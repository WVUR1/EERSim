"""Post-step ovphysx 0.5.11 contacts in the PyBullet contact tuple layout.

Create after scene attachment, before its first normal simulation step. The
exporter must author PhysxContactReportAPI on every subscribed dynamic actor/link. Recreate
the reader after topology changes; cached native actor IDs are scene-local.

The SDK exposes one world contact point, not separate surface witnesses. Both
point fields retain that reported point, with separation preserved separately.
Raw contact reports do not expose friction vectors; their optional tuple fields
are placeholders, not measured zero friction.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from numbers import Integral

import numpy as np


@dataclass(frozen=True)
class _Actor:
    path: str
    body: int
    link: int
    dynamic: bool

    @property
    def endpoint(self):
        return self.body, self.link


class ContactReader:
    """Reuse one unfiltered binding; capture only after step_sync completes."""

    def __init__(self, physx, manifest, capacity=131072):
        if not isinstance(capacity, Integral) or isinstance(capacity, bool) or capacity <= 0:
            raise ValueError("Contact capacity must be a positive integer")
        self.capacity = int(capacity)
        self._binding = None
        self._closed = False
        self._id_cache = {}
        self._actors = {}
        endpoints = set()
        for item in manifest["actors"]:
            path = item["path"]
            if not isinstance(path, str) or not path.startswith("/") or any(
                    part in ("", ".", "..") for part in path[1:].split("/")):
                raise ValueError(f"Invalid manifest actor path: {path!r}")
            body, link = item["body_id"], item["link_index"]
            if any(not isinstance(value, Integral) or isinstance(value, bool) for value in (body, link)):
                raise ValueError(f"Actor IDs must be integers: {path}")
            if body < 0 or link < -1:
                raise ValueError(f"Invalid body/link IDs: {path}")
            mass = float(item["mass_kg"])
            if not math.isfinite(mass) or mass < 0:
                raise ValueError(f"Invalid actor mass: {path}")
            actor = _Actor(path, int(body), int(link), mass > 0)
            if path in self._actors or actor.endpoint in endpoints:
                raise ValueError(f"Duplicate manifest actor path or body/link: {path}")
            self._actors[path] = actor
            endpoints.add(actor.endpoint)
        self._prefixes = sorted(self._actors, key=len, reverse=True)
        dynamics = {path for path, actor in self._actors.items() if actor.dynamic}
        requested = manifest.get('contact_sensor_paths', sorted(dynamics))
        if not isinstance(requested, list) or len(requested) != len(set(requested)) or not set(requested) <= dynamics:
            raise ValueError('Contact subscriptions must name unique dynamic manifest actors')
        expected = set(requested)
        if not expected:
            raise ValueError("Contact capture requires at least one dynamic manifest actor")
        try:
            self._binding = physx.create_contact_binding(
                sensor_patterns=sorted(expected), max_contact_data_count=self.capacity)
            paths = list(self._binding.sensor_paths)
            if len(paths) != len(set(paths)) or set(paths) != expected:
                raise RuntimeError("Contact sensor coverage differs from subscribed dynamic manifest actors; "
                                   "check PhysxContactReportAPI on each exact actor/link prim. "
                                   f"Missing={sorted(expected-set(paths))}; unexpected={sorted(set(paths)-expected)}")
            if self._binding.max_contact_data_count != self.capacity:
                raise RuntimeError("Native contact capacity differs from requested capacity")
            self._sensors = [self._actors[path] for path in paths]
            self._sensor_endpoints = {actor.endpoint for actor in self._sensors}
            count = len(paths)
            self._forces = np.zeros((self.capacity, 1), dtype=np.float32)
            self._points = np.zeros((self.capacity, 3), dtype=np.float32)
            self._normals = np.zeros((self.capacity, 3), dtype=np.float32)
            self._separations = np.zeros((self.capacity, 1), dtype=np.float32)
            self._counts = np.zeros(count, dtype=np.int32)
            self._starts = np.zeros(count, dtype=np.int32)
            self._other_ids = np.zeros(self.capacity, dtype=np.uint64)
        except Exception:
            self.close()
            raise

    def _actor_at_path(self, path):
        if not isinstance(path, str) or not path:
            raise RuntimeError("Native contact actor ID has no resolvable physics-object path")
        for prefix in self._prefixes:
            if path == prefix or path.startswith(prefix + "/"):
                return self._actors[prefix]
        raise RuntimeError(f"Native contact actor is absent from the export manifest: {path}")

    def capture(self):
        """Return every manifold point once, with normals from tuple bodyB to A.

        Mirrored reports for two subscribed dynamics are matched as multisets;
        identical contacts within one manifold remain separate. Missing mirrors,
        unknown actor mappings and native buffer overflow raise rather than
        silently dropping contact information.
        """
        if self._closed:
            raise RuntimeError("ContactReader is closed")
        self._counts.fill(0)
        self._starts.fill(0)
        try:
            self._binding.read_raw_contact_data(self._forces, self._points, self._normals,
                                                self._separations, self._counts, self._starts,
                                                self._other_ids)
        except RuntimeError as exc:
            required = int(np.max(self._starts.astype(np.int64) + self._counts.astype(np.int64)))
            if required > self.capacity:
                raise RuntimeError(f"Contact buffer overflow: capacity={self.capacity}, required={required}. "
                                   "This step's payload is unavailable; recreate the reader with more capacity.") from exc
            raise
        total = 0
        for start, count in zip(self._starts, self._counts):
            if count < 0 or start != total:
                raise RuntimeError("Native contact count/start layout is malformed")
            total += int(count)
        if total > self.capacity:
            raise RuntimeError(f"Native contact payload exceeds capacity: {total}>{self.capacity}")
        for data in (self._forces, self._points, self._normals, self._separations):
            if not np.isfinite(data[:total]).all():
                raise RuntimeError("Native contact payload contains nonfinite values")
        if total and not np.allclose(np.linalg.norm(self._normals[:total], axis=1), 1, atol=1e-3, rtol=0):
            raise RuntimeError("Native contact normals are not unit vectors")
        missing = sorted({int(value) for value in self._other_ids[:total]} - self._id_cache.keys())
        if missing:
            paths = self._binding.get_other_actor_paths_from_ids(np.asarray(missing, dtype=np.uint64))
            if len(paths) != len(missing):
                raise RuntimeError("Native contact ID resolver returned an incomplete path list")
            resolved = {key: self._actor_at_path(path) for key, path in zip(missing, paths)}
            self._id_cache.update(resolved)

        # Each bucket contains all reports of one unordered body/link pair,
        # separated by reporting endpoint. Keep contact multiplicity, not merely
        # a set of positions: compound leaves can have distinct coincident hits.
        pairs = {}
        for sensor, start, count in zip(self._sensors, self._starts, self._counts):
            for index in range(int(start), int(start + count)):
                other = self._id_cache[int(self._other_ids[index])]
                if sensor.endpoint == other.endpoint:
                    raise RuntimeError(f"Contact reports an actor colliding with itself: {sensor.path}")
                force = float(self._forces[index, 0])
                normal = self._normals[index].astype(float)
                # Pinned CPU and CUDA raw APIs keep the original actor1->actor0
                # normal while negating force for a sensor on actor1. signbit
                # preserves that orientation even for signed zero force.
                if np.signbit(force):
                    normal = -normal
                a, b = sensor.endpoint, other.endpoint
                side = 0
                if a > b:
                    a, b, side = b, a, 1
                    normal = -normal
                point = tuple(float(value) for value in self._points[index])
                contact = (0, a[0], b[0], a[1], b[1], point, point,
                           tuple(float(value) for value in normal),
                           float(self._separations[index, 0]), abs(force),
                           0., (0., 0., 0.), 0., (0., 0., 0.))
                reports = pairs.setdefault((a, b), ([], []))
                reports[side].append(contact)
        contacts = []
        for (a, b), (first, second) in pairs.items():
            if a in self._sensor_endpoints and b in self._sensor_endpoints:
                if Counter(first) != Counter(second):
                    raise RuntimeError(f"Mirrored contact reports disagree for body/link pair {a}, {b}")
                contacts.extend(first)
            else:
                contacts.extend(first)
                contacts.extend(second)
        return contacts

    def close(self):
        if self._closed:
            return
        self._closed = True
        binding, self._binding = self._binding, None
        self._id_cache.clear()
        if binding is not None:
            binding.destroy()
