"""Bounded recorded-pose playback; capture never advances or edits physics."""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import math
from pathlib import Path
import shlex
import shutil
import tempfile
from types import MappingProxyType
from typing import Mapping
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_BUDGET = 64 << 20


def _readonly(values, dtype=None):
    array = np.ascontiguousarray(values, dtype=dtype)
    # Bytes-backed views cannot be made writable by a frame consumer.
    return np.frombuffer(array.tobytes(), dtype=array.dtype).reshape(array.shape)


def _finite_tuple(values, size, label):
    result = tuple(float(x) for x in values)
    if len(result) != size or not all(math.isfinite(x) for x in result):
        raise ValueError(f'Replay {label} requires {size} finite values')
    return result


def _quat_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array((aw*bx + ax*bw + ay*bz - az*by,
                     aw*by - ax*bz + ay*bw + az*bx,
                     aw*bz + ax*by - ay*bx + az*bw,
                     aw*bw - ax*bx - ay*by - az*bz))


def _rotate(q, point):
    x, y, z, w = q
    scale = 2. / float(np.dot(q, q))
    matrix = np.array(((1-scale*(y*y+z*z), scale*(x*y-z*w), scale*(x*z+y*w)),
                       (scale*(x*y+z*w), 1-scale*(x*x+z*z), scale*(y*z-x*w)),
                       (scale*(x*z-y*w), scale*(y*z+x*w), 1-scale*(x*x+y*y))))
    return matrix @ point


def _link_from_com(position, quaternion, inertial_position, inertial_quaternion):
    """T_world_link = T_world_COM * inverse(T_link_COM), in float64."""
    position, quaternion = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    local, rotation = np.asarray(inertial_position, dtype=float), np.asarray(inertial_quaternion, dtype=float)
    if np.array_equal(local, (0., 0., 0.)) and np.array_equal(rotation, (0., 0., 0., 1.)):
        return np.concatenate((position, quaternion))
    norm = float(np.dot(rotation, rotation))
    if norm <= 0 or not math.isfinite(norm):
        raise ValueError('Replay scene has an invalid base inertial quaternion')
    inverse = np.array((-rotation[0], -rotation[1], -rotation[2], rotation[3])) / norm
    link_quaternion = _quat_product(quaternion, inverse)
    return np.concatenate((position - _rotate(link_quaternion, local), link_quaternion))


class _Assets:
    """One owned asset tree, transferred from recorder to its finalized clip."""
    def __init__(self, root=None, limit=512 << 20):
        if root is None:
            from .runtime_paths import user_data_dir
            root = user_data_dir() / 'replays'
        root = Path(root).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        self.root = root
        self.path = Path(tempfile.mkdtemp(prefix='recording-', dir=root))
        self.limit, self.bytes = int(limit), 0
        self.files, self.active, self.albedo = {}, set(), {}
        self.closed = False

    def pin(self, source):
        source = Path(source).expanduser().resolve()
        if source in self.files:
            return self.files[source]
        if source in self.active:
            raise ValueError(f'Replay asset dependency cycle: {source.name}')
        if not source.is_file():
            raise ValueError(f'Replay asset is missing: {source.name}')
        size = source.stat().st_size
        if size > self.limit - self.bytes:
            raise ValueError('Replay visual assets exceed the configured cache budget')
        data = source.read_bytes()
        if len(data) > self.limit - self.bytes:
            raise ValueError('Replay visual asset grew beyond the cache budget')
        self.bytes += len(data)
        digest = hashlib.sha256(str(source).encode() + b'\0' + data).hexdigest()[:24]
        original_size = len(data)
        destination = self.path / digest / ('asset' + source.suffix.lower())
        destination.parent.mkdir()
        self.active.add(source)
        try:
            suffix = source.suffix.lower()
            if suffix in ('.obj', '.mtl'):
                lines = []
                for line in data.decode('utf-8-sig').splitlines():
                    lexer = shlex.shlex(line, posix=True)
                    lexer.whitespace_split, lexer.escape = True, ''
                    tokens = list(lexer)
                    if tokens and suffix == '.obj' and tokens[0] == 'mtllib':
                        references = [self.pin(source.parent / name) for name in tokens[1:]]
                        line = 'mtllib ' + ' '.join(self._relative(p, destination.parent) for p in references)
                        self.albedo.setdefault(source, set()).update(set().union(*(
                            self.albedo.get((source.parent / name).resolve(), set()) for name in tokens[1:])))
                    elif tokens and suffix == '.mtl' and (tokens[0].lower().startswith('map_') or
                                                          tokens[0].lower() in ('bump', 'disp', 'decal', 'refl')):
                        split = self._texture_start(tokens)
                        texture = self.pin(source.parent / ' '.join(tokens[split:]))
                        line = ' '.join(tokens[:split]) + ' ' + self._relative(texture, destination.parent)
                        if tokens[0].lower() == 'map_kd':
                            self.albedo.setdefault(source, set()).add(texture)
                    lines.append(line)
                data = ('\n'.join(lines) + '\n').encode()
            elif suffix == '.dae':
                tree = ET.fromstring(data)
                for library in tree.iter():
                    if library.tag.rsplit('}', 1)[-1] != 'library_images':
                        continue
                    for node in library.iter():
                        if node.tag.rsplit('}', 1)[-1] == 'init_from' and node.text:
                            reference = node.text.strip()
                            if '://' in reference:
                                raise ValueError('Replay Collada textures must use local file paths')
                            texture = self.pin(source.parent / reference)
                            node.text = self._relative(texture, destination.parent)
                data = ET.tostring(tree, encoding='utf-8', xml_declaration=True)
            difference = len(data) - original_size
            if difference > self.limit - self.bytes:
                raise ValueError('Replay rewritten assets exceed the cache budget')
            self.bytes += difference
            destination.write_bytes(data)
            self.files[source] = destination
            return destination
        finally:
            self.active.remove(source)

    @staticmethod
    def _texture_start(tokens):
        index = 1
        fixed = {'-blendu': 1, '-blendv': 1, '-cc': 1, '-clamp': 1, '-texres': 1,
                 '-bm': 1, '-boost': 1, '-imfchan': 1, '-type': 1, '-mm': 2}
        while index < len(tokens) and tokens[index].startswith('-'):
            option = tokens[index]
            index += 1
            if option in ('-o', '-s', '-t'):
                count = 0
                while index < len(tokens) and count < 3:
                    try:
                        float(tokens[index])
                    except ValueError:
                        break
                    index += 1
                    count += 1
                if not count:
                    raise ValueError(f'Material texture option {option} requires numbers')
            elif option in fixed:
                index += fixed[option]
            else:
                raise ValueError(f'Unsupported material texture option: {option}')
        if index >= len(tokens):
            raise ValueError('Material texture filename is missing')
        return index

    def _relative(self, path, directory):
        import os
        return Path(os.path.relpath(path, directory)).as_posix()

    def close(self):
        if self.closed:
            return
        resolved = self.path.resolve()
        if resolved.parent != self.root or not resolved.name.startswith('recording-') or self.path.is_symlink():
            raise RuntimeError('Replay cache ownership changed; refusing cleanup')
        if resolved.exists():
            shutil.rmtree(resolved, ignore_errors=False)
        self.closed = True


@dataclass(frozen=True)
class ReplayVisual:
    row: int
    kind: int
    dimensions: tuple[float, ...]
    mesh_path: str | None
    local_position: tuple[float, float, float]
    local_orientation: tuple[float, float, float, float]
    rgba: tuple[float, float, float, float]
    texture_path: str | None = None


@dataclass(frozen=True)
class ReplayScene:
    name: str
    row_keys: tuple[tuple[int, int], ...]
    visuals: tuple[ReplayVisual, ...]
    initial_poses: np.ndarray = field(repr=False, compare=False)
    _assets: _Assets = field(repr=False, compare=False)

    @property
    def asset_directory(self):
        return self._assets.path

    def close(self):
        self._assets.close()


def _urdf_textures(path):
    from .importers import _asset_path
    path = Path(path).expanduser().resolve()
    tree = ET.parse(path).getroot()
    materials = {node.get('name'): node for node in tree.findall('material')}
    children = {node.find('child').get('link') for node in tree.findall('joint')}
    roots = [node.get('name') for node in tree.findall('link') if node.get('name') not in children]
    result = {}
    for link in tree.findall('link'):
        textures = []
        for visual in link.findall('visual'):
            material = visual.find('material')
            if material is not None and material.find('texture') is None:
                material = materials.get(material.get('name'), material)
            texture = None if material is None else material.find('texture')
            textures.append(None if texture is None else
                            _asset_path(texture.get('filename', ''), path.parent, 'Replay URDF texture'))
        result[link.get('name')] = textures
    return (roots[0] if len(roots) == 1 else None), result


class _CaptureSource:
    def __init__(self, world, assets):
        self.world, self.api, self.client = world, world._physics, world.client
        self.inertials, self.native = {}, callable(getattr(self.api, '_actor_pose', None))
        self.native_rows = dict(getattr(self.api, 'keys', {})) if self.native else {}
        self.native_actors = getattr(self.api, 'actors', {}) if self.native else {}
        api = self.api
        bodies = [api.getBodyUniqueId(i, physicsClientId=self.client)
                  for i in range(api.getNumBodies(physicsClientId=self.client))]
        urdfs = {}
        robot_path = getattr(world.config, 'robot_urdf', None)
        if robot_path:
            urdfs[world.robot] = Path(robot_path)
        for body, item, directory in getattr(world, '_export_items', []):
            if 'urdf' in item:
                urdfs[body] = Path(directory) / item['urdf']
        rows, visuals, moving = [], [], set()
        supported = {api.GEOM_BOX, api.GEOM_SPHERE, api.GEOM_CYLINDER, api.GEOM_CAPSULE, api.GEOM_MESH}
        self.indices = {}
        for body in bodies:
            count = api.getNumJoints(body, physicsClientId=self.client)
            dynamic = body == world.robot or body in getattr(world, '_dynamic_field_bodies', ())
            if not dynamic:
                dynamic = any(api.getDynamicsInfo(body, link, physicsClientId=self.client)[0] > 0
                              for link in range(-1, count))
            if not self.native:
                inertia = getattr(world, 'base_inertial_frames', {}).get(body)
                if inertia is None:
                    info = api.getDynamicsInfo(body, -1, physicsClientId=self.client)
                    inertia = info[3:5]
                self.inertials[body] = (_finite_tuple(inertia[0], 3, 'inertial position'),
                                        _finite_tuple(inertia[1], 4, 'inertial orientation'))
            texture_layout = None
            shape_ordinals = {}
            for shape in api.getVisualShapeData(body, flags=api.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS,
                                                physicsClientId=self.client) or ():
                _, link, kind, dimensions, filename, local, orient, color = shape[:8]
                ordinal = shape_ordinals.get(link, 0)
                shape_ordinals[link] = ordinal + 1
                if color[3] <= 0:
                    continue
                if kind not in supported:
                    raise ValueError(f'Replay does not support visual geometry type {kind}')
                if kind == api.GEOM_MESH and not filename:
                    raise ValueError('Replay requires a file for each visual mesh')
                key = (body, link)
                if key not in self.indices:
                    self.indices[key] = len(rows)
                    rows.append(key)
                    if dynamic:
                        moving.add(self.indices[key])
                mesh = None
                source_mesh = None
                if filename:
                    source_mesh = Path(filename.decode() if isinstance(filename, bytes) else filename)
                    if not source_mesh.is_absolute() and body in urdfs:
                        relative = urdfs[body].expanduser().resolve().parent / source_mesh
                        if relative.is_file():
                            source_mesh = relative
                    mesh = str(assets.pin(source_mesh))
                texture = None
                if len(shape) > 8 and shape[8] >= 0:
                    if texture_layout is None and body in urdfs:
                        texture_layout = _urdf_textures(urdfs[body])
                    source_texture = None
                    if texture_layout is not None:
                        root_name, by_link = texture_layout
                        name = root_name if link < 0 else api.getJointInfo(body, link, physicsClientId=self.client)[12].decode()
                        choices = by_link.get(name, [])
                        if ordinal < len(choices):
                            source_texture = choices[ordinal]
                    if source_texture is not None:
                        texture = str(assets.pin(source_texture))
                    elif source_mesh is not None:
                        choices = assets.albedo.get(source_mesh.resolve(), set())
                        if len(choices) == 1:
                            texture = str(next(iter(choices)))
                    if texture is None:
                        raise ValueError('Replay cannot recover the file for an active visual texture')
                visuals.append(ReplayVisual(self.indices[key], int(kind), _finite_tuple(dimensions, 3, 'visual dimensions'),
                                            mesh, _finite_tuple(local, 3, 'visual origin'),
                                            _finite_tuple(orient, 4, 'visual rotation'),
                                            _finite_tuple(color, 4, 'visual color'), texture))
        self.rows = tuple(rows)
        self.moving = tuple(sorted(moving))
        initial = self.read(tuple(range(len(rows))))
        self.scene = ReplayScene(str(getattr(world, 'game', {}).get('name', 'Recorded scene')),
                                 self.rows, tuple(visuals), _readonly(initial), assets)

    def read(self, indices):
        world, api = self.world, self.api
        if world._physics is not api or world.client != self.client:
            raise ValueError('The live scene changed while recording')
        values = np.empty((len(indices), 7), dtype=np.float64)
        if self.native:
            state = api._state
            for target, index in enumerate(indices):
                key = self.rows[index]
                row = self.native_rows.get(key)
                if row is not None:
                    values[target] = state['poses'][row]
                else:
                    actor = self.native_actors[key]
                    values[target] = (*actor['position'], *actor['orientation'])
        else:
            for target, index in enumerate(indices):
                body, link = self.rows[index]
                if link < 0:
                    values[target] = _link_from_com(*api.getBasePositionAndOrientation(body, physicsClientId=self.client),
                                                     *self.inertials[body])
                else:
                    state = api.getLinkState(body, link, computeForwardKinematics=1, physicsClientId=self.client)
                    values[target] = (*state[4], *state[5])
        if not np.isfinite(values).all() or np.any(np.sum(values[:, 3:]**2, axis=1) <= 0):
            raise ValueError('Replay capture received non-finite poses or invalid quaternions')
        return values


@dataclass(frozen=True)
class ReplayFrame:
    time_s: float
    tick: int
    phase: str
    mode_time: float
    observed: Mapping
    poses: np.ndarray = field(repr=False, compare=False)

    @property
    def time(self):
        return self.time_s


@dataclass(frozen=True)
class _Sample:
    tick: int
    phase: str
    mode_time: float
    observed: Mapping


def _observed(values):
    values = values or {}
    result = {}
    for name, size in (('pose', 4), ('otos', 3)):
        if name in values:
            result[name] = _finite_tuple(values[name], size, name)
    heading = values.get('imu_heading_deg', values.get('imu', {}).get('heading'))
    if heading is not None and math.isfinite(float(heading)):
        result['imu_heading_deg'] = float(heading)
    placed = values.get('placed_pins', values.get('scoring', {}).get('placed_pins'))
    if placed is not None and isinstance(placed, (int, float)):
        result['placed_pins'] = int(placed)
    return MappingProxyType(result)


class ReplayClip:
    def __setattr__(self, name, value):
        if getattr(self, '_sealed', False):
            raise AttributeError('Replay clips are immutable')
        object.__setattr__(self, name, value)

    def __init__(self, scene, mode, dt, origin_tick, moving, chunks, samples, stride, reason):
        self.scene, self.mode, self.dt = scene, str(mode), float(dt)
        self.origin_tick, self.moving = int(origin_tick), tuple(moving)
        self._chunks = tuple(chunks)
        for chunk in self._chunks:
            chunk.setflags(write=False)
        self._samples = tuple(samples)
        self._ticks = _readonly([sample.tick for sample in samples], np.int64)
        self.effective_hz, self.termination = 1. / (stride * self.dt), str(reason)
        self.closed = False
        self._sealed = True

    @property
    def frame_count(self):
        return len(self._samples)

    @property
    def duration(self):
        return (int(self._ticks[-1]) - self.origin_tick) * self.dt if self.frame_count else 0.

    @property
    def bytes_used(self):
        return (self.scene.initial_poses.nbytes +
                sum((chunk.base if isinstance(chunk.base, np.ndarray) else chunk).nbytes for chunk in self._chunks) +
                self.frame_count * 1024)

    def index_at(self, seconds):
        seconds = float(seconds)
        if not math.isfinite(seconds):
            raise ValueError('Replay time must be finite')
        target = self.origin_tick + max(0., min(self.duration, seconds)) / self.dt
        return max(0, min(self.frame_count - 1, int(np.searchsorted(self._ticks, target + 1e-9, side='right')) - 1))

    def frame_at(self, index):
        if self.closed:
            raise RuntimeError('Replay clip is closed')
        if not isinstance(index, (int, np.integer)) or not 0 <= index < self.frame_count:
            raise IndexError('Replay frame index is out of range')
        sample = self._samples[index]
        poses = self.scene.initial_poses.copy()
        remaining = int(index)
        for chunk in self._chunks:
            if remaining < len(chunk):
                if self.moving:
                    poses[list(self.moving)] = chunk[remaining]
                break
            remaining -= len(chunk)
        return ReplayFrame((sample.tick - self.origin_tick) * self.dt, sample.tick, sample.phase,
                           sample.mode_time, sample.observed, _readonly(poses))

    def close(self):
        if not self.closed:
            # Frame memory and access are released even if Windows temporarily
            # locks a cached mesh. Keep the scene owner so another close can
            # retry its idempotent asset cleanup after that lock is released.
            object.__setattr__(self, '_chunks', ())
            object.__setattr__(self, 'closed', True)
        self.scene.close()


class ReplayRecorder:
    """Engine-owned active recording; a failed/empty attempt yields no clip."""
    @classmethod
    def start(cls, world, *, mode, duration, capture_hz=30., budget_bytes=DEFAULT_BUDGET,
              cache_root=None, phase='initialize', mode_time=0., observed=None):
        return cls(world, mode=mode, duration=duration, capture_hz=capture_hz,
                   budget_bytes=budget_bytes, cache_root=cache_root,
                   phase=phase, mode_time=mode_time, observed=observed)

    def __init__(self, world, *, mode, duration, capture_hz=30., budget_bytes=DEFAULT_BUDGET,
                 cache_root=None, phase='initialize', mode_time=0., observed=None):
        self.source = self.assets = None
        self.finished = False
        self.chunks, self.samples = [], []
        self._count = 0
        self.mode, self.dt = str(mode), float(world.dt)
        duration, capture_hz = float(duration), float(capture_hz)
        if not all(math.isfinite(v) and v > 0 for v in (self.dt, duration, capture_hz)):
            raise ValueError('Replay duration, time step and capture rate must be positive and finite')
        self.origin_tick = int(world.tick)
        try:
            self.assets = _Assets(cache_root)
            self.source = _CaptureSource(world, self.assets)
            # Reserve a conservative fixed allowance for each compact sample's
            # metadata, plus immutable initial scene poses. Never store sensors
            # or a mutable world's status/diagnostic tree in this budget.
            frame_bytes = len(self.source.moving) * 7 * 8 + 1024
            self.max_frames = (int(budget_bytes) - self.source.scene.initial_poses.nbytes) // frame_bytes
            if self.max_frames < 2:
                raise ValueError('Replay memory budget cannot hold the initial and final scene poses')
            expected_ticks = math.ceil((duration + 30. + self.dt) / self.dt)
            self.stride = max(1, math.ceil(1. / (capture_hz * self.dt)),
                              math.ceil(expected_ticks / max(1, self.max_frames - 2)))
            self._next_tick = self.origin_tick + self.stride
            self.chunk_size = min(64, self.max_frames, max(1, (512 << 10) // max(1, len(self.source.moving) * 7 * 8)))
            self.capture(world, phase=phase, mode_time=mode_time, observed=observed, force=True)
        except BaseException:
            self.close()
            raise

    @property
    def effective_hz(self):
        return 1. / (self.stride * self.dt)

    @property
    def frame_count(self):
        return self._count

    def capture(self, world, *, phase, mode_time, observed=None, force=False):
        if self.finished:
            return False
        if world is not self.source.world:
            raise ValueError('Replay capture belongs to a different world')
        tick = int(world.tick)
        if tick < self.origin_tick or (self.samples and tick < self.samples[-1].tick):
            raise ValueError('Replay simulation ticks moved backwards')
        if self.samples and tick == self.samples[-1].tick:
            if force:
                self.samples[-1] = _Sample(tick, str(phase), float(mode_time), _observed(observed))
            return False
        if not force and tick < self._next_tick:
            return False
        limit = self.max_frames if force else self.max_frames - 1
        if self._count >= limit:
            raise ValueError('Replay recording exceeded its bounded duration/memory allocation')
        poses = self.source.read(self.source.moving)
        if self._count % self.chunk_size == 0:
            remaining = min(self.chunk_size, self.max_frames - self._count)
            self.chunks.append(np.empty((remaining, len(self.source.moving), 7), np.float64))
        self.chunks[-1][self._count % self.chunk_size] = poses
        self.samples.append(_Sample(tick, str(phase), float(mode_time), _observed(observed)))
        self._count += 1
        self._next_tick = self.origin_tick + ((tick - self.origin_tick) // self.stride + 1) * self.stride
        return True

    def finish(self, world, *, phase, mode_time, observed=None, reason='completed'):
        if self.finished:
            return None
        if int(world.tick) <= self.origin_tick:
            self.close()
            return None
        self.capture(world, phase=phase, mode_time=mode_time, observed=observed, force=True)
        count = self._count
        chunks = []
        for index, chunk in enumerate(self.chunks):
            used = min(count, len(chunk))
            # Freeze one bounded chunk at a time and release its mutable source.
            # Earlier recorder references cannot edit a completed clip. The
            # temporary copy is bounded by one chunk, normally at most 512 KiB.
            chunks.append(_readonly(chunk[:used]))
            self.chunks[index] = None
            count -= used
        result = ReplayClip(self.source.scene, self.mode, self.dt, self.origin_tick,
                            self.source.moving, chunks, self.samples, self.stride, reason)
        self.finished = True
        self.chunks, self.samples = [], []
        self.source = self.assets = None
        return result

    def close(self):
        if self.assets is not None:
            self.assets.close()
        self.assets = self.source = None
        self.chunks, self.samples = [], []
        self.finished = True


class ReplayCursor:
    """Playback wall pacing only; clip lookup separately selects saved samples."""
    def __init__(self, duration_s):
        self.duration_s = float(duration_s)
        if not math.isfinite(self.duration_s) or self.duration_s < 0:
            raise ValueError('Replay duration must be finite and nonnegative')
        self.position_s = 0.
        self.playing = False

    def seek(self, seconds):
        seconds = float(seconds)
        if not math.isfinite(seconds):
            raise ValueError('Replay time must be finite')
        self.position_s = max(0., min(self.duration_s, seconds))
        if self.position_s >= self.duration_s:
            self.playing = False
        return self.position_s

    def play(self):
        if self.position_s >= self.duration_s:
            self.position_s = 0.
        self.playing = self.duration_s > 0

    def pause(self):
        self.playing = False

    def advance(self, elapsed_wall_seconds, speed=1.):
        elapsed_wall_seconds, speed = float(elapsed_wall_seconds), float(speed)
        if (not math.isfinite(elapsed_wall_seconds) or elapsed_wall_seconds < 0 or
                not math.isfinite(speed) or speed <= 0):
            raise ValueError('Replay pacing requires finite nonnegative elapsed time and positive speed')
        return self.seek(self.position_s + elapsed_wall_seconds * speed if self.playing else self.position_s)
