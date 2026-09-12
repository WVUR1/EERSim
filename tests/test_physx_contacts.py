"""Contact compatibility checks with mocked SDK buffers; no physics runtime."""
import numpy as np
import pytest

from eersim.physx_contacts import ContactReader


def actor(path, body, link=-1, mass=1.):
    return {"path": path, "body_id": body, "link_index": link, "mass_kg": mass}


class Binding:
    def __init__(self, sensors, reports=None, paths=None, capacity=32):
        self.sensor_paths = sensors
        self.max_contact_data_count = capacity
        self.reports = reports or {}
        self.paths = paths or {}
        self.resolutions = []
        self.destroyed = 0

    def read_raw_contact_data(self, forces, points, normals, separations, counts, starts, ids):
        cursor = 0
        for row, sensor in enumerate(self.sensor_paths):
            entries = self.reports.get(sensor, [])
            starts[row], counts[row] = cursor, len(entries)
            cursor += len(entries)
        if cursor > self.max_contact_data_count:
            raise RuntimeError("OVPHYSX_API_BUFFER_TOO_SMALL")
        for row, sensor in enumerate(self.sensor_paths):
            for index, (other, point, normal, separation, force) in enumerate(self.reports.get(sensor, []), int(starts[row])):
                forces[index, 0] = force
                points[index], normals[index] = point, normal
                separations[index, 0], ids[index] = separation, other

    def get_other_actor_paths_from_ids(self, ids):
        assert ids.dtype == np.uint64
        self.resolutions.append(ids.tolist())
        return [self.paths.get(int(value), "") for value in ids]

    def destroy(self):
        self.destroyed += 1


class Physics:
    def __init__(self, binding):
        self.binding = binding
        self.arguments = None

    def create_contact_binding(self, **kwargs):
        self.arguments = kwargs
        return self.binding


def make_reader(binding, actors):
    physics = Physics(binding)
    reader = ContactReader(physics, {"actors": actors}, capacity=binding.max_contact_data_count)
    assert physics.arguments == {"sensor_patterns": sorted(a["path"] for a in actors if a["mass_kg"] > 0),
                                 "max_contact_data_count": binding.max_contact_data_count}
    return reader


@pytest.mark.parametrize("force,normal", [(7., (0, 0, 1)), (-7., (0, 0, -1)), (-0., (0, 0, -1))])
def test_static_contact_orientation_signed_zero_and_normal_force(force, normal):
    binding = Binding(["/World/wheel"], {
        "/World/wheel": [(101, (1, 2, 3), normal, -0.001, force)]},
        {101: "/World/floor/collider_0"})
    reader = make_reader(binding, [actor("/World/floor", 1, mass=0), actor("/World/wheel", 2, link=3)])
    contacts = reader.capture()
    assert len(contacts) == 1
    contact = contacts[0]
    assert contact[:5] == (0, 1, 2, -1, 3)
    assert contact[5] == contact[6] == (1., 2., 3.)
    assert contact[7] == (0., 0., -1.)  # Canonical bodyB wheel -> bodyA floor.
    assert contact[8] == pytest.approx(-0.001)
    assert contact[9] == abs(force)
    assert len(contact) == 14
    assert reader.capture() == contacts
    assert binding.resolutions == [[101]]
    reader.close()
    reader.close()
    assert binding.destroyed == 1
    with pytest.raises(RuntimeError, match="closed"):
        reader.capture()


def test_mirrored_reports_keep_manifold_points_and_contact_multiplicity():
    points = [(0, 0, 0), (0, 0, 0), (1, 0, 0)]
    binding = Binding(["/World/b", "/World/a"], {
        "/World/a": [(12, point, (1, 0, 0), -0.01, 4.) for point in points],
        "/World/b": [(11, point, (1, 0, 0), -0.01, -4.) for point in reversed(points)],
    }, {11: "/World/a", 12: "/World/b"})
    reader = make_reader(binding, [actor("/World/a", 7, link=0), actor("/World/b", 7, link=1)])
    contacts = reader.capture()
    assert len(contacts) == 3
    assert sum(contact[5] == (0., 0., 0.) for contact in contacts) == 2
    assert all(contact[1:5] == (7, 7, 0, 1) and contact[7] == (1., 0., 0.) for contact in contacts)


def test_prefix_mapping_prefers_nearest_actor_and_requires_path_boundary():
    binding = Binding(["/World/wheel"], {
        "/World/wheel": [(101, (0, 0, 0), (0, 0, 1), 0, 1.)]},
        {101: "/World/field/goal/collider"})
    reader = make_reader(binding, [actor("/World/field", 1, mass=0),
                                   actor("/World/field/goal", 4, mass=0), actor("/World/wheel", 9)])
    assert reader.capture()[0][1:3] == (4, 9)
    binding.reports["/World/wheel"][0] = (102, (0, 0, 0), (0, 0, 1), 0, 1.)
    binding.paths[102] = "/World/field_other/collider"
    with pytest.raises(RuntimeError, match="absent from the export manifest"):
        reader.capture()


def test_unknown_native_id_is_an_error_even_when_force_is_zero():
    binding = Binding(["/World/a"], {"/World/a": [(999, (0, 0, 0), (0, 0, 1), 0, 0.)]})
    reader = make_reader(binding, [actor("/World/a", 1)])
    with pytest.raises(RuntimeError, match="no resolvable"):
        reader.capture()


def test_missing_sensor_api_cannot_silently_reduce_coverage():
    binding = Binding(["/World/a"])
    with pytest.raises(RuntimeError, match="PhysxContactReportAPI"):
        make_reader(binding, [actor("/World/a", 1), actor("/World/b", 2)])
    assert binding.destroyed == 1


def test_overflow_reports_required_capacity_without_partial_contacts():
    binding = Binding(["/World/a"], {"/World/a": [(11, (0, 0, 0), (0, 0, 1), 0, 1.)] * 3}, capacity=2)
    reader = make_reader(binding, [actor("/World/a", 1)])
    with pytest.raises(RuntimeError, match="capacity=2, required=3"):
        reader.capture()
    assert binding.resolutions == []


def test_missing_mirrored_report_is_an_error():
    binding = Binding(["/World/a", "/World/b"], {
        "/World/a": [(12, (0, 0, 0), (1, 0, 0), 0, 2.)]}, {12: "/World/b"})
    reader = make_reader(binding, [actor("/World/a", 1), actor("/World/b", 2)])
    with pytest.raises(RuntimeError, match="Mirrored contact reports disagree"):
        reader.capture()


@pytest.mark.parametrize("normal,force", [((0, 0, 0), 1.), ((0, 0, 1), float("nan"))])
def test_invalid_native_payload_is_an_error(normal, force):
    binding = Binding(["/World/a"], {"/World/a": [(11, (0, 0, 0), normal, 0, force)]},
                      {11: "/World/floor"})
    reader = make_reader(binding, [actor("/World/a", 1), actor("/World/floor", 2, mass=0)])
    with pytest.raises(RuntimeError, match="nonfinite|not unit"):
        reader.capture()


def test_selected_contact_sensors_resolve_unsubscribed_dynamic_neighbors():
    # Read only a robot wheel. Its cup still collides and must resolve correctly,
    # without requiring a mirrored observation from the unsubscribed cup.
    actors = [actor('/World/wheel', 2, link=3), actor('/World/cup', 4)]
    binding = Binding(['/World/wheel'], {
        '/World/wheel': [(101, (0, 0, 1), (0, 0, 1), -.002, 9.)]},
        {101: '/World/cup/collider_0'})
    physics = Physics(binding)
    reader = ContactReader(physics, {'actors': actors, 'contact_sensor_paths': ['/World/wheel']}, capacity=32)
    assert physics.arguments['sensor_patterns'] == ['/World/wheel']
    assert reader.capture()[0][1:5] == (2, 4, 3, -1)
    reader.close()


@pytest.mark.parametrize('requested', [['/World/missing'], ['/World/wheel', '/World/wheel'], ['/World/floor']])
def test_invalid_contact_subscription_fails_before_native_binding(requested):
    physics = Physics(None)
    with pytest.raises(ValueError, match='unique dynamic'):
        ContactReader(physics, {'actors': [actor('/World/wheel', 2), actor('/World/floor', 0, mass=0)],
                               'contact_sensor_paths': requested})
    assert physics.arguments is None
