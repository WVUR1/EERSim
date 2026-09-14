from copy import deepcopy
from types import SimpleNamespace

import pytest

from eersim.tuning_simulation import SimulationTuner, motor_sample


TARGET = {"id":"arm", "ports":[1,-2], "axes":{"shaft":[1,1]}}


class World:
    motor_bindings = [{"port": 1}, {"port": 2}]
    def __init__(self):
        self.time = 0.
        self.commands = []
        self.disabled = 0
        self.position = 0
    def snapshot(self):
        return {"time":self.time,"motors":{"1":{"position":self.position,"velocity":0},
                "2":{"position":-self.position,"velocity":0}}}
    def step(self, command):
        self.commands.append(deepcopy(command))
        self.time += .01
        return self.snapshot()
    def disable(self): self.disabled += 1


def fixture():
    world = World()
    def stop():
        session.running = False
        world.disable()
    session = SimpleNamespace(world=world,dt=.01,manual_joints={},paused=False,running=True,
                              stop=stop,state=lambda snapshot=None:snapshot or world.snapshot())
    samples, states = [], []
    return SimulationTuner(samples.append,states.append), session, samples, states


def test_simulation_characterization_stops_user_code_uses_bound_ports_and_disables_at_end():
    tuner, session, samples, states = fixture()
    tuner.experiment(session, {"target":TARGET,"axis":"shaft","duration_s":.4,"voltage_mv":2000})
    assert not session.running
    for _ in range(130):
        if tuner.active: tuner.step(session)
    assert not tuner.active and session.world.disabled == 2
    powered = [c for c in session.world.commands if c["motors"]["1"]["value"]]
    assert powered and all(c["motors"]["2"]["value"] == -2000 for c in powered)
    assert samples[0]["motors"]["1"]["voltage_mv"] == 0
    assert samples[-1]["motors"]["1"]["voltage_mv"] == 0
    assert states[-1]["state"] == "completed"
    assert samples[-1]["time"] > 1


def test_travel_guard_checks_each_motor_not_only_cancelled_group_average():
    tuner, session, samples, states = fixture()
    tuner.experiment(session, {"target":TARGET,"axis":"shaft","max_travel_deg":30})
    session.world.position = 1
    tuner.step(session)
    assert not tuner.active and "Travel limit" in states[-1]["message"]
    assert not session.world.commands


def test_invalid_port_does_not_stop_existing_robot():
    tuner, session, samples, states = fixture()
    with pytest.raises(ValueError, match="Map"):
        tuner.experiment(session, {"target":{"ports":[5],"axes":{"shaft":[1]}}, "axis":"shaft"})
    assert session.running and not session.world.disabled


def test_cancel_detaches_observer_and_disables_active_actuators():
    tuner, session, samples, states = fixture()
    tuner.capture(session,True,TARGET)
    observer = session.motor_observer
    observer(session.world.snapshot(), {"motors":{"1":{"mode":"velocity","value":100}}})
    assert samples[-1]["motors"]["1"]["voltage_mv"] is None
    tuner.cancel(session)
    assert session.motor_observer is None
    tuner.experiment(session, {"target":TARGET,"axis":"shaft"})
    tuner.cancel(session)
    assert not tuner.active and session.paused and session.world.disabled == 2


def test_capture_restart_has_new_id_after_reset():
    tuner, session, samples, states = fixture()
    tuner.capture(session,True,TARGET)
    first = states[-1]["capture_id"]
    tuner.capture(session,True,TARGET)
    assert first != states[-1]["capture_id"]


def test_voltage_edges_between_nominal_capture_ticks_are_recorded():
    tuner, session, samples, states = fixture()
    tuner.capture(session,True,TARGET)
    tuner.observe(session.world.snapshot(), {"motors":{}})
    session.world.time = .01
    tuner.observe(session.world.snapshot(), {"motors":{"1":{"mode":"voltage","value":2000}}})
    assert [s["time"] for s in samples] == [0,.01]
    assert samples[-1]["motors"]["1"]["voltage_mv"] == 2000
