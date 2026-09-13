"""Replay lifecycle and queue checks with fake sessions/renderers only."""
import bisect
import copy
import queue
import random
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from eersim.performance import PerformanceRecorder
from eersim.replay import ReplayCursor
from eersim.replay_engine import ReplayController


def frame(tick, dt=.1):
    return SimpleNamespace(tick=tick, time_s=tick*dt, phase='skills_auto', mode_time=tick*dt,
                           observed={'pose': [tick*.01, 0., 0., 0.]}, poses=np.array([[tick*.01, 0, 0, 0, 0, 0, 1.]]))


class Clip:
    mode = 'skills_auto'
    effective_hz = 30.
    termination = 'completed'

    def __init__(self, frames=None, events=None):
        self.frames = frames or [frame(i) for i in range(11)]
        self.duration = self.frames[-1].time_s
        self.frame_count = len(self.frames)
        self.events = events if events is not None else []
        self.closed = False

    def frame_at(self, index):
        assert not self.closed
        return self.frames[index]

    def index_at(self, seconds):
        return max(0, min(self.frame_count-1, bisect.bisect_right([f.time_s for f in self.frames], seconds)-1))

    def close(self):
        self.closed = True
        self.events.append('clip.close')


class Renderer:
    def __init__(self, clip, *, renderer):
        self.clip = clip
        self.current = None

    def set_frame(self, value):
        self.current = value

    def set_camera(self, **values):
        self.camera = values

    def render(self, width, height):
        return np.array([self.current.tick])

    def close(self):
        self.clip.events.append('renderer.close')


class Recorder:
    @classmethod
    def start(cls, world, **kwargs):
        result = cls()
        result.frames = [frame(world.tick, world.dt)]
        result.closed = False
        return result

    def capture(self, world, **kwargs):
        self.frames.append(frame(world.tick, world.dt))
        return True

    def finish(self, world, **kwargs):
        return Clip(self.frames) if self.frames[-1].tick > self.frames[0].tick else None

    def close(self):
        self.closed = True


class Session:
    instances = []

    def __init__(self, config=None, log=None):
        self.instances.append(self)
        self.world = self
        self.dt = 1/120
        self.performance = PerformanceRecorder(False)
        self.renderer_status = {'device': 'fake rendering'}
        self.motor_bindings = []
        self.stop_calls = self.start_calls = 0
        self.rng = random.Random(3)
        self.reset()

    def reset(self):
        self.tick = self.elapsed_ticks = 0
        self.duration = 8*self.dt
        self.mode, self.phase = 'disabled', 'disabled'
        self.paused = self.running = False
        self.manual_joints = {}
        self.controller = {'analog': {'left_y': 127}, 'digital': {'A': True}}
        self.pose = [0., 0., 0., 0.]

    def start(self, mode, executable):
        self.start_calls += 1
        self.reset()
        self.mode, self.phase = mode, 'skills_auto'
        self.running = True

    def step(self):
        self.tick += 1
        self.elapsed_ticks += 1
        self.pose[0] += .01 + self.rng.random()*.0001
        if self.elapsed_ticks >= 8:
            self.running = False
        return self.state()

    def state(self):
        return {'pose': list(self.pose), 'tick': self.tick, 'running': self.running, 'paused': self.paused}

    def render(self, *args, **kwargs):
        return np.array([self.tick])

    def stop(self):
        self.stop_calls += 1
        self.running = False

    def close(self):
        pass


def controller(clip=None, **kwargs):
    result = ReplayController(lambda message: None, recorder_type=Recorder, renderer_type=Renderer, **kwargs)
    result.clip = clip or Clip()
    result.cursor = ReplayCursor(result.clip.duration)
    return result


def test_scrubbing_is_read_only_for_live_physics_and_keeps_input_neutral():
    live = Session()
    live.start('skills_auto', None)
    live.step()
    before = (live.tick, live.elapsed_ticks, copy.deepcopy(live.pose), live.rng.getstate(), live.stop_calls, live.start_calls)
    replay = controller()
    replay.enter(live, 'cpu')
    assert live.paused and live.controller == {'analog': {}, 'digital': {}}
    for seconds in (.7, .1, 1., 0., .45):
        replay.seek(seconds)
        replay.set_camera({'yaw': 35, 'pitch': -20, 'distance': 3., 'target': [0, 0, 0]})
        pixels = replay.render(10, 10)
        assert pixels.tolist() == [replay.selected_frame().tick]
    replay.play()
    replay.advance(10)
    assert not replay.cursor.playing
    replay.leave()
    assert live.paused
    assert before == (live.tick, live.elapsed_ticks, live.pose, live.rng.getstate(), live.stop_calls, live.start_calls)
    replay.close()


def test_begin_failure_and_discard_preserve_previous_clip():
    class BrokenRecorder:
        @classmethod
        def start(cls, *args, **kwargs):
            raise ValueError('unsupported visual')
    replay, live = controller(), Session()
    previous = replay.clip
    replay._recorder_type = BrokenRecorder
    replay.begin(live, replay.camera)
    assert replay.recorder is None and replay.clip is previous and not previous.closed
    assert 'unsupported visual' in replay.metadata()['reason']
    replay._recorder_type = Recorder
    replay.begin(live, replay.camera)
    active = replay.recorder
    replay.discard()
    assert active.closed and replay.clip is previous and not previous.closed
    replay.close()


def test_rejected_enter_still_acknowledges_the_request_without_stopping_robot():
    replay = ReplayController(lambda message: None, recorder_type=Recorder, renderer_type=Renderer)
    live = Session()
    before = replay.generation
    replay.enter(live, 'cpu')
    assert replay.generation > before and not replay.active
    assert replay.metadata()['reason'] == 'No recorded run yet'
    assert live.paused and live.stop_calls == 0
    replay.close()


def test_empty_run_cannot_replace_previous_and_new_completed_run_closes_it():
    replay, live = controller(), Session()
    previous = replay.clip
    replay.begin(live, replay.camera)
    replay.finish(live, 'failed')
    assert replay.clip is previous and not previous.closed
    replay.begin(live, replay.camera)
    replay.capture(live, live.step())
    replay.finish(live, 'stopped')
    assert replay.clip is not previous and previous.closed
    replay.close()


def test_capture_error_and_render_error_do_not_stop_live_session():
    live, replay = Session(), controller()
    replay.begin(live, replay.camera)
    active = replay.recorder
    active.capture = lambda *args, **kwargs: (_ for _ in ()).throw(ValueError('sample failure'))
    replay.capture(live, live.state())
    assert active.closed and replay.recorder is None and not replay.clip.closed
    replay.enter(live, 'cpu')
    replay.play()
    replay.renderer.render = lambda *args: (_ for _ in ()).throw(RuntimeError('renderer failure'))
    assert replay.render(10, 10) is None
    assert not replay.active and not replay.cursor.playing and live.paused and live.stop_calls == 0
    assert 'renderer failure' in replay.metadata()['reason']
    replay.close()


def test_renderer_is_closed_before_its_clip_assets():
    replay, live = controller(), Session()
    events = replay.clip.events
    replay.enter(live, 'cpu')
    replay.close()
    assert events == ['renderer.close', 'clip.close']


def test_cleanup_failure_is_isolated_from_live_robot_and_new_recording():
    replay, live = controller(), Session()
    messages = []
    replay.log = messages.append
    replay.begin(live, replay.camera)
    replay.recorder.close = lambda: (_ for _ in ()).throw(OSError('locked cache file'))
    replay.discard()
    assert replay.recorder is None and live.stop_calls == 0
    replay.clip.close = lambda: (_ for _ in ()).throw(OSError('locked old mesh'))
    replay.begin(live, replay.camera)
    replay.capture(live, live.step())
    replay.finish(live, 'stopped')
    assert replay.clip.frame_count == 2 and live.stop_calls == 0
    assert any('locked cache file' in text for text in messages)
    assert any('locked old mesh' in text for text in messages)
    replay.close()


def test_metadata_reuses_immutable_selected_frame_until_selected_sample_changes():
    replay, live = controller(), Session()
    calls = []
    original = replay.clip.frame_at
    replay.clip.frame_at = lambda index: (calls.append(index), original(index))[1]
    replay.enter(live, 'cpu')
    for _ in range(100):
        replay.metadata()
    assert calls == [0]
    replay.seek(.45)
    replay.metadata()
    replay.render(10, 10)
    assert calls == [0, 4]
    replay.close()


def test_replay_renderer_fallback_is_logged_only_when_status_changes():
    replay, live = controller(), Session()
    messages = []
    replay.log = messages.append
    replay.enter(live, 'auto')
    replay.renderer.renderer_status = {'device': 'CPU', 'reason': 'No supported graphics driver'}
    replay.render(10, 10)
    replay.render(10, 10)
    assert len(messages) == 1 and 'CPU' in messages[0]
    assert replay.metadata()['renderer']['reason'] == 'No supported graphics driver'
    replay.close()


def test_seek_queue_retains_only_latest_drag_position():
    from eersim.engine import SimulationEngine
    facade = SimpleNamespace(_replay_seek_lock=threading.Lock(), _replay_seek_pending=False,
                             _replay_seek_value=0., _closing=threading.Event(), _queue=queue.Queue())
    for value in range(1000):
        SimulationEngine.replay_seek(facade, value/10)
    assert facade._queue.qsize() == 1
    assert SimulationEngine._take_replay_seek(facade) == 99.9


def test_engine_replay_controls_cannot_advance_or_drive_paused_live_run(monkeypatch):
    from PySide6.QtCore import QCoreApplication
    from eersim import engine
    from eersim.config import default_config
    app = QCoreApplication.instance() or QCoreApplication([])
    states, pixels = [], []
    Session.instances = []
    monkeypatch.setattr(engine, 'Session', Session)
    monkeypatch.setattr(engine, 'ReplayController', lambda log: ReplayController(log, recorder_type=Recorder, renderer_type=Renderer))
    worker = engine.SimulationEngine(defer_configuration=True)
    worker.replay_state.connect(states.append)
    worker.replay_frame.connect(pixels.append)

    def wait_for(predicate):
        deadline = time.monotonic()+4
        while not predicate() and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(.002)
        assert predicate(), states[-3:]

    try:
        worker.configure(default_config())
        worker.start('skills_auto')
        wait_for(lambda: states and states[-1]['available'])
        # Keep a new run paused while looking at the preceding completed run.
        worker.start('skills_auto')
        worker.pause()
        wait_for(lambda: Session.instances[-1].start_calls == 2 and Session.instances[-1].paused)
        live = Session.instances[-1]
        worker.replay_enter()
        wait_for(lambda: states[-1]['active'])
        before = (live.tick, live.elapsed_ticks, list(live.pose), live.rng.getstate(), live.stop_calls)
        for value in range(100):
            worker.replay_seek(value/1000)
            worker.set_controller({'analog': {'left_y': 127}})
            worker.set_joint('claw', 1.)
            worker.single_step()
            worker.resume()
            worker.stop()
        worker.replay_play()
        wait_for(lambda: pixels and pixels[-1]['pixels'].tolist() == [8]
                 and not states[-1]['playing'] and states[-1]['position_s'] == states[-1]['duration_s'])
        assert pixels[-1]['pixels'].tolist() == [8]
        assert pixels[-1]['generation'] == states[-1]['generation']
        assert before == (live.tick, live.elapsed_ticks, live.pose, live.rng.getstate(), live.stop_calls)
        assert live.controller == {'analog': {}, 'digital': {}} and not live.manual_joints
        worker.replay_live()
        wait_for(lambda: not states[-1]['active'])
        assert live.paused and live.running
        worker.reset()
        wait_for(lambda: live.tick == 0 and not live.running)
        assert states[-1]['available']
    finally:
        worker.close()
