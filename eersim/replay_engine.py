"""Worker-owned replay lifecycle; recorded presentation never steps a world."""
from __future__ import annotations

import copy

from .replay import ReplayCursor, ReplayRecorder
from .replay_rendering import ReplayRenderer


class ReplayController:
    def __init__(self, log, *, recorder_type=ReplayRecorder, renderer_type=ReplayRenderer):
        self.log = log
        self._recorder_type, self._renderer_type = recorder_type, renderer_type
        self.recorder = self.clip = self.renderer = self.cursor = None
        self.active = False
        self.generation = 0
        self.reason = ""
        self.camera = {"yaw": -45., "pitch": -55., "distance": 5., "target": None}
        self._recording_camera = copy.deepcopy(self.camera)
        self._clip_camera = copy.deepcopy(self.camera)
        self._cached_clip = self._cached_index = self._cached_frame = None
        self._last_renderer_status = None

    def changed_view(self):
        self.generation += 1

    def _dispose(self, value, label):
        try:
            value.close()
        except Exception as exc:
            self.log(f"Replay {label} cleanup: {exc}")

    @staticmethod
    def _observation(session, observed=None):
        return {"phase": session.phase, "mode_time": session.elapsed_ticks * session.dt,
                "observed": session.state() if observed is None else observed}

    def begin(self, session, camera):
        self.discard()
        self.reason = ""
        self._recording_camera = copy.deepcopy(camera)
        try:
            self.recorder = self._recorder_type.start(
                session.world, mode=session.mode, duration=session.duration,
                **self._observation(session))
        except Exception as exc:
            self.reason = f"Replay recording unavailable: {exc}"
            self.log(self.reason)

    def capture(self, session, observed):
        if self.recorder is None:
            return
        try:
            with session.performance.span("session.replay.capture"):
                added = self.recorder.capture(session.world, **self._observation(session, observed))
            if added:
                session.performance.increment("session.replay.frames")
        except Exception as exc:
            self.discard()
            self.reason = f"Replay recording stopped: {exc}"
            self.log(self.reason)

    def finish(self, session, reason, observed=None):
        recorder, self.recorder = self.recorder, None
        if recorder is None:
            return
        try:
            clip = recorder.finish(session.world, reason=reason, **self._observation(session, observed))
        except Exception as exc:
            self._dispose(recorder, "recording")
            self.reason = f"Replay recording could not be finalized: {exc}"
            self.log(self.reason)
            return
        if clip is None:
            self._dispose(recorder, "recording")
            return
        self.leave()
        previous, self.clip = self.clip, clip
        self._cached_clip = self._cached_index = self._cached_frame = None
        self._clip_camera = copy.deepcopy(self._recording_camera)
        self.camera = copy.deepcopy(self._clip_camera)
        self.cursor = ReplayCursor(clip.duration)
        self.reason = ""
        if previous is not None:
            self._dispose(previous, "previous clip")
        self.log(f"Replay ready: {clip.duration:.2f} simulated seconds")

    def discard(self):
        recorder, self.recorder = self.recorder, None
        if recorder is not None:
            self._dispose(recorder, "recording")

    def enter(self, session, renderer):
        # These are pacing/input changes only. Never call stop/disable or write
        # historical poses, joint targets, sensors or RNG state to this session.
        session.paused = True
        session.controller = {"analog": {}, "digital": {}}
        self.changed_view()
        if self.clip is None:
            self.reason = "No recorded run yet"
            return
        if self.active:
            return
        self.reason = ""
        self._last_renderer_status = None
        try:
            self.renderer = self._renderer_type(self.clip, renderer=renderer)
            self.cursor = ReplayCursor(self.clip.duration)
            self.active = True
        except Exception as exc:
            self._close_renderer()
            self.active = False
            self.reason = f"Replay could not be opened: {exc}"
            self.log(self.reason)

    def _close_renderer(self):
        renderer, self.renderer = self.renderer, None
        if renderer is not None:
            try:
                renderer.close()
            except Exception as exc:
                self.log(f"Replay renderer cleanup: {exc}")

    def leave(self):
        if self.cursor is not None:
            self.cursor.pause()
        self._close_renderer()
        self.active = False
        self.changed_view()

    def seek(self, seconds):
        if self.active:
            self.cursor.seek(seconds)

    def play(self):
        if self.active:
            self.cursor.play()

    def pause(self):
        if self.cursor is not None:
            self.cursor.pause()

    def advance(self, elapsed):
        if self.active:
            self.cursor.advance(elapsed)

    def set_camera(self, camera):
        self.camera = copy.deepcopy(camera)

    def selected_frame(self):
        if self.clip is None or self.cursor is None:
            return None
        index = self.clip.index_at(self.cursor.position_s)
        if self._cached_clip is not self.clip or self._cached_index != index:
            self._cached_frame = self.clip.frame_at(index)
            self._cached_clip, self._cached_index = self.clip, index
        return self._cached_frame

    def render(self, width, height):
        if not self.active or self.renderer is None:
            return None
        try:
            self.renderer.set_frame(self.selected_frame())
            self.renderer.set_camera(**self.camera)
            pixels = self.renderer.render(width, height)
            status = getattr(self.renderer, "renderer_status", {})
            if status and status != self._last_renderer_status:
                self._last_renderer_status = copy.deepcopy(status)
                self.log(f"Replay renderer: {status.get('device', 'unknown')} · {status.get('reason', '')}")
            return pixels
        except Exception as exc:
            self.leave()
            self.reason = f"Replay rendering failed: {exc}"
            self.log(self.reason)
            return None

    def metadata(self):
        result = {"available": self.clip is not None, "active": self.active,
                  "playing": bool(self.active and self.cursor and self.cursor.playing),
                  "position_s": 0., "duration_s": 0., "sample_index": 0,
                  "sample_count": 0, "effective_hz": 0., "generation": self.generation,
                  "camera": copy.deepcopy(self.camera), "reason": self.reason,
                  "renderer": copy.deepcopy(self._last_renderer_status or {}),
                  "recording": self.recorder is not None}
        if self.clip is not None:
            result.update(duration_s=self.clip.duration, sample_count=self.clip.frame_count,
                          effective_hz=self.clip.effective_hz, mode=self.clip.mode,
                          termination=self.clip.termination)
            frame = self.selected_frame()
            if frame is not None:
                result.update(position_s=frame.time_s,
                              sample_index=self.clip.index_at(self.cursor.position_s),
                              phase=frame.phase, mode_time=frame.mode_time,
                              pose=list(frame.observed.get("pose", (0., 0., 0., 0.))))
        return result

    def close(self):
        self.leave()
        self.discard()
        clip, self.clip = self.clip, None
        self._cached_clip = self._cached_index = self._cached_frame = None
        if clip is not None:
            self._dispose(clip, "clip")
