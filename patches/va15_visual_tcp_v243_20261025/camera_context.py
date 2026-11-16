"""Read display/tracker phase only; cannot request any robot motion."""
import json


class Context:
    def __init__(self):
        self.phase = None

    def update(self, path, nonce, now, required):
        try:
            packet = json.loads(path.read_text())
            phase, stamp = packet['phase'], packet['monotonic_ns']
            valid = (packet['session_id'] == nonce and type(phase) is int and phase in (0, 1, 2, 3) and
                     type(stamp) is int and 0 <= now-stamp <= 1_000_000_000)
        except (OSError, ValueError, TypeError, KeyError):
            valid, phase = False, None
        if not valid:
            return dict(valid=not required, phase=None, reset=False, reacquire=not required)
        # 0=prone/not policy, 1=quiet standby, 2=following, 3=preparing/stopping.
        # Acquiring while stationary never grants permission to start walking.
        reset = phase != self.phase and (phase == 0 or (phase == 1 and self.phase != 2))
        self.phase = phase
        return dict(valid=True, phase=phase, reset=reset, reacquire=phase in (0, 1))
