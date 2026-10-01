"""GLFW keyboard device: polls PHYSICAL key state every call (a held-key deadman needs
state, not events) and derives press edges itself. Key codes probed empirically against
this repo's pinned glfw 2.10.2 [measured: 2026-09-20, every KEY_<letter>/KEY_<digit>/
KEY_SPACE/KEY_ESCAPE/KEY_EQUAL/KEY_MINUS/KEY_LEFT_BRACKET/KEY_RIGHT_BRACKET name resolves]."""
import glfw

from testbed.control.keyboard_rc import KeyState

# Key bindings per docs/guide/teleop.md; codes are read from the glfw module itself,
# never hardcoded scancodes. [policy default]
KEYMAP = {**{c: getattr(glfw, f"KEY_{c}") for c in "WASDQEIJKLUOMRXHF"}, **{d: getattr(glfw, f"KEY_{d}") for d in "1234567"},
          "SPACE": glfw.KEY_SPACE, "ESC": glfw.KEY_ESCAPE, "=": glfw.KEY_EQUAL, "-": glfw.KEY_MINUS,
          "[": glfw.KEY_LEFT_BRACKET, "]": glfw.KEY_RIGHT_BRACKET}


class KeyboardDevice:
    def __init__(self, window):
        self.window = window
        self._prev: frozenset = frozenset()

    def poll(self) -> KeyState:
        glfw.poll_events()
        focused = bool(glfw.get_window_attrib(self.window, glfw.FOCUSED))
        held = frozenset(name for name, code in KEYMAP.items() if glfw.get_key(self.window, code) == glfw.PRESS)
        pressed = held - self._prev
        self._prev = held
        return KeyState(held, pressed, focused)
