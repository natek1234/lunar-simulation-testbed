"""The seam: a Controller turns an Observation into an Action. ScriptedSequencer runs a
list of (skill, params) through the executor registry -- the single-placement baseline's shape."""
from typing import Protocol

from testbed.control.records import Action, Observation, SkillAction, TaskAction


class Controller(Protocol):
    name: str
    def reset(self, robot, seed: int) -> None: ...
    def act(self, obs: Observation) -> Action: ...
    def done(self, obs: Observation) -> bool: ...


class ScriptedSequencer:
    """Emits SkillActions in order; the runtime executes each through the registry and
    calls back `advance()` when the executor reports done."""
    name = "scripted"

    def __init__(self, script):
        self.script = tuple((n, dict(p)) for n, p in script)
        self.i = 0

    def reset(self, robot, seed):
        self.i = 0

    def act(self, obs):
        if self.i >= len(self.script):
            return TaskAction()
        name, params = self.script[self.i]
        return SkillAction(name=name, params=params)

    def advance(self):
        self.i += 1

    def done(self, obs):
        return self.i >= len(self.script)

    @property
    def phase(self) -> str | None:
        return self.script[self.i][0] if self.i < len(self.script) else None
