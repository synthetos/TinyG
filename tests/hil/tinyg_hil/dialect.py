"""Per-build protocol differences: 440 (master) and 449 (edge-0.97)."""
import json
import pathlib
from dataclasses import dataclass, field

DATA = pathlib.Path(__file__).parent / "data"

# Combined machine state ("stat"), from cmCombinedState in canonical_machine.h
_STAT_440 = ["INITIALIZING", "READY", "ALARM", "PROGRAM_STOP", "PROGRAM_END", "RUN", "HOLD", "PROBE",
             "CYCLE", "HOMING", "JOG", "SHUTDOWN"]
_STAT_449 = ["INITIALIZING", "READY", "ALARM", "PROGRAM_STOP", "PROGRAM_END", "RUN", "HOLD", "PROBE",
             "CYCLE", "HOMING", "JOG", "INTERLOCK", "SHUTDOWN", "PANIC"]
# Feedhold state ("hold"), from cmFeedholdState
_HOLD_440 = ["OFF", "SYNC", "PLAN", "DECEL", "HOLD", "END_HOLD"]
_HOLD_449 = ["OFF", "REQUESTED", "SYNC", "DECEL_CONTINUE", "DECEL_TO_ZERO", "PENDING", "HOLD"]


@dataclass
class Dialect:
    name: str                   # "d440" or "d449"
    stat_names: list
    hold_names: list
    footer_revs: tuple
    table: dict = field(repr=False)

    @property
    def tokens(self) -> dict:
        return self.table["tokens"]

    def status(self, name: str) -> int:
        """Status code by name, e.g. status("MCODE_COMMAND_UNSUPPORTED") -> 132."""
        return self.table["status"][name]

    def status_name(self, code: int) -> str:
        for name, value in self.table["status"].items():
            if value == code:
                return name
        return str(code)

    def stat(self, name: str) -> int:
        return self.stat_names.index(name)

    def hold(self, name: str) -> int:
        return self.hold_names.index(name)

    @property
    def idle_stats(self) -> set:
        return {self.stat("READY"), self.stat("PROGRAM_STOP"), self.stat("PROGRAM_END")}

    def persisted_tokens(self) -> list:
        return [t for t, row in self.tokens.items() if row["persist"]]


def load(name: str) -> Dialect:
    table = json.loads((DATA / f"{name}.json").read_text())
    if name == "d440":
        return Dialect(name, _STAT_440, _HOLD_440, (1,), table)
    return Dialect(name, _STAT_449, _HOLD_449, (2, 3), table)


def for_build(fb: float) -> Dialect:
    return load("d440" if fb < 445 else "d449")
