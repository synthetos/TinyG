"""Machine configurations as data: profiles, matrices, and applying them to the board.

    tests/hil/config/profiles/<name>.json   a named layer of settings (may "extends" other profiles)
    tests/hil/config/matrices/<name>.json   dimensions of variation, combined "full" or "pairwise"

A test declares what it is sensitive to with markers; the harness builds one ConfigSpec per combination,
applies only the settings that differ from what is on the board (EEPROM wear), reads each one back,
and restores the user's settings at the end of the session:

    @pytest.mark.matrix("motor-geometry")          # one run per combination
    @pytest.mark.profile("homing-z", "switches-nc") # extra layers for this test
    @pytest.mark.settings(zsv=300)                  # inline one-off (prefer a profile)
    def test_x(tg, machine): ...machine.steps_per_unit("x")...

Command line: --profile NAME (layer applied to every hardware test, e.g. run the whole suite on a
0.9 degree machine), --matrix NAME (expand every hardware test), --list-configs.

Profile file:
    {"description": "...", "extends": ["other"], "requires": {"dialect": ["d449"], "ice": false},
     "motors": {"1-4": {"sa": 0.9}},          -> 1sa..4sa
     "axes": {"xyz": {"vm": 800}},            -> xvm, yvm, zvm
     "settings": {"st": 1},                   -> literal tokens
     "dialects": {"d449": {"settings": {...}}}}  -> extra layer only on that firmware dialect
"""
import itertools
import json
import math
import pathlib
from dataclasses import dataclass, field

CONFIG = pathlib.Path(__file__).resolve().parent.parent / "config"


class ConfigError(Exception):
    pass


def _expand_block(block: dict) -> dict:
    out = {}
    for motors, values in block.get("motors", {}).items():
        if "-" in motors:
            lo, hi = (int(x) for x in motors.split("-"))
            numbers = range(lo, hi + 1)
        else:
            numbers = [int(m) for m in motors.split(",")]
        for m in numbers:
            for tok, v in values.items():
                out[f"{m}{tok}"] = v
    for axes, values in block.get("axes", {}).items():
        for a in axes.replace(",", ""):
            for tok, v in values.items():
                out[f"{a}{tok}"] = v
    out.update(block.get("settings", {}))
    return out


@dataclass
class Profile:
    name: str
    settings: dict
    requires: dict = field(default_factory=dict)
    by_dialect: dict = field(default_factory=dict)
    description: str = ""


def load_profile(name: str, _seen=()) -> Profile:
    if name in _seen:
        raise ConfigError(f"profile cycle: {' -> '.join(_seen + (name,))}")
    path = CONFIG / "profiles" / f"{name}.json"
    if not path.exists():
        raise ConfigError(f"no profile {name!r} ({path})")
    data = json.loads(path.read_text())
    settings, requires, by_dialect = {}, {}, {}
    for parent in data.get("extends", []):
        p = load_profile(parent, _seen + (name,))
        settings.update(p.settings)
        requires.update(p.requires)
        for d, s in p.by_dialect.items():
            by_dialect.setdefault(d, {}).update(s)
    settings.update(_expand_block(data))
    requires.update(data.get("requires", {}))
    for d, block in data.get("dialects", {}).items():
        by_dialect.setdefault(d, {}).update(_expand_block(block))
    return Profile(name, settings, requires, by_dialect, data.get("description", ""))


@dataclass(frozen=True)
class ConfigSpec:
    layers: tuple = ()          # profile names, or (("inline", ((tok, value), ...)),) entries
    label: str = "default"

    def resolve(self, dialect=None):
        """Flat settings (later layers win) and merged requirements."""
        settings, requires = {}, {}
        for layer in self.layers:
            if isinstance(layer, tuple) and layer and layer[0] == "inline":
                settings.update(dict(layer[1]))
                continue
            p = load_profile(layer)
            settings.update(p.settings)
            if dialect:
                settings.update(p.by_dialect.get(dialect, {}))
            for k, v in p.requires.items():
                requires[k] = sorted(set(requires.get(k, [])) & set(v)) if isinstance(v, list) and k in requires else v
        return settings, requires

    def plus(self, *layers, label=None):
        layers = tuple(l for l in layers if l)
        return ConfigSpec(self.layers + layers, label or "+".join(x for x in (self.label, *[_layer_label(l) for l in layers]) if x and x != "default") or "default")


def _layer_label(layer):
    if isinstance(layer, tuple):
        return ",".join(f"{k}={v}" for k, v in layer[1])
    return layer


def inline(**settings):
    return ("inline", tuple(sorted(settings.items())))


def pairwise(dimensions: dict) -> list:
    """Greedy all-pairs: every value pair of every two dimensions appears in at least one combination."""
    names = list(dimensions)
    if len(names) < 2:
        return [dict(zip(names, combo)) for combo in itertools.product(*dimensions.values())]
    uncovered = {(a, va, b, vb) for a, b in itertools.combinations(names, 2)
                 for va in dimensions[a] for vb in dimensions[b]}
    combos = []
    while uncovered:
        best, best_cover = None, -1
        for combo in itertools.product(*(dimensions[n] for n in names)):
            pick = dict(zip(names, combo))
            cover = sum(1 for a, b in itertools.combinations(names, 2) if (a, pick[a], b, pick[b]) in uncovered)
            if cover > best_cover:
                best, best_cover = pick, cover
        combos.append(best)
        uncovered -= {(a, best[a], b, best[b]) for a, b in itertools.combinations(names, 2)}
    return combos


def load_matrix(name: str) -> list:
    """ConfigSpecs for every combination of a matrix file."""
    path = CONFIG / "matrices" / f"{name}.json"
    if not path.exists():
        raise ConfigError(f"no matrix {name!r} ({path})")
    data = json.loads(path.read_text())
    dims = {d: list(variants) for d, variants in data["dimensions"].items()}
    combos = pairwise(dims) if data.get("combine", "full") == "pairwise" else \
        [dict(zip(dims, c)) for c in itertools.product(*dims.values())]
    specs = []
    for combo in combos:
        layers = []
        for dim, variant in combo.items():
            ref = data["dimensions"][dim][variant]
            layers.append(ref if isinstance(ref, str) else inline(**_expand_block(ref)))
        specs.append(ConfigSpec(tuple(layers), "-".join(f"{variant}" for variant in combo.values())))
    return specs


def validate(settings: dict, dialect) -> list:
    """Problems with a resolved settings dict for this firmware's token table."""
    return [f"{tok}: not a setting on {dialect.name}" for tok in settings if tok not in dialect.tokens]


class Machine:
    """The configuration a test runs under, as actually read back from the board."""

    def __init__(self, spec: ConfigSpec, settings: dict, dialect):
        self.spec, self.settings, self.dialect = spec, settings, dialect

    @property
    def label(self):
        return self.spec.label

    def get(self, token, default=None):
        return self.settings.get(token, default)

    def motors_for(self, axis):
        index = "xyzabc".index(axis)
        return [m for m in range(1, 7) if self.settings.get(f"{m}ma") == index]

    def steps_per_unit(self, axis):
        """Steps per mm (or degree) of the first motor mapped to an axis: 360 / sa * mi / tr."""
        motors = self.motors_for(axis)
        if not motors:
            raise ConfigError(f"no motor mapped to {axis}")
        m = motors[0]
        return 360.0 / self.settings[f"{m}sa"] * self.settings[f"{m}mi"] / self.settings[f"{m}tr"]

    def info(self):
        return {"label": self.label, "layers": [_layer_label(l) for l in self.spec.layers], "settings": self.settings}


GEOMETRY_TOKENS = [f"{m}{t}" for m in range(1, 5) for t in ("ma", "sa", "tr", "mi")]


def apply(tg, spec: ConfigSpec, base: dict) -> Machine:
    """Make the board's settings equal base + spec layers, writing only differences; read back and verify."""
    settings, requires = spec.resolve(tg.dialect.name)
    if requires.get("dialect") and tg.dialect.name not in requires["dialect"]:
        raise ConfigError(f"{spec.label}: needs dialect {requires['dialect']}, board is {tg.dialect.name}")
    problems = validate(settings, tg.dialect)
    if problems:
        raise ConfigError(f"{spec.label}: " + "; ".join(problems))
    current = getattr(tg, "_config_applied", None)
    if current is None:
        current = tg._config_applied = dict(base)
    desired = dict(base)
    desired.update(settings)
    for token in set(current) | set(desired):
        if token not in desired or token in ("fb", "fv", "hp", "hv", "id", "baud"):
            continue
        if token in current and tg.values_equal(token, current[token], desired[token]):
            continue
        tg.set(token, desired[token], allow=f"apply config {spec.label}")
        current[token] = desired[token]
    readback = {}
    for token in sorted(set(settings) | set(GEOMETRY_TOKENS)):
        if token not in tg.dialect.tokens:
            continue
        value = tg.get(token)
        readback[token] = value
        if token in settings and not tg.values_equal(token, value, settings[token]):
            raise ConfigError(f"{spec.label}: {token} reads back {value}, wanted {settings[token]} (clamped?)")
    return Machine(spec, readback, tg.dialect)


def specs_for_test(global_profiles, test_profiles, matrices, inline_settings):
    """Every ConfigSpec a test must run under."""
    base = ConfigSpec().plus(*global_profiles) if global_profiles else ConfigSpec()
    base = base.plus(*test_profiles) if test_profiles else base
    matrix_specs = [load_matrix(m) for m in matrices]
    extra = inline(**inline_settings) if inline_settings else None
    if not matrix_specs:
        return [base.plus(extra) if extra else base]
    out = []
    for combo in itertools.product(*matrix_specs):
        spec = base
        for s in combo:
            spec = spec.plus(*s.layers, label="+".join(x for x in (spec.label, s.label) if x != "default"))
        out.append(spec.plus(extra) if extra else spec)
    return out


def describe(spec: ConfigSpec) -> str:
    settings, requires = spec.resolve()
    body = ", ".join(f"{k}={v}" for k, v in sorted(settings.items())) or "(board settings unchanged)"
    return f"{spec.label}: {body}" + (f"  requires {requires}" if requires else "")


__all__ = ["ConfigSpec", "Machine", "apply", "load_profile", "load_matrix", "pairwise", "specs_for_test",
           "describe", "inline", "ConfigError", "math"]
