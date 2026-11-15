import math
import os
import random
import tempfile
from collections.abc import Callable
from pathlib import Path

from jinja2 import StrictUndefined, Template


def appearance_rng(seed: int | None, stream: str = "scene") -> random.Random:
    """The random stream the looks of a run are drawn from: its own, so that the seed's
    placements stay the same whether the looks vary or not."""
    return random.Random() if seed is None else random.Random(f"appearance {stream} {seed}")


def vary_appearance(rng: random.Random, appearance: float, base: float, low: float, high: float):
    """`base` moved towards a value drawn from [low, high] by `appearance` (0 to 1)."""
    drawn = rng.uniform(low, high)  # always drawn, so the stream does not depend on `appearance`
    return base + min(max(appearance, 0.0), 1.0) * (drawn - base)


def randomize_scenario_locations(
    scenario_template_file: Path | str,
    DX: float,
    DY: float,
    DZ: float,
    DYAW: float,
    render_robot: Callable[..., str],
    seed: int | None = None,
    appearance: float = 0.0,
    **variables,
) -> Path:
    """Render the scenario template; `variables` are passed to it as they are.

    A variable the template uses but is not given raises jinja2.UndefinedError.

    `appearance` (0 to 1) is how far the scene's look may stray from the one it was
    tuned to: the template's vary(base, low, high) gives `base` at 0 and a value
    drawn from all of [low, high] at 1. It draws from its own random stream, so the
    placements a seed gives do not depend on it.
    """
    scenario_template_file = Path(scenario_template_file)

    rng = random.Random()
    if seed is not None:
        rng.seed(seed)

    def fuzz(base, mag):
        return base + rng.uniform(-mag, mag)

    def fuzz_z(base, mag):
        return max(0.0, fuzz(base, mag))

    def rand(a, b):
        return rng.uniform(a, b)

    def choose(options):
        return rng.choice(options)

    look_rng = appearance_rng(seed)

    def vary(base, low, high):
        return vary_appearance(look_rng, appearance, base, low, high)

    with open(scenario_template_file) as f:
        scenario_template = Template(f.read(), undefined=StrictUndefined)

    rendered = scenario_template.render(
        fuzz=fuzz,
        fuzz_z=fuzz_z,
        rand=rand,
        choose=choose,
        vary=vary,
        DX=DX,
        DY=DY,
        DZ=DZ,
        DYAW=DYAW,
        render_robot=render_robot,
        PI=math.pi,
        **variables,
    )

    fd, temp_path = tempfile.mkstemp(prefix=scenario_template_file.stem, suffix=".scn")
    with os.fdopen(fd, "w") as f:
        f.write(rendered)

    return Path(temp_path)
