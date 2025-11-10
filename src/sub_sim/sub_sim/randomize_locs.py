import os
import math
import random
import tempfile
from pathlib import Path

from jinja2 import Template

def randomize_scenario_locations(scenario_template_file: Path | str, DX: float, DY: float, DZ: float, DYAW: float, seed: None | int=None):
    rng = random.Random()
    if seed:
        rng.seed(seed)

    def fuzz(base, mag):
        return base + rng.uniform(-mag, mag)

    def fuzz_z(base, mag):
        return max(0.0, fuzz(base, mag))

    def rand(a, b):
        return rng.uniform(a, b)

    def choose(options):
        return rng.choice(options)

    with open(scenario_template_file, "r") as f:
        scenario_template = Template(f.read())

    scn_text = scenario_template.render(
        fuzz=fuzz, fuzz_z=fuzz_z, rand=rand, choose=choose, DX=DX, DY=DY, DZ=DZ, DYAW=DYAW, PI=math.pi
    )

    fd, temp_path = tempfile.mkstemp(prefix=Path(scenario_template_file).stem, suffix=".scn")
    with os.fdopen(fd, "w") as f:
        f.write(scn_text)

    return temp_path
