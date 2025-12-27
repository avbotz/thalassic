import os
import math
import tempfile
from pathlib import Path

from jinja2 import Environment, FileSystemLoader


def render_robot_scenario(robot_template_file: Path | str) -> Path:
    robot_template_file = Path(robot_template_file)

    # Let Jinja resolve `{% from "thrusters.jinja" import ... %}`
    env = Environment(loader=FileSystemLoader(str(robot_template_file.parent)))

    template = env.get_template(robot_template_file.name)

    rendered = template.render(PI=math.pi)

    fd, temp_path = tempfile.mkstemp(
        prefix=robot_template_file.stem,
        suffix=".scn",
    )
    with os.fdopen(fd, "w") as f:
        f.write(rendered)

    return Path(temp_path)
