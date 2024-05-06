import math
import numpy as np
from visualization import *


if __name__ == "__main__":

  viz_out = threejs_group(js_dir="../js")

  viz_out.to_html("out/generate_scene.html");