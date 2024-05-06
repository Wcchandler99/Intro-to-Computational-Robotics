import math
from geometry import * 
from threejs_group import *

if __name__ == "__main__":

  red="0xff0000"
  green="0x00ff00"
  purple="0xff00ff"
  blue="0x0000ff"

  viz_out = threejs_group()
  viz_out.js_dir = "../js/"
  line_0 = []
  line_1 = []
  line_2 = []

  for t in np.arange(0, 10, .1, dtype = float):
    line_0.append([t, t, t]) # line from origin to 10 10 10
    line_1.append([math.cos(t), math.sin(t), t])
  viz_out.add_line(line_0, red) #actually draws line
  viz_out.add_line(line_1, blue)

  viz_out.to_html("out/lines.html");
