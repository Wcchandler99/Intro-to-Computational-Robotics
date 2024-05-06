from geometry import * 
from threejs_group import *

if __name__ == "__main__":

  red="0xff0000"
  green="0x00ff00"
  viz_out = threejs_group(js_dir="../js")

  geom = sphere("sphere_0", 1, [2, 3, 4], [1, 0, 0, 0]) # name, radius, position, rotation
  geom1 = box("box_0", 1,1,1, [2, -3, 4], [.707, .707, 0, 0]) # name, width, length, height, position, rotation
  viz_out.add_obstacle(geom, green) #actullay draws obstacle
  viz_out.add_obstacle(geom1, red)
    

  viz_out.to_html("out/obstacles.html");
