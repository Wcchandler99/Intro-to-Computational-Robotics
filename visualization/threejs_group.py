import os
import numpy as np
# from geometry import * 

class threejs_group(object):
    def __init__(self, canvas_width = 600, canvas_height = 600, js_dir="./js/"):
        super().__init__()
        js_dir = js_dir if js_dir[-1] == "/" else js_dir+"/"
        self._color="0xffffff"
        self._js_dir=js_dir
        self._obstacles=[]
        self._axis = []
        self._animation = []
        self._size = [canvas_width, canvas_height]
        self._lines = []

    @property
    def axis(self):
        return self._axis

    @property
    def js_dir(self):
        return self._js_dir
    @js_dir.setter
    def js_dir(self, value):
        value = value if value[-1] == "/" else value+"/"
        self._js_dir = value

    def html_header(self):
        var =  "<!DOCTYPE html>"
        var += "<html>"
        var += "<head>"
        var += f"  <link rel=stylesheet href={self._js_dir}style.css> "
        var += "  <meta charset=utf-8>"
        var += "  <title>CS560: Robotics</title>"
        var += "  <style> body { margin: 0; } canvas { width: 100%; height: 100%; display: block; }</style>"
        var += "</head>\n"
        return var;

    def html_body(self):
        var = "<button id=shot>Screenshot</button>"
        var += "  <button id=bt_play>Play</button>"
        var += "  <div class=slidecontainer>"
        var += "    <input type=range min=0 max=1000 value=500 class=slider id=time_slider>"
        var += "  </div>"
        var += f"  <script src={self._js_dir}three.js></script>"
        var += f"  <script src={self._js_dir}map_controls.js></script>"
        var += f"  <script src={self._js_dir}cs560.js></script>"
        var += "   <script>"
        var += " if ( webgl_available() ) {\n"
        var += f"    init_threejs({self._size[0]}, {self._size[1]});"
        return var
    
    def html_footer(self):
        var  = "  animate();"
        var += "  } else {"
        var += "  const warning = WebGL.getWebGLErrorMessage();"
        var += "  document.getElementById( 'container' ).appendChild( warning );"
        var += "  }"
        var += "  </script>"
        var += "</body>"
        var += "</html>"
        return var

    def add_line(self, line, color="0xaaaaaa"):
        self._lines.append([line, color])

    def html_lines(self):
        js_string = ""
        for line in self._lines:
            points = line[0]
            r,g,b = self.color_hex_to_rgb(line[1])

            js_string += "{var geometry = new THREE.BufferGeometry();";
            js_string += "var material = new THREE.LineBasicMaterial( { vertexColors: THREE.VertexColors } );";
            js_string += "var positions = [];";
            js_string += "var colors = [];";

            for x,y,z in points:
                js_string += f"positions.push({x},{y},{z});";
                js_string += f"colors.push({r},{g},{b});";
            js_string += "geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );";
            js_string += "geometry.setAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );";
            js_string += "geometry.computeBoundingSphere();";
            js_string += "mesh = new THREE.Line( geometry, material );";
            js_string += "scene.add( mesh );}\n";
        return js_string;

    def add_obstacle(self, geom, color="0xff0000"):
        self._obstacles.append([geom, color])

    def add_animation(self, box, trajectory):
        self._animation.append([box, trajectory])

    def color_hex_to_rgb(self, color):
        r = round(int(color[2:4],16) / 255, 2)
        g = round(int(color[4:6],16) / 255, 2)
        b = round(int(color[6:8],16) / 255, 2)
        return r,g,b

    def html_animation(self):
        js_string = ""
        for geom, trajectory in self._animation:
            time_stamps=""
            positions=""
            quaternions=""
            colors = ""
            for t, pos, quat, color in trajectory:
                time_stamps += f"{t},"
                positions += f"{pos[0]},{pos[1]},{pos[2]},"
                quaternions += f"{quat[1]},{quat[2]},{quat[3]},{quat[0]},"
                r,g,b = self.color_hex_to_rgb(color)
                colors += f"{r},{g},{b},"
            time_stamps = time_stamps[:-1]
            positions = positions[:-1]
            quaternions = quaternions[:-1]
            colors = colors[:-1]
            js_string += "{";
            js_string += geom.to_threejs("0xff0000");
            js_string += f"var positionKF = new THREE.VectorKeyframeTrack( '.position',[{time_stamps}],[{positions}],THREE.InterpolateDiscrete);";
            js_string += f"var quaternionKF = new THREE.QuaternionKeyframeTrack( '.quaternion',[{time_stamps}],[{quaternions}],THREE.InterpolateDiscrete);";
            js_string += f"var colorsKF = new THREE.ColorKeyframeTrack( '.material.color',[{time_stamps}],[{colors}],THREE.InterpolateDiscrete);";
            js_string += "var clip = new THREE.AnimationClip( 'Action', -1, [positionKF,quaternionKF,colorsKF] );";
            js_string += f"var mixer = new THREE.AnimationMixer( mesh_{geom.name} );";
            js_string += "var clipAction = mixer.clipAction( clip );";
            js_string += "clipAction.play();";
            js_string += "mixers.push(mixer);}\n";
        return js_string
    
    # Add a floor 
    def html_floor(self):
        var = "const floor = new THREE.PlaneBufferGeometry(100,100);"
        var += f"var floor_material = new THREE.MeshLambertMaterial({{ color : {self._color} }});"
        var += "var floor_mesh = new THREE.Mesh(floor, floor_material);"
        var += "floor_mesh.receiveShadow = true;"
        var += "floor_mesh.position.set(0,0,0);"
        var += "floor_mesh.quaternion = new THREE.Quaternion(0.707, 0, 0, 0.707 );"
        var += "scene.add(floor_mesh);\n"
        return var

    # Add axis at position, quat with lines of specified length
    def add_axis(self, position, quaternion, length = 1):
        self._axis.append([position, quaternion, length])

    def html_axis(self):
        var = ""
        for axis in self._axis:
            pos = axis[0]
            quat = axis[1]
            length = axis[2]
            var += f"{{const position = new THREE.Vector3({pos[0]}, {pos[1]}, {pos[2]});"
            var += f"const quaternion = new THREE.Quaternion({quat[1]},{quat[2]},{quat[3]},{quat[0]});"
            var += f"set_custom_axis(position, quaternion, {length});}}"
        return var

    def html_obstacles(self):
        var = ""
        for geom, color in self._obstacles:
            var += geom.to_threejs(color);
        return var;
    
    def to_html(self, filename, path = ""):
        if path == "":
            path = os.getcwd()
        filename = os.path.join(os.path.abspath(path), filename)
        file = open(filename, 'w')
        file.write(self.html_header())
        file.write(self.html_body())
        file.write(self.html_floor())
        file.write(self.html_axis())
        file.write(self.html_obstacles())
        file.write(self.html_animation())
        file.write(self.html_lines())
        file.write(self.html_footer())

        file.close();
        print(f"Output: {filename}")
        
