import re

class geometry(object):

    def __init__(self, name, position = [0,0,0], quaternion = [1,0,0,0]):
        super().__init__()
        assert re.match("[A-Za-z][A-Za-z0-9_]*$", name)
        self._name = name
        self._position = position
        self._quaternion = quaternion

    @property
    def name(self):
        return self._name
    @name.setter
    def name(self, value):
        self._name = value

    @property
    def position(self):
        return self._position
    @position.setter
    def position(self, value):
        self._position = value

    @property
    def position(self):
        return self._position
    @position.setter
    def position(self, value):
        self._position = value

    @property
    def quaternion(self):
        return self._quaternion
    @quaternion.setter
    def quaternion(self, value):
        self._quaternion = value

    def __str__(self):
        val =  "Position: ["+ str(position[0]) + str(position[1]) + str(position[2]) + "]\n"
        val +=  "Quat: [" + str(quaternion[0]) + str(quaternion[1]) + str(quaternion[2])+ str(quaternion[3]) +"]\n"

class sphere(geometry):

    def __init__(self, name, radius, position = [0,0,0], quaternion = [1,0,0,0]):
        super().__init__(name, position, quaternion)
        self._radius = radius

    @property
    def radius(self):
        return self._radius
    @radius.setter
    def radius(self, value):
        self._radius = value

    def to_threejs(self, color):
        pos = self.position;
        quat = self.quaternion;
        val = f"const geometry_{self._name} = new THREE.SphereGeometry( {self.radius}, 32, 16);"
        val += f"const material_{self._name} = new THREE.MeshBasicMaterial( {{ color: {color} }} );"
        val += f"const mesh_{self._name} = new THREE.Mesh( geometry_{self._name}, material_{self._name} );"
        val += f"const quat_{self._name} = new THREE.Quaternion({quat[1]},{quat[2]},{quat[3]},{quat[0]});"
        val += f"mesh_{self._name}.setRotationFromQuaternion(quat_{self._name});"
        val += f"mesh_{self._name}.position.set({pos[0]},{pos[1]},{pos[2]});"
        val += f"scene.add( mesh_{self._name} );"
        return val

class box(geometry):

    def __init__(self, name, width, height , depth, position = [0,0,0], quaternion = [1,0,0,0]):
        super().__init__(name, position, quaternion)
        self._width = width
        self._height = height
        self._depth = depth
    @property
    def width(self):
        return self._width
    @width.setter
    def width(self, value):
        self._width = value
    @property
    def height(self):
        return self._height
    @height.setter
    def height(self, value):
        self._height = value
    @property
    def depth(self):
        return self._depth
    @depth.setter
    def depth(self, value):
        self._depth = value

    def to_threejs(self, color):
        pos = self.position;
        quat = self.quaternion;
        val = f"const geometry_{self._name} = new THREE.BoxGeometry({self.width},{self.height},{self.depth});"
        val += f"const material_{self._name} = new THREE.MeshBasicMaterial( {{ color: {color} }} );"
        val += f"const mesh_{self._name} = new THREE.Mesh( geometry_{self._name}, material_{self._name} );"
        val += f"const quat_{self._name} = new THREE.Quaternion({quat[1]},{quat[2]},{quat[3]},{quat[0]});"
        val += f"mesh_{self._name}.setRotationFromQuaternion(quat_{self._name});"
        val += f"mesh_{self._name}.position.set({pos[0]},{pos[1]},{pos[2]});"
        val += f"scene.add( mesh_{self._name} );"
        return val
