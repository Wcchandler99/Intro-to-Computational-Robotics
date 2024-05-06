import pytest
from .geometry import * 


def test_geometry_default():
	geom = geometry("geom")
	assert geom.name == "geom"
	assert len(geom.position) == 3
	assert len(geom.quaternion) == 4

def test_geometry_constructor():
	pos = [1,2,3]
	quat = [0.0, 0.707, 0.0, 0.707]

	geom = geometry("geom", pos, quat)
	assert geom.name == "geom"
	assert geom.quaternion == quat
	assert geom.position == pos

def test_geometry_constructor():
	pos = [1,2,3]
	quat = [0.0, 0.707, 0.0, 0.707]

	geom = geometry("geom")
	geom.position = pos
	geom.quaternion = quat
	assert geom.quaternion == quat
	assert geom.position == pos

def test_sphere_default():
	geom = sphere("sphere", 1)
	assert geom.name == "sphere"
	assert geom.radius == 1
	assert len(geom.position) == 3
	assert len(geom.quaternion) == 4

def test_sphere_to_threejs():
	pos = [1,2,3]
	quat = [0.0, 0.707, 0.0, 0.707]
	color = "0x0f0f0f"
	geom = sphere("sphere", 1, pos, quat)

	js = geom.to_threejs(color)
	assert "mesh_sphere.position.set(1,2,3);" in js
	assert "THREE.Quaternion(0.707,0.0,0.707,0.0);" in js
	assert "color: 0x0f0f0f" in js
	assert "THREE.SphereGeometry( 1, 32, 16);" in js

def test_box_default():
	geom = box("box",1,2,3)
	assert geom.width == 1
	assert geom.height == 2
	assert geom.depth == 3
	assert len(geom.position) == 3
	assert len(geom.quaternion) == 4

def test_cylinder_to_threejs():
	pos = [1,2,3]
	quat = [0.0, 0.707, 0.0, 0.707]
	color = "0x0f0f0f"
	geom = box("box", 4,5,6, pos, quat)

	js = geom.to_threejs(color)
	assert "mesh_box.position.set(1,2,3);" in js
	assert "THREE.Quaternion(0.707,0.0,0.707,0.0);" in js
	assert "color: 0x0f0f0f" in js
	assert "THREE.BoxGeometry(4,5,6);" in js
