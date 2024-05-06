function webgl_available() {
  try {
    const canvas = document.createElement('canvas');
    return !!window.WebGLRenderingContext &&
           (canvas.getContext('webgl') ||
            canvas.getContext('experimental-webgl'));
  } catch (e) {
    return false;
  }
};

function set_custom_axis(position, quaternion, scale = 1) {
  const x_geom = new THREE.CylinderGeometry(0.1, 0.1, scale, 32);
  const y_geom = new THREE.CylinderGeometry(0.1, 0.1, scale, 32);
  const z_geom = new THREE.CylinderGeometry(0.1, 0.1, scale, 32);
  x_geom.translate(0.0, scale / 2.0, 0.0);
  y_geom.translate(0.0, scale / 2.0, 0.0);
  z_geom.translate(0.0, scale / 2.0, 0.0);
  x_geom.rotateZ(-1.57);
  // y_geom.rotateZ(0.0);
  z_geom.rotateX(1.57);

  const x_material = new THREE.MeshBasicMaterial({color : 0xff0000});
  const y_material = new THREE.MeshBasicMaterial({color : 0x00ff00});
  const z_material = new THREE.MeshBasicMaterial({color : 0x0000ff});
  const x_mesh = new THREE.Mesh(x_geom, x_material);
  const y_mesh = new THREE.Mesh(y_geom, y_material);
  const z_mesh = new THREE.Mesh(z_geom, z_material);
  x_mesh.setRotationFromQuaternion(quaternion);
  y_mesh.setRotationFromQuaternion(quaternion);
  z_mesh.setRotationFromQuaternion(quaternion);
  x_mesh.position.set(position.x, position.y, position.z);
  y_mesh.position.set(position.x, position.y, position.z);
  z_mesh.position.set(position.x, position.y, position.z);
  scene.add(x_mesh);
  scene.add(y_mesh);
  scene.add(z_mesh);
};

function init_threejs(image_width, image_height) {
  scene = new THREE.Scene();
  camera =
      new THREE.PerspectiveCamera(45, image_width / image_height, 0.1, 1000);
  renderer = new THREE.WebGLRenderer({alpha : true});
  renderer.setSize(image_width, image_height);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  document.body.appendChild(renderer.domElement);
  camera.position.x = 0;
  camera.position.y = -5;
  camera.position.z = 20;
  camera.up.set(0, 0, 1);
  camera.lookAt(0, 0, 0);

  const size = 100;
  const divisions = 20;
  const gridHelper = new THREE.GridHelper(size, divisions);
  scene.add(gridHelper);
  gridHelper.rotation.x = Math.PI / 2;
  gridHelper.position.z += .1
  // scene.add(gridHelper);

  controls = new THREE.MapControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.25;
  controls.screenSpacePanning = true;
  controls.minDistance = 0;
  controls.maxDistance = 200;
  controls.maxPolarAngle = Math.PI / 2;
  var light = new THREE.DirectionalLight(0xffffff);
  light.position.set(100, 100, 200);

  scene.background = new THREE.Color(0xaaaaaa);

  scene.add(light);
  var light = new THREE.DirectionalLight(0xaaaaaa);
  light.position.set(-50, -50, 50);
  light.castShadow = true;
  light.shadow.camera.near = 1;
  light.shadow.camera.far = 350;
  light.shadow.camera.right = 150;
  light.shadow.camera.left = -150;
  light.shadow.camera.top = 150;
  light.shadow.camera.bottom = -150;
  light.shadow.mapSize.width = 4000;
  light.shadow.mapSize.height = 4000;
  scene.add(light);

  var light = new THREE.AmbientLight(0x222222);
  scene.add(light);

  document.getElementById('shot').addEventListener('click', takeScreenshot);
  document.getElementById('bt_play').addEventListener('click',
                                                      start_stop_animation);
  const axis_vec = new THREE.Vector3(0, 0, 0);
  const quaternion = new THREE.Quaternion(0, 0, 0, 1);
  set_custom_axis(axis_vec, quaternion, 10);
}
var scene, camera, renderer;
var mixers = [];
var play_animation = false;
var clock = new THREE.Clock();

function start_stop_animation() {
  if (play_animation) {
    play_animation = false;
    clock.stop();
  } else {
    play_animation = true;
    clock.start();
  }
}

function takeScreenshot() {

  var a = document.createElement('a');
  // Without 'preserveDrawingBuffer' set to true, we must render now
  renderer.render(scene, camera);
  a.href = renderer.domElement.toDataURL().replace('image/png',
                                                   'image/octet-stream');
  a.download = 'screenshot.png'
  a.click();
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  if (play_animation) {
    var delta = clock.getDelta();
    for (i = 0; i < mixers.length; i++) {
      mixers[i].update(delta)
    }
  } else {
    var slider = document.getElementById('time_slider');
    for (i = 0; i < mixers.length; i++) {
      mixers[i].setTime((slider.value / slider.max) * clip.duration);
    }
  }

  renderer.render(scene, camera);
}