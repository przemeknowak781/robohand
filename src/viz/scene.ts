/** Three.js scene, camera, lights, renderer, orbit controls. */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

export class SceneManager {
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  readonly renderer: THREE.WebGLRenderer;
  readonly controls: OrbitControls;

  constructor(container: HTMLElement) {
    this.scene.background = new THREE.Color(0x0d1117);
    this.scene.fog = new THREE.Fog(0x0d1117, 0.8, 2.5);

    this.camera = new THREE.PerspectiveCamera(
      45,
      window.innerWidth / window.innerHeight,
      0.01,
      10,
    );
    // palmar three-quarter view (palmar side is -Z): flexion faces the camera
    this.camera.position.set(-0.17, -0.30, -0.20);
    this.camera.up.set(0, 0, 1);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(this.renderer.domElement);

    // image-based lighting so the metallic plates read well from every side
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    this.scene.environmentIntensity = 0.55;

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.045, -0.03);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 0.08;
    this.controls.maxDistance = 1.2;

    const hemi = new THREE.HemisphereLight(0xbfd4ff, 0x1a2030, 0.9);
    this.scene.add(hemi);

    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(0.25, -0.35, 0.5);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.left = -0.25;
    key.shadow.camera.right = 0.25;
    key.shadow.camera.top = 0.25;
    key.shadow.camera.bottom = -0.25;
    key.shadow.camera.near = 0.1;
    key.shadow.camera.far = 1.5;
    this.scene.add(key);

    const fill = new THREE.DirectionalLight(0x6688cc, 0.7);
    fill.position.set(-0.3, 0.2, 0.25);
    this.scene.add(fill);

    // palmar fill: the default camera looks at the palm (-Z side), which the
    // overhead key light leaves in shadow
    const palmFill = new THREE.DirectionalLight(0xd8e4f5, 1.5);
    palmFill.position.set(-0.15, -0.3, -0.5);
    this.scene.add(palmFill);

    // ground plane a bit below the hand
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(2, 2),
      new THREE.MeshStandardMaterial({ color: 0x131a24, roughness: 0.95 }),
    );
    ground.position.z = -0.28;
    ground.receiveShadow = true;
    this.scene.add(ground);

    const grid = new THREE.GridHelper(1.2, 24, 0x27313f, 0x1b232e);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.279;
    this.scene.add(grid);

    window.addEventListener('resize', () => this.onResize());
  }

  private onResize(): void {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  render(): void {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
