/**
 * 3D representation of the hand: palm + phalanx capsules driven directly by
 * simulator FK frames, plus tendon tubes colored by live tension.
 */

import * as THREE from 'three';
import type { HandParams } from '../sim/types';
import type { HandSimulator, FKFrame } from '../sim/simulator';
import { mat3FromEulerXYZ } from '../sim/math3';

const SKIN = new THREE.MeshStandardMaterial({
  color: 0x9aa7b8,
  roughness: 0.55,
  metalness: 0.35,
});
const JOINT_MAT = new THREE.MeshStandardMaterial({
  color: 0x3d4a5c,
  roughness: 0.4,
  metalness: 0.6,
});

interface TendonView {
  mesh: THREE.Mesh;
  material: THREE.MeshStandardMaterial;
  fingerIndex: number;
  /** Local joint indices the tendon routes over. */
  jointLocals: number[];
  momentArms: number[];
  side: 1 | -1; // -1 palmar (flexor), +1 dorsal (extensor)
  nRouted: number;
}

interface PhalanxView {
  mesh: THREE.Mesh;
  fingerIndex: number;
  local: number;
  length: number;
}

const tmpV = new THREE.Vector3();
const tmpM = new THREE.Matrix4();
const COLOR_COLD = new THREE.Color(0x3282e0);
const COLOR_WARM = new THREE.Color(0xf5c542);
const COLOR_HOT = new THREE.Color(0xe8442e);

export class HandView {
  readonly group = new THREE.Group();
  private phalanxViews: PhalanxView[] = [];
  private jointViews: PhalanxView[] = [];
  private tendonViews: TendonView[] = [];
  private sim: HandSimulator;

  constructor(sim: HandSimulator, parent: THREE.Object3D) {
    this.sim = sim;
    parent.add(this.group);
    this.rebuild();
  }

  /** Recreate all meshes from current sim params (after geometry changes). */
  rebuild(): void {
    this.group.clear();
    this.phalanxViews = [];
    this.jointViews = [];
    this.tendonViews = [];
    const params: HandParams = this.sim.params;

    // palm
    const palmGeo = new THREE.BoxGeometry(
      params.palm.width,
      params.palm.length,
      params.palm.thickness,
      2, 2, 2,
    );
    const palm = new THREE.Mesh(palmGeo, SKIN);
    palm.position.set(0, 0, -params.palm.thickness * 0.1);
    palm.castShadow = true;
    palm.receiveShadow = true;
    this.group.add(palm);

    // wrist stub
    const wrist = new THREE.Mesh(
      new THREE.CylinderGeometry(
        params.palm.width * 0.32,
        params.palm.width * 0.36,
        params.palm.length * 0.5,
        20,
      ),
      SKIN,
    );
    wrist.position.set(0, -params.palm.length * 0.68, -params.palm.thickness * 0.1);
    wrist.castShadow = true;
    this.group.add(wrist);

    params.fingers.forEach((finger, fi) => {
      finger.phalanges.forEach((ph, li) => {
        const capsule = new THREE.Mesh(
          new THREE.CapsuleGeometry(ph.radius, ph.length, 6, 14),
          SKIN,
        );
        capsule.castShadow = true;
        this.group.add(capsule);
        this.phalanxViews.push({
          mesh: capsule,
          fingerIndex: fi,
          local: li,
          length: ph.length,
        });

        const joint = new THREE.Mesh(
          new THREE.SphereGeometry(ph.radius * 1.12, 16, 12),
          JOINT_MAT,
        );
        joint.castShadow = true;
        this.group.add(joint);
        this.jointViews.push({
          mesh: joint,
          fingerIndex: fi,
          local: li,
          length: ph.length,
        });
      });

      finger.tendons.forEach((tendon) => {
        const material = new THREE.MeshStandardMaterial({
          color: 0x3282e0,
          roughness: 0.35,
          metalness: 0.1,
          emissive: 0x000000,
        });
        const mesh = new THREE.Mesh(new THREE.BufferGeometry(), material);
        mesh.frustumCulled = false;
        this.group.add(mesh);
        this.tendonViews.push({
          mesh,
          material,
          fingerIndex: fi,
          jointLocals: tendon.routing.map((r) => r.jointIndex),
          momentArms: tendon.routing.map((r) => r.momentArm),
          side: tendon.kind === 'flexor' ? -1 : 1,
          nRouted: tendon.routing.length,
        });
      });
    });

    this.update();
  }

  /** Sync meshes with the simulator's current FK frames and tensions. */
  update(): void {
    const frames = this.sim.fkFrames;
    const params = this.sim.params;

    for (const pv of this.phalanxViews) {
      const frame = frames[pv.fingerIndex];
      this.placeAlongSegment(pv.mesh, frame, pv.local, pv.length, 0.5);
    }
    for (const jv of this.jointViews) {
      const frame = frames[jv.fingerIndex];
      const p = frame.jointPos[jv.local];
      jv.mesh.position.set(p[0], p[1], p[2]);
    }

    const tensions = this.sim.tension;
    const maxSafe = this.sim.maxSafeTensions;
    let ti = 0;
    for (const tv of this.tendonViews) {
      const frame = frames[tv.fingerIndex];
      const finger = params.fingers[tv.fingerIndex];
      const pts = this.tendonWaypoints(tv, frame, finger.phalanges.map((p) => p.radius), finger.phalanges.map((p) => p.length));
      const curve = new THREE.CatmullRomCurve3(pts, false, 'catmullrom', 0.5);
      const radius = 0.0017 * params.scale;
      const geo = new THREE.TubeGeometry(curve, 22, radius, 5, false);
      tv.mesh.geometry.dispose();
      tv.mesh.geometry = geo;

      const t01 = Math.min(1, tensions[ti] / maxSafe[ti]);
      this.applyTensionColor(tv.material, t01);
      ti++;
    }
  }

  private applyTensionColor(mat: THREE.MeshStandardMaterial, t01: number): void {
    if (t01 < 0.5) {
      mat.color.lerpColors(COLOR_COLD, COLOR_WARM, t01 * 2);
    } else {
      mat.color.lerpColors(COLOR_WARM, COLOR_HOT, (t01 - 0.5) * 2);
    }
    mat.emissive.copy(mat.color).multiplyScalar(t01 * 0.45);
  }

  private placeAlongSegment(
    mesh: THREE.Mesh,
    frame: FKFrame,
    local: number,
    length: number,
    frac: number,
  ): void {
    const p = frame.jointPos[local];
    const m = frame.jointMat[local];
    tmpM.set(
      m[0], m[3], m[6], 0,
      m[1], m[4], m[7], 0,
      m[2], m[5], m[8], 0,
      0, 0, 0, 1,
    );
    mesh.quaternion.setFromRotationMatrix(tmpM);
    tmpV.set(0, length * frac, 0).applyMatrix4(tmpM);
    mesh.position.set(p[0] + tmpV.x, p[1] + tmpV.y, p[2] + tmpV.z);
  }

  /** Polyline the tendon follows: palm anchor → joint pulleys → attachment. */
  private tendonWaypoints(
    tv: TendonView,
    frame: FKFrame,
    radii: number[],
    lengths: number[],
  ): THREE.Vector3[] {
    const pts: THREE.Vector3[] = [];
    const side = tv.side;
    const finger = this.sim.params.fingers[tv.fingerIndex];

    // anchor proximal of the first joint, on the routing side of the palm
    const anchorLocal: [number, number, number] = [
      0,
      -0.028 * this.sim.params.scale,
      side * (radii[0] * 0.9),
    ];
    pts.push(
      this.localToWorld(
        frame.jointPos[0],
        mat3FromEulerXYZ(finger.baseEulerXYZ),
        anchorLocal,
      ),
    );

    const lastRouted = tv.jointLocals[tv.jointLocals.length - 1];
    for (let k = 0; k < tv.jointLocals.length; k++) {
      const lj = tv.jointLocals[k];
      const arm = tv.momentArms[k];
      // pulley point at the joint, offset by the moment arm on the tendon side
      pts.push(
        this.localToWorld(frame.jointPos[lj], frame.jointMat[lj], [
          0,
          0,
          side * arm * 1.15,
        ]),
      );
      // guide along the phalanx (skin-level) unless this is the attachment
      const guideZ = side * (radii[lj] * 0.75);
      if (lj === lastRouted) {
        pts.push(
          this.localToWorld(frame.jointPos[lj], frame.jointMat[lj], [
            0,
            lengths[lj] * 0.85,
            guideZ,
          ]),
        );
      } else {
        pts.push(
          this.localToWorld(frame.jointPos[lj], frame.jointMat[lj], [
            0,
            lengths[lj] * 0.55,
            guideZ,
          ]),
        );
      }
    }
    return pts;
  }

  private localToWorld(
    origin: [number, number, number],
    m: ArrayLike<number>,
    v: [number, number, number],
  ): THREE.Vector3 {
    return new THREE.Vector3(
      origin[0] + m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
      origin[1] + m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
      origin[2] + m[2] * v[0] + m[5] * v[1] + m[8] * v[2],
    );
  }
}
