/**
 * Automated regression check for the tendon-tube twist bug: sweeps every
 * finger's curl from 0..1 and, for each tendon, replicates HandView's exact
 * pulley-arc quaternion sequence, then checks that consecutive orientations
 * never jump (dot product of consecutive quaternions stays near 1) — a
 * flip would show up as a sudden sign/direction reversal. Run with:
 * npx tsx scripts/check-tendon-twist.ts
 */
import * as THREE from 'three';
import { buildHandParams } from '../src/sim/defaultHand';
import { HandSimulator } from '../src/sim/simulator';
import { computeLinkMech } from '../src/sim/mechGeometry';
import { mat3FromEulerXYZ, type Mat3 } from '../src/sim/math3';
import { curlsToAction } from '../src/sim/presets';

const sim = new HandSimulator(buildHandParams());

function quatFromMat3(m: Mat3): THREE.Quaternion {
  const m4 = new THREE.Matrix4().set(
    m[0], m[3], m[6], 0,
    m[1], m[4], m[7], 0,
    m[2], m[5], m[8], 0,
    0, 0, 0, 1,
  );
  return new THREE.Quaternion().setFromRotationMatrix(m4);
}

function pulleyArc(parentMat: Mat3, childMat: Mat3, samples = 4): THREE.Quaternion[] {
  const qA = quatFromMat3(parentMat);
  const qB = quatFromMat3(childMat);
  const quats: THREE.Quaternion[] = [];
  for (let i = 0; i <= samples; i++) quats.push(qA.clone().slerp(qB, i / samples));
  return quats;
}

let worstDot = 1;
let flips = 0;
let checkedCurves = 0;

for (let step = 0; step <= 40; step++) {
  const curl = step / 40;
  sim.setAction(curlsToAction(sim, { thumb: curl, index: curl, middle: curl, ring: curl, pinky: curl }));
  for (let i = 0; i < 30; i++) sim.step();

  const params = sim.params;
  const frames = sim.fkFrames;

  params.fingers.forEach((finger, fi) => {
    const frame = frames[fi];
    const baseMat = mat3FromEulerXYZ(finger.baseEulerXYZ);
    const baseQuat = quatFromMat3(baseMat);

    for (const tendon of finger.tendons) {
      const quats: THREE.Quaternion[] = [baseQuat, baseQuat];
      const jointLocals = tendon.routing.map((r) => r.jointIndex);
      for (const lj of jointLocals) {
        const parentMat = lj === 0 ? baseMat : frame.jointMat[lj - 1];
        const childMat = frame.jointMat[lj];
        const arc = pulleyArc(parentMat, childMat);
        quats.push(...arc);
        const childQuat = arc[arc.length - 1];
        quats.push(childQuat, childQuat);
      }

      checkedCurves++;
      for (let i = 1; i < quats.length; i++) {
        const d = Math.abs(quats[i - 1].dot(quats[i]));
        if (d < worstDot) worstDot = d;
        if (d < 0.9) {
          flips++;
          console.log(`JUMP: ${finger.name}.${tendon.name} curl=${curl.toFixed(2)} i=${i}/${quats.length} |dot|=${d.toFixed(3)}`);
        }
      }
    }
  });
}

console.log(`\nChecked ${checkedCurves} tendon quaternion sequences across 41 curl steps.`);
console.log(`Worst consecutive |quat dot|: ${worstDot.toFixed(4)} (near 0 = discontinuity/flip)`);
console.log(flips === 0 ? 'PASS: orientation sequence is continuous everywhere.' : `FAIL: ${flips} discontinuities detected.`);
process.exit(flips === 0 ? 0 : 1);
