/**
 * Deterministic, fixed-timestep simulator of a tendon-driven hand.
 *
 * Model summary
 * -------------
 * Each finger is a serial chain of revolute flexion joints. For every tendon:
 *
 *   excursion    e(q) = Σ_j sign_j · r_j · q_j          (pulley model)
 *   stretch      s    = pretension + x_a − e(q)
 *   tension      T    = max(0, k·s + c·ṡ)               (tendons cannot push)
 *   joint torque τ_j += sign_j · r_j · T
 *
 * where x_a is the actuator (spool) position following a first-order lag
 * toward its target. Joints additionally have passive elasticity, damping,
 * soft limit springs, and gravity torques computed from forward kinematics.
 * Integration: semi-implicit Euler with velocity damping folded implicitly.
 *
 * No DOM / three.js dependencies — runs headless in Node for ML training.
 */

import type {
  HandParams,
  Observation,
  SimConfig,
  SimSpec,
  Vec3,
} from './types';
import {
  Mat3,
  mat3FromEulerXYZ,
  mat3Mul,
  mat3RotX,
  mat3Apply,
  mat3Col,
  v3Add,
  v3Cross,
  v3Dot,
  v3LenSq,
  v3Scale,
  v3Sub,
} from './math3';

export const DEFAULT_SIM_CONFIG: SimConfig = {
  controlDt: 1 / 60,
  substeps: 8,
};

/** Flattened (whole-hand) tendon description with global joint indices. */
interface CompiledTendon {
  name: string;
  kind: 'flexor' | 'extensor';
  fingerIndex: number;
  jointIndices: number[]; // global
  momentArms: number[];
  signs: number[];
  stiffness: number;
  damping: number;
  pretension: number;
  maxActuatorPull: number;
  actuatorTimeConstant: number;
  maxSafeTension: number;
}

interface CompiledFinger {
  name: string;
  jointOffset: number; // index of first joint in global arrays
  nJoints: number;
  basePos: Vec3;
  baseMat: Mat3;
  lengths: number[];
  masses: number[];
}

const MIN_INERTIA = 2e-6; // kg·m² floor to keep integration stable

export interface FKFrame {
  /** World position of each joint of the finger. */
  jointPos: Vec3[];
  /** World orientation after each joint's rotation (column-major 3x3). */
  jointMat: Mat3[];
  /** World position of each phalanx center of mass. */
  comPos: Vec3[];
  /** Fingertip world position. */
  tip: Vec3;
}

export class HandSimulator {
  params: HandParams;
  config: SimConfig;

  nJoints = 0;
  nTendons = 0;

  /** Joint state (global order: finger by finger, proximal→distal). */
  q!: Float64Array;
  qd!: Float64Array;
  /** Actuator spool positions and their targets [m]. */
  actuatorPos!: Float64Array;
  actuatorTarget!: Float64Array;
  /** Per-tendon measurements refreshed every substep. */
  tension!: Float64Array;
  excursion!: Float64Array;
  stretch!: Float64Array;

  time = 0;

  private fingers: CompiledFinger[] = [];
  private tendons: CompiledTendon[] = [];
  private jointMin!: Float64Array;
  private jointMax!: Float64Array;
  private jointRest!: Float64Array;
  private jointKp!: Float64Array;
  private jointCp!: Float64Array;
  private fk: FKFrame[] = [];
  private inertia!: Float64Array;
  private torqueScratch!: Float64Array;

  constructor(params: HandParams, config: SimConfig = DEFAULT_SIM_CONFIG) {
    this.params = params;
    this.config = { ...config };
    this.compile(params);
    this.reset();
  }

  /** Physics timestep [s]. */
  get physicsDt(): number {
    return this.config.controlDt / this.config.substeps;
  }

  get jointNames(): string[] {
    return this.params.fingers.flatMap((f) => f.joints.map((j) => j.name));
  }

  get tendonNames(): string[] {
    return this.tendons.map((t) => t.name);
  }

  get tendonKinds(): ('flexor' | 'extensor')[] {
    return this.tendons.map((t) => t.kind);
  }

  get tendonFingerIndex(): number[] {
    return this.tendons.map((t) => t.fingerIndex);
  }

  get maxSafeTensions(): number[] {
    return this.tendons.map((t) => t.maxSafeTension);
  }

  /** Forward-kinematics frames per finger (recomputed every substep). */
  get fkFrames(): FKFrame[] {
    return this.fk;
  }

  private compile(params: HandParams): void {
    this.fingers = [];
    this.tendons = [];
    let jointOffset = 0;

    params.fingers.forEach((f, fi) => {
      this.fingers.push({
        name: f.name,
        jointOffset,
        nJoints: f.joints.length,
        basePos: [...f.basePosition],
        baseMat: mat3FromEulerXYZ(f.baseEulerXYZ),
        lengths: f.phalanges.map((p) => p.length),
        masses: f.phalanges.map((p) => p.mass),
      });
      for (const t of f.tendons) {
        this.tendons.push({
          name: t.name,
          kind: t.kind,
          fingerIndex: fi,
          jointIndices: t.routing.map((r) => jointOffset + r.jointIndex),
          momentArms: t.routing.map((r) => r.momentArm),
          signs: t.routing.map((r) => r.sign),
          stiffness: t.stiffness,
          damping: t.damping,
          pretension: t.pretension,
          maxActuatorPull: t.maxActuatorPull,
          actuatorTimeConstant: t.actuatorTimeConstant,
          maxSafeTension: t.maxSafeTension,
        });
      }
      jointOffset += f.joints.length;
    });

    this.nJoints = jointOffset;
    this.nTendons = this.tendons.length;

    this.jointMin = new Float64Array(this.nJoints);
    this.jointMax = new Float64Array(this.nJoints);
    this.jointRest = new Float64Array(this.nJoints);
    this.jointKp = new Float64Array(this.nJoints);
    this.jointCp = new Float64Array(this.nJoints);
    let ji = 0;
    for (const f of params.fingers) {
      for (const j of f.joints) {
        this.jointMin[ji] = j.minAngle;
        this.jointMax[ji] = j.maxAngle;
        this.jointRest[ji] = j.restAngle;
        this.jointKp[ji] = j.passiveStiffness;
        this.jointCp[ji] = j.passiveDamping;
        ji++;
      }
    }

    this.inertia = new Float64Array(this.nJoints);
    this.torqueScratch = new Float64Array(this.nJoints);
    this.fk = this.fingers.map((f) => ({
      jointPos: Array.from({ length: f.nJoints }, () => [0, 0, 0] as Vec3),
      jointMat: Array.from({ length: f.nJoints }, () => mat3RotX(0)),
      comPos: Array.from({ length: f.nJoints }, () => [0, 0, 0] as Vec3),
      tip: [0, 0, 0] as Vec3,
    }));
  }

  /**
   * Replace hand parameters, preserving dynamic state when the topology
   * (joint/tendon counts) is unchanged — lets the UI retune live.
   */
  setParams(params: HandParams): void {
    const prevQ = this.q;
    const prevQd = this.qd;
    const prevXa = this.actuatorPos;
    const prevTarget = this.actuatorTarget;
    const prevJoints = this.nJoints;
    const prevTendons = this.nTendons;
    const prevTime = this.time;

    this.params = params;
    this.compile(params);

    if (prevJoints === this.nJoints && prevTendons === this.nTendons) {
      this.q = prevQ;
      this.qd = prevQd;
      this.actuatorTarget = prevTarget;
      // clamp spool positions into the (possibly rescaled) actuator range
      this.actuatorPos = prevXa;
      for (let i = 0; i < this.nTendons; i++) {
        const max = this.tendons[i].maxActuatorPull;
        if (this.actuatorPos[i] > max) this.actuatorPos[i] = max;
        if (this.actuatorTarget[i] > max) this.actuatorTarget[i] = max;
      }
      this.tension = new Float64Array(this.nTendons);
      this.excursion = new Float64Array(this.nTendons);
      this.stretch = new Float64Array(this.nTendons);
      this.time = prevTime;
      this.computeFK();
      this.updateTendonMeasurements();
    } else {
      this.reset();
    }
  }

  /** Reset to the rest pose with slack actuators. Deterministic. */
  reset(): Observation {
    this.q = new Float64Array(this.nJoints);
    this.qd = new Float64Array(this.nJoints);
    for (let i = 0; i < this.nJoints; i++) this.q[i] = this.jointRest[i];
    this.actuatorPos = new Float64Array(this.nTendons);
    this.actuatorTarget = new Float64Array(this.nTendons);
    this.tension = new Float64Array(this.nTendons);
    this.excursion = new Float64Array(this.nTendons);
    this.stretch = new Float64Array(this.nTendons);
    this.time = 0;
    this.computeFK();
    this.updateTendonMeasurements();
    return this.getObservation();
  }

  /**
   * Set normalized actuator commands (one per tendon, clamped to [0, 1]);
   * 0 = fully released spool, 1 = maximum pull.
   */
  setAction(action: ArrayLike<number>): void {
    const n = Math.min(action.length, this.nTendons);
    for (let i = 0; i < n; i++) {
      if (!Number.isFinite(action[i])) continue; // NaN/Inf must not poison state
      const u = Math.min(1, Math.max(0, action[i]));
      this.actuatorTarget[i] = u * this.tendons[i].maxActuatorPull;
    }
  }

  /** Current normalized action (targets / maxPull). */
  getAction(): number[] {
    return this.tendons.map((t, i) =>
      t.maxActuatorPull > 0 ? this.actuatorTarget[i] / t.maxActuatorPull : 0,
    );
  }

  /**
   * Advance one control step (config.controlDt), optionally applying an
   * action first. This is the primary ML-facing entry point.
   */
  step(action?: ArrayLike<number>): Observation {
    if (action) this.setAction(action);
    const h = this.physicsDt;
    for (let s = 0; s < this.config.substeps; s++) this.substep(h);
    return this.getObservation();
  }

  private substep(h: number): void {
    this.computeFK(); // refreshes FK frames + gravity data + inertias

    const tau = this.torqueScratch;
    tau.fill(0);
    // extra velocity-proportional damping folded implicitly per joint
    const implicitDamp = new Float64Array(this.nJoints);

    // --- actuator first-order lag ---
    const xaDot = new Float64Array(this.nTendons);
    for (let i = 0; i < this.nTendons; i++) {
      const t = this.tendons[i];
      xaDot[i] = (this.actuatorTarget[i] - this.actuatorPos[i]) /
        Math.max(1e-3, t.actuatorTimeConstant);
    }

    // --- tendon forces ---
    for (let i = 0; i < this.nTendons; i++) {
      const t = this.tendons[i];
      let e = 0;
      let eDot = 0;
      for (let k = 0; k < t.jointIndices.length; k++) {
        const j = t.jointIndices[k];
        e += t.signs[k] * t.momentArms[k] * this.q[j];
        eDot += t.signs[k] * t.momentArms[k] * this.qd[j];
      }
      this.excursion[i] = e;
      const s = t.pretension + this.actuatorPos[i] - e;
      let T = 0;
      if (s > 0) {
        T = t.stiffness * s + t.damping * (xaDot[i] - eDot);
        if (T < 0) T = 0;
      }
      this.stretch[i] = Math.max(0, s);
      this.tension[i] = T;
      for (let k = 0; k < t.jointIndices.length; k++) {
        const j = t.jointIndices[k];
        tau[j] += t.signs[k] * t.momentArms[k] * T;
        // tendon damping acts like joint damping through the moment arm
        if (s > 0) {
          implicitDamp[j] +=
            t.damping * t.momentArms[k] * t.momentArms[k];
        }
      }
    }

    // --- passive joint elasticity, limits, gravity ---
    const g = this.params.gravity;
    const gravityOn =
      this.params.gravityEnabled && v3LenSq(g as Vec3) > 1e-9;

    for (let fi = 0; fi < this.fingers.length; fi++) {
      const f = this.fingers[fi];
      const frame = this.fk[fi];
      for (let lj = 0; lj < f.nJoints; lj++) {
        const j = f.jointOffset + lj;
        // passive capsule spring
        tau[j] -= this.jointKp[j] * (this.q[j] - this.jointRest[j]);
        implicitDamp[j] += this.jointCp[j];

        // soft joint limits
        if (this.q[j] < this.jointMin[j]) {
          tau[j] += this.params.limitStiffness * (this.jointMin[j] - this.q[j]);
          implicitDamp[j] += this.params.limitDamping;
        } else if (this.q[j] > this.jointMax[j]) {
          tau[j] += this.params.limitStiffness * (this.jointMax[j] - this.q[j]);
          implicitDamp[j] += this.params.limitDamping;
        }

        // gravity torque about this joint from all downstream segments
        if (gravityOn) {
          // joint axis in q-positive (flexion) sense = -X of the parent frame
          const parentMat = lj === 0
            ? f.baseMat
            : frame.jointMat[lj - 1];
          const axis = v3Scale(mat3Col(parentMat, 0), -1);
          let tg = 0;
          for (let k = lj; k < f.nJoints; k++) {
            const rel = v3Sub(frame.comPos[k], frame.jointPos[lj]);
            const force: Vec3 = [
              f.masses[k] * g[0],
              f.masses[k] * g[1],
              f.masses[k] * g[2],
            ];
            tg += v3Dot(axis, v3Cross(rel, force));
          }
          tau[j] += tg;
        }
      }
    }

    // --- integrate joints (semi-implicit Euler, implicit damping) ---
    for (let j = 0; j < this.nJoints; j++) {
      const I = this.inertia[j];
      let qd = this.qd[j] + (h * tau[j]) / I;
      qd /= 1 + (h * implicitDamp[j]) / I;
      this.qd[j] = qd;
      this.q[j] += h * qd;

      // hard safety clamp well past the soft limits
      const lo = this.jointMin[j] - 0.25;
      const hi = this.jointMax[j] + 0.25;
      if (this.q[j] < lo) {
        this.q[j] = lo;
        if (this.qd[j] < 0) this.qd[j] = 0;
      } else if (this.q[j] > hi) {
        this.q[j] = hi;
        if (this.qd[j] > 0) this.qd[j] = 0;
      }
    }

    // --- integrate actuators ---
    for (let i = 0; i < this.nTendons; i++) {
      this.actuatorPos[i] += h * xaDot[i];
    }

    this.time += h;
  }

  /** Forward kinematics + per-joint effective inertia from downstream mass. */
  private computeFK(): void {
    for (let fi = 0; fi < this.fingers.length; fi++) {
      const f = this.fingers[fi];
      const frame = this.fk[fi];
      let pos: Vec3 = f.basePos;
      let mat: Mat3 = f.baseMat;
      for (let lj = 0; lj < f.nJoints; lj++) {
        const j = f.jointOffset + lj;
        // flexion q > 0 rotates about local -X (curls toward palmar -Z)
        mat = mat3Mul(mat, mat3RotX(-this.q[j]));
        frame.jointPos[lj] = pos;
        frame.jointMat[lj] = mat;
        const seg = mat3Apply(mat, [0, f.lengths[lj], 0]);
        frame.comPos[lj] = v3Add(pos, v3Scale(seg, 0.5));
        pos = v3Add(pos, seg);
      }
      frame.tip = pos;

      // effective inertia: point-mass approximation of downstream segments
      for (let lj = 0; lj < f.nJoints; lj++) {
        const j = f.jointOffset + lj;
        let I = MIN_INERTIA;
        for (let k = lj; k < f.nJoints; k++) {
          I += f.masses[k] * v3LenSq(v3Sub(frame.comPos[k], frame.jointPos[lj]));
        }
        this.inertia[j] = I;
      }
    }
  }

  /** Refresh tension/excursion/stretch without advancing time (after reset). */
  private updateTendonMeasurements(): void {
    for (let i = 0; i < this.nTendons; i++) {
      const t = this.tendons[i];
      let e = 0;
      for (let k = 0; k < t.jointIndices.length; k++) {
        e += t.signs[k] * t.momentArms[k] * this.q[t.jointIndices[k]];
      }
      this.excursion[i] = e;
      const s = t.pretension + this.actuatorPos[i] - e;
      this.stretch[i] = Math.max(0, s);
      this.tension[i] = s > 0 ? t.stiffness * s : 0;
    }
  }

  getObservation(): Observation {
    const tips: number[] = [];
    for (const frame of this.fk) tips.push(...frame.tip);
    return {
      time: this.time,
      q: Array.from(this.q),
      qd: Array.from(this.qd),
      tendonTension: Array.from(this.tension),
      tendonExcursion: Array.from(this.excursion),
      tendonStretch: Array.from(this.stretch),
      actuatorPosition: Array.from(this.actuatorPos),
      actuatorTarget: Array.from(this.actuatorTarget),
      fingertipPositions: tips,
    };
  }

  /** Flatten an observation into a single vector (fixed, documented order). */
  static flattenObservation(obs: Observation): Float64Array {
    return Float64Array.from([
      ...obs.q,
      ...obs.qd,
      ...obs.tendonTension,
      ...obs.tendonExcursion,
      ...obs.tendonStretch,
      ...obs.actuatorPosition,
      ...obs.actuatorTarget,
      ...obs.fingertipPositions,
    ]);
  }

  /** Machine-readable spec of spaces + full parameters, for ML tooling. */
  getSpec(): SimSpec {
    const jointNames = this.jointNames;
    const tendonNames = this.tendonNames;
    const fingerNames = this.params.fingers.map((f) => f.name);
    const tipLabels = fingerNames.flatMap((n) => [`${n}.x`, `${n}.y`, `${n}.z`]);
    const field = (name: string, labels: string[]) => ({
      name,
      shape: [labels.length] as [number],
      labels,
    });
    return {
      version: 1,
      controlDt: this.config.controlDt,
      physicsDt: this.physicsDt,
      actionSpace: {
        type: 'box',
        low: 0,
        high: 1,
        shape: [this.nTendons],
        names: tendonNames,
      },
      observationSpace: {
        type: 'dict',
        fields: [
          field('q', jointNames),
          field('qd', jointNames),
          field('tendonTension', tendonNames),
          field('tendonExcursion', tendonNames),
          field('tendonStretch', tendonNames),
          field('actuatorPosition', tendonNames),
          field('actuatorTarget', tendonNames),
          field('fingertipPositions', tipLabels),
        ],
      },
      jointNames,
      tendonNames,
      handParams: this.params,
    };
  }
}
