/**
 * Core parameter and state types for the tendon-driven hand simulator.
 *
 * This module (and everything under src/sim/) is intentionally free of any
 * DOM / three.js dependency so the whole physics core can run headless in
 * Node.js for ML training pipelines.
 */

export type Vec3 = [number, number, number];

/** A single revolute (flexion) joint. Angles in radians, flexion positive. */
export interface JointParams {
  name: string;
  /** Lower limit (slight hyperextension allowed, e.g. -0.1 rad). */
  minAngle: number;
  /** Upper limit (full flexion, e.g. ~1.6 rad). */
  maxAngle: number;
  /** Passive elastic rest angle of the joint capsule. */
  restAngle: number;
  /** Passive joint capsule stiffness [N·m/rad]. */
  passiveStiffness: number;
  /** Passive joint damping [N·m·s/rad]. */
  passiveDamping: number;
}

/** A rigid finger segment between two joints (or joint and fingertip). */
export interface PhalanxParams {
  name: string;
  /** Segment length [m]. */
  length: number;
  /** Visual/collision radius [m]. */
  radius: number;
  /** Segment mass [kg] (used for gravity torques and effective inertia). */
  mass: number;
}

/**
 * One routing entry: the tendon passes over the pulley of a joint.
 * `sign = +1` means tension produces flexion torque (palmar routing),
 * `sign = -1` means extension torque (dorsal routing).
 */
export interface TendonRouting {
  /** Joint index local to the finger (0 = most proximal). */
  jointIndex: number;
  /** Pulley radius / moment arm [m]. */
  momentArm: number;
  sign: 1 | -1;
}

/**
 * A tendon: an elastic cable driven by a linear actuator (motor + spool).
 * Tendon tension:  T = max(0, k·s + c·ṡ)  where the stretch
 * s = pretension + x_a − e(q)  and the excursion e(q) = Σ sign·r·q.
 */
export interface TendonParams {
  name: string;
  kind: 'flexor' | 'extensor';
  routing: TendonRouting[];
  /** Tendon elastic stiffness [N/m]. */
  stiffness: number;
  /** Tendon material damping [N·s/m]. */
  damping: number;
  /** Pre-stretch at rest pose [m] (gives extensors a return-spring bias). */
  pretension: number;
  /** Actuator travel mapped to action = 1.0 [m]. */
  maxActuatorPull: number;
  /** First-order actuator lag time constant [s]. */
  actuatorTimeConstant: number;
  /** Tension at which the tendon is considered overloaded [N] (for UI/ML). */
  maxSafeTension: number;
}

/** A serial kinematic chain: base frame on the palm + phalanges + joints. */
export interface FingerParams {
  name: string;
  /** Position of the first joint (MCP/CMC) in the palm frame [m]. */
  basePosition: Vec3;
  /** Orientation of the finger base frame, XYZ Euler [rad]. */
  baseEulerXYZ: Vec3;
  phalanges: PhalanxParams[];
  joints: JointParams[];
  tendons: TendonParams[];
}

export interface HandParams {
  name: string;
  /** Global geometric scale multiplier. */
  scale: number;
  palm: { width: number; length: number; thickness: number };
  gravity: Vec3;
  gravityEnabled: boolean;
  /** Joint-limit penalty stiffness [N·m/rad] and damping [N·m·s/rad]. */
  limitStiffness: number;
  limitDamping: number;
  fingers: FingerParams[];
}

export interface SimConfig {
  /** Control (ML) timestep [s]; one `step()` advances this much sim time. */
  controlDt: number;
  /** Physics substeps per control step. physicsDt = controlDt / substeps. */
  substeps: number;
}

/** Full observation returned by the simulator each control step. */
export interface Observation {
  /** Simulation time [s]. */
  time: number;
  /** Joint angles [rad], global joint order. */
  q: number[];
  /** Joint angular velocities [rad/s]. */
  qd: number[];
  /** Tendon tensions [N], global tendon order. */
  tendonTension: number[];
  /** Tendon excursions e(q) [m]. */
  tendonExcursion: number[];
  /** Tendon elastic stretch s [m] (0 when slack). */
  tendonStretch: number[];
  /** Actuator positions x_a [m]. */
  actuatorPosition: number[];
  /** Actuator position targets [m]. */
  actuatorTarget: number[];
  /** Fingertip world positions, flattened [x,y,z] per finger [m]. */
  fingertipPositions: number[];
}

/** Machine-readable description of action/observation spaces for ML. */
export interface SimSpec {
  version: number;
  controlDt: number;
  physicsDt: number;
  actionSpace: {
    type: 'box';
    low: number;
    high: number;
    shape: [number];
    /** One name per action dimension (tendon actuator). */
    names: string[];
  };
  observationSpace: {
    type: 'dict';
    fields: { name: string; shape: [number]; labels: string[] }[];
  };
  jointNames: string[];
  tendonNames: string[];
  handParams: HandParams;
}
