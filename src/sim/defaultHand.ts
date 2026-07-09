/**
 * Parametric builder of a human-like, tendon-driven hand.
 *
 * Geometry conventions (palm frame = world frame of the hand):
 *   - palm lies in the XY plane, fingers point along +Y
 *   - dorsal side (back of the hand) is +Z, palmar side is -Z
 *   - flexion (positive q) curls fingertips toward -Z
 *
 * Tendon layout per finger (anthropomorphic naming):
 *   - FDP-like deep flexor   : routed palmar over MCP, PIP, DIP
 *   - FDS-like superf. flexor: routed palmar over MCP, PIP
 *   - EDC-like extensor      : routed dorsal over MCP, PIP, DIP
 * Thumb: FPL flexor (3 joints) + EPL extensor. Total 4×3 + 2 = 14 tendons.
 */

import type {
  FingerParams,
  HandParams,
  JointParams,
  PhalanxParams,
  TendonParams,
  TendonRouting,
} from './types';

/** High-level knobs exposed in the UI; everything else derives from these. */
export interface HandTunables {
  scale: number;
  fingerLengthMul: number;
  fingerRadiusMul: number;
  momentArmMul: number;
  flexorStiffness: number;   // N/m
  extensorStiffness: number; // N/m
  tendonDamping: number;     // N·s/m
  extensorPretension: number; // m
  passiveStiffness: number;  // N·m/rad
  passiveDamping: number;    // N·m·s/rad
  actuatorTimeConstant: number; // s
  actuatorMaxPullMul: number;
  gravityEnabled: boolean;
  gravityZ: number; // m/s², applied along -Z when negative
  maxSafeTension: number; // N
}

export const DEFAULT_TUNABLES: HandTunables = {
  scale: 1.0,
  fingerLengthMul: 1.0,
  fingerRadiusMul: 1.0,
  momentArmMul: 1.0,
  flexorStiffness: 4000,
  extensorStiffness: 1200,
  tendonDamping: 8,
  extensorPretension: 0.002,
  passiveStiffness: 0.03,
  passiveDamping: 0.004,
  actuatorTimeConstant: 0.06,
  actuatorMaxPullMul: 1.0,
  gravityEnabled: true,
  gravityZ: -9.81,
  maxSafeTension: 60,
};

interface FingerSpec {
  name: string;
  /** Phalanx lengths [m] proximal→distal, human-ish proportions. */
  lengths: number[];
  radius: number;
  baseX: number; // lateral placement on the palm
  baseY: number;
  baseZ: number;
  baseEuler: [number, number, number];
  /** Flexor moment arms per joint proximal→distal [m]. */
  flexArms: number[];
  /** Extensor moment arms per joint [m]. */
  extArms: number[];
  maxFlex: number[]; // per-joint max flexion [rad]
  hasFds: boolean;
}

const FINGER_SPECS: FingerSpec[] = [
  {
    name: 'thumb',
    lengths: [0.042, 0.032, 0.026],
    radius: 0.0105,
    baseX: -0.044,
    baseY: -0.005,
    baseZ: -0.010,
    // opposed thumb: swung toward -X in the palm plane, tilted palmar (-Z);
    // flexion then curls the tip across the palm toward the fingers
    baseEuler: [0, -0.45, 0.95],
    flexArms: [0.011, 0.009, 0.007],
    extArms: [0.008, 0.0065, 0.005],
    maxFlex: [1.05, 1.1, 1.35],
    hasFds: false,
  },
  {
    name: 'index',
    lengths: [0.045, 0.026, 0.020],
    radius: 0.0085,
    baseX: -0.030,
    baseY: 0.045,
    baseZ: 0,
    baseEuler: [0, 0, 0.06],
    flexArms: [0.0105, 0.0080, 0.0055],
    extArms: [0.0075, 0.0058, 0.0042],
    maxFlex: [1.55, 1.85, 1.35],
    hasFds: true,
  },
  {
    name: 'middle',
    lengths: [0.049, 0.030, 0.021],
    radius: 0.0088,
    baseX: -0.010,
    baseY: 0.049,
    baseZ: 0,
    baseEuler: [0, 0, 0],
    flexArms: [0.0110, 0.0084, 0.0057],
    extArms: [0.0078, 0.0060, 0.0043],
    maxFlex: [1.55, 1.9, 1.4],
    hasFds: true,
  },
  {
    name: 'ring',
    lengths: [0.046, 0.029, 0.020],
    radius: 0.0084,
    baseX: 0.010,
    baseY: 0.047,
    baseZ: 0,
    baseEuler: [0, 0, -0.05],
    flexArms: [0.0105, 0.0080, 0.0055],
    extArms: [0.0075, 0.0058, 0.0042],
    maxFlex: [1.55, 1.9, 1.4],
    hasFds: true,
  },
  {
    name: 'pinky',
    lengths: [0.037, 0.022, 0.017],
    radius: 0.0074,
    baseX: 0.030,
    baseY: 0.042,
    baseZ: 0,
    baseEuler: [0, 0, -0.12],
    flexArms: [0.0095, 0.0072, 0.0050],
    extArms: [0.0068, 0.0052, 0.0038],
    maxFlex: [1.55, 1.85, 1.4],
    hasFds: true,
  },
];

const JOINT_NAMES_FINGER = ['mcp', 'pip', 'dip'];
const JOINT_NAMES_THUMB = ['cmc', 'mp', 'ip'];

function buildFinger(spec: FingerSpec, t: HandTunables): FingerParams {
  const s = t.scale;
  const isThumb = spec.name === 'thumb';
  const jointNames = isThumb ? JOINT_NAMES_THUMB : JOINT_NAMES_FINGER;

  const phalanges: PhalanxParams[] = spec.lengths.map((len, i) => {
    const length = len * s * t.fingerLengthMul;
    const radius = spec.radius * s * t.fingerRadiusMul * (1 - 0.08 * i);
    // density-ish mass: cylinder of soft tissue+bone ~ 1100 kg/m³
    const mass = Math.PI * radius * radius * length * 1100;
    return {
      name: `${spec.name}.phalanx${i}`,
      length,
      radius,
      mass,
    };
  });

  const joints: JointParams[] = spec.lengths.map((_, i) => ({
    name: `${spec.name}.${jointNames[i]}`,
    minAngle: i === 0 ? -0.20 : -0.05,
    maxAngle: spec.maxFlex[i],
    restAngle: 0.15,
    passiveStiffness: t.passiveStiffness,
    passiveDamping: t.passiveDamping,
  }));

  const armScale = s * t.momentArmMul;
  const flexRouting: TendonRouting[] = spec.flexArms.map((r, i) => ({
    jointIndex: i,
    momentArm: r * armScale,
    sign: 1 as const,
  }));
  const extRouting: TendonRouting[] = spec.extArms.map((r, i) => ({
    jointIndex: i,
    momentArm: r * armScale,
    sign: -1 as const,
  }));

  const excursion = (routing: TendonRouting[]) =>
    routing.reduce(
      (acc, r) => acc + r.momentArm * spec.maxFlex[r.jointIndex],
      0,
    );

  const tendons: TendonParams[] = [];

  tendons.push({
    name: `${spec.name}.${isThumb ? 'fpl' : 'fdp'}`,
    kind: 'flexor',
    routing: flexRouting,
    stiffness: t.flexorStiffness,
    damping: t.tendonDamping,
    pretension: 0.0005,
    maxActuatorPull: excursion(flexRouting) * 1.25 * t.actuatorMaxPullMul,
    actuatorTimeConstant: t.actuatorTimeConstant,
    maxSafeTension: t.maxSafeTension,
  });

  if (spec.hasFds) {
    // superficial flexor terminates on the middle phalanx: MCP + PIP only
    const fdsRouting = flexRouting.slice(0, 2).map((r) => ({
      ...r,
      momentArm: r.momentArm * 1.1,
    }));
    tendons.push({
      name: `${spec.name}.fds`,
      kind: 'flexor',
      routing: fdsRouting,
      stiffness: t.flexorStiffness,
      damping: t.tendonDamping,
      pretension: 0.0005,
      maxActuatorPull: excursion(fdsRouting) * 1.25 * t.actuatorMaxPullMul,
      actuatorTimeConstant: t.actuatorTimeConstant,
      maxSafeTension: t.maxSafeTension,
    });
  }

  tendons.push({
    name: `${spec.name}.${isThumb ? 'epl' : 'edc'}`,
    kind: 'extensor',
    routing: extRouting,
    stiffness: t.extensorStiffness,
    damping: t.tendonDamping,
    pretension: t.extensorPretension,
    maxActuatorPull: excursion(extRouting) * 1.0 * t.actuatorMaxPullMul,
    actuatorTimeConstant: t.actuatorTimeConstant,
    maxSafeTension: t.maxSafeTension,
  });

  return {
    name: spec.name,
    basePosition: [spec.baseX * s, spec.baseY * s, spec.baseZ * s],
    baseEulerXYZ: [...spec.baseEuler],
    phalanges,
    joints,
    tendons,
  };
}

export function buildHandParams(
  tunables: Partial<HandTunables> = {},
): HandParams {
  const t: HandTunables = { ...DEFAULT_TUNABLES, ...tunables };
  const s = t.scale;
  return {
    name: 'robohand-v1',
    scale: s,
    palm: { width: 0.085 * s, length: 0.095 * s, thickness: 0.022 * s },
    gravity: [0, 0, t.gravityEnabled ? t.gravityZ : 0],
    gravityEnabled: t.gravityEnabled,
    limitStiffness: 0.6,
    limitDamping: 0.01,
    fingers: FINGER_SPECS.map((spec) => buildFinger(spec, t)),
  };
}
