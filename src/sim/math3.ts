/** Minimal 3D math (column-major 3x3 matrices as number[9]) — no deps. */

import type { Vec3 } from './types';

export type Mat3 = [
  number, number, number,
  number, number, number,
  number, number, number,
];

export function mat3Identity(): Mat3 {
  return [1, 0, 0, 0, 1, 0, 0, 0, 1];
}

/** Rotation about X axis. */
export function mat3RotX(a: number): Mat3 {
  const c = Math.cos(a), s = Math.sin(a);
  return [1, 0, 0, 0, c, s, 0, -s, c];
}

/** Rotation about Y axis. */
export function mat3RotY(a: number): Mat3 {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, 0, -s, 0, 1, 0, s, 0, c];
}

/** Rotation about Z axis. */
export function mat3RotZ(a: number): Mat3 {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, s, 0, -s, c, 0, 0, 0, 1];
}

/** out = a * b (column-major). */
export function mat3Mul(a: Mat3, b: Mat3): Mat3 {
  const o = new Array(9) as Mat3;
  for (let col = 0; col < 3; col++) {
    for (let row = 0; row < 3; row++) {
      o[col * 3 + row] =
        a[row] * b[col * 3] +
        a[3 + row] * b[col * 3 + 1] +
        a[6 + row] * b[col * 3 + 2];
    }
  }
  return o;
}

/** Rotation from XYZ intrinsic Euler angles: R = Rx·Ry·Rz. */
export function mat3FromEulerXYZ(e: Vec3): Mat3 {
  return mat3Mul(mat3Mul(mat3RotX(e[0]), mat3RotY(e[1])), mat3RotZ(e[2]));
}

/** v' = M * v */
export function mat3Apply(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
    m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
    m[2] * v[0] + m[5] * v[1] + m[8] * v[2],
  ];
}

/** Column i of M as a vector (basis axis in world frame). */
export function mat3Col(m: Mat3, i: 0 | 1 | 2): Vec3 {
  return [m[i * 3], m[i * 3 + 1], m[i * 3 + 2]];
}

export function v3Add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function v3Sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function v3Scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

export function v3Cross(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

export function v3Dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function v3LenSq(a: Vec3): number {
  return v3Dot(a, a);
}
