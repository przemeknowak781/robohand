/**
 * High-level pose helpers: map a per-finger "curl" command (0 = open,
 * 1 = fully flexed) onto normalized tendon actions. Flexors are pulled
 * proportionally to curl; extensors get a light antagonist bias when open.
 */

import type { HandSimulator } from './simulator';

export type FingerCurls = Record<string, number>;

export function curlsToAction(
  sim: HandSimulator,
  curls: FingerCurls,
): number[] {
  const fingerNames = sim.params.fingers.map((f) => f.name);
  const kinds = sim.tendonKinds;
  const fingerIdx = sim.tendonFingerIndex;
  const action = new Array<number>(sim.nTendons).fill(0);
  for (let i = 0; i < sim.nTendons; i++) {
    const curl = Math.min(1, Math.max(0, curls[fingerNames[fingerIdx[i]]] ?? 0));
    if (kinds[i] === 'flexor') {
      action[i] = curl;
    } else {
      // antagonist: hold the finger open when relaxed, release when curling
      action[i] = 0.35 * (1 - curl);
    }
  }
  return action;
}

export interface PosePreset {
  name: string;
  curls: FingerCurls;
}

export const POSE_PRESETS: PosePreset[] = [
  {
    name: 'Open',
    curls: { thumb: 0, index: 0, middle: 0, ring: 0, pinky: 0 },
  },
  {
    name: 'Fist',
    curls: { thumb: 0.9, index: 1, middle: 1, ring: 1, pinky: 1 },
  },
  {
    name: 'Pinch',
    curls: { thumb: 0.55, index: 0.55, middle: 0.1, ring: 0.1, pinky: 0.1 },
  },
  {
    name: 'Point',
    curls: { thumb: 0.7, index: 0, middle: 1, ring: 1, pinky: 1 },
  },
  {
    name: 'OK sign',
    curls: { thumb: 0.6, index: 0.65, middle: 0.15, ring: 0.15, pinky: 0.15 },
  },
];
