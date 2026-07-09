/**
 * Looping animated pose presets: unlike POSE_PRESETS (a single static curl
 * target), each of these is a pure function of elapsed time that the caller
 * samples every control step, producing a continuously repeating motion
 * (grasp/release, a finger wave, idle tapping, a pinch cycle). No DOM/
 * three.js dependency — usable from the headless core too (e.g. to script
 * a demo trajectory for a dataset recording).
 */

import type { FingerCurls } from './presets';

export interface AnimationPreset {
  name: string;
  /** Curls at time t [s] since the animation was selected; loops forever. */
  curlsAt(t: number): FingerCurls;
}

const FINGER_ORDER = ['thumb', 'index', 'middle', 'ring', 'pinky'] as const;

/** 0 → 1 → 0 continuously, one full cycle per `period` seconds. */
function triangle01(t: number, period: number): number {
  const phase = ((t % period) + period) % period / period;
  return phase < 0.5 ? phase * 2 : 2 - phase * 2;
}

/** Smooth out a linear 0..1 ramp (ease in/out) so motion doesn't snap. */
function easeInOut(x: number): number {
  return 0.5 - 0.5 * Math.cos(Math.PI * x);
}

export const ANIMATION_PRESETS: AnimationPreset[] = [
  {
    name: 'Grasp cycle',
    curlsAt(t) {
      const c = easeInOut(triangle01(t, 3.2));
      const curls: FingerCurls = {};
      for (const f of FINGER_ORDER) curls[f] = f === 'thumb' ? c * 0.9 : c;
      return curls;
    },
  },
  {
    name: 'Finger wave',
    curlsAt(t) {
      const period = 2.6;
      const perFingerDelay = 0.16;
      const curls: FingerCurls = { thumb: 0.15 };
      const fingers = ['index', 'middle', 'ring', 'pinky'];
      fingers.forEach((f, i) => {
        curls[f] = easeInOut(triangle01(t - i * period * perFingerDelay, period));
      });
      return curls;
    },
  },
  {
    name: 'Piano tap',
    curlsAt(t) {
      const freqHz: Record<string, number> = {
        thumb: 0.9, index: 1.35, middle: 1.65, ring: 1.15, pinky: 1.9,
      };
      const phase: Record<string, number> = {
        thumb: 0, index: 0.6, middle: 1.7, ring: 2.9, pinky: 4.1,
      };
      const curls: FingerCurls = {};
      for (const f of FINGER_ORDER) {
        const raw = 0.5 + 0.5 * Math.sin(2 * Math.PI * freqHz[f] * t + phase[f]);
        // sharpen: mostly open, brief sharp taps rather than a smooth sine
        curls[f] = Math.max(0, Math.min(1, raw * 1.6 - 0.6));
      }
      return curls;
    },
  },
  {
    name: 'Pinch cycle',
    curlsAt(t) {
      const c = easeInOut(triangle01(t, 2.0));
      return {
        thumb: 0.15 + 0.5 * c,
        index: 0.15 + 0.5 * c,
        middle: 0.05 * c,
        ring: 0.05 * c,
        pinky: 0.05 * c,
      };
    },
  },
];
