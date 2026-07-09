/**
 * Headless demo: runs the physics core in Node with no browser/three.js.
 * Closes the hand into a fist over 2 s, then opens it, printing tendon
 * tension telemetry — the same loop an RL environment wrapper would run.
 *
 *   npm run demo:headless
 */

import { buildHandParams } from '../src/sim/defaultHand';
import { HandSimulator } from '../src/sim/simulator';
import { curlsToAction } from '../src/sim/presets';
import { DatasetRecorder } from '../src/sim/recorder';

const sim = new HandSimulator(buildHandParams());
const recorder = new DatasetRecorder();
const spec = sim.getSpec();

console.log(`model: ${spec.handParams.name}`);
console.log(`action space : ${spec.actionSpace.shape[0]} tendon actuators`);
console.log(`observation  : ${spec.observationSpace.fields
  .map((f) => `${f.name}[${f.shape[0]}]`)
  .join(' ')}`);
console.log(`control dt   : ${spec.controlDt.toFixed(4)} s ` +
  `(physics ${spec.physicsDt.toExponential(2)} s)\n`);

recorder.start(spec);

const seconds = 4;
const steps = Math.round(seconds / spec.controlDt);
let peak = 0;
let peakName = '';

for (let i = 0; i < steps; i++) {
  const t = i * spec.controlDt;
  // 0→2 s: close into a fist; 2→4 s: open again
  const curl = t < 2 ? Math.min(1, t / 1.2) : Math.max(0, 1 - (t - 2) / 1.2);
  const action = curlsToAction(sim, {
    thumb: curl * 0.9,
    index: curl,
    middle: curl,
    ring: curl,
    pinky: curl,
  });
  const obs = sim.step(action);
  recorder.record(action, obs);

  obs.tendonTension.forEach((T, k) => {
    if (T > peak) {
      peak = T;
      peakName = spec.tendonNames[k];
    }
  });

  if (i % Math.round(0.5 / spec.controlDt) === 0) {
    const maxT = Math.max(...obs.tendonTension);
    const meanQ =
      obs.q.reduce((a, b) => a + b, 0) / obs.q.length;
    console.log(
      `t=${obs.time.toFixed(2)}s  curl=${curl.toFixed(2)}  ` +
      `mean q=${(meanQ * 57.2958).toFixed(1)}°  max T=${maxT.toFixed(1)} N`,
    );
  }
}

recorder.stop();
const jsonl = recorder.toJSONL();
console.log(`\npeak tension : ${peak.toFixed(1)} N (${peakName})`);
console.log(`dataset      : ${recorder.steps} steps, ` +
  `${(jsonl.length / 1024).toFixed(0)} KiB JSONL in memory`);

// sanity checks — non-zero motion and tensions, finite values
const finalObs = sim.getObservation();
const allFinite = [
  ...finalObs.q, ...finalObs.qd, ...finalObs.tendonTension,
].every(Number.isFinite);
if (!allFinite) {
  console.error('FAIL: non-finite state detected');
  process.exit(1);
}
if (peak < 1) {
  console.error('FAIL: tendons never developed tension');
  process.exit(1);
}
console.log('sanity checks: OK');
