/**
 * RoboHand — parametric tendon-driven humanoid hand simulator.
 * Entry point: wires the headless physics core to the three.js view,
 * HUD, tension plot, and control panel. Fixed-timestep control loop.
 */

import { buildHandParams, DEFAULT_TUNABLES, HandTunables } from './sim/defaultHand';
import { HandSimulator, DEFAULT_SIM_CONFIG } from './sim/simulator';
import { DatasetRecorder } from './sim/recorder';
import { curlsToAction, POSE_PRESETS } from './sim/presets';
import { SceneManager } from './viz/scene';
import { HandView } from './viz/handView';
import { TensionHud } from './viz/hud';
import { TensionPlot } from './viz/plot';
import { ControlPanel, RunState } from './ui/panel';

const tunables: HandTunables = { ...DEFAULT_TUNABLES };
const sim = new HandSimulator(buildHandParams(tunables), {
  ...DEFAULT_SIM_CONFIG,
});
const recorder = new DatasetRecorder();
const runState: RunState = { paused: false, timeScale: 1 };

const container = document.getElementById('app')!;
const sceneMgr = new SceneManager(container);
const handView = new HandView(sim, sceneMgr.scene);
const hud = new TensionHud(
  sim,
  document.getElementById('tension-table') as HTMLTableElement,
);
const plot = new TensionPlot(
  sim,
  document.getElementById('plot') as HTMLCanvasElement,
);
const statusEl = document.getElementById('status')!;

const panel = new ControlPanel({
  sim,
  tunables,
  runState,
  recorder,
  onTunablesChanged: () => {
    sim.setParams(buildHandParams(tunables));
    handView.rebuild();
  },
  onResetPose: () => {
    sim.reset();
    panel.syncTendonTargetsFromSim();
  },
});

// --- fixed-timestep control loop -------------------------------------------
let lastTime = performance.now();
let accumulator = 0;
let fps = 0;
let fpsSmoothed = 60;
let hudTimer = 0;
let stepsPerSecond = 0;
let stepCounter = 0;
let stepTimer = 0;

function frame(now: number): void {
  const realDt = Math.min(0.1, (now - lastTime) / 1000);
  lastTime = now;
  fps = 1 / Math.max(realDt, 1e-4);
  fpsSmoothed += (fps - fpsSmoothed) * 0.05;

  if (!runState.paused) {
    accumulator += realDt * runState.timeScale;
    const dt = sim.config.controlDt;
    let steps = 0;
    while (accumulator >= dt && steps < 6) {
      sim.step();
      recorder.record(sim.getAction(), sim.getObservation());
      plot.sample();
      accumulator -= dt;
      steps++;
      stepCounter++;
    }
    if (steps === 6) accumulator = 0; // drop backlog if tab was throttled
  }

  handView.update();
  plot.draw();
  panel.update();

  hudTimer += realDt;
  if (hudTimer > 0.08) {
    hud.update();
    hudTimer = 0;
  }

  stepTimer += realDt;
  if (stepTimer >= 1) {
    stepsPerSecond = stepCounter / stepTimer;
    stepCounter = 0;
    stepTimer = 0;
    statusEl.textContent =
      `t = ${sim.time.toFixed(1)} s · ${fpsSmoothed.toFixed(0)} fps · ` +
      `${stepsPerSecond.toFixed(0)} ctrl steps/s · ` +
      `physics ${(1 / sim.physicsDt).toFixed(0)} Hz · ` +
      `${sim.nJoints} joints · ${sim.nTendons} tendons`;
  }

  sceneMgr.render();
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

// --- programmatic / ML access ----------------------------------------------
// Expose the simulator for external harnesses (e.g. RL driving the sim from
// a WebSocket bridge or Playwright): window.robohand.sim.step([...14 actions])
declare global {
  interface Window {
    robohand: {
      sim: HandSimulator;
      recorder: DatasetRecorder;
      curlsToAction: typeof curlsToAction;
      presets: typeof POSE_PRESETS;
      setCurls: (curls: Record<string, number>) => void;
    };
  }
}

window.robohand = {
  sim,
  recorder,
  curlsToAction,
  presets: POSE_PRESETS,
  setCurls: (curls) => {
    sim.setAction(curlsToAction(sim, curls));
    panel.syncTendonTargetsFromSim();
  },
};
