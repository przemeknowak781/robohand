/**
 * lil-gui control panel: pose presets, per-finger curls, raw tendon targets,
 * physical parameter tuning (live rebuild), simulation and dataset controls.
 */

import GUI from 'lil-gui';
import type { HandSimulator } from '../sim/simulator';
import type { HandTunables } from '../sim/defaultHand';
import { POSE_PRESETS, curlsToAction, FingerCurls } from '../sim/presets';
import { ANIMATION_PRESETS, type AnimationPreset } from '../sim/animations';
import type { DatasetRecorder } from '../sim/recorder';
import { downloadText } from './download';

export interface RunState {
  paused: boolean;
  timeScale: number;
  /** Looping animation currently driving the curls, or null for manual control. */
  activeAnimation: AnimationPreset | null;
  /** sim.time when activeAnimation was selected (animation's t=0). */
  animationStartTime: number;
}

export interface PanelDeps {
  sim: HandSimulator;
  tunables: HandTunables;
  runState: RunState;
  recorder: DatasetRecorder;
  /** Rebuild params from tunables and push into sim + views. */
  onTunablesChanged: () => void;
  onResetPose: () => void;
  /** Fabrication exports (STL assembly / STL parts kit / DXF profiles). */
  onExportAssemblySTL: () => void;
  onExportPartsKitSTL: () => void;
  onExportPlatesDXF: () => void;
}

export class ControlPanel {
  readonly gui: GUI;
  private deps: PanelDeps;
  private curls: FingerCurls & { master: number };
  private tendonTargets: Record<string, number> = {};
  private tendonControllers: ReturnType<GUI['add']>[] = [];
  private curlControllers: ReturnType<GUI['add']>[] = [];
  private recInfo = { steps: 0, status: 'idle' };

  constructor(deps: PanelDeps) {
    this.deps = deps;
    this.gui = new GUI({ title: 'RoboHand — controls' });
    this.curls = { master: 0, thumb: 0, index: 0, middle: 0, ring: 0, pinky: 0 };

    this.buildPresets();
    this.buildCurls();
    this.buildAnimations();
    this.buildTendonTargets();
    this.buildParameters();
    this.buildSimulation();
    this.buildData();
  }

  private fingerNames(): string[] {
    return this.deps.sim.params.fingers.map((f) => f.name);
  }

  private applyCurls(): void {
    this.deps.runState.activeAnimation = null; // manual control overrides any loop
    const action = curlsToAction(this.deps.sim, this.curls);
    this.deps.sim.setAction(action);
    this.syncTendonTargetsFromSim();
  }

  /** Reflect the sim's normalized targets in the advanced sliders. */
  syncTendonTargetsFromSim(): void {
    const a = this.deps.sim.getAction();
    this.deps.sim.tendonNames.forEach((n, i) => {
      this.tendonTargets[n] = a[i];
    });
    for (const c of this.tendonControllers) c.updateDisplay();
  }

  private buildPresets(): void {
    const folder = this.gui.addFolder('Pose presets');
    const actions: Record<string, () => void> = {};
    for (const preset of POSE_PRESETS) {
      actions[preset.name] = () => {
        for (const name of this.fingerNames()) {
          this.curls[name] = preset.curls[name] ?? 0;
        }
        for (const c of this.curlControllers) c.updateDisplay();
        this.applyCurls();
      };
      folder.add(actions, preset.name);
    }
  }

  private buildCurls(): void {
    const folder = this.gui.addFolder('Finger curl (0 = open, 1 = fist)');
    this.curlControllers.push(
      folder.add(this.curls, 'master', 0, 1, 0.01).name('all fingers')
        .onChange((v: number) => {
          for (const name of this.fingerNames()) this.curls[name] = v;
          for (const c of this.curlControllers) c.updateDisplay();
          this.applyCurls();
        }),
    );
    for (const name of this.fingerNames()) {
      this.curlControllers.push(
        folder.add(this.curls, name, 0, 1, 0.01)
          .onChange(() => this.applyCurls()),
      );
    }
  }

  private buildTendonTargets(): void {
    const folder = this.gui.addFolder('Tendon actuators (advanced)');
    folder.close();
    const sim = this.deps.sim;
    sim.tendonNames.forEach((name, i) => {
      this.tendonTargets[name] = 0;
      this.tendonControllers.push(
        folder.add(this.tendonTargets, name, 0, 1, 0.01).onChange((v: number) => {
          this.deps.runState.activeAnimation = null;
          const a = sim.getAction();
          a[i] = v;
          sim.setAction(a);
        }),
      );
    });
  }

  private buildParameters(): void {
    const t = this.deps.tunables;
    const changed = () => this.deps.onTunablesChanged();

    const geo = this.gui.addFolder('Hand geometry');
    geo.close();
    geo.add(t, 'scale', 0.5, 2, 0.01).name('global scale').onFinishChange(changed);
    geo.add(t, 'fingerLengthMul', 0.6, 1.6, 0.01).name('finger length ×').onFinishChange(changed);
    geo.add(t, 'fingerRadiusMul', 0.6, 1.6, 0.01).name('finger radius ×').onFinishChange(changed);
    geo.add(t, 'momentArmMul', 0.5, 2, 0.01).name('pulley radius ×').onFinishChange(changed);

    const tendon = this.gui.addFolder('Tendon physics');
    tendon.close();
    tendon.add(t, 'flexorStiffness', 500, 12000, 50).name('flexor k [N/m]').onFinishChange(changed);
    tendon.add(t, 'extensorStiffness', 200, 6000, 50).name('extensor k [N/m]').onFinishChange(changed);
    tendon.add(t, 'tendonDamping', 0, 40, 0.5).name('damping [N·s/m]').onFinishChange(changed);
    tendon.add(t, 'extensorPretension', 0, 0.008, 0.0002).name('ext. pretension [m]').onFinishChange(changed);
    tendon.add(t, 'maxSafeTension', 10, 200, 5).name('max safe T [N]').onFinishChange(changed);

    const joints = this.gui.addFolder('Joints & actuators');
    joints.close();
    joints.add(t, 'passiveStiffness', 0, 0.2, 0.001).name('joint k [N·m/rad]').onFinishChange(changed);
    joints.add(t, 'passiveDamping', 0, 0.05, 0.0005).name('joint c [N·m·s/rad]').onFinishChange(changed);
    joints.add(t, 'actuatorTimeConstant', 0.01, 0.5, 0.005).name('actuator τ [s]').onFinishChange(changed);
    joints.add(t, 'actuatorMaxPullMul', 0.5, 2, 0.05).name('actuator travel ×').onFinishChange(changed);

    const mech = this.gui.addFolder('Mechanical design (fabrication)');
    mech.close();
    const mm = (v: number) => v * 1000;
    const asMm = (obj: HandTunables, key: keyof HandTunables, min: number, max: number, step: number, label: string) => {
      const proxy = { [key]: mm(obj[key] as number) };
      mech.add(proxy, key as string, min, max, step).name(label)
        .onFinishChange((v: number) => {
          (obj[key] as number) = v / 1000;
          changed();
        });
    };
    asMm(t, 'plateThickness', 1.5, 5, 0.1, 'plate thickness [mm]');
    asMm(t, 'pinDiameter', 2, 6, 0.1, 'hinge pin ⌀ [mm]');
    asMm(t, 'hingeClearance', 0.1, 1, 0.05, 'fit clearance [mm]');
    asMm(t, 'channelDiameter', 1.5, 5, 0.1, 'tendon channel ⌀ [mm]');
    asMm(t, 'guideWall', 0.6, 3, 0.1, 'min wall [mm]');
    asMm(t, 'palmPlateThickness', 2, 5, 0.1, 'palm plate [mm]');

    const fab = this.gui.addFolder('Export (fabrication)');
    fab.add({ f: () => this.deps.onExportAssemblySTL() }, 'f')
      .name('STL — assembly (current pose)');
    fab.add({ f: () => this.deps.onExportPartsKitSTL() }, 'f')
      .name('STL — parts kit (print layout)');
    fab.add({ f: () => this.deps.onExportPlatesDXF() }, 'f')
      .name('DXF — plate profiles (CNC)');

    const env = this.gui.addFolder('Environment');
    env.close();
    env.add(t, 'gravityEnabled').name('gravity').onFinishChange(changed);
    env.add(t, 'gravityZ', -25, 0, 0.1).name('g [m/s²]').onFinishChange(changed);
  }

  private buildAnimations(): void {
    const folder = this.gui.addFolder('Animation loops');
    const { sim, runState } = this.deps;
    for (const preset of ANIMATION_PRESETS) {
      folder.add({
        f: () => {
          runState.activeAnimation = preset;
          runState.animationStartTime = sim.time;
        },
      }, 'f').name(`▶ ${preset.name}`);
    }
    folder.add({
      f: () => {
        runState.activeAnimation = null;
      },
    }, 'f').name('■ stop (manual control)');
  }

  private buildSimulation(): void {
    const folder = this.gui.addFolder('Simulation');
    folder.close();
    folder.add(this.deps.runState, 'paused');
    folder.add(this.deps.runState, 'timeScale', 0.1, 3, 0.05).name('time scale');
    folder.add(this.deps.sim.config, 'substeps', 2, 32, 1).name('physics substeps');
    folder.add({
      reset: () => {
        this.deps.runState.activeAnimation = null;
        this.deps.onResetPose();
      },
    }, 'reset').name('reset pose');
  }

  private buildData(): void {
    const folder = this.gui.addFolder('Data / ML');
    const { sim, recorder } = this.deps;

    const ops = {
      startStop: () => {
        if (recorder.active) {
          recorder.stop();
          this.recInfo.status = `stopped (${recorder.steps} steps)`;
        } else {
          recorder.start(sim.getSpec());
          this.recInfo.status = 'recording…';
        }
      },
      downloadDataset: () => {
        if (recorder.steps === 0) {
          this.recInfo.status = 'nothing recorded yet';
          return;
        }
        downloadText(
          `robohand-dataset-${Date.now()}.jsonl`,
          recorder.toJSONL(),
          'application/jsonl',
        );
      },
      downloadSpec: () => {
        downloadText(
          'robohand-spec.json',
          JSON.stringify(sim.getSpec(), null, 2),
        );
      },
    };

    folder.add(ops, 'startStop').name('record: start / stop');
    folder.add(this.recInfo, 'status').name('recorder').listen().disable();
    folder.add(this.recInfo, 'steps').name('recorded steps').listen().disable();
    folder.add(ops, 'downloadDataset').name('download dataset (.jsonl)');
    folder.add(ops, 'downloadSpec').name('download sim spec (.json)');
  }

  /** Called every frame to refresh live readouts. */
  update(): void {
    this.recInfo.steps = this.deps.recorder.steps;
    if (this.deps.recorder.active) {
      this.recInfo.status = `recording… (${this.deps.recorder.steps})`;
    }
    if (this.deps.runState.activeAnimation) this.syncTendonTargetsFromSim();
  }
}
