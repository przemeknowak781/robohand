/**
 * JSONL dataset recorder for ML pipelines.
 * Each recorded control step becomes one line:
 *   {"t": <sim time>, "action": [...normalized 0..1], "obs": {...Observation}}
 * The first line is a header: {"type":"spec", "spec": <SimSpec>}.
 */

import type { Observation, SimSpec } from './types';

export class DatasetRecorder {
  private lines: string[] = [];
  private _active = false;
  private _steps = 0;

  get active(): boolean {
    return this._active;
  }

  get steps(): number {
    return this._steps;
  }

  start(spec: SimSpec): void {
    this.lines = [JSON.stringify({ type: 'spec', spec })];
    this._steps = 0;
    this._active = true;
  }

  record(action: number[], obs: Observation): void {
    if (!this._active) return;
    this.lines.push(
      JSON.stringify({ type: 'step', t: obs.time, action, obs }),
    );
    this._steps++;
  }

  stop(): void {
    this._active = false;
  }

  /** Serialized JSONL content of the current recording. */
  toJSONL(): string {
    return this.lines.join('\n') + '\n';
  }

  clear(): void {
    this.lines = [];
    this._steps = 0;
    this._active = false;
  }
}
