/** Rolling time-series plot of tendon tensions on a 2D canvas. */

import type { HandSimulator } from '../sim/simulator';

const FINGER_COLORS: Record<string, string> = {
  thumb: '#e8734a',
  index: '#4ea1ff',
  middle: '#52d68a',
  ring: '#c77dff',
  pinky: '#ffd166',
};

export class TensionPlot {
  windowSeconds = 8;

  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private sim: HandSimulator;
  private buffers: Float32Array[] = [];
  private colors: string[] = [];
  private dashes: boolean[] = [];
  private head = 0;
  private count = 0;
  private capacity = 0;

  constructor(sim: HandSimulator, canvas: HTMLCanvasElement) {
    this.sim = sim;
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d')!;
    this.rebuild();
  }

  rebuild(): void {
    this.capacity = Math.ceil(this.windowSeconds / this.sim.config.controlDt);
    this.buffers = this.sim.tendonNames.map(
      () => new Float32Array(this.capacity),
    );
    this.colors = this.sim.tendonNames.map(
      (n) => FINGER_COLORS[n.split('.')[0]] ?? '#8899aa',
    );
    // extensors dashed so antagonist pairs are distinguishable
    this.dashes = this.sim.tendonKinds.map((k) => k === 'extensor');
    this.head = 0;
    this.count = 0;
  }

  /** Push one sample per control step. */
  sample(): void {
    const t = this.sim.tension;
    for (let i = 0; i < this.buffers.length; i++) {
      this.buffers[i][this.head] = t[i];
    }
    this.head = (this.head + 1) % this.capacity;
    if (this.count < this.capacity) this.count++;
  }

  draw(): void {
    const { canvas, ctx } = this;
    const cssW = canvas.clientWidth;
    const cssH = canvas.clientHeight;
    if (cssW === 0 || cssH === 0) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (canvas.width !== cssW * dpr || canvas.height !== cssH * dpr) {
      canvas.width = cssW * dpr;
      canvas.height = cssH * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssW, cssH);

    // autoscale to the visible max (with a sane floor)
    let maxT = 5;
    for (const buf of this.buffers) {
      for (let k = 0; k < this.count; k++) {
        if (buf[k] > maxT) maxT = buf[k];
      }
    }
    maxT *= 1.1;

    // horizontal gridlines
    ctx.strokeStyle = 'rgba(120,140,160,0.15)';
    ctx.fillStyle = 'rgba(150,165,180,0.6)';
    ctx.font = '10px system-ui, sans-serif';
    ctx.lineWidth = 1;
    const gridN = 4;
    for (let g = 0; g <= gridN; g++) {
      const y = cssH - (g / gridN) * (cssH - 12) - 6;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(cssW, y);
      ctx.stroke();
      ctx.fillText(((g / gridN) * maxT).toFixed(0), 4, y - 2);
    }

    if (this.count < 2) return;

    for (let i = 0; i < this.buffers.length; i++) {
      const buf = this.buffers[i];
      ctx.strokeStyle = this.colors[i];
      ctx.globalAlpha = this.dashes[i] ? 0.55 : 0.9;
      ctx.setLineDash(this.dashes[i] ? [4, 3] : []);
      ctx.lineWidth = this.dashes[i] ? 1 : 1.5;
      ctx.beginPath();
      for (let k = 0; k < this.count; k++) {
        const idx = (this.head - this.count + k + this.capacity) % this.capacity;
        const x = (k / (this.capacity - 1)) * cssW;
        const y = cssH - 6 - (buf[idx] / maxT) * (cssH - 12);
        if (k === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    ctx.setLineDash([]);
  }
}
