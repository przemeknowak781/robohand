/** DOM overlay: live per-tendon tension table with bars. */

import type { HandSimulator } from '../sim/simulator';

const FINGER_COLORS: Record<string, string> = {
  thumb: '#e8734a',
  index: '#4ea1ff',
  middle: '#52d68a',
  ring: '#c77dff',
  pinky: '#ffd166',
};

export class TensionHud {
  private rows: { val: HTMLElement; bar: HTMLElement }[] = [];
  private table: HTMLTableElement;
  private sim: HandSimulator;

  constructor(sim: HandSimulator, table: HTMLTableElement) {
    this.sim = sim;
    this.table = table;
    this.rebuild();
  }

  rebuild(): void {
    this.table.innerHTML = '';
    this.rows = [];
    const names = this.sim.tendonNames;
    for (const name of names) {
      const fingerName = name.split('.')[0];
      const color = FINGER_COLORS[fingerName] ?? '#8899aa';
      const tr = document.createElement('tr');

      const tdName = document.createElement('td');
      tdName.textContent = name;
      tdName.style.color = color;

      const tdVal = document.createElement('td');
      tdVal.className = 'val';
      tdVal.textContent = '0.0';

      const tdBar = document.createElement('td');
      tdBar.className = 'bar-cell';
      const bar = document.createElement('div');
      bar.className = 'bar';
      const fill = document.createElement('i');
      fill.style.background = color;
      bar.appendChild(fill);
      tdBar.appendChild(bar);

      tr.append(tdName, tdVal, tdBar);
      this.table.appendChild(tr);
      this.rows.push({ val: tdVal, bar: fill });
    }
  }

  update(): void {
    const t = this.sim.tension;
    const maxSafe = this.sim.maxSafeTensions;
    for (let i = 0; i < this.rows.length; i++) {
      const row = this.rows[i];
      row.val.textContent = t[i].toFixed(1);
      const frac = Math.min(1, t[i] / maxSafe[i]);
      row.bar.style.width = `${(frac * 100).toFixed(1)}%`;
      row.val.style.color = frac > 0.85 ? '#ff6b5e' : '';
    }
  }
}
