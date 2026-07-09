---
name: verify
description: Build, run, and drive the RoboHand tendon-hand simulator to verify changes end-to-end (browser GUI + headless physics core).
---

# Verifying RoboHand

## Build & launch

```bash
npm install                # once
npm run build              # vite build → dist/
npm run preview -- --port 4173 --host 127.0.0.1 &   # serve dist/
```

Dev server alternative: `npm run dev` (port 5173).

## Surfaces

1. **Browser GUI** (primary): drive with playwright-core against the
   preinstalled Chromium:
   ```js
   chromium.launch({
     executablePath: '/opt/pw-browsers/chromium',
     args: ['--no-sandbox', '--use-gl=swiftshader', '--enable-unsafe-swiftshader'],
   })
   ```
   Software GL gives ~15–140 fps — low fps here is the renderer, not a
   regression.
2. **Headless physics core**: `npm run demo:headless` — runs the sim in Node,
   prints tension telemetry, exits non-zero on non-finite state or zero
   tension (built-in sanity checks).
3. **Tendon-tube orientation regression**: `npm run check:tendon-twist` —
   sweeps every finger's curl 0→1 and checks the tendon tube's per-point
   orientation quaternions never jump (a jump is what a Frenet-frame flip /
   "tube twists on itself" bug looks like). Exits non-zero on any
   discontinuity. Re-run this after touching `handView.ts`'s tendon routing.

## Flows worth driving

- Click the **Fist** preset button → within ~2.5 s expect
  `max(sim.tension) ≈ 41.7 N` and mean joint angle ≈ 86° (deterministic —
  identical values in headless and browser runs at default params). Note:
  per-joint max flexion is clamped by mechanical guide-block clearance
  (`computeMechJointLimits`), e.g. PIP/DIP joints cap around 75-80° even
  though the anthropomorphic spec allows more — that's intentional, not a
  regression.
- `window.robohand` exposes `{ sim, recorder, setCurls }` for programmatic
  driving; `sim.step(action[14])` is the ML entry point.
- **Data / ML folder**: record start → stop → "download dataset (.jsonl)";
  intercept with Playwright's `download` event. Line 1 is
  `{"type":"spec",...}` with `actionSpace.shape[0] === 14`.
- **Animation loops folder**: click e.g. "▶ Grasp cycle" → `sim.q` keeps
  changing every frame without further input (poll `sim.getAction()`
  before/after, not `sim.q` — q keeps drifting briefly after stop from
  spring relaxation, that's expected, not a bug). Click "■ stop (manual
  control)" → `sim.getAction()` freezes immediately (compare two samples a
  few hundred ms apart, expect bit-identical). Any pose preset / curl
  slider / tendon actuator interaction should also silently cancel a
  running animation.
- **Hand geometry → global scale**: fill the number input + Enter
  (onFinishChange) → live rebuild must preserve pose/time and stay finite.
- Robustness probe: `sim.step([NaN, Infinity, -5, 99])` then more steps —
  all of `q`, `tension`, `actuatorTarget` must stay finite.

## Gotchas

- lil-gui buttons are matched by text (`page.getByText('Fist', { exact: true })`);
  folders must be clicked open before their controls are visible.
- The page has no test IDs; the HUD table is `#tension-table`, plot is `#plot`.
- A 404 in console = missing asset only; the app itself logs no errors on a
  clean run.
