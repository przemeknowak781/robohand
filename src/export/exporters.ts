/**
 * Fabrication exports:
 *  - STL of the assembled hand in its current pose (binary, millimeters)
 *  - STL "parts kit": every part instance laid out flat on a grid in its
 *    print orientation (plates flat, tendon channels vertical)
 *  - DXF of all flat-plate profiles (outlines + hole circles, millimeters)
 *    for CNC / laser cutting.
 */

import * as THREE from 'three';
import { STLExporter } from 'three/addons/exporters/STLExporter.js';
import type { PartsManifest, PlateProfile } from '../viz/parts';

const M_TO_MM = 1000;

/** Binary STL of an object tree, scaled meters → millimeters. */
export function exportAssemblySTL(root: THREE.Object3D): DataView {
  const wrapper = new THREE.Group();
  wrapper.scale.setScalar(M_TO_MM);
  // temporarily reparent (keeps world transforms of children intact
  // relative to the wrapper)
  const parent = root.parent;
  wrapper.add(root);
  wrapper.updateMatrixWorld(true);
  const exporter = new STLExporter();
  const data = exporter.parse(wrapper, { binary: true }) as unknown as DataView;
  if (parent) parent.add(root);
  else wrapper.remove(root);
  root.updateMatrixWorld(true);
  return data;
}

/** Lay every part instance flat on a grid and export one binary STL. */
export function exportPartsKitSTL(manifest: PartsManifest): DataView {
  const kit = new THREE.Group();
  const margin = 0.006;
  const box = new THREE.Box3();
  const size = new THREE.Vector3();

  // place parts row by row, wrapping at a fixed bed width
  const bedWidth = 0.24;
  let cursorX = 0;
  let cursorY = 0;
  let rowDepth = 0;

  for (const entry of manifest.list()) {
    for (let i = 0; i < entry.count; i++) {
      const mesh = new THREE.Mesh(entry.raw);
      box.setFromObject(mesh);
      box.getSize(size);
      if (cursorX + size.x > bedWidth && cursorX > 0) {
        cursorX = 0;
        cursorY += rowDepth + margin;
        rowDepth = 0;
      }
      mesh.position.set(
        cursorX - box.min.x,
        cursorY - box.min.y,
        -box.min.z, // rest on the build plate
      );
      kit.add(mesh);
      cursorX += size.x + margin;
      rowDepth = Math.max(rowDepth, size.y);
    }
  }

  kit.scale.setScalar(M_TO_MM);
  kit.updateMatrixWorld(true);
  const exporter = new STLExporter();
  return exporter.parse(kit, { binary: true }) as unknown as DataView;
}

// ------------------------------------------------------------------ DXF ---

function dxfEntity(lines: (string | number)[]): string {
  return lines.join('\n') + '\n';
}

function dxfPolyline(
  pts: { x: number; y: number }[],
  ox: number,
  oy: number,
): string {
  const lines: (string | number)[] = [
    0, 'LWPOLYLINE', 8, 'CUT', 90, pts.length, 70, 1,
  ];
  for (const p of pts) {
    lines.push(10, ((p.x + ox) * M_TO_MM).toFixed(3));
    lines.push(20, ((p.y + oy) * M_TO_MM).toFixed(3));
  }
  return dxfEntity(lines);
}

function dxfCircle(
  c: { x: number; y: number; r: number },
  ox: number,
  oy: number,
): string {
  return dxfEntity([
    0, 'CIRCLE', 8, 'CUT',
    10, ((c.x + ox) * M_TO_MM).toFixed(3),
    20, ((c.y + oy) * M_TO_MM).toFixed(3),
    40, (c.r * M_TO_MM).toFixed(3),
  ]);
}

function dxfText(text: string, x: number, y: number, h: number): string {
  return dxfEntity([
    0, 'TEXT', 8, 'LABEL',
    10, (x * M_TO_MM).toFixed(3),
    20, (y * M_TO_MM).toFixed(3),
    40, (h * M_TO_MM).toFixed(3),
    1, text,
  ]);
}

/**
 * All flat-plate profiles laid out in rows, labelled with part name, count
 * and stock thickness. Units: millimeters ($INSUNITS = 4).
 */
export function exportPlatesDXF(profiles: PlateProfile[]): string {
  let entities = '';
  const margin = 0.008;
  let ox = 0;
  let oy = 0;
  let rowH = 0;
  const rowWidth = 0.28;

  for (const prof of profiles) {
    const xs = prof.outline.map((p) => p.x);
    const ys = prof.outline.map((p) => p.y);
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);
    const w = maxX - minX;
    const h = maxY - minY;
    if (ox + w > rowWidth && ox > 0) {
      ox = 0;
      oy += rowH + margin + 0.004; // extra room for the label line
      rowH = 0;
    }
    const px = ox - minX;
    const py = oy - minY;
    entities += dxfPolyline(prof.outline, px, py);
    for (const c of prof.circles) entities += dxfCircle(c, px, py);
    entities += dxfText(
      `${prof.label} x${prof.count} t=${(prof.thickness * M_TO_MM).toFixed(1)}mm`,
      ox,
      oy + h + 0.0012,
      0.0022,
    );
    ox += w + margin;
    rowH = Math.max(rowH, h + 0.005);
  }

  return [
    '0', 'SECTION', '2', 'HEADER',
    '9', '$INSUNITS', '70', '4',
    '0', 'ENDSEC',
    '0', 'SECTION', '2', 'ENTITIES',
    entities.trimEnd(),
    '0', 'ENDSEC',
    '0', 'EOF',
  ].join('\n') + '\n';
}
