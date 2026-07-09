/** Trigger a browser download of a text payload. */
export function downloadText(
  filename: string,
  text: string,
  mime = 'application/json',
): void {
  downloadBlob(filename, new Blob([text], { type: mime }));
}

/** Trigger a browser download of binary data (e.g. STL). */
export function downloadBinary(
  filename: string,
  data: DataView | ArrayBuffer,
  mime = 'application/octet-stream',
): void {
  const bytes = data instanceof DataView
    ? new Uint8Array(data.buffer as ArrayBuffer, data.byteOffset, data.byteLength)
    : new Uint8Array(data);
  downloadBlob(filename, new Blob([bytes], { type: mime }));
}

function downloadBlob(filename: string, blob: Blob): void {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}
