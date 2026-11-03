import { bundle } from "./map";

// Same linear distance / road half-width field as the backend. Cache geometry
// separately so dragging the cost slider only recolors existing pixels.
export function roadPreferenceField(blocked: string[], resolution = 0.5) {
  const [ox, oy] = bundle.grid.origin;
  const width = Math.ceil(
    (bundle.grid.width * bundle.grid.resolution) / resolution,
  );
  const height = Math.ceil(
    (bundle.grid.height * bundle.grid.resolution) / resolution,
  );
  const values = new Float32Array(width * height).fill(1);
  for (const road of bundle.network.roads) {
    if (blocked.includes(road.id)) continue;
    const radius =
      bundle.road_widths_m[road.id as keyof typeof bundle.road_widths_m] / 2;
    for (let i = 1; i < road.points.length; i++) {
      const a = road.points[i - 1],
        b = road.points[i];
      const dx = b.x - a.x,
        dy = b.y - a.y,
        norm = dx * dx + dy * dy;
      const c0 = Math.max(
        0,
        Math.floor((Math.min(a.x, b.x) - radius - ox) / resolution),
      );
      const c1 = Math.min(
        width,
        Math.ceil((Math.max(a.x, b.x) + radius - ox) / resolution),
      );
      const r0 = Math.max(
        0,
        Math.floor((Math.min(a.y, b.y) - radius - oy) / resolution),
      );
      const r1 = Math.min(
        height,
        Math.ceil((Math.max(a.y, b.y) + radius - oy) / resolution),
      );
      for (let r = r0; r < r1; r++)
        for (let c = c0; c < c1; c++) {
          const x = ox + (c + 0.5) * resolution,
            y = oy + (r + 0.5) * resolution;
          const t = norm
            ? Math.max(0, Math.min(1, ((x - a.x) * dx + (y - a.y) * dy) / norm))
            : 0;
          const fraction = Math.min(
            1,
            Math.hypot(x - a.x - t * dx, y - a.y - t * dy) / radius,
          );
          values[r * width + c] = Math.min(values[r * width + c], fraction);
        }
    }
  }
  return { width, height, values };
}

export function roadPreferenceImage(
  field: ReturnType<typeof roadPreferenceField>,
  cost: number,
) {
  const canvas = document.createElement("canvas");
  canvas.width = field.width;
  canvas.height = field.height;
  const ctx = canvas.getContext("2d");
  if (!ctx) return undefined;
  const image = ctx.createImageData(field.width, field.height);
  for (let r = 0; r < field.height; r++)
    for (let c = 0; c < field.width; c++) {
      const pixel = ((field.height - 1 - r) * field.width + c) * 4;
      image.data[pixel] = 181;
      image.data[pixel + 1] = 142;
      image.data[pixel + 2] = 255;
      image.data[pixel + 3] = Math.round(
        ((255 * 0.32 * cost) / 99) * field.values[r * field.width + c],
      );
    }
  ctx.putImageData(image, 0, 0);
  return canvas.toDataURL();
}
