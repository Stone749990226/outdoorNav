import { describe, expect, it } from "vitest";
import {
  bundle,
  svgOrigin,
  campus,
  geoToMap,
  mapToGeo,
  mapToSvg,
  mapYaw,
  toMap,
} from "./map";
describe("shared measured coordinate frame", () => {
  it("places the saved survey origin at zero with east/north axes", () => {
    expect(toMap(svgOrigin)).toEqual({ x: 0, y: 0 });
    expect(mapToGeo({ x: 0, y: 0 }).latitude).toBeCloseTo(
      bundle.origin_wgs84.latitude,
      8,
    );
    expect(bundle.route_version).toBe(campus.version);
    expect(bundle.network.quality_status).toBe("survey_dgps_unverified");
    expect(
      toMap({
        x: svgOrigin.x + 8,
        y: svgOrigin.y - 8,
      }),
    ).toEqual({ x: 1, y: 1 });
  });
  it("round trips every node and preserves heading", () => {
    for (const drawing of campus.nodes) {
      const map = toMap(drawing),
        geo = mapToGeo(map),
        restored = geoToMap(geo.latitude, geo.longitude);
      expect(restored.x).toBeCloseTo(map.x, 3);
      expect(restored.y).toBeCloseTo(map.y, 3);
      expect(mapToSvg(map).x).toBeCloseTo(drawing.x, 7);
      expect(mapToSvg(map).y).toBeCloseTo(drawing.y, 7);
    }
    expect(mapYaw(Math.PI / 2)).toBeCloseTo(Math.PI / 2, 10);
  });
});
