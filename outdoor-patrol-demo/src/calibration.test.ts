import { describe, expect, it } from "vitest";
import { calibration, campus, geoToMap, mapToGeo, mapToSvg, mapYaw, toMap } from "./map";
describe("shared measured coordinate frame", () => {
  it("places the saved survey origin at zero with east/north axes", () => {
    expect(toMap(calibration.map_origin_svg)).toEqual({ x: 0, y: 0 });
    expect(mapToGeo({ x: 0, y: 0 }).latitude).toBeCloseTo(calibration.origin_wgs84.latitude, 8);
    expect(calibration.rotation_rad).toBe(0);
    expect(calibration.map_version).toBe(campus.version);
    expect(calibration.robot_executable).toBe(false);
    expect(toMap({ x: calibration.map_origin_svg.x + 8, y: calibration.map_origin_svg.y - 8 })).toEqual({ x: 1, y: 1 });
  });
  it("round trips every node and preserves heading", () => {
    for (const drawing of campus.nodes) {
      const map = toMap(drawing), geo = mapToGeo(map), restored = geoToMap(geo.latitude, geo.longitude);
      expect(restored.x).toBeCloseTo(map.x, 3);
      expect(restored.y).toBeCloseTo(map.y, 3);
      expect(mapToSvg(map).x).toBeCloseTo(drawing.x, 7);
      expect(mapToSvg(map).y).toBeCloseTo(drawing.y, 7);
    }
    expect(mapYaw(Math.PI / 2)).toBeCloseTo(Math.PI / 2, 10);
  });
});
