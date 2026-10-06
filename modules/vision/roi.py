"""Image-space geometry of the camera ROI: the ROI frame (pixels <-> mm) and the trigger line."""
from __future__ import annotations

from modules.vision.tracking import Track


class TriggerLine:
    """One-shot trigger-line crossing detector.

    A board fires exactly once: when its tracked centroid transitions from the
    UPSTREAM side of the fixed line to the DOWNSTREAM side.
    """

    def __init__(self, y_px: int, direction: str = "down") -> None:
        self.y_px = int(y_px)
        # +1 means downstream is the side with LARGER py (belt moves top->bottom)
        self.sign = 1 if direction == "down" else -1

    def _side(self, cy: float) -> int:
        return 1 if (cy - self.y_px) * self.sign > 0 else -1

    def crossed(self, track: Track) -> bool:
        side = self._side(track.cy)
        crossed = (not track.triggered) and track.prev_side == -1 and side == 1
        track.prev_side = side
        if crossed:
            track.triggered = True
        return crossed


class RoiFrame:
    """ROI polygon + precomputed coordinate basis.

    Origin O = polygon[3] (bottom-left); X axis = O→polygon[2] (bottom-right);
    Y axis = O→polygon[0] (top-left). `to_mm` projects a pixel onto that frame
    and scales by pixels_per_mm. The basis is computed once (the old code
    recomputed it for every detection).
    """

    def __init__(self, polygon, pixels_per_mm: float, np_mod, cv2_mod) -> None:
        self._np = np_mod
        self._cv2 = cv2_mod
        self.pixels_per_mm = float(pixels_per_mm)
        self.poly = np_mod.array(polygon, dtype=np_mod.int32) if polygon else None
        self._ok = False
        if self.poly is not None and len(self.poly) >= 4 and self.pixels_per_mm > 0:
            O = self.poly[3].astype(float)
            Xp = self.poly[2].astype(float)
            Yp = self.poly[0].astype(float)
            xlen = float(np_mod.linalg.norm(Xp - O))
            ylen = float(np_mod.linalg.norm(Yp - O))
            if xlen > 0 and ylen > 0:
                self._O = O
                self._Xu = (Xp - O) / xlen
                self._Yu = (Yp - O) / ylen
                self._ok = True

    def contains(self, cx: float, cy: float) -> bool:
        if self.poly is None:
            return True
        return self._cv2.pointPolygonTest(self.poly, (float(cx), float(cy)), False) >= 0

    def to_mm(self, cx: float, cy: float) -> tuple[float, float]:
        """Project (cx, cy) px onto the ROI frame and return (x_mm, y_mm)."""
        if not self._ok:
            return cx / self.pixels_per_mm, cy / self.pixels_per_mm
        v = self._np.array([cx, cy], dtype=float) - self._O
        x_px = float(self._np.dot(v, self._Xu))
        y_px = float(self._np.dot(v, self._Yu))
        return x_px / self.pixels_per_mm, y_px / self.pixels_per_mm

    def mm_to_pixel(self, x_mm: float, y_mm: float) -> tuple[float, float]:
        """Inverse of to_mm: project (x_mm, y_mm) in the ROI frame back to pixels."""
        if not self._ok:
            return x_mm * self.pixels_per_mm, y_mm * self.pixels_per_mm
        px = self._O + x_mm * self.pixels_per_mm * self._Xu + y_mm * self.pixels_per_mm * self._Yu
        return float(px[0]), float(px[1])
