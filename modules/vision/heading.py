"""Board orientation from YOLO-OBB boxes and their orientation markers (basis-theory §5)."""
from __future__ import annotations

import math

# An OBB detection tuple: (cx, cy, w, h, theta_rad, cls_id, conf)
Obb = tuple[float, float, float, float, float, int, float]


def normalize_angle_deg(theta_rad: float) -> float:
    """Map an OBB angle (radians) to degrees in [-90, 90).

    A rectangular board is symmetric under 180° rotation, so we fold the angle
    into a half-open 180° window centred on 0 — the smallest rotation a gripper
    must apply.
    """
    deg = math.degrees(theta_rad)
    return (deg + 90.0) % 180.0 - 90.0


def extract_obb(result, np_mod) -> list[Obb]:
    """Return (cx, cy, w, h, theta_rad, cls_id, conf) for each OBB detection."""
    obb = getattr(result, "obb", None)
    if obb is None or obb.xywhr is None:
        return []
    xywhr = obb.xywhr.cpu().numpy()       # (N, 5): cx, cy, w, h, theta
    cls = obb.cls.cpu().numpy().astype(int)
    conf = obb.conf.cpu().numpy()
    out: list[Obb] = []
    for i in range(len(xywhr)):
        cx, cy, w, h, theta = xywhr[i]
        out.append((float(cx), float(cy), float(w), float(h), float(theta), int(cls[i]), float(conf[i])))
    return out


def _obb_corners(board: Obb, cv2_mod, np_mod):
    cx, cy, w, h, theta = board[0], board[1], board[2], board[3], board[4]
    return cv2_mod.boxPoints(((cx, cy), (w, h), math.degrees(theta))).astype(np_mod.float32)


def pick_marker(board: Obb, markers: list[Obb], names: dict, marker_map: dict,
                cv2_mod, np_mod, max_dist_px: float | None = None):
    """Pick the marker belonging to `board` and the PCB type it implies.

    A marker whose centre lies INSIDE the board OBB is a definitive association,
    so the closest such marker is returned unconditionally (no distance gate) —
    this is what makes a larger board like TQFP work, whose own marker sits ~22 mm
    from the centre, beyond any fixed `max_dist_px`. Only when no marker lies
    inside the OBB do we fall back to the globally closest marker, gated by
    `max_dist_px` (prevents cross-board association when boards sit close together).
    Returns (marker_tuple_or_None, inferred_type_or_None).
    """
    if not markers:
        return None, None
    bx, by = board[0], board[1]
    corners = _obb_corners(board, cv2_mod, np_mod)
    inside = [m for m in markers
              if cv2_mod.pointPolygonTest(corners, (float(m[0]), float(m[1])), False) >= 0]
    if inside:
        m = min(inside, key=lambda mk: (mk[0] - bx) ** 2 + (mk[1] - by) ** 2)
        return m, marker_map.get(names[m[5]])

    # Fallback: no marker inside the board — take the globally closest, but only
    # if it is near enough to plausibly belong to this board.
    m = min(markers, key=lambda mk: (mk[0] - bx) ** 2 + (mk[1] - by) ** 2)
    if max_dist_px is not None and math.hypot(m[0] - bx, m[1] - by) > max_dist_px:
        return None, None
    return m, marker_map.get(names[m[5]])


def heading_from_marker_vector(board: Obb, marker, offset_deg: float = 0.0) -> float | None:
    """Heading in [0, 360) — angle of the board→marker vector vs downward (0,1),
    clockwise (0°=marker below, 90°=right, 180°=above, 270°=left). None if no marker."""
    if marker is None:
        return None
    dx = marker[0] - board[0]
    dy = marker[1] - board[1]
    return (math.degrees(math.atan2(dx, dy)) + offset_deg) % 360.0


def resolve_heading_360(board: Obb, marker, offset_deg: float, symmetry_deg: float) -> float:
    """Board heading in [0, 360) using the marker to break shape symmetry.
    Falls back to the OBB angle folded into [0, symmetry_deg) when no marker."""
    theta = math.degrees(board[4])
    if marker is None:
        return (theta % symmetry_deg + offset_deg) % 360.0
    to_marker = math.degrees(math.atan2(marker[1] - board[1], marker[0] - board[0]))
    n = max(1, round(360.0 / symmetry_deg))
    best, best_d = theta, 1e9
    for k in range(n):
        cand = theta + k * symmetry_deg
        d = abs(((cand - to_marker + 180.0) % 360.0) - 180.0)
        if d < best_d:
            best_d, best = d, cand
    return (best + offset_deg) % 360.0
