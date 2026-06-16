"""
Navigation algorithms: angle helpers, A* path planning, Unscented Kalman Filter,
and a waypoint-following controller.

Ported from the AllGoHome simulator (github.com/Nepike/allgohome). Units are SI for
the real system: positions in metres, angles in radians, velocities in m/s and
rad/s. The grid is in cells; one cell is `cfg.cell_size_m` metres.
"""

import math
from typing import List, Optional, Set, Tuple

import numpy as np


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def world_to_cell(x: float, y: float, cfg) -> Tuple[int, int]:
    """Convert world coordinates (metres) to a grid cell, clamped to the grid."""
    cx = int(x // cfg.cell_size_m)
    cy = int(y // cfg.cell_size_m)
    cx = min(max(cx, 0), cfg.grid_w - 1)
    cy = min(max(cy, 0), cfg.grid_h - 1)
    return (cx, cy)


def a_star(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    blocked: Set[Tuple[int, int]],
    grid_w: int,
    grid_h: int,
) -> Optional[List[Tuple[int, int]]]:
    """
    Shortest path on a 4-connected grid from `start` to `goal`.

    Heuristic: Manhattan distance (admissible for 4-connectivity -> optimal path).
    `blocked` already includes the inflation buffer around obstacles.
    Returns the list of cells start..goal, or None if no path exists.
    """
    if start in blocked or goal in blocked:
        return None

    def h(c: Tuple[int, int]) -> float:
        return abs(c[0] - goal[0]) + abs(c[1] - goal[1])

    open_set = {start}
    came_from: dict = {}
    g = {start: 0.0}
    f = {start: h(start)}

    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while open_set:
        current = min(open_set, key=lambda c: f.get(c, float("inf")))
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        open_set.remove(current)
        for dx, dy in neighbors:
            nb = (current[0] + dx, current[1] + dy)
            if not (0 <= nb[0] < grid_w and 0 <= nb[1] < grid_h):
                continue
            if nb in blocked:
                continue
            tentative_g = g[current] + 1.0
            if tentative_g < g.get(nb, float("inf")):
                came_from[nb] = current
                g[nb] = tentative_g
                f[nb] = tentative_g + h(nb)
                open_set.add(nb)
    return None


class UKF:
    """
    Unscented Kalman Filter for the unicycle state [x, y, theta].

    Approximates the distribution with 2n+1 = 7 sigma points and propagates them
    through the nonlinear motion model without computing a Jacobian. Angles are
    averaged via atan2(sum sin, sum cos) so wrap-around is handled correctly.

    Measurement model is identity (the camera/ArUco gives x, y, theta directly).
    Call update(None) to skip the correction step on a dropped measurement.
    """

    def __init__(self, x0, P0, Q, R, alpha: float = 0.5, beta: float = 2.0, kappa: float = 0.0):
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa

    def _sigma_points(self, x, P):
        n = len(x)
        lam = self.alpha ** 2 * (n + self.kappa) - n
        c = n + lam
        try:
            S = np.linalg.cholesky(c * P)
        except np.linalg.LinAlgError:
            S = np.linalg.cholesky(c * (P + 1e-6 * np.eye(n)))

        pts = [x]
        for i in range(n):
            pts.append(x + S[:, i])
            pts.append(x - S[:, i])

        Wm = np.full(2 * n + 1, 1.0 / (2.0 * c))
        Wc = np.full(2 * n + 1, 1.0 / (2.0 * c))
        Wm[0] = lam / c
        Wc[0] = lam / c + (1 - self.alpha ** 2 + self.beta)
        return np.array(pts), Wm, Wc

    @staticmethod
    def _mean_state(pts, Wm):
        x = np.sum(pts[:, 0] * Wm)
        y = np.sum(pts[:, 1] * Wm)
        sin_sum = np.sum(np.sin(pts[:, 2]) * Wm)
        cos_sum = np.sum(np.cos(pts[:, 2]) * Wm)
        th = math.atan2(sin_sum, cos_sum)
        return np.array([x, y, th], dtype=float)

    def predict(self, u, dt):
        v, omega = u
        pts, Wm, Wc = self._sigma_points(self.x, self.P)
        pred_pts = []
        for p in pts:
            x, y, th = p
            pred_pts.append([
                x + v * math.cos(th) * dt,
                y + v * math.sin(th) * dt,
                normalize_angle(th + omega * dt),
            ])
        pred_pts = np.array(pred_pts, dtype=float)
        x_pred = self._mean_state(pred_pts, Wm)
        P_pred = self.Q.copy()
        for i in range(pred_pts.shape[0]):
            d = pred_pts[i] - x_pred
            d[2] = normalize_angle(d[2])
            P_pred += Wc[i] * np.outer(d, d)
        self.x = x_pred
        self.P = P_pred

    def update(self, z, R=None):
        """
        Correct the state with a measurement z = (x, y, theta).

        `R` overrides the default measurement-noise covariance for this update — used
        for fusion: call update once per camera measurement, each with that camera's
        R. Consecutive updates with no predict between them combine optimally (this
        is equivalent to a single stacked measurement for independent noise).
        """
        if z is None:
            return
        z = np.array(z, dtype=float)
        R = self.R if R is None else np.asarray(R, dtype=float)
        pts, Wm, Wc = self._sigma_points(self.x, self.P)
        Z = pts.copy()  # identity measurement model
        z_mean = self._mean_state(Z, Wm)

        Pzz = R.copy()
        Pxz = np.zeros((3, 3), dtype=float)
        for i in range(Z.shape[0]):
            dz = Z[i] - z_mean
            dz[2] = normalize_angle(dz[2])
            dx = pts[i] - self.x
            dx[2] = normalize_angle(dx[2])
            Pzz += Wc[i] * np.outer(dz, dz)
            Pxz += Wc[i] * np.outer(dx, dz)

        y = z - z_mean
        y[2] = normalize_angle(y[2])
        K = Pxz @ np.linalg.inv(Pzz)
        self.x = self.x + K @ y
        self.x[2] = normalize_angle(self.x[2])
        self.P = self.P - K @ Pzz @ K.T


class WaypointController:
    """
    Follows an A* path waypoint by waypoint, producing (v, omega) for the unicycle.

    Strategy each step:
      1. advance to the next waypoint once close enough to the current one;
      2. aim at the waypoint centre: theta_target = atan2(dy, dx);
      3. omega = k * (theta_target - theta), clamped to max_angular_speed;
      4. full linear speed when roughly aligned, smoothly reduced (and zero when the
         heading error is large) -> "turn first, then drive".
    """

    def __init__(self, path: List[Tuple[int, int]], cfg):
        self.path = path
        self.cfg = cfg
        self.i = 0

    def _target(self, idx: int) -> Tuple[float, float]:
        cx, cy = self.path[idx]
        return (cx + 0.5) * self.cfg.cell_size_m, (cy + 0.5) * self.cfg.cell_size_m

    def compute(self, x: float, y: float, theta: float) -> Tuple[float, float]:
        cfg = self.cfg
        if not self.path:
            return 0.0, 0.0

        wp_thresh = cfg.waypoint_threshold * cfg.cell_size_m
        while self.i < len(self.path):
            tx, ty = self._target(self.i)
            if math.hypot(tx - x, ty - y) < wp_thresh and self.i < len(self.path) - 1:
                self.i += 1
                continue
            break

        if self.i >= len(self.path):
            return 0.0, 0.0

        tx, ty = self._target(self.i)
        desired = math.atan2(ty - y, tx - x)
        err = normalize_angle(desired - theta)

        omega = float(np.clip(cfg.angular_speed_k * err, -cfg.max_angular_speed, cfg.max_angular_speed))

        if abs(err) < cfg.angle_threshold:
            v = cfg.max_linear_speed
        else:
            v = cfg.max_linear_speed * max(0.0, math.cos(err))
            if abs(err) > 1.0:
                v = 0.0
        return v, omega

    def reached(self, x: float, y: float) -> bool:
        cx, cy = self.path[-1]
        gx, gy = (cx + 0.5) * self.cfg.cell_size_m, (cy + 0.5) * self.cfg.cell_size_m
        return math.hypot(gx - x, gy - y) < self.cfg.waypoint_threshold * 0.5 * self.cfg.cell_size_m
