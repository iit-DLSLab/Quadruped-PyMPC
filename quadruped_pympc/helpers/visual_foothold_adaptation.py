import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
from gym_quadruped.utils.quadruped_utils import LegsAttr


class VisualFootholdAdaptation:
    def __init__(
        self,
        legs_order,
        adaptation_strategy='height',
        edge_threshold=0.03,
        safety_margin=0.0,
        max_box_half_cells=2,
        foot_radius=0.02,
    ):
        """
        Args:
            legs_order: list of leg names
            adaptation_strategy: 'height' (only z adaptation) or 'vfa' (edge-aware xyz adaptation)
            edge_threshold: max height difference [m] inside a 3x3 neighborhood for a cell to be considered flat
            safety_margin: additional clearance [m] from detected edges (dilation of the unsafe mask)
            max_box_half_cells: max half size (in cells) of the safe box returned as foothold constraint
            foot_radius: offset added to the terrain height, since the foot position is the center of the foot sphere
        """
        self.footholds_adaptation = LegsAttr(
            FL=np.array([0, 0, 0]), FR=np.array([0, 0, 0]), RL=np.array([0, 0, 0]), RR=np.array([0, 0, 0])
        )
        self.footholds_constraints = LegsAttr(FL=None, FR=None, RL=None, RR=None)
        self.initialized = False

        self.adaptation_strategy = adaptation_strategy

        self.edge_threshold = edge_threshold
        self.safety_margin = safety_margin
        self.max_box_half_cells = max_box_half_cells
        self.foot_radius = foot_radius

        # Last safe map computed for each leg (useful for debugging/visualization)
        self.safe_maps = LegsAttr(FL=None, FR=None, RL=None, RR=None)

    def update_footholds_adaptation(self, update_footholds_adaptation):
        self.footholds_adaptation = update_footholds_adaptation
        self.initialized = True

    def reset(self):
        self.initialized = False

    def get_footholds_adapted(self, reference_footholds):
        # If the adaptation is not initialized, return the reference footholds
        if self.initialized == False:
            self.footholds_adaptation = reference_footholds
            return reference_footholds, self.footholds_constraints
        else:
            return self.footholds_adaptation, self.footholds_constraints

    @staticmethod
    def _window_max_min(z, k):
        """Max and min of z over a (2k+1)x(2k+1) window, borders replicated. NaNs propagate."""
        windows = sliding_window_view(np.pad(z, k, mode='edge'), (2 * k + 1, 2 * k + 1))
        return windows.max(axis=(-1, -2)), windows.min(axis=(-1, -2))

    def compute_safe_map(self, z, resolution):
        """Edge detection on the heightmap via morphological gradient (3x3 max - min).

        A cell is safe if the terrain around it is flat (no step/hole edge within one cell),
        it is valid (no NaN nearby) and it is at least safety_margin away from any edge.

        Returns:
            safe (np.ndarray): boolean map, True where it is safe to step
            edge_magnitude (np.ndarray): local height variation, NaN where invalid
        """
        z_max, z_min = self._window_max_min(z, 1)
        edge_magnitude = z_max - z_min
        unsafe = ~(edge_magnitude <= self.edge_threshold)  # also catches NaN

        margin_cells = int(np.ceil(self.safety_margin / resolution - 1e-9))
        if margin_cells > 0:
            unsafe, _ = self._window_max_min(unsafe, margin_cells)

        return ~unsafe, edge_magnitude

    def compute_safe_box(self, safe, r, c):
        """Greedily grow the largest safe rectangle around cell (r, c), up to max_box_half_cells per side.

        Returns:
            (r0, c0, r1, c1): inclusive corner indices of the box
        """
        n_rows, n_cols = safe.shape
        r0, r1, c0, c1 = r, r, c, c
        grow = [True, True, True, True]
        for _ in range(self.max_box_half_cells):
            if grow[0] and r0 > 0 and safe[r0 - 1, c0 : c1 + 1].all():
                r0 -= 1
            else:
                grow[0] = False
            if grow[1] and r1 < n_rows - 1 and safe[r1 + 1, c0 : c1 + 1].all():
                r1 += 1
            else:
                grow[1] = False
            if grow[2] and c0 > 0 and safe[r0 : r1 + 1, c0 - 1].all():
                c0 -= 1
            else:
                grow[2] = False
            if grow[3] and c1 < n_cols - 1 and safe[r0 : r1 + 1, c1 + 1].all():
                c1 += 1
            else:
                grow[3] = False
            if not any(grow):
                break
        return r0, c0, r1, c1

    def compute_safe_foothold(self, heightmap_data, reference_foothold, resolution):
        """Select the safe cell of the heightmap closest to the nominal foothold.

        Args:
            heightmap_data (np.ndarray): (rows, cols, 1, 3) world frame points of the heightmap
            reference_foothold (np.ndarray): nominal foothold in world frame
            resolution (float): cell size [m]

        Returns:
            foothold (np.ndarray): safe foothold xyz in world frame
            constraint (list): two opposite vertices (world frame) of the safe box around the foothold
            safe (np.ndarray): boolean safe map
        """
        points = heightmap_data[:, :, 0, :]
        z = points[:, :, 2]

        safe, edge_magnitude = self.compute_safe_map(z, resolution)

        dist2 = np.sum((points[:, :, 0:2] - reference_foothold[0:2]) ** 2, axis=-1)
        if safe.any():
            cost = np.where(safe, dist2, np.inf)
        else:
            # No safe cell at all: pick the flattest valid cell, tie-break on the distance
            cost = np.where(np.isfinite(edge_magnitude), edge_magnitude + 1e-3 * dist2, np.inf)
            if not np.isfinite(cost).any():
                return None, None, safe

        r, c = np.unravel_index(np.argmin(cost), cost.shape)

        foothold = points[r, c].copy()
        foothold[2] += self.foot_radius

        r0, c0, r1, c1 = self.compute_safe_box(safe, r, c)
        constraint = [points[r0, c0].copy(), points[r1, c1].copy()]

        return foothold, constraint, safe

    def compute_adaptation(
        self,
        legs_order,
        reference_footholds,
        hip_positions,
        heightmaps,
        forward_vel,
        base_orientation,
        base_orientation_rate,
    ):
        for leg_id, leg_name in enumerate(legs_order):
            if heightmaps[leg_name].data is None:
                return False

        if self.adaptation_strategy == 'height':
            for leg_id, leg_name in enumerate(legs_order):
                height_adjustment = heightmaps[leg_name].get_height(reference_footholds[leg_name])
                if height_adjustment is not None:
                    reference_footholds[leg_name][2] = height_adjustment

        elif self.adaptation_strategy == 'vfa':
            for leg_id, leg_name in enumerate(legs_order):
                foothold, constraint, safe = self.compute_safe_foothold(
                    heightmaps[leg_name].data, reference_footholds[leg_name], heightmaps[leg_name].dist_x
                )
                self.safe_maps[leg_name] = safe
                if foothold is None:
                    continue

                reference_footholds[leg_name] = foothold
                self.footholds_constraints[leg_name] = constraint

        self.update_footholds_adaptation(reference_footholds)

        return True
