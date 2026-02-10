# mapping.py
import numpy as np
from ground_plane import fit_plane_ransac, signed_height, flip_plane

class Mapper:
    """
    Depth -> point cloud -> ground plane (RANSAC) -> obstacle endpoints -> occupancy grid.

    Deprojection coords:
      X: right
      Y: down
      Z: forward

    We output endpoints for the grid as:
      x_forward = Z
      y_left    = -X
    """

    def __init__(
        self,
        grid,
        point_stride=4,
        z_min=0.15,
        z_max=6.0,
        use_roi=True,
        roi_top=0.30,
        roi_side=0.15,

        # plane fitting
        plane_fit_rows_frac=0.40,   # bottom fraction for plane candidates
        plane_sample_max=8000,
        plane_ransac_iters=200,
        plane_inlier_tol=0.03,
        plane_refit_every=3,

        # obstacle selection
        ground_band_m=0.03,         # points within ± this of plane are "ground"
        min_obstacle_height_m=0.12, # must be above ground by this much
        max_obstacle_height_m=1.50, # cap to avoid ceiling / far junk
        obstacle_min_cluster_size=2,   # min points per cell to keep (drops noise)
        obstacle_downsample_res_m=0.05,  # bucket resolution; one rep per cell
        crater_height_below_m=0.05,    # h < -this => crater
        max_crater_depth_m=0.35,       # ignore h < -this
        filter_walls=False,            # if True, drop wall-like clusters (merged footprint or dense)
        wall_footprint_max_m=0.70,     # merged group larger than this (x or y extent) = wall
        wall_merge_dist_m=0.12,        # merge clusters within this distance before checking footprint
        wall_max_points_per_cluster=50, # single cluster with more points = dense wall, excluded
    ):
        self.grid = grid
        self.point_stride = int(max(1, point_stride))
        self.z_min = float(z_min)
        self.z_max = float(z_max)

        self.use_roi = bool(use_roi)
        self.roi_top = float(roi_top)
        self.roi_side = float(roi_side)

        self.plane_fit_rows_frac = float(plane_fit_rows_frac)
        self.plane_sample_max = int(max(500, plane_sample_max))
        self.plane_ransac_iters = int(max(50, plane_ransac_iters))
        self.plane_inlier_tol = float(plane_inlier_tol)
        self.plane_refit_every = int(max(1, plane_refit_every))

        self.ground_band_m = float(ground_band_m)
        self.min_obstacle_height_m = float(min_obstacle_height_m)
        self.max_obstacle_height_m = float(max_obstacle_height_m)
        self.obstacle_min_cluster_size = int(max(1, obstacle_min_cluster_size))
        self.obstacle_downsample_res_m = float(max(1e-3, obstacle_downsample_res_m))
        self.crater_height_below_m = float(crater_height_below_m)
        self.max_crater_depth_m = float(max_crater_depth_m)
        self.filter_walls = bool(filter_walls)
        self.wall_footprint_max_m = float(wall_footprint_max_m)
        self.wall_merge_dist_m = float(wall_merge_dist_m)
        self.wall_max_points_per_cluster = int(max(2, wall_max_points_per_cluster))

        self._frame_i = 0
        self.plane = None
        self.plane_inliers = 0
        self.plane_flipped = False

    def _roi_slices(self, H, W):
        if not self.use_roi:
            return slice(0, H, self.point_stride), slice(0, W, self.point_stride)
        y0 = int(self.roi_top * H)
        y1 = int(0.98 * H)
        x0 = int(self.roi_side * W)
        x1 = int((1.0 - self.roi_side) * W)
        return slice(y0, y1, self.point_stride), slice(x0, x1, self.point_stride)

    def _cluster_and_downsample(self, xy, uv=None, pad_m=None, pad_px=2, max_points_per_cluster=None):
        """
        Bucket (x_forward, y_left) into 2D grid. Drop cells with < min_cluster_size
        points (noise). If max_points_per_cluster is set, drop cells with more points (dense wall).
        Emit centroid per kept cell, AABB, and optionally UV AABB for depth-viz boxes.
        """
        r = self.obstacle_downsample_res_m
        m = self.obstacle_min_cluster_size
        pad = float(pad_m if pad_m is not None else 0.5 * r)
        buckets = {}
        for i in range(xy.shape[0]):
            ix = int(np.floor(xy[i, 0] / r))
            iy = int(np.floor(xy[i, 1] / r))
            k = (ix, iy)
            if k not in buckets:
                buckets[k] = []
            row = (float(xy[i, 0]), float(xy[i, 1]))
            if uv is not None:
                row = row + (float(uv[i, 0]), float(uv[i, 1]))
            buckets[k].append(row)
        centroids = []
        bboxes = []
        bboxes_uv = [] if uv is not None else None
        for pts in buckets.values():
            if len(pts) < m:
                continue
            if max_points_per_cluster is not None and len(pts) > max_points_per_cluster:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            cx_ = sum(xs) / len(xs)
            cy_ = sum(ys) / len(ys)
            x_min = min(xs) - pad
            x_max = max(xs) + pad
            y_min = min(ys) - pad
            y_max = max(ys) + pad
            centroids.append((cx_, cy_))
            bboxes.append((x_min, y_min, x_max, y_max))
            if uv is not None:
                us = [p[2] for p in pts]
                vs = [p[3] for p in pts]
                u_min = max(0, min(us) - pad_px)
                u_max = max(us) + pad_px
                v_min = max(0, min(vs) - pad_px)
                v_max = max(vs) + pad_px
                bboxes_uv.append((u_min, v_min, u_max, v_max))
        if not centroids:
            return np.zeros((0, 2), dtype=np.float32), [], (bboxes_uv or [])
        return np.array(centroids, dtype=np.float32), bboxes, (bboxes_uv or [])

    def _drop_wall_like(self, endpoints_xy, bboxes, bboxes_uv=None):
        """
        Drop wall-like clusters: (1) merge clusters within wall_merge_dist_m;
        (2) drop any merged group whose union footprint exceeds wall_footprint_max_m.
        Used so navigation does not use walls (per guidebook).
        """
        if not self.filter_walls or endpoints_xy.shape[0] == 0:
            return endpoints_xy, bboxes, (bboxes_uv or [])
        n = len(bboxes)
        half = 0.5 * self.wall_merge_dist_m

        def boxes_overlap(i, j):
            (a0, a1, a2, a3) = bboxes[i]
            (b0, b1, b2, b3) = bboxes[j]
            a0e, a1e, a2e, a3e = a0 - half, a1 - half, a2 + half, a3 + half
            b0e, b1e, b2e, b3e = b0 - half, b1 - half, b2 + half, b3 + half
            if a2e < b0e or b2e < a0e:
                return False
            if a3e < b1e or b3e < a1e:
                return False
            return True

        # Union-find: parent[i] = representative of cluster i
        parent = list(range(n))

        def find(i):
            if parent[i] != i:
                parent[i] = find(parent[i])
            return parent[i]

        def union(i, j):
            pi, pj = find(i), find(j)
            if pi != pj:
                parent[pi] = pj

        for i in range(n):
            for j in range(i + 1, n):
                if boxes_overlap(i, j):
                    union(i, j)

        # For each group, compute union bbox; if extent > threshold, drop all in group
        drop = set()
        seen_root = set()
        for i in range(n):
            root = find(i)
            if root in seen_root:
                continue
            seen_root.add(root)
            group = [k for k in range(n) if find(k) == root]
            x_min = min(bboxes[k][0] for k in group)
            y_min = min(bboxes[k][1] for k in group)
            x_max = max(bboxes[k][2] for k in group)
            y_max = max(bboxes[k][3] for k in group)
            w, h = x_max - x_min, y_max - y_min
            if w > self.wall_footprint_max_m or h > self.wall_footprint_max_m:
                drop.update(group)

        keep = [i for i in range(n) if i not in drop]
        if not keep:
            return np.zeros((0, 2), dtype=np.float32), [], (bboxes_uv or [])[:0]
        keep = np.asarray(keep)
        out_xy = endpoints_xy[keep]
        out_boxes = [bboxes[j] for j in keep]
        out_uv = [bboxes_uv[j] for j in keep] if bboxes_uv else []
        return out_xy, out_boxes, out_uv

    def update_from_depth(self, depth_m, fx, fy, cx, cy):
        self._frame_i += 1
        H, W = depth_m.shape
        ys, xs = self._roi_slices(H, W)

        d = depth_m[ys, xs]
        if d.size == 0:
            return np.zeros((0, 2), np.float32), np.zeros((0, 2), np.float32), {"n": 0, "n_craters": 0, "plane": False, "bboxes_boulders": [], "bboxes_craters": [], "bboxes_boulders_uv": [], "bboxes_craters_uv": []}

        vv, uu = np.mgrid[ys, xs]
        z = d.astype(np.float32)

        valid = (z > self.z_min) & (z < self.z_max)
        if not np.any(valid):
            return np.zeros((0, 2), np.float32), np.zeros((0, 2), np.float32), {"n": 0, "n_craters": 0, "plane": bool(self.plane), "bboxes_boulders": [], "bboxes_craters": [], "bboxes_boulders_uv": [], "bboxes_craters_uv": []}

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z  = z[valid].astype(np.float32)

        X = (uu - cx) * z / fx
        Y = (vv - cy) * z / fy
        Z = z
        P = np.stack([X, Y, Z], axis=1).astype(np.float32)

        # -----------------------------
        # 1) Fit / refresh plane
        # -----------------------------
        refit = (self.plane is None) or ((self._frame_i % self.plane_refit_every) == 1)
        if refit:
            y_cut = int((1.0 - self.plane_fit_rows_frac) * H)
            bottom = (vv >= y_cut)
            Pb = P[bottom]
            vb = vv[bottom]

            if Pb.shape[0] >= 800:
                if Pb.shape[0] > self.plane_sample_max:
                    idx = np.random.choice(Pb.shape[0], self.plane_sample_max, replace=False)
                    Pb = Pb[idx]
                    vb = vb[idx]

                seed = 0xBEEF + (self._frame_i % 10000)
                plane = fit_plane_ransac(Pb, iters=self.plane_ransac_iters, tol=self.plane_inlier_tol, seed=seed)
                if plane is not None:
                    # inlier health
                    dist = np.abs(signed_height(Pb, plane))
                    self.plane_inliers = int(np.sum(dist < self.plane_inlier_tol))

                    # AUTO ORIENT:
                    # positive heights should generally appear higher in image (smaller v)
                    h_all = signed_height(P, plane)
                    use = np.abs(h_all) > self.ground_band_m
                    hp = h_all[use]
                    vp = vv[use]
                    flipped = False
                    if hp.size > 200:
                        v_pos = np.median(vp[hp > 0]) if np.any(hp > 0) else None
                        v_neg = np.median(vp[hp < 0]) if np.any(hp < 0) else None
                        if (v_pos is not None) and (v_neg is not None):
                            # If positive heights are LOWER in image (bigger v), flip plane
                            if v_pos > v_neg:
                                plane = flip_plane(plane)
                                flipped = True

                    self.plane = plane
                    self.plane_flipped = flipped

        if self.plane is None:
            return np.zeros((0, 2), np.float32), np.zeros((0, 2), np.float32), {"n": 0, "n_craters": 0, "plane": False, "bboxes_boulders": [], "bboxes_craters": []}

        # -----------------------------
        # 2) Ground / obstacle selection (boulders + craters)
        # -----------------------------
        h = signed_height(P, self.plane)
        not_ground = np.abs(h) > self.ground_band_m

        # Boulders: above ground
        obstacles = (h > self.min_obstacle_height_m) & (h < self.max_obstacle_height_m) & not_ground
        Po = P[obstacles]
        uu_o = uu[obstacles]
        vv_o = vv[obstacles]
        if Po.shape[0] > 0:
            xy_b = np.stack([Po[:, 2], -Po[:, 0]], axis=1).astype(np.float32)
            uv_b = np.stack([uu_o, vv_o], axis=1).astype(np.float32)
            max_pts = self.wall_max_points_per_cluster if self.filter_walls else None
            endpoints_xy, bboxes_boulders, bboxes_boulders_uv = self._cluster_and_downsample(
                xy_b, uv=uv_b, max_points_per_cluster=max_pts
            )
            # Exclude wall-like clusters so we do not use walls for navigation (guidebook)
            endpoints_xy, bboxes_boulders, bboxes_boulders_uv = self._drop_wall_like(
                endpoints_xy, bboxes_boulders, bboxes_boulders_uv
            )
        else:
            endpoints_xy = np.zeros((0, 2), dtype=np.float32)
            bboxes_boulders = []
            bboxes_boulders_uv = []

        # Craters: below ground (negative obstacles); project footprint to 2D
        craters = (h < -self.crater_height_below_m) & (h > -self.max_crater_depth_m) & not_ground
        Pc = P[craters]
        uu_c = uu[craters]
        vv_c = vv[craters]
        if Pc.shape[0] > 0:
            xy_c = np.stack([Pc[:, 2], -Pc[:, 0]], axis=1).astype(np.float32)
            uv_c = np.stack([uu_c, vv_c], axis=1).astype(np.float32)
            crater_xy, bboxes_craters, bboxes_craters_uv = self._cluster_and_downsample(xy_c, uv=uv_c)
        else:
            crater_xy = np.zeros((0, 2), dtype=np.float32)
            bboxes_craters = []
            bboxes_craters_uv = []

        n_b = int(endpoints_xy.shape[0])
        n_c = int(crater_xy.shape[0])

        return endpoints_xy, crater_xy, {
            "n": n_b,
            "n_craters": n_c,
            "plane": True,
            "inliers": int(self.plane_inliers),
            "flip": bool(self.plane_flipped),
            "bboxes_boulders": bboxes_boulders,
            "bboxes_craters": bboxes_craters,
            "bboxes_boulders_uv": bboxes_boulders_uv,
            "bboxes_craters_uv": bboxes_craters_uv,
        }

    def integrate(self, endpoints_xy, crater_xy=None, sensor_origin_xy=(0.0, 0.0), inflate_radius_m=0.30, inflate_occ_th=0.65):
        """Update grid: boulder rays + crater footprints. Inflation at read-time."""
        self.grid.update_rays(endpoints_xy, sensor_origin_xy=sensor_origin_xy)
        if crater_xy is not None and crater_xy.shape[0] > 0:
            self.grid.mark_occupied_cells(crater_xy)