# ground_plane.py
import numpy as np

def fit_plane_ransac(P, iters=200, tol=0.03, seed=0xBEEF):
    """
    Fit plane ax+by+cz+d=0 to 3D points P (N,3) using RANSAC.
    Returns (a,b,c,d) with ||n||=1 (normal direction may be flipped later).
    """
    if P is None or P.shape[0] < 3:
        return None

    rng = np.random.default_rng(seed)
    best_in = -1
    best_model = None
    N = P.shape[0]

    for _ in range(iters):
        idx = rng.choice(N, size=3, replace=False)
        p1, p2, p3 = P[idx[0]], P[idx[1]], P[idx[2]]

        n = np.cross(p2 - p1, p3 - p1)
        norm = float(np.linalg.norm(n))
        if norm < 1e-8:
            continue
        n = n / norm
        d = -float(np.dot(n, p1))

        dists = np.abs(P @ n + d)
        inliers = int(np.sum(dists < tol))
        if inliers > best_in:
            best_in = inliers
            best_model = (float(n[0]), float(n[1]), float(n[2]), float(d))

    if best_model is None:
        return None

    # Refine using SVD on inliers
    n = np.array(best_model[:3], dtype=np.float32)
    d = float(best_model[3])

    dists = np.abs(P @ n + d)
    mask = dists < tol
    Q = P[mask]
    if Q.shape[0] >= 3:
        mu = Q.mean(axis=0)
        _, _, Vt = np.linalg.svd(Q - mu)
        n = Vt[-1].astype(np.float32)
        n = n / (np.linalg.norm(n) + 1e-9)
        d = -float(np.dot(n, mu))

    return (float(n[0]), float(n[1]), float(n[2]), float(d))

def signed_height(points_xyz, plane_abcd):
    a, b, c, d = plane_abcd
    n = np.array([a, b, c], dtype=np.float32)
    return points_xyz @ n + float(d)

def flip_plane(plane_abcd):
    a, b, c, d = plane_abcd
    return (-a, -b, -c, -d)
