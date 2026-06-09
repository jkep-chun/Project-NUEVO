import numpy as np

def generate_open_rounded_path(vertices, R, ds):
    """
    Generates a dense, open path of waypoints from a start point to an end point 
    with rounded intermediate corners.
    
    :param vertices: List of (x, y) tuples/arrays marking the path skeleton in order.
    :param R: Desired turning radius for intermediate corners.
    :param ds: Desired linear spacing between generated waypoints.
    """
    vertices = np.atleast_2d(vertices)
    num_verts = len(vertices)
    rounded_path = []
    
    if num_verts < 3:
        return vertices

    # Start the path exactly at the first vertex
    rounded_path.append(vertices[0])

    # Only loop through intermediate vertices (index 1 to num_verts - 2)
    # The first and last vertices are treated strictly as start/end endpoints.
    for i in range(1, num_verts - 1):
        V = vertices[i]
        A = vertices[i - 1] # Previous vertex
        B = vertices[i + 1] # Next vertex

        # 1. Vectors and Normalization
        vec_u = A - V
        vec_v = B - V
        len_u = np.linalg.norm(vec_u)
        len_v = np.linalg.norm(vec_v)
        
        u_hat = vec_u / len_u
        v_hat = vec_v / len_v

        # 2. Angle and Tangent Distance Calculations
        dot_prod = np.clip(np.dot(u_hat, v_hat), -1.0, 1.0)
        theta = np.arccos(dot_prod)

        # Safety Check: Handle collinear or near-collinear points
        if theta > np.pi - 1e-4:
            # Points are essentially on a straight line, skip rounding
            continue

        d = R / np.tan(theta / 2)

        # Safety Check: Ensure the radius fits within the straight edge constraints
        # Use min() to ensure we don't overshoot the shorter segment.
        d_limit = min(len_u, len_v) / 2
        if d > d_limit:
            d = d_limit
            # We must update the effective radius to match the capped distance d
            # to maintain geometric continuity (the arc must be tangent at T1 and T2).
            R_eff = d * np.tan(theta / 2)
        else:
            R_eff = R

        # 3. Calculate Tangent Points and Center
        T1 = V + d * u_hat
        T2 = V + d * v_hat

        bisector = u_hat + v_hat
        bisector_norm = np.linalg.norm(bisector)
        if bisector_norm < 1e-6:
            # Fallback for collinear cases that might have slipped through
            continue

        bisector_hat = bisector / bisector_norm
        center = V + (R_eff / np.sin(theta / 2)) * bisector_hat

        # 4. Straightaway Segment (From last exit to the entry point T1 of this turn)
        last_exit = rounded_path[-1]
        dist_to_T1 = np.linalg.norm(T1 - last_exit)
        num_line_pts = max(1, int(dist_to_T1 / ds))
        for t in np.linspace(0, 1, num_line_pts, endpoint=False):
            rounded_path.append(last_exit + t * (T1 - last_exit))

        # 5. Circular Arc Segment (From T1 to T2 around the calculated center)
        angle_start = np.arctan2(T1[1] - center[1], T1[0] - center[0])
        angle_end = np.arctan2(T2[1] - center[1], T2[0] - center[0])

        # Shortest angular path correction
        angle_diff = angle_end - angle_start
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        elif angle_diff < -np.pi:
            angle_diff += 2 * np.pi

        arc_length = R_eff * abs(angle_diff)
        num_arc_pts = max(3, int(arc_length / ds))

        for t in np.linspace(0, 1, num_arc_pts):
            current_angle = angle_start + t * angle_diff
            arc_point = center + R_eff * np.array([np.cos(current_angle), np.sin(current_angle)])

            # Prevent duplicating identical contiguous coordinates
            if np.linalg.norm(rounded_path[-1] - arc_point) > 1e-5:
                rounded_path.append(arc_point)

    # 6. Final Straightaway (From the last turn's exit T2 to the final absolute endpoint)
    final_endpoint = vertices[-1]
    last_exit = rounded_path[-1]
    dist_to_end = np.linalg.norm(final_endpoint - last_exit)
    num_line_pts = max(1, int(dist_to_end / ds))
    for t in np.linspace(0, 1, num_line_pts):
        rounded_path.append(last_exit + t * (final_endpoint - last_exit))

    return np.array(rounded_path)