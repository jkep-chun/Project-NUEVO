import numpy as np

def generate_rounded_path(vertices, R, ds=0.05):
    """
    Generates a dense path of waypoints with rounded corners.
    
    :param vertices: List of (x, y) tuples/arrays marking the shape corners.
    :param R: Desired turning radius.
    :param ds: Desired linear spacing between generated waypoints.
    """
    vertices = np.array(vertices)
    num_verts = len(vertices)
    rounded_path = []

    for i in range(num_verts):
        # Handle indexing for closed loops
        V = vertices[i]
        A = vertices[(i - 1) % num_verts]
        B = vertices[(i + 1) % num_verts]

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
        
        d = R / np.tan(theta / 2)

        # Safety Check: If radius is too big for the side lengths, cap it
        if d > len_u / 2 or d > len_v / 2:
            raise ValueError(f"Radius R={R} is too large for the side lengths near vertex {V}")

        # 3. Calculate Tangent Points and Center
        T1 = V + d * u_hat
        T2 = V + d * v_hat
        
        bisector = u_hat + v_hat
        bisector_hat = bisector / np.linalg.norm(bisector)
        center = V + (R / np.sin(theta / 2)) * bisector_hat

        # 4. Generate the Straightaway (from previous vertex's exit to current entry T1)
        if i == 0:
            # For the very first point, start at T1
            rounded_path.append(T1)
        else:
            # Linearly interpolate from the last corner's exit to this corner's entry
            last_exit = rounded_path[-1]
            dist_to_T1 = np.linalg.norm(T1 - last_exit)
            num_line_pts = max(1, int(dist_to_T1 / ds))
            for t in np.linspace(0, 1, num_line_pts, endpoint=False):
                rounded_path.append(last_exit + t * (T1 - last_exit))

        # 5. Generate the Circular Arc (from T1 to T2 around Center)
        # Find start and end angles relative to the circle center
        angle_start = np.arctan2(T1[1] - center[1], T1[0] - center[0])
        angle_end = np.arctan2(T2[1] - center[1], T2[0] - center[0])

        # Ensure we interpolate the short way around the circle
        angle_diff = angle_end - angle_start
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        elif angle_diff < -np.pi:
            angle_diff += 2 * np.pi

        # Determine number of points needed for the arc based on arc length
        arc_length = R * abs(angle_diff)
        num_arc_pts = max(3, int(arc_length / ds))

        for t in np.linspace(0, 1, num_arc_pts):
            current_angle = angle_start + t * angle_diff
            arc_point = center + R * np.array([np.cos(current_angle), np.sin(current_angle)])
            
            # Avoid duplicating the exact entry point
            if len(rounded_path) == 0 or np.linalg.norm(rounded_path[-1] - arc_point) > 1e-5:
                rounded_path.append(arc_point)

    # # Close the loop by connecting back to the very first point
    # first_pt = rounded_path[0]
    # last_exit = rounded_path[-1]
    # dist_to_start = np.linalg.norm(first_pt - last_exit)
    # num_line_pts = max(1, int(dist_to_start / ds))
    # for t in np.linspace(0, 1, num_line_pts):
    #     rounded_path.append(last_exit + t * (first_pt - last_exit))

    return np.array(rounded_path)