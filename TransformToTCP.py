# Description:
#   Reports the transform from the top-level component's origin to the
#   component named "TCP"'s origin: translation, quaternion, and Euler ZYX. 
#   Also prints the ABB definition format for the pose.
#
# Usage:
#   - Open a design that contains a component named "TCP" (case-insensitive ok).
#   - Run as a Script.

import adsk.core, adsk.fusion, math, traceback

def run(context):
    app = adsk.core.Application.get()
    ui  = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            _msg(ui, "Open a Fusion design first.")
            return

        root = design.rootComponent

        # Find all occurrences whose component name is "TCP" (exact or case-insensitive)
        exact, fallback = _find_tcp_occurrences(root)
        tcp_occs = exact if exact else fallback

        if not tcp_occs:
            _msg(ui, "No occurrence found for a component named 'TCP'.")
            return

        if len(tcp_occs) > 1:
            ui.messageBox("Multiple 'TCP' occurrences found; using the first:\n\n" +
                          "\n".join(_occ_path(o) for o in tcp_occs))

        tcp_occ = tcp_occs[0]

        # Transform from top-level (world) -> TCP component origin
        T = tcp_occ.transform.copy()

        # Correct Python call: returns (origin, xAxis, yAxis, zAxis)
        origin, x_axis, y_axis, z_axis = T.getAsCoordinateSystem()

        # Orientation
        qx, qy, qz, qw = _axes_to_quaternion(x_axis, y_axis, z_axis)
        rx, ry, rz = _axes_to_euler_zyx(x_axis, y_axis, z_axis)
        
        # Convert radians → degrees
        rx_deg = math.degrees(rx)
        ry_deg = math.degrees(ry)
        rz_deg = math.degrees(rz)
    
        # Convert cm → mm
        tx_mm = origin.x * 10
        ty_mm = origin.y * 10
        tz_mm = origin.z * 10
        
        # ABB definition format
        abb_pose = f"[[{tx_mm:.3f}, {ty_mm:.3f}, {tz_mm:.3f}], [{qw:.9f}, {qx:.9f}, {qy:.9f}, {qz:.9f}]]"


        # Popup display
        ui.messageBox(
            "Top-level → TCP transform\n"
            f"TCP position (mm): [{tx_mm:.3f}, {ty_mm:.3f}, {tz_mm:.3f}]\n"
            f"Quaternion (x,y,z,w): [{qx:.9f}, {qy:.9f}, {qz:.9f}, {qw:.9f}]\n"
            f"Euler ZYX (deg): X={rx_deg:.3f}, Y={ry_deg:.3f}, Z={rz_deg:.3f}\n"
            f"ABB pose: {abb_pose}\n\n"
            f"(Occurrence used: {_occ_path(tcp_occ)})"
        )

        # Print to Text Command window for copy/paste
        app.log(f"ABB pose: {abb_pose}")

    except:
        if ui:
            ui.messageBox(traceback.format_exc())


# ---------- helpers ----------

def _find_tcp_occurrences(root: adsk.fusion.Component):
    """Return (exact_matches, fallback_matches) for component name 'TCP'."""
    exact, fallback = [], []
    for occ in root.allOccurrences:
        name = occ.component.name
        if name == "TCP":
            exact.append(occ)
        elif "tcp" in name.lower():
            fallback.append(occ)
    return exact, fallback

def _occ_path(occ: adsk.fusion.Occurrence) -> str:
    """Readable path like Root/Asm/SubAsm/TCP."""
    parts = []
    o = occ
    while o:
        parts.append(o.component.name)
        o = o.assemblyContext
    parts.reverse()
    return "/".join(parts)

def _axes_to_quaternion(xa, ya, za):
    # Rotation matrix from basis vectors (columns = axes)
    m00, m01, m02 = xa.x, ya.x, za.x
    m10, m11, m12 = xa.y, ya.y, za.y
    m20, m21, m22 = xa.z, ya.z, za.z
    tr = m00 + m11 + m22

    if tr > 0:
        S = (tr + 1.0) ** 0.5 * 2
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = ((1.0 + m00 - m11 - m22) ** 0.5) * 2
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = ((1.0 + m11 - m00 - m22) ** 0.5) * 2
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = ((1.0 + m22 - m00 - m11) ** 0.5) * 2
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    return qx, qy, qz, qw

def _axes_to_euler_zyx(xa, ya, za):
    """
    Right-handed, orthonormal basis.
    Returns (Rx, Ry, Rz) with intrinsic ZYX (R = Rz * Ry * Rx).
    """
    m00, m01, m02 = xa.x, ya.x, za.x
    m10, m11, m12 = xa.y, ya.y, za.y
    m20, m21, m22 = xa.z, ya.z, za.z

    m20c = max(-1.0, min(1.0, m20))  # clamp for safety

    if abs(m20c) < 1.0:
        y = -math.asin(m20c)          # pitch about Y
        x = math.atan2(m21, m22)      # roll about X
        z = math.atan2(m10, m00)      # yaw about Z
    else:
        # gimbal lock
        y = math.pi/2 if m20c <= -1.0 else -math.pi/2
        x = math.atan2(-m01, m11)
        z = 0.0
    return x, y, z

def _msg(ui, s):
    ui.messageBox(s)
