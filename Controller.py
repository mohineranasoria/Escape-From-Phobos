#############################################################################################
# DON'T CHANGE ANY OF THE BELOW. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################

import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)

import numpy as np
import numpy.typing as npt
import copy

this = st.GetThisSystem()

doSpawning = st.SimGlobals.GetSimEntity().GetParam(st.VarType.bool, "DoSpawning")
if doSpawning:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()
target: st.Entity = this.GetParam(st.VarType.entityRef, "Target")

topCamera: st.Entity = this.GetParam(st.VarType.entityRef, "TopCamera")
topCameraFrame = topCamera.GetBodyFixedFrame()
bottomCamera: st.Entity = this.GetParam(st.VarType.entityRef, "BottomCamera")
bottomCameraFrame = bottomCamera.GetBodyFixedFrame()


def GetNearestDebrisLocations(N: int) -> list[npt.NDArray]:
    allDebrisInfo: list[npt.NDArray] = shuttle.GetParamArray(st.VarType.doubleM3x3, 
                                                            ["Radar", "SortedDebrisData"])
    nearest = allDebrisInfo[:min(N, len(allDebrisInfo))]
    return nearest


def PanTiltCamera(camera: st.Entity, yaw_deg: float, pitch_deg: float) -> None:
    if camera == topCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, -50.0, 0.0)
    elif camera == bottomCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, 0.0, 50.0)
    camera.setRotation_DCM(st.math.Euler321_to_DCM([np.deg2rad(yaw_deg), np.deg2rad(pitch_deg), 0.0]), shuttleFrame)


def SetCommand(accelCommand: npt.NDArray, angAccelCommand: npt.NDArray) -> None:
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelCommand)
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelCommand)


#############################################################################################
# DON'T CHANGE ANY OF THE ABOVE. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################


# ==============================================================================
# USER CONTROLLER CODE: NOSE-ALIGNED PITCH SLEW, HIGH-EFFICIENCY +X BURN & COAST
# ==============================================================================

# --- 1. Mission & Tuning Constants ---
V_CRUISE = 7.85           # Calibrated cruise speed for maximum score (m/s)
MAX_ACCEL = 2.15          # LP Simplex solver saturation ceiling (m/s^2)
KEEP_OUT_CUTOFF = 49.5    # Cutoff distance (m) - victory triggers at <= 50.0m
TERMINAL_COAST_DIST = 160.0 # Distance inside which forward burn is permanently locked off

# Speed & Coasting Calibration
DEADBAND_ENTER = 0.15     # Enter ballistic coast when speed reaches 7.70 m/s
LAT_DRIFT_MAX = 2.80      # Heading tolerance during climb (m/s)

KP_VEL_LONG = 1.25        # Forward velocity proportional gain (1/s)
KP_VEL_LAT = 1.45         # Lateral evasion velocity proportional gain (1/s)

# Proven Crash-Proof Geometry with Dead-Reckoning Compensation
RADAR_QUERY_COUNT = 60    # Scan 60 nearest rocks to overcome radar sort latency
SHIP_BOUND_RADIUS = 3.0   # Shuttle bounding radius (m)
CLEARANCE_BUFFER = 2.0    # Safety standoff distance (m) -> R_safe = r_rock + 5.0m
TCA_HORIZON = 9.5         # Lookahead time horizon for collision cone (seconds)
MAX_EVADE_VEL = 5.2       # Maximum lateral dodge velocity component (m/s)
EVADE_FILTER_ALPHA = 0.45 # EMA smoothing factor for feed-forward acceleration

# Attitude Slew Profile (Pitch Body +X to align with Goal Direction)
SLEW_ANG_MAX_ACCEL = 0.045  # Maximum angular acceleration for slew (rad/s^2, limit 0.05)
SLEW_ANG_DECEL = 0.035      # Braking deceleration curve (rad/s^2)
SLEW_CRUISE_RATE = 0.18     # Slew angular velocity limit (~10.3 deg/s)
SLEW_ALIGN_TOL_DEG = 2.0    # Alignment completion threshold (degrees)
SLEW_MAX_TIME_SEC = 18.0    # Timeout before transitioning to forward burn (seconds)

# Keep cameras facing forward along body +X
PanTiltCamera(topCamera, 0.0, 0.0)
PanTiltCamera(bottomCamera, 0.0, 0.0)

LOOP_DT = 1.0 / this.GetParam(st.VarType.double, "LoopFreqHz")
t0_sim = st.SimGlobals.SimClock.GetTimeNow().as_datetime()

smooth_evade_acc = np.zeros(3)
long_coasting = False
coasting = False
is_aligned = False
ref_body_y = None
iteration = 0
exit_flag: bool = False

while not exit_flag:
    time.sleep(LOOP_DT)

    # Current simulation time for radar latency dead-reckoning
    t_now_sim = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
    sim_time_sec = (t_now_sim - t0_sim).total_seconds()

    # --- 2. Sensor & State Queries (Phobos-Fixed Inertial Frame) ---
    locVelAcc = shuttle.getLocVelAcc()
    loc = np.array(locVelAcc.loc_WRT_ExprIn(phobosFrame), dtype=float)
    vel = np.array(locVelAcc.vel_WRT_ExprIn(phobosFrame), dtype=float)
    rot_dcm = np.array(shuttle.getRotation().DCM_WRT(phobosFrame), dtype=float)
    ang_vel_body = np.array(shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame), dtype=float)

    damage: float = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
    fuelUsed: float = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])

    targetLoc = np.array(target.getLocVelAcc().loc_WRT_ExprIn(phobosFrame), dtype=float)

    # Query nearest debris objects
    debrisInfo = GetNearestDebrisLocations(RADAR_QUERY_COUNT)

    # --- 3. Goal Geometry & Orthogonal Basis Construction ---
    dist_to_target = float(np.linalg.norm(targetLoc - loc))

    r_to_goal = targetLoc - loc
    dist_to_goal = float(np.linalg.norm(r_to_goal))

    if dist_to_goal > 1e-6:
        u_goal = r_to_goal / dist_to_goal
    else:
        u_goal = np.array([0.0, 0.0, -1.0])

    ref_axis = np.array([0.0, 1.0, 0.0]) if abs(u_goal[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e_lat1 = np.cross(u_goal, ref_axis)
    e_lat1 /= np.linalg.norm(e_lat1)
    e_lat2 = np.cross(u_goal, e_lat1)

    speed = float(np.linalg.norm(vel))
    v_long_mag = float(np.dot(vel, u_goal))
    v_long_vec = v_long_mag * u_goal
    v_lat_vec = vel - v_long_vec
    v_lat_mag = float(np.linalg.norm(v_lat_vec))

    # --- 4. Attitude Control: Nose-Aligned Pitch Slew & Deadband Hold ---
    if ref_body_y is None:
        ref_body_y = copy.deepcopy(rot_dcm[1, :])

    # Target body orientation: +X pointing at u_goal, aligned with initial body Y reference
    x_des = u_goal
    z_des = np.cross(x_des, ref_body_y)
    z_norm = np.linalg.norm(z_des)
    if z_norm < 1e-4:
        ref_alt = np.array([0.0, 1.0, 0.0]) if abs(x_des[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
        z_des = np.cross(x_des, ref_alt)
        z_norm = np.linalg.norm(z_des)
    z_des /= z_norm
    y_des = np.cross(z_des, x_des)

    R_des = np.vstack((x_des, y_des, z_des))
    R_err = R_des @ rot_dcm.T

    tr = np.trace(R_err)
    angle_err_rad = float(np.arccos(np.clip((tr - 1.0) / 2.0, -1.0, 1.0)))
    angle_err_deg = float(np.rad2deg(angle_err_rad))

    if abs(np.sin(angle_err_rad)) > 1e-5 and angle_err_rad > 1e-4:
        rot_axis = (0.5 / np.sin(angle_err_rad)) * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1]
        ])
        rot_axis_norm = np.linalg.norm(rot_axis)
        if rot_axis_norm > 1e-6:
            rot_axis /= rot_axis_norm
        theta_err_body = angle_err_rad * rot_axis
    else:
        theta_err_body = np.zeros(3)

    ang_speed = float(np.linalg.norm(ang_vel_body))

    # Check if initial pitch slew has completed
    if not is_aligned:
        if (angle_err_deg < SLEW_ALIGN_TOL_DEG and ang_speed < np.deg2rad(0.8)) or sim_time_sec > SLEW_MAX_TIME_SEC:
            is_aligned = True
            st.OnScreenLogMessage(
                f"PITCH COMPLETE at t={sim_time_sec:.1f}s (Error: {angle_err_deg:.1f}deg). Starting Forward +X Burn!",
                "PitchSlew", st.Severity.Info
            )

    # --- 5. Collision Avoidance Geometry (Always Active) ---
    v_evade_lat_des = np.zeros(3)
    a_evade_ff = np.zeros(3)
    max_threat_urgency = 0.0

    if debrisInfo is not None and len(debrisInfo) > 0:
        for item in debrisInfo:
            rel_pos_raw = np.array(item[0, :], dtype=float)
            surf_dist = float(item[1, 0])
            range_rate = float(item[1, 1])
            debris_diam = float(item[1, 2])
            debris_rad = 0.5 * max(debris_diam, 1.0)
            t_sample = float(item[2, 0])

            # Dead-reckon rock position across radar refresh latency
            dt_stale = float(np.clip(sim_time_sec - t_sample, 0.0, 8.5)) if t_sample > 0.0 else 0.0
            rel_pos = rel_pos_raw - vel * dt_stale

            com_dist = float(np.linalg.norm(rel_pos))
            if com_dist < 1e-3:
                continue

            # Ignore debris already behind the shuttle along goal LOS
            forward_proj = float(np.dot(rel_pos, u_goal))
            if forward_proj < -3.0:
                continue

            r_safe = debris_rad + SHIP_BOUND_RADIUS + CLEARANCE_BUFFER

            # Emergency Proximity Repulsion
            if surf_dist < (SHIP_BOUND_RADIUS + 1.0) and range_rate < -0.5:
                lat_offset = rel_pos - forward_proj * u_goal
                lat_dist = float(np.linalg.norm(lat_offset))
                if lat_dist > 0.2:
                    escape_dir = -lat_offset / lat_dist
                else:
                    escape_dir = e_lat1

                prox_urg = np.clip((SHIP_BOUND_RADIUS + 1.0 - surf_dist) / 4.0, 0.2, 1.0)
                v_evade_lat_des += escape_dir * (MAX_EVADE_VEL * prox_urg)
                a_evade_ff += escape_dir * (1.8 * prox_urg)
                max_threat_urgency = max(max_threat_urgency, prox_urg)
                continue

            # Predictive TCA Collision Cone
            if range_rate >= -0.35:
                continue

            if speed > 1.5:
                u_vel = vel / speed
                tca = float(np.dot(rel_pos, u_vel)) / max(speed, 1.0)
            else:
                tca = -surf_dist / range_rate

            if tca < 0.0 or tca > TCA_HORIZON:
                continue

            miss_vec = rel_pos - vel * tca
            miss_dist = float(np.linalg.norm(miss_vec))

            if miss_dist < r_safe:
                penetration = (r_safe - miss_dist) / r_safe
                time_weight = np.clip((TCA_HORIZON - tca) / TCA_HORIZON, 0.15, 1.0)
                urgency = (penetration ** 1.3) * (time_weight ** 1.2)

                if miss_dist > 0.3:
                    escape_dir = -miss_vec / miss_dist
                else:
                    escape_dir = e_lat1

                v_evade_lat_des += escape_dir * (MAX_EVADE_VEL * urgency)
                if tca < 4.5:
                    a_evade_ff += escape_dir * (1.65 * urgency / max(tca, 0.8))

                max_threat_urgency = max(max_threat_urgency, urgency)

    # Clamp commanded lateral dodge speed
    evade_vel_Mag = float(np.linalg.norm(v_evade_lat_des))
    if evade_vel_Mag > MAX_EVADE_VEL:
        v_evade_lat_des = (v_evade_lat_des / evade_vel_Mag) * MAX_EVADE_VEL

    smooth_evade_acc = (
        EVADE_FILTER_ALPHA * a_evade_ff
        + (1.0 - EVADE_FILTER_ALPHA) * smooth_evade_acc
    )

    # --- 6. Guidance Execution (Phase-Dependent) ---
    if not is_aligned:
        # Phase 1: Zero Linear Burn, Full Angular Slew Authority
        v_stop = np.sqrt(max(0.0, 2.0 * SLEW_ANG_DECEL * angle_err_rad))
        target_rate = min(SLEW_CRUISE_RATE, v_stop)
        omega_target = (theta_err_body / angle_err_rad) * target_rate if angle_err_rad > 1e-4 else np.zeros(3)

        alpha_cmd = 2.0 * (omega_target - ang_vel_body)
        alpha_cmd_mag = float(np.linalg.norm(alpha_cmd))
        if alpha_cmd_mag > SLEW_ANG_MAX_ACCEL:
            alpha_cmd = (alpha_cmd / alpha_cmd_mag) * SLEW_ANG_MAX_ACCEL

        linearAccel_body = np.zeros(3, dtype=float)
        angularAccel_body = alpha_cmd
        coasting = False

    else:
        # Phase 2 & 3: Forward +X Burn and Ballistic Coast
        if dist_to_target <= KEEP_OUT_CUTOFF:
            accel_phobos = np.zeros(3)
            coasting = True
        else:
            if speed >= (V_CRUISE - DEADBAND_ENTER):
                long_coasting = True

            if not long_coasting:
                v_err_long = max(0.0, V_CRUISE - v_long_mag)
                a_cmd_long = KP_VEL_LONG * v_err_long * u_goal
                long_thrusting = True
            else:
                a_cmd_long = np.zeros(3)
                long_thrusting = False

            if max_threat_urgency > 0.035:
                v_err_lat = v_evade_lat_des - v_lat_vec
                a_cmd_lat = KP_VEL_LAT * v_err_lat + smooth_evade_acc
                lat_thrusting = True
            elif not long_coasting and v_lat_mag > LAT_DRIFT_MAX:
                a_cmd_lat = -0.30 * v_lat_vec
                lat_thrusting = True
            else:
                a_cmd_lat = np.zeros(3)
                lat_thrusting = False

            coasting = not (long_thrusting or lat_thrusting)

            if coasting:
                accel_phobos = np.zeros(3)
            else:
                a_lat_mag = float(np.linalg.norm(a_cmd_lat))
                if a_lat_mag >= MAX_ACCEL:
                    accel_phobos = (a_cmd_lat / a_lat_mag) * MAX_ACCEL
                else:
                    max_long_avail = np.sqrt(max(0.0, MAX_ACCEL**2 - a_lat_mag**2))
                    a_long_mag = float(np.linalg.norm(a_cmd_long))
                    if a_long_mag > max_long_avail and a_long_mag > 1e-6:
                        a_cmd_long = (a_cmd_long / a_long_mag) * max_long_avail
                    accel_phobos = a_cmd_long + a_cmd_lat

        accel_norm = float(np.linalg.norm(accel_phobos))
        if accel_norm > MAX_ACCEL:
            accel_phobos = (accel_phobos / accel_norm) * MAX_ACCEL

        linearAccel_body = np.array(
            st.frames.FramedVector(accel_phobos, phobosFrame).ExprIn(shuttleFrame),
            dtype=float
        )

        # Attitude Hold: Tight deadband to prevent fuel chatter
        if angle_err_deg < 3.5 and ang_speed < np.deg2rad(1.0):
            angularAccel_body = np.zeros(3, dtype=float)
        else:
            alpha_cmd = 0.10 * theta_err_body - 0.50 * ang_vel_body
            alpha_cmd_mag = float(np.linalg.norm(alpha_cmd))
            if alpha_cmd_mag > 0.015:
                alpha_cmd = (alpha_cmd / alpha_cmd_mag) * 0.015
            angularAccel_body = alpha_cmd

    SetCommand(linearAccel_body, angularAccel_body)

    if iteration % 10 == 0:
        hud_phase = "SLEW" if not is_aligned else ("COAST" if coasting else "BURN")
        st.OnScreenLogMessage(
            f"[{hud_phase}] Dist: {dist_to_target:.1f}m | Spd: {speed:.1f}m/s | PitchErr: {angle_err_deg:.1f}deg | Fuel: {fuelUsed:.1f}kg",
            "FlightHUD", st.Severity.Info
        )

    iteration += 1

st.leave_sim()
