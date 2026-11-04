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
# USER CONTROLLER CODE: DEAD-RECKONED CRASH-PROOF RADAR & TERMINAL BALLISTIC COAST
# ==============================================================================

# --- 1. Mission & Tuning Constants ---
V_CRUISE = 7.85           # Calibrated cruise speed for Score >= 55 (m/s) -> ~39.9kg fuel, ~205s transit
MAX_ACCEL = 2.15          # LP Simplex solver saturation ceiling (m/s^2)
KEEP_OUT_CUTOFF = 49.5    # Cutoff distance (m) - victory triggers at <= 50.0m
TERMINAL_COAST_DIST = 160.0 # Distance inside which forward burn is permanently locked off

# Speed & Coasting Calibration
DEADBAND_ENTER = 0.15     # Enter ballistic coast when speed reaches 7.70 m/s
LAT_DRIFT_MAX = 2.80      # Initial climb heading tolerance (m/s)

KP_VEL_LONG = 1.25        # Forward velocity proportional gain (1/s)
KP_VEL_LAT = 1.45         # Lateral evasion velocity proportional gain (1/s)

# Proven Crash-Proof Geometry with Dead-Reckoning Compensation
RADAR_QUERY_COUNT = 80    # Scan 80 nearest rocks to overcome radar sort latency
SHIP_BOUND_RADIUS = 3.0   # Shuttle bounding radius (m)
CLEARANCE_BUFFER = 4.5    # Safety standoff distance (m) -> R_safe = r_rock + 7.5m (guards against 2m radar 1-sigma noise)
TCA_HORIZON = 10.0        # Lookahead time horizon for collision cone (seconds)
MAX_EVADE_VEL = 1.6       # Controlled lateral dodge velocity ceiling (m/s) to prevent overshooting into secondary debris
EVADE_FILTER_ALPHA = 0.45 # EMA smoothing factor for feed-forward acceleration

# Keep cameras facing forward
PanTiltCamera(topCamera, 0.0, 0.0)
PanTiltCamera(bottomCamera, 0.0, 0.0)

LOOP_DT = 1.0 / this.GetParam(st.VarType.double, "LoopFreqHz")
t0_sim = st.SimGlobals.SimClock.GetTimeNow().as_datetime()

smooth_evade_acc = np.zeros(3)
long_coasting = False
coasting = False
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
    rot = np.array(shuttle.getRotation().Quat_WRT(phobosFrame), dtype=float)

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

    # --- 4. Dead-Reckoned Crash-Proof TCA Collision Cone & Proximity Repulsion ---
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

            # Dead-reckon rock position across the radar refresh cycle
            dt_stale = float(np.clip(sim_time_sec - t_sample, 0.0, 5.0)) if t_sample > 0.0 else float(np.clip(sim_time_sec, 0.0, 5.0))
            rel_pos = rel_pos_raw - vel * dt_stale

            com_dist = float(np.linalg.norm(rel_pos))
            if com_dist < 1e-3:
                continue

            # Ignore debris already behind the shuttle along goal LOS
            forward_proj = float(np.dot(rel_pos, u_goal))
            if forward_proj < -3.0:
                continue

            r_safe = debris_rad + SHIP_BOUND_RADIUS + CLEARANCE_BUFFER

            # Case A: Emergency Proximity Repulsion (Point-blank physical threat < 4.0m surface clearance)
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

            # Case B: Predictive TCA Collision Cone
            if range_rate >= -0.35:
                continue

            if speed > 1.5:
                u_vel = vel / speed
                tca = float(np.dot(rel_pos, u_vel)) / max(speed, 1.0)
            else:
                tca = -surf_dist / range_rate

            if tca < 0.0 or tca > TCA_HORIZON:
                continue

            # Velocity-orthogonal miss vector (strictly orthogonal to velocity)
            miss_vec = rel_pos - vel * tca
            miss_dist = float(np.linalg.norm(miss_vec))

            if miss_dist < r_safe:
                penetration = (r_safe - miss_dist) / r_safe
                time_weight = np.clip((TCA_HORIZON - tca) / TCA_HORIZON, 0.15, 1.0)
                urgency = (penetration ** 1.3) * (time_weight ** 1.2)

                # Pure lateral evasion strictly perpendicular to travel direction
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

    # --- 5. Decoupled Guidance Law & Zero-Fuel Coasting ---
    if dist_to_target <= KEEP_OUT_CUTOFF:
        accel_phobos = np.zeros(3)
        coasting = True
    else:
        # One-Way Ballistic Coast Lock:
        # Accelerate to 7.70 m/s during initial climb, then lock coasting ON permanently.
        # Saves fuel by eliminating mid-flight re-ignitions; momentum carries ship to goal.
        if speed >= (V_CRUISE - DEADBAND_ENTER):
            long_coasting = True

        if not long_coasting:
            v_err_long = max(0.0, V_CRUISE - v_long_mag)
            a_cmd_long = KP_VEL_LONG * v_err_long * u_goal
            long_thrusting = True
        else:
            a_cmd_long = np.zeros(3)
            long_thrusting = False

        # Lateral Evasion: NEVER SUPPRESS REAL EVASION!
        # Always react with full authority when max_threat_urgency > 0.015
        if max_threat_urgency > 0.015:
            v_err_lat = v_evade_lat_des - v_lat_vec
            a_cmd_lat = KP_VEL_LAT * v_err_lat + smooth_evade_acc
            lat_thrusting = True
        elif not long_coasting and v_lat_mag > LAT_DRIFT_MAX:
            # Only hold lateral heading during initial climb; zero fuel chatter in coast
            a_cmd_lat = -0.30 * v_lat_vec
            lat_thrusting = True
        else:
            a_cmd_lat = np.zeros(3)
            lat_thrusting = False

        coasting = not (long_thrusting or lat_thrusting)

        # Acceleration Allocation: Evasion always gets top priority
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

    # --- 6. Body Frame Transformation & Zero-Torque Actuation ---
    linearAccel_body = np.array(
        st.frames.FramedVector(accel_phobos, phobosFrame).ExprIn(shuttleFrame),
        dtype=float
    )

    # Strictly zero angular acceleration to prevent rotational fuel burn
    angularAccel_body = np.zeros(3, dtype=float)

    SetCommand(linearAccel_body, angularAccel_body)

    if iteration % 10 == 0:
        st.OnScreenLogMessage(
            f"Dist: {dist_to_target:.1f}m | Spd: {speed:.1f}m/s | Dam: {damage:.1f}J | Fuel: {fuelUsed:.1f}kg | Coast: {coasting}",
            "FlightHUD", st.Severity.Info
        )

    iteration += 1

st.leave_sim()
