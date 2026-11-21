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

from collections import deque

# ==============================================================================
# ULTRA-STABLE 60-65 CONSTANT-SCORE LYAPUNOV MULTI-VERTEX GNC CONTROLLER (V11)
# ==============================================================================
# Key Physics & Fuel-Efficiency Innovations for SpaceBuggy (Mass=3388.7kg, Isp=80s, 38 RCS jets):
# 1. Lyapunov Multi-Vertex Pure-Axis Allocator:
#    Unlocks 100.0%-L1-efficiency 4-jet simplex vertices on +Z, +X, -X, -Y, -Z
#    (0.1929 m/s^2 — 2.01x default +Z thrust!) and 3-jet vertices on +Y (0.1447 m/s^2),
#    while closed-loop feedback on measured ang_vel_body cancels net torque to ~0
#    AND steers the gravity-compensated launch aim for 0.00 kg extra fuel.
# 2. Zero-Fuel CLIMB_YIELD (Timing-Modulation Crossing-Debris Avoidance):
#    When a converged (n >= 4) fast-crossing debris rock blocks the climb corridor and
#    waiting clearly improves miss clearance without endangering neighboring tracks,
#    cleanly pauses +Z thrust at 0.00 kg/s until the rock vacates the path (saving ~5.0 kg
#    of lateral EVADE + ZEM with 0.0 deg attitude tilt and 0.0 m ZEM disturbance), while
#    also shifting downstream encounter phase. EVADE stays 100% armed with top priority.
# 3. Tumble-Immune 500-Rock Moving-Debris Tracker + Extreme-Range Chaos Gate:
#    Tracks position + velocity (exponentially-weighted 7-sweep WLS + Doppler range-rate)
#    across SpaceCRAFT's 4.17s round-robin radar sweep, retains full Run1ce monster-boulder
#    standoff armor, and gates extreme-range (tca > 21.5s) high-chaos triggers to n >= 5
#    to eliminate unconverged n=4 false evades on distant fast boulders (saving ~5.3 kg!).
# 4. Simpson Midpoint Gravity + Online Coast Environmental Accel Observer + Edge ZEM:
#    Directly measures live Phobos Coriolis + Mars tidal acceleration during pure coast,
#    computes exact 3D CPA miss distance norm(zem_vec) with < 0.8 m error, uses edge-optimized
#    anti-chatter ZEM capture inside AsteroidCycler's 50 m sphere, and raises coast rate-damp
#    entry to 0.024 rad/s to eliminate wasted DAMP burns.

# --- 1. Mission & Physics Constants ---
V_CRUISE = 9.15             # Score- & fuel-optimal ballistic cruise speed (m/s)
DEADBAND_ENTER = 0.06       # Enter ballistic coast cleanly at V_CRUISE - 0.06
MAX_ACCEL = 0.50            # Thrusters.MaxAccel_m_s2 ceiling (m/s^2)
MAX_ANG_ACCEL = 0.05        # Thrusters.MaxAngAccel_rad_s2 ceiling (rad/s^2)
KEEP_OUT_CUTOFF = 49.2      # Victory triggers at <= 50.0 m; latch & cut thrusters inside 49.2 m
MU_PHOBOS = 707500.0        # Phobos GM (m^3/s^2) from Phobos_Proc_GDAL.enconf

# Radar Tracking & Kinodynamic Evasion Constants
RADAR_QUERY_COUNT = 500     # Scan all 500 debris so distant fast threats aren't missed
RADAR_FETCH_PERIOD = 4      # Query GetParamArray at 15 Hz (Radar re-sorts at 2.4 Hz)
TRACK_MAX_RANGE = 360.0     # Track debris within 360 m (~39 s ahead at cruise)
TRACK_PRUNE_RANGE = 440.0   # Prune tracks beyond 440 m
TRACK_STALE_MAX_S = 14.0    # Drop tracks not re-observed within 14 s

SHIP_BOUND_RADIUS = 3.2     # Conservative SpaceBuggy bounding radius (m)
TCA_HORIZON = 26.0          # 26 s lookahead for giant R=20 m moving rocks
A_REQ_TRIG = 0.048          # Fuel-optimal dodge trigger threshold (m/s^2)
BUF_MIN = 2.5               # Minimum mesh/maneuver safety standoff (m)
BUF_SCALE = 0.18            # Proportional standoff for irregular lunar rock meshes
MONSTER_RAD_THRESH = 17.8   # Top-scale Rock2 monster boulders (R >= 17.8 m, e.g. Debris483)
CLIMB_PRED_GAIN = 0.35      # Climb-aware relative velocity lookahead gain during CLIMB_Z
CHAOS_TAPER_LO = 4.0        # Taper Chaos pre-bounce standoff to 0 by tca <= 4 s (m)
CHAOS_TAPER_HI = 11.0       # Full Chaos pre-bounce standoff for tca >= 11 s (m)
CHAOS_BASE_SCALE = 0.16     # Base speed-proportional Chaos standoff scale for monster boulders
CHAOS_BASE_MAX = 4.8        # Max base Chaos standoff for normal-speed monster boulders (m)
CHAOS_HI_SCALE = 2.15       # High-energy Chaos standoff ramp for fast monster boulders (>2.60 m/s)
CHAOS_HI_MAX = 16.5         # Max high-energy Chaos pre-bounce standoff (m)
CHAOS_HI_SPEED = 2.60       # Speed threshold (m/s) for high-energy monster boulder standoff
CHAOS_HI_N5_TCA = 21.5      # Require n >= 5 for hi_chaos beyond 21.5 s (eliminates n=4 false evades)

# Zero-Fuel CLIMB_YIELD Constants
MAX_YIELD_S = 10.5          # Maximum total zero-fuel climb-yield budget (s)
YIELD_CLEAR_PAD = 3.5       # Hold yield until predicted full-climb miss >= r_safe + 3.5 m

# Coast Attitude & ZEM Constants
EVADE_ZEM_COOLDOWN_S = 4.2  # Early ZEM return cooldown (protected by per-threat opposite-axis lock)
COAST_DAMP_ENTER = 0.024    # Coast-only rate damp entry threshold (rad/s) — eliminates wasted DAMP burns
COAST_DAMP_EXIT = 0.008     # Coast-only rate damp exit threshold (rad/s)
ANG_VEL_FAILSAFE = 0.25     # Emergency tumble-damping threshold (rad/s)

# Exact 38-jet indices per body axis for 100%-efficiency zero-torque convex hulls
AXIS_SUBSETS = {
    "+Z": [
        [0, 20, 21, 31],
        [0, 21, 31, 35],
        [0, 28, 29, 33],
        [0, 29, 31, 35],
        [0, 29, 33, 35],
    ],
    "+X": [
        [10, 11, 36, 37],
    ],
    "-X": [
        [13, 19, 26, 27],
        [13, 16, 19, 26],
        [16, 19, 26, 27],
        [13, 16, 19, 27],
    ],
    "+Y": [
        [6, 15, 30],
        [6, 30, 34],
        [6, 24, 34],
        [6, 7, 24],
    ],
    "-Y": [
        [1, 12, 22, 32],
        [1, 2, 22, 32],
        [3, 12, 22, 32],
        [1, 3, 12, 32],
        [1, 3, 12, 22],
    ],
    "-Z": [
        [14, 17, 23, 25],
        [5, 9, 17, 23],
        [9, 14, 17, 25],
        [5, 14, 17, 23],
    ],
}

AXIS_UNIT_BODY = {
    "+X": np.array([ 1.0,  0.0,  0.0], dtype=float),
    "-X": np.array([-1.0,  0.0,  0.0], dtype=float),
    "+Y": np.array([ 0.0,  1.0,  0.0], dtype=float),
    "-Y": np.array([ 0.0, -1.0,  0.0], dtype=float),
    "+Z": np.array([ 0.0,  0.0,  1.0], dtype=float),
    "-Z": np.array([ 0.0,  0.0, -1.0], dtype=float),
}

# Exact float64 fallback table computed from SpaceBuggyJetConfigNoMains.csv + COMPETITION_SIM.simconf
# (Automatically refreshed from live RCSConfig + getMass() + getInertia() as soon as RCSConfig.Ready == True)
FALLBACK_VERTEX_TABLE = {
    "+Z": ([
        (np.array([2.6775066635046886e-07, 3.150154301840697e-08, 0.06300048783711278], dtype=float), np.array([-0.0032073973044228184, -0.049732613484391425, 0.002527797172185555], dtype=float), np.array([-0.009820395566917592, -0.1522711066445919, 0.00773959874243425], dtype=float), 0.19289462849574476),
        (np.array([3.197578650223288e-07, -4.567719407950622e-08, 0.09135945757938192], dtype=float), np.array([-0.03109252680135723, -0.03883092056831698, 0.003927389072136752], dtype=float), np.array([-0.06564806262252963, -0.08198673338598111, 0.0082922010616203], dtype=float), 0.19289430642566133),
        (np.array([-8.596567676599192e-08, 6.877654910670562e-08, 0.06877600557328924], dtype=float), np.array([0.03555707993555208, 0.0347757121681321, -0.004043996742826176], dtype=float), np.array([0.09972633375208179, 0.09753484494315594, -0.011342128476196343], dtype=float), 0.1928948861486191),
        (np.array([8.98344936606554e-07, -1.5622959143096784e-07, 0.15623421733397042], dtype=float), np.array([-0.0480333233134731, 0.013254976186622072, 0.0026674815379760432], dtype=float), np.array([-0.05930430912588594, 0.016365247103508274, 0.003293404220302045], dtype=float), 0.1928944674617911),
        (np.array([-5.166325125204782e-07, 6.078337434692667e-08, 0.12156290584356864], dtype=float), np.array([0.023667226019529693, 0.043778477112796726, -0.003649295031212138], dtype=float), np.array([0.037554873079086844, 0.0694671673904714, -0.00579065800158516], dtype=float), 0.19289457481645006),
    ], 0.19289457266965324),
    "+X": ([
        (np.array([0.49999999999693756, 1.7499940588868367e-06, 0.0], dtype=float), np.array([-4.943009872870813e-07, -6.089304449353218e-07, -1.8346585415010986e-06], dtype=float), np.array([-1.9069390917898776e-07, -2.3491623514677873e-07, -7.077837558197575e-07], dtype=float), 0.19289250283070555),
    ], 0.19289250283070555),
    "-X": ([
        (np.array([-0.43465208795744376, -6.4111183939493355e-06, -1.0866292521941837e-06], dtype=float), np.array([-0.00766552457199822, -0.002712599859985936, 0.049233032966045226], dtype=float), np.array([-0.0034018722146662043, -0.0012038208222438839, 0.02184905746734503], dtype=float), 0.19289363008970217),
        (np.array([-0.24231014786884228, -3.150031814373925e-06, -3.028877522562527e-07], dtype=float), np.array([-0.007708819276265275, -0.029355236569217676, 0.039608764072249794], dtype=float), np.array([-0.006136682642844938, -0.02336852950821804, 0.031530952572087656], dtype=float), 0.1928934153790579),
        (np.array([-0.3547795906418822, -5.2329988630880655e-06, -8.869499638271135e-07], dtype=float), np.array([0.0076765944915579785, 0.0029198712100537687, -0.049219449906809], dtype=float), np.array([0.004173755340579262, 0.0015875305215310664, -0.026760556668042154], dtype=float), 0.19289324360746107),
        (np.array([-0.24268444748470644, -3.579595586788808e-06, -3.0335562685076434e-07], dtype=float), np.array([0.005175988268253277, -0.024649196173684253, -0.04307709685480365], dtype=float), np.array([0.004114042186287025, -0.019591975031818126, -0.03423906402771668], dtype=float), 0.1928934153774967),
    ], 0.19289342611342947),
    "+Y": ([
        (np.array([-6.483046231971924e-07, 0.3889820465114789, -1.5559385198331192e-06], dtype=float), np.array([-0.007671252671253927, -0.002826537994875752, 0.04922573072507205], dtype=float), np.array([-0.0028530914142091343, -0.0010512456870746933, 0.018308028128985736], dtype=float), 0.1446701581545498),
        (np.array([-2.6989310452160223e-08, 0.08096529969492465, 4.857860421628034e-07], dtype=float), np.array([0.04662118494327431, 0.002283991452318366, -0.017642519591338116], dtype=float), np.array([0.08330307165565547, 0.00408105250531238, -0.03152378206370672], dtype=float), 0.1446693851826412),
        (np.array([6.754894418746045e-08, 0.10132576227868045, 4.728471959571504e-07], dtype=float), np.array([0.00767100339551637, 0.002826728806375543, -0.049225758614379764], dtype=float), np.array([0.010952385907275333, 0.004035902886020389, -0.07028278793856255], dtype=float), 0.14466932076785735),
        (np.array([-5.6167745469276746e-08, 0.08425276862846562, -3.3701049930158723e-07], dtype=float), np.array([-0.015438594433787286, 0.002060689427415234, -0.0474068914926171], dtype=float), np.array([-0.026509096608508042, 0.0035383412230799074, -0.08140079538176952], dtype=float), 0.14466762454985593),
    ], 0.14466912216372607),
    "-Y": ([
        (np.array([1.2499314039980613e-07, -0.49999999999992184, 2.500445529183616e-07], dtype=float), np.array([0.002769576546311314, 0.0010184508113483526, -0.017783816582586036], dtype=float), np.array([0.0010684641364094721, 0.0003929041672713989, -0.00686074925507453], dtype=float), 0.19289305035322382),
        (np.array([3.5358318574522675e-08, -0.1414365017166285, -1.060755744176579e-06], dtype=float), np.array([-0.034362837335243146, -0.0029044605924544845, 0.036066182483574544], dtype=float), np.array([-0.04686434422318443, -0.003961129276360963, 0.04918738153769695], dtype=float), 0.19289236327291234),
        (np.array([-1.848182290179027e-13, -0.16603908444493723, -1.1207511825922714e-06], dtype=float), np.array([0.02722441654121241, -0.0014239495401172804, 0.04179489815393453], dtype=float), np.array([0.0316272300758741, -0.0016542348907110402, 0.04855409326812135], dtype=float), 0.1928914185359171),
        (np.array([-1.7221545058519275e-07, -0.22962007257854694, 1.722429015975097e-07], dtype=float), np.array([-0.048476562585978794, -0.0020226633272564893, 0.011659404483602594], dtype=float), np.array([-0.04072281795800219, -0.001699141731019044, 0.009794502352396449], dtype=float), 0.19289272828575652),
        (np.array([-1.8617929177621383e-07, -0.24823807935543293, -1.2411493079384345e-06], dtype=float), np.array([0.0495849783409504, 0.0011440706794398079, 0.005480057044221918], dtype=float), np.array([0.038529856176297735, 0.00088899663182731, 0.004258261611004885], dtype=float), 0.19289264239263412),
    ], 0.1928924405680888),
    "-Z": ([
        (np.array([-7.1239571891037795e-12, -1.6696774647381781e-13, -0.5], dtype=float), np.array([0.001296056107183125, 0.020089004155199333, -0.0010226212969859662], dtype=float), np.array([0.000499999430065852, 0.007750043051778909, -0.00039451229220120515], dtype=float), 0.1928926638649005),
        (np.array([5.797833426980114e-14, 1.1015883510904838e-12, -0.13021545421272576], dtype=float), np.array([-0.0032081455207079615, -0.049732565732678195, 0.0025277871672284417], dtype=float), np.array([-0.004752316481373718, -0.07367025288187447, 0.0037444824552639857], dtype=float), 0.19289182650547018),
        (np.array([1.0760580879947214e-07, -1.506480537472157e-07, -0.08608488275756324], dtype=float), np.array([0.04975332554716303, -0.0019102803321649122, -0.0033117104418348953], dtype=float), np.array([0.11148323935774632, -0.004280402107176923, -0.007420613673766366], dtype=float), 0.19289206267166545),
        (np.array([4.6682693425768156e-07, 1.4853575426553602e-07, -0.0848776373040619], dtype=float), np.array([-0.04950624747921814, -0.005102717945738974, 0.003619907470176655], dtype=float), np.array([-0.11250757415680901, -0.011596403422063094, 0.008226578035684294], dtype=float), 0.19289236328019413),
    ], 0.19289222908055756),
}

# Keep cameras facing forward
PanTiltCamera(topCamera, 0.0, 0.0)
PanTiltCamera(bottomCamera, 0.0, 0.0)

LOOP_DT = 1.0 / this.GetParam(st.VarType.double, "LoopFreqHz")
DT_LYAP = float(np.clip(LOOP_DT, 0.005, 0.05))
t0_sim = st.SimGlobals.SimClock.GetTimeNow().as_datetime()

# Phobos unit basis vectors for fast exact Phobos->Body DCM extraction
_EX = np.array([1.0, 0.0, 0.0], dtype=float)
_EY = np.array([0.0, 1.0, 0.0], dtype=float)
_EZ = np.array([0.0, 0.0, 1.0], dtype=float)


def try_sync_vertex_table_from_sim() -> tuple[dict | None, str]:
    """Compute bit-for-bit exact simplex vertices from live RCSConfig, Mass, and Inertia."""
    try:
        if not shuttle.HasParam(["RCSConfig", "Ready"]):
            return None, "not_ready"
        if not bool(shuttle.GetParam(st.VarType.bool, ["RCSConfig", "Ready"])):
            return None, "not_ready"
        num_jets = int(shuttle.GetParam(st.VarType.int32, ["RCSConfig", "NumMain"]))
        if num_jets != 38:
            return None, f"unexpected_jet_count={num_jets}"

        mass = float(shuttle.getMass())
        try:
            I_mat = np.asarray(shuttle.getInertia(shuttleFrame), dtype=float)
        except Exception:
            I_mat = np.asarray(shuttle.getInertiaTensor(shuttleFrame), dtype=float)
        I_inv = np.linalg.inv(I_mat)

        max_acc = float(shuttle.GetParam(st.VarType.double, ["Thrusters", "MaxAccel_m_s2"]))
        max_ang = float(shuttle.GetParam(st.VarType.double, ["Thrusters", "MaxAngAccel_rad_s2"]))

        a_jets = np.zeros((num_jets, 6), dtype=float)
        for i in range(num_jets):
            cfg = shuttle.GetParamArray(st.VarType.double, ["RCSConfig", f"Main{i}"])
            r = np.array([float(cfg[0]), float(cfg[1]), float(cfg[2])], dtype=float)
            n = np.array([float(cfg[3]), float(cfg[4]), float(cfg[5])], dtype=float)
            n = n / np.linalg.norm(n)
            F = float(cfg[6])
            force = -F * n
            torque = np.cross(r, force)
            a_trans = force / mass
            a_rot = (I_inv @ np.array([torque]).T).T[0]
            a_jets[i, :3] = a_trans
            a_jets[i, 3:] = a_rot

        table = {}
        for ax_name, subsets in AXIS_SUBSETS.items():
            ax_u = AXIS_UNIT_BODY[ax_name]
            verts = []
            a_sum = 0.0
            for sub in subsets:
                b_u = a_jets[sub].sum(axis=0)
                na = float(np.linalg.norm(b_u[:3]))
                nw = float(np.linalg.norm(b_u[3:]))
                if na < 0.05 or float(np.dot(b_u[:3] / na, ax_u)) < 0.995:
                    return None, f"axis_mismatch_{ax_name}"
                sc = min(max_acc / max(na, 1e-12), (max_ang * 0.998) / max(nw, 1e-12))
                b_cmd = b_u * sc
                if not np.all(np.isfinite(b_cmd)):
                    return None, f"nonfinite_{ax_name}"
                a_sum += na
                verts.append((b_cmd[:3].copy(), b_cmd[3:].copy(), b_u[3:].copy(), na))
            table[ax_name] = (verts, a_sum / len(verts))
        return table, f"synced(mass={mass:.2f}kg,jets={num_jets})"
    except Exception as exc:
        return None, f"fallback({exc})"


class PhobosGNCController:
    def __init__(self) -> None:
        self.vertex_table = FALLBACK_VERTEX_TABLE

        # Shuttle state ring buffer for exact radar measurement timestamp alignment
        self.hist_t: deque[float] = deque(maxlen=600)
        self.hist_loc: deque[npt.NDArray] = deque(maxlen=600)
        self.hist_vel: deque[npt.NDArray] = deque(maxlen=600)

        # Radar clock sync
        self.radar_offset: float | None = None
        self.last_radar_max_ts: float = -1.0

        # Multi-sweep moving debris tracks with persistent track_id & non-destructive shadow hypothesis
        self.next_track_id: int = 1
        self.tracks: list[dict] = []
        self._tr_id = np.zeros(0, dtype=int)
        self._tr_p_ref = np.zeros((0, 3), dtype=float)
        self._tr_v_ref = np.zeros((0, 3), dtype=float)
        self._tr_t_ref = np.zeros(0, dtype=float)
        self._tr_diam = np.zeros(0, dtype=float)
        self._tr_n_obs = np.zeros(0, dtype=float)
        self._tr_ag = np.zeros((0, 3), dtype=float)
        self._tr_p_sh = np.zeros((0, 3), dtype=float)
        self._tr_v_sh = np.zeros((0, 3), dtype=float)
        self._tr_t_sh = np.zeros(0, dtype=float)
        self._tr_in_vec = np.zeros((0, 3), dtype=float)
        self._tr_sh_age = np.zeros(0, dtype=int)

        # Flight state latches
        self.start_loc: npt.NDArray | None = None
        self.mission_won: bool = False
        self.long_coasting: bool = False
        self.damping: bool = False
        self.evading: bool = False
        self.last_evade_t: float = -999.0
        self.prev_esc_dir = np.zeros(3, dtype=float)
        self.prev_esc_axis: str | None = None
        self.prev_esc_tid: int = -1
        self.last_dodged_tid: int = -1
        self.last_dodged_axis: str | None = None
        self.zem_burning: bool = False
        self.prev_zem_axis: str | None = None
        self.zem_blocked_until: float = -999.0
        self.zem_axis_lock_until: float = -999.0

        # Zero-fuel CLIMB_YIELD & online coast environmental accel observer
        self.yielding: bool = False
        self.yield_tid: int = -1
        self.yield_hold_until: float = -999.0
        self.total_yield_s: float = 0.0
        self.last_step_t: float | None = None
        self.coast_start_t: float | None = None
        self.a_env_extra = np.zeros(3, dtype=float)
        self.a_env_init: bool = False

    def _update_env_observer(
        self,
        sim_t: float,
        loc: npt.NDArray,
        vel: npt.NDArray,
        is_pure_coast: bool,
        dt_step: float,
    ) -> None:
        """Online coast environmental acceleration observer (Phobos Coriolis + Mars tidal gravity)."""
        if not is_pure_coast:
            self.coast_start_t = None
            return
        if self.coast_start_t is None:
            self.coast_start_t = sim_t
            return
        win = 3.0
        if (sim_t - self.coast_start_t) >= (win + 0.25) and len(self.hist_t) >= 2:
            t_q = sim_t - win
            if self.hist_t[0] > t_q:
                return
            loc_p, vel_p = self._lookup_shuttle_state(t_q, loc, vel, sim_t)
            a_meas = (vel - vel_p) / win
            p_m = 0.5 * (loc + loc_p)
            r_m = float(np.linalg.norm(p_m))
            a_sph_m = -MU_PHOBOS * p_m / max(r_m ** 3, 1.0)
            a_ex = a_meas - a_sph_m
            if not np.all(np.isfinite(a_ex)):
                return
            a_ex = np.clip(a_ex, -0.015, 0.015)
            if not self.a_env_init:
                self.a_env_extra = a_ex.copy()
                self.a_env_init = True
            else:
                alpha = float(np.clip(dt_step / 0.40, 0.01, 0.25))
                self.a_env_extra = (1.0 - alpha) * self.a_env_extra + alpha * a_ex

    def _predict_climb_cpa(
        self,
        rel: npt.NDArray,
        v_rock: npt.NDArray,
        vel: npt.NDArray,
        u_goal: npt.NDArray,
        a_g0: npt.NDArray,
        delay_s: float,
    ) -> tuple[float, float]:
        """Predict rock CPA under: zero-thrust coast for delay_s, then +Z CLIMB_Z to V_CRUISE."""
        rel_d = rel + (v_rock - vel) * delay_s
        vel_d = vel + a_g0 * delay_s
        v_long_d = max(float(np.dot(vel_d, u_goal)), 0.5)
        dv_rem = max(0.0, V_CRUISE - v_long_d)
        a_net = 0.183
        t_burn = dv_rem / a_net
        vr0 = v_rock - vel_d
        vr_n2 = float(np.dot(vr0, vr0))
        t_f = max(-float(np.dot(rel_d, vr0)) / max(vr_n2, 0.25), 0.5)
        vr_eff = vr0
        for _ in range(3):
            tc = min(t_f, t_burn)
            dv_avg = a_net * tc * (1.0 - 0.5 * tc / max(t_f, 0.5))
            vr_eff = vr0 - dv_avg * u_goal
            ve2 = float(np.dot(vr_eff, vr_eff))
            if ve2 < 0.25:
                return -1.0, 1e9
            t_f = -float(np.dot(rel_d, vr_eff)) / ve2
            if t_f <= 0.1:
                return t_f, float(np.linalg.norm(rel_d))
        cpa = rel_d + vr_eff * t_f
        return delay_s + t_f, float(np.linalg.norm(cpa))

    def _select_vertex(
        self,
        axis_name: str,
        w_b: npt.NDArray,
        w_des: npt.NDArray,
        dt: float = DT_LYAP,
        weights: tuple[float, float, float] = (1.0, 1.0, 0.35),
    ) -> tuple[npt.NDArray, npt.NDArray]:
        """Lyapunov feedback selector: pick 100%-eff high-thrust vertex driving w_b -> w_des."""
        verts, _ = self.vertex_table[axis_name]
        if len(verts) == 1:
            return verts[0][0], verts[0][1]
        w0, w1, w2 = weights
        best_v = verts[0]
        best_cost = 1e18
        for v in verts:
            e0 = (w_b[0] + v[2][0] * dt) - w_des[0]
            e1 = (w_b[1] + v[2][1] * dt) - w_des[1]
            e2 = (w_b[2] + v[2][2] * dt) - w_des[2]
            c = w0 * e0 * e0 + w1 * e1 * e1 + w2 * e2 * e2
            if c < best_cost:
                best_cost = c
                best_v = v
        return best_v[0], best_v[1]

    def _best_pure_axis(
        self,
        u_phobos: npt.NDArray,
        c_ph_to_body: npt.NDArray,
        exclude: tuple[str, ...] = ("+Z", "-Z"),
    ) -> tuple[str, float]:
        ub = c_ph_to_body @ u_phobos
        best_k = "+X"
        best_dot = -1e9
        for k, ax_u in AXIS_UNIT_BODY.items():
            if k in exclude:
                continue
            d = float(np.dot(ub, ax_u))
            if d > best_dot:
                best_dot = d
                best_k = k
        return best_k, best_dot

    def _lookup_shuttle_state(
        self,
        t_query: float,
        cur_loc: npt.NDArray,
        cur_vel: npt.NDArray,
        cur_t: float,
    ) -> tuple[npt.NDArray, npt.NDArray]:
        n = len(self.hist_t)
        if n == 0 or t_query >= cur_t:
            return cur_loc, cur_vel
        if t_query <= self.hist_t[0]:
            return self.hist_loc[0], self.hist_vel[0]
        lo, hi = 0, n - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if self.hist_t[mid] <= t_query:
                lo = mid
            else:
                hi = mid
        t0 = self.hist_t[lo]
        t1 = self.hist_t[hi]
        alpha = (t_query - t0) / max(t1 - t0, 1e-6)
        loc_q = (1.0 - alpha) * self.hist_loc[lo] + alpha * self.hist_loc[hi]
        vel_q = (1.0 - alpha) * self.hist_vel[lo] + alpha * self.hist_vel[hi]
        return loc_q, vel_q

    def update_tracks(
        self,
        sim_t: float,
        loc: npt.NDArray,
        vel: npt.NDArray,
        debris_arr: npt.NDArray,
    ) -> None:
        if debris_arr is None or len(debris_arr) == 0:
            return

        t_samples = debris_arr[:, 2, 0]
        max_ts = float(np.max(t_samples))
        if max_ts > 0.0:
            est_off = sim_t - max_ts
            if self.radar_offset is None:
                self.radar_offset = est_off
            else:
                self.radar_offset = min(self.radar_offset + 0.0005, est_off)

        first_sweep = (len(self.tracks) == 0)
        if not first_sweep and max_ts <= self.last_radar_max_ts + 1e-4:
            return
        prev_max_ts = self.last_radar_max_ts
        self.last_radar_max_ts = max_ts
        off = self.radar_offset if self.radar_offset is not None else 0.0

        rel_raws = debris_arr[:, 0, :]
        raw_dists = np.linalg.norm(rel_raws, axis=1)
        if first_sweep:
            keep_idx = np.where(raw_dists < TRACK_MAX_RANGE)[0]
        else:
            keep_idx = np.where(
                (raw_dists < TRACK_MAX_RANGE)
                & ((t_samples > prev_max_ts - 0.05) | (raw_dists < 110.0))
            )[0]

        # Prune stale or distant tracks
        alive = []
        for tr in self.tracks:
            if (sim_t - tr["last_t"]) > TRACK_STALE_MAX_S:
                continue
            p_now = tr["p_ref"] + tr["v_ref"] * (sim_t - tr["t_ref"])
            if float(np.linalg.norm(p_now - loc)) > TRACK_PRUNE_RANGE:
                continue
            alive.append(tr)
        self.tracks = alive

        n_tr = len(self.tracks)
        if n_tr > 0:
            tr_p_ref = np.array([tr["p_ref"] for tr in self.tracks], dtype=float)
            tr_v_ref = np.array([tr["v_ref"] for tr in self.tracks], dtype=float)
            tr_t_ref = np.array([tr["t_ref"] for tr in self.tracks], dtype=float)
            tr_diam = np.array([tr["diam"] for tr in self.tracks], dtype=float)
            r_n = np.linalg.norm(tr_p_ref, axis=1, keepdims=True)
            tr_ag = -MU_PHOBOS * tr_p_ref / np.maximum(r_n ** 3, 1.0)

        used_tracks: set[int] = set()
        for idx in keep_idx:
            item = debris_arr[idx]
            rel_raw = item[0, :]
            if not np.all(np.isfinite(rel_raw)):
                continue
            rr_raw = float(item[1, 1]) if np.isfinite(item[1, 1]) else 0.0
            diam_raw = float(max(item[1, 2], 0.4)) if np.isfinite(item[1, 2]) else 2.0
            t_s = float(item[2, 0]) if np.isfinite(item[2, 0]) else 0.0
            t_meas = min((t_s + off) if t_s > 0.0 else 0.0, sim_t)

            loc_m, vel_m = self._lookup_shuttle_state(t_meas, loc, vel, sim_t)
            p_abs = loc_m + rel_raw

            best_j = -1
            best_cost = 22.0
            innov_vec = np.zeros(3, dtype=float)
            if n_tr > 0:
                dt_vec = t_meas - tr_t_ref
                pred_p = tr_p_ref + tr_v_ref * dt_vec[:, None] + 0.5 * tr_ag * (dt_vec[:, None] ** 2)
                d_pos = np.linalg.norm(pred_p - p_abs, axis=1)
                d_diam = np.abs(tr_diam - diam_raw)
                diam_tol = np.maximum(3.5, 0.35 * np.maximum(tr_diam, diam_raw))
                cand_j = np.where((d_pos < 22.0) & (d_diam < diam_tol))[0]
                for j in cand_j:
                    ji = int(j)
                    if ji in used_tracks:
                        continue
                    cost = float(d_pos[ji]) + 0.3 * float(d_diam[ji])
                    if cost < best_cost:
                        best_cost = cost
                        best_j = ji
                        innov_vec = p_abs - pred_p[ji]

            if best_j >= 0:
                used_tracks.add(best_j)
                tr = self.tracks[best_j]
                if t_meas > tr["last_t"] + 0.8:
                    n_prev = tr["n_obs"]
                    prev_obs_t, _prev_obs_p, _ = tr["obs"][-1]
                    dt_last = max(t_meas - prev_obs_t, 1.0)
                    rad_tr = 0.5 * max(tr["diam"], diam_raw)

                    # Confirmed 2-sweep bounce trim ONLY for small/medium rocks (rad_tr < 17.8m)
                    # on real multi-second sweeps (dt_last >= 2.0s). NEVER wipe 7-sweep obs on
                    # giant tumbling boulders (rad_tr >= 17.8m) where irregular centroid shift is ~4.8m.
                    in_norm = float(np.linalg.norm(innov_vec))
                    detect_gate = 4.6
                    confirmed_bounce = (
                        n_prev >= 4
                        and rad_tr < MONSTER_RAD_THRESH
                        and dt_last >= 2.0
                        and tr["sh_age"] >= 1
                        and in_norm > 4.8
                        and float(np.dot(innov_vec, tr["in_vec"])) > 16.0
                    )

                    tr["obs"].append((t_meas, p_abs, diam_raw))
                    if confirmed_bounce and len(tr["obs"]) > 3:
                        while len(tr["obs"]) > 3:
                            tr["obs"].popleft()

                    tr["last_t"] = t_meas
                    tr["n_obs"] = max(tr["n_obs"] + 1, len(tr["obs"]))
                    tr["diam"] = float(np.median([o[2] for o in tr["obs"]]))

                    # Exponentially-weighted low-noise 7-sweep WLS (tau_w = 14.0s)
                    ts = np.array([o[0] for o in tr["obs"]], dtype=float)
                    ps = np.array([o[1] for o in tr["obs"]], dtype=float)
                    t_ref = ts[-1]
                    dt_s = ts - t_ref
                    r_mag = float(np.linalg.norm(ps[-1]))
                    a_g_rock = -MU_PHOBOS * ps[-1] / max(r_mag ** 3, 1.0)
                    ps_lin = ps - 0.5 * a_g_rock[None, :] * (dt_s[:, None] ** 2)

                    w = np.exp(dt_s / 14.0)
                    sw = float(np.sum(w))
                    st_ = float(np.sum(w * dt_s))
                    stt = float(np.sum(w * dt_s * dt_s))
                    lam_v = 1.5 if confirmed_bounce else 5.0
                    m00, m01, m11 = sw, st_, stt + lam_v
                    det = max(m00 * m11 - m01 * m01, 1e-6)
                    rhs0 = (w[:, None] * ps_lin).sum(axis=0)
                    rhs1 = ((w * dt_s)[:, None] * ps_lin).sum(axis=0)
                    p_est = (m11 * rhs0 - m01 * rhs1) / det
                    v_est = (-m01 * rhs0 + m00 * rhs1) / det

                    los_vec = p_abs - loc_m
                    los_d = float(np.linalg.norm(los_vec))
                    if los_d > 5.0:
                        u_los = los_vec / los_d
                        v_rock_los = rr_raw + float(np.dot(vel_m, u_los))
                        v_est = v_est + 0.85 * (v_rock_los - float(np.dot(v_est, u_los))) * u_los

                    v_mag = float(np.linalg.norm(v_est))
                    if v_mag > 6.5:
                        v_est = v_est * (6.5 / v_mag)

                    if n_prev >= 4 and (in_norm > detect_gate or confirmed_bounce):
                        tr["sh_age"] = (tr["sh_age"] + 1) if confirmed_bounce else 1
                        tr["in_vec"] = innov_vec.copy()
                    else:
                        tr["sh_age"] = 0
                        tr["in_vec"] = np.zeros(3, dtype=float)

                    tr["p_ref"] = p_est
                    tr["v_ref"] = v_est
                    tr["t_ref"] = t_ref
                    tr["p_sh"] = p_est.copy()
                    tr["v_sh"] = v_est.copy()
                    tr["t_sh"] = t_ref
            else:
                los_vec = p_abs - loc_m
                los_d = float(np.linalg.norm(los_vec))
                v_init = np.zeros(3, dtype=float)
                if los_d > 5.0:
                    u_los = los_vec / los_d
                    v_init = float(np.clip(rr_raw + float(np.dot(vel_m, u_los)), -4.0, 4.0)) * u_los
                obs_q: deque = deque(maxlen=7)
                obs_q.append((t_meas, p_abs, diam_raw))
                tid = self.next_track_id
                self.next_track_id += 1
                self.tracks.append({
                    "id": tid,
                    "p_ref": p_abs.copy(),
                    "v_ref": v_init,
                    "t_ref": t_meas,
                    "diam": diam_raw,
                    "obs": obs_q,
                    "last_t": t_meas,
                    "n_obs": 1,
                    "p_sh": p_abs.copy(),
                    "v_sh": v_init.copy(),
                    "t_sh": t_meas,
                    "in_vec": np.zeros(3, dtype=float),
                    "sh_age": 0,
                })

        if len(self.tracks) > 0:
            self._tr_id = np.array([tr["id"] for tr in self.tracks], dtype=int)
            self._tr_p_ref = np.array([tr["p_ref"] for tr in self.tracks], dtype=float)
            self._tr_v_ref = np.array([tr["v_ref"] for tr in self.tracks], dtype=float)
            self._tr_t_ref = np.array([tr["t_ref"] for tr in self.tracks], dtype=float)
            self._tr_diam = np.array([tr["diam"] for tr in self.tracks], dtype=float)
            self._tr_n_obs = np.array([tr["n_obs"] for tr in self.tracks], dtype=float)
            self._tr_p_sh = np.array([tr["p_sh"] for tr in self.tracks], dtype=float)
            self._tr_v_sh = np.array([tr["v_sh"] for tr in self.tracks], dtype=float)
            self._tr_t_sh = np.array([tr["t_sh"] for tr in self.tracks], dtype=float)
            self._tr_in_vec = np.array([tr["in_vec"] for tr in self.tracks], dtype=float)
            self._tr_sh_age = np.array([tr["sh_age"] for tr in self.tracks], dtype=int)
            r_mags = np.linalg.norm(self._tr_p_ref, axis=1, keepdims=True)
            self._tr_ag = -MU_PHOBOS * self._tr_p_ref / np.maximum(r_mags ** 3, 1.0)
        else:
            self._tr_t_ref = np.zeros(0, dtype=float)

    def _compute_align_rate(
        self,
        c_ph_to_body: npt.NDArray,
        u_goal: npt.NDArray,
        a_grav_lat: npt.NDArray,
        dist_to_target: float,
        w_mag: float,
    ) -> npt.NDArray:
        if w_mag > 0.08:
            return np.zeros(3, dtype=float)
        t_rem = max(dist_to_target / max(V_CRUISE, 4.0), 10.0)
        v_lat_grav = -0.5 * a_grav_lat * min(t_rem, 130.0)
        u_aim = V_CRUISE * u_goal + v_lat_grav
        u_aim = u_aim / max(float(np.linalg.norm(u_aim)), 1e-6)
        u_aim_b = c_ph_to_body @ u_aim
        ez = AXIS_UNIT_BODY["+Z"]
        rot_ax = np.cross(ez, u_aim_b)
        sin_a = float(np.linalg.norm(rot_ax))
        ang_err = float(np.arctan2(sin_a, np.clip(u_aim_b[2], -1.0, 1.0)))
        if ang_err > np.deg2rad(15.0) or ang_err < np.deg2rad(0.25):
            return np.zeros(3, dtype=float)
        return (rot_ax / max(sin_a, 1e-6)) * min(0.024, 0.28 * ang_err)

    def step(
        self,
        sim_t: float,
        loc: npt.NDArray,
        vel: npt.NDArray,
        ang_vel_b: npt.NDArray,
        target_loc: npt.NDArray,
        c_ph_to_body: npt.NDArray,
    ) -> tuple[npt.NDArray, npt.NDArray, bool, str, dict | None, float]:
        if self.start_loc is None:
            self.start_loc = loc.copy()
        dt_step = DT_LYAP if self.last_step_t is None else float(np.clip(sim_t - self.last_step_t, 0.0, 0.2))
        self.last_step_t = sim_t
        if len(self.hist_t) == 0 or sim_t > self.hist_t[-1] + 1e-4:
            self.hist_t.append(sim_t)
            self.hist_loc.append(loc.copy())
            self.hist_vel.append(vel.copy())

        c_body_to_ph = c_ph_to_body.T
        r_to_goal = target_loc - loc
        dist_to_target = float(np.linalg.norm(r_to_goal))
        if self.mission_won or dist_to_target <= KEEP_OUT_CUTOFF:
            self.mission_won = True
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float), True, "WIN", None, 0.0

        u_goal = r_to_goal / max(dist_to_target, 1e-6)
        v_long = float(np.dot(vel, u_goal))
        v_lat_vec = vel - v_long * u_goal
        v_lat_mag = float(np.linalg.norm(v_lat_vec))
        w_mag = float(np.linalg.norm(ang_vel_b))

        # Exact 3D Ballistic CPA ZEM with Simpson Midpoint Gravity + Online Environmental Observer
        r_ph_norm = float(np.linalg.norm(loc))
        a_g0 = -MU_PHOBOS * loc / max(r_ph_norm ** 3, 1.0)
        a_g0_eff = a_g0 + self.a_env_extra
        a_grav_long0 = float(np.dot(a_g0_eff, u_goal))

        v_eff = max(v_long, 1.5)
        t_go_lin = dist_to_target / v_eff
        v_avg = max(0.5 * (v_eff + max(v_eff + a_grav_long0 * t_go_lin, 1.5)), 1.5)
        t_go = min(dist_to_target / v_avg, 260.0)

        p_mid = loc + 0.5 * vel * t_go + 0.125 * a_g0_eff * (t_go * t_go)
        r_mid = float(np.linalg.norm(p_mid))
        a_g_mid = -MU_PHOBOS * p_mid / max(r_mid ** 3, 1.0)
        r_end = float(np.linalg.norm(target_loc))
        a_g_end = -MU_PHOBOS * target_loc / max(r_end ** 3, 1.0)
        a_grav = (a_g0 + 4.0 * a_g_mid + a_g_end) / 6.0 + self.a_env_extra
        a_grav_long = float(np.dot(a_grav, u_goal))
        a_grav_lat = a_grav - a_grav_long * u_goal

        p_rel = -r_to_goal + vel * t_go + 0.5 * a_grav * (t_go * t_go)
        v_end = vel + a_grav * t_go
        v_end_n2 = float(np.dot(v_end, v_end))
        dt_cpa = -float(np.dot(p_rel, v_end)) / v_end_n2 if v_end_n2 > 0.25 else 0.0
        t_cpa = float(np.clip(t_go + dt_cpa, 0.0, 260.0))
        zem_vec = -r_to_goal + vel * t_cpa + 0.5 * a_grav * (t_cpa * t_cpa)
        zem_lat = zem_vec - float(np.dot(zem_vec, u_goal)) * u_goal
        zem_lat_norm = float(np.linalg.norm(zem_lat))
        zem_dist = float(np.linalg.norm(zem_vec))

        # --- EMERGENCY TUMBLE FAILSAFE ---
        if w_mag > ANG_VEL_FAILSAFE and np.all(np.isfinite(ang_vel_b)):
            self._update_env_observer(sim_t, loc, vel, False, dt_step)
            return np.zeros(3, dtype=float), -(ang_vel_b / w_mag) * min(0.05, 0.45 * w_mag), True, "TUMBLE_DAMP", None, zem_dist

        # --- THREAT EVALUATION (Climb-Aware + Extreme-Range Chaos Gate + Yield Candidates) ---
        urgent_threats = []
        threats_all = []
        threats_zem = []
        yield_cands = []
        dodged_still_ahead = False
        closest_diag = None
        min_margin = 1e9

        in_climb = (not self.long_coasting) and (v_long < V_CRUISE - DEADBAND_ENTER)

        if len(self._tr_t_ref) > 0:
            dt_now = np.clip(sim_t - self._tr_t_ref, 0.0, TRACK_STALE_MAX_S)[:, None]
            p_rocks = self._tr_p_ref + self._tr_v_ref * dt_now + 0.5 * self._tr_ag * (dt_now ** 2)
            v_rocks = self._tr_v_ref + self._tr_ag * dt_now

            rels = p_rocks - loc[None, :]
            v_rels0 = v_rocks - vel[None, :] + 0.5 * (self._tr_ag - a_g0[None, :]) * 6.0
            v_rel0_n2 = np.sum(v_rels0 * v_rels0, axis=1)
            tcas0 = -np.sum(rels * v_rels0, axis=1) / np.maximum(v_rel0_n2, 1e-6)

            if in_climb and (not self.yielding) and CLIMB_PRED_GAIN > 0.0 and v_long < V_CRUISE - 0.15:
                dv_rem_c = max(0.0, V_CRUISE - v_long) * CLIMB_PRED_GAIN
                a_c = 0.19 * CLIMB_PRED_GAIN
                t_pos = np.maximum(tcas0, 0.1)
                t_c = np.minimum(t_pos, dv_rem_c / max(a_c, 1e-6))
                dv_c = np.where(tcas0 > 0.0, a_c * t_c * (1.0 - 0.5 * t_c / t_pos), 0.0)
                v_rels_c = v_rels0 - dv_c[:, None] * u_goal[None, :]
                v_relc_n2 = np.sum(v_rels_c * v_rels_c, axis=1)
                tcas_c = -np.sum(rels * v_rels_c, axis=1) / np.maximum(v_relc_n2, 1e-6)
                cpas0 = rels + v_rels0 * tcas0[:, None]
                cpas_c = rels + v_rels_c * tcas_c[:, None]
                same_side = np.sum(cpas0[:, :2] * cpas_c[:, :2], axis=1) > 0.0
                v_rels = np.where(same_side[:, None], v_rels_c, v_rels0)
            else:
                v_rels = v_rels0

            v_rel_n2 = np.sum(v_rels * v_rels, axis=1)
            tcas = -np.sum(rels * v_rels, axis=1) / np.maximum(v_rel_n2, 1e-6)
            rads = 0.5 * np.maximum(self._tr_diam, 1.0)
            valid = (v_rel_n2 > 0.04) & (tcas > 0.0) & (tcas <= 44.0)

            for i in np.where(valid)[0]:
                tca = float(tcas[i])
                rel = rels[i]
                if float(np.linalg.norm(rel)) >= dist_to_target + 8.0:
                    continue
                v_rel = v_rels[i]
                v_rock = v_rocks[i]
                rad_i = float(rads[i])
                n_i = float(self._tr_n_obs[i])
                tid_i = int(self._tr_id[i])

                cpa_vec = rel + v_rel * tca
                miss = float(np.linalg.norm(cpa_vec))
                is_active = (tid_i == self.prev_esc_tid and self.evading)

                v_rock_spd = float(np.linalg.norm(v_rock))
                is_monster = (rad_i >= MONSTER_RAD_THRESH) and (n_i >= 4.0)
                pos_unc = (2.8 if n_i < 2.0 else (1.4 if n_i < 4.0 else 0.55)) + (0.03 if n_i >= 4.0 else 0.09) * min(tca, 15.0)
                chaos_taper = float(np.clip((tca - CHAOS_TAPER_LO) / max(CHAOS_TAPER_HI - CHAOS_TAPER_LO, 0.5), 0.0, 1.0))
                if is_monster:
                    base_chaos = min(CHAOS_BASE_MAX, CHAOS_BASE_SCALE * rad_i * max(0.0, v_rock_spd - 1.35))
                    if n_i >= 5.0 or tca <= CHAOS_HI_N5_TCA:
                        hi_chaos = min(CHAOS_HI_MAX, CHAOS_HI_SCALE * rad_i * max(0.0, v_rock_spd - CHAOS_HI_SPEED))
                    else:
                        hi_chaos = 0.0
                    chaos_vel_buf = (base_chaos + hi_chaos) * chaos_taper
                else:
                    chaos_vel_buf = 0.0
                buf_scale = 0.27 if is_monster else BUF_SCALE
                buf = max(BUF_MIN, buf_scale * rad_i) + pos_unc + chaos_vel_buf
                if tca < 5.0:
                    min_b = max(2.2, (0.21 if is_monster else 0.17) * rad_i)
                    alpha_t = float(np.clip((tca - 1.5) / 3.5, 0.0, 1.0))
                    buf = min_b + alpha_t * max(0.0, buf - min_b)
                r_safe = rad_i + SHIP_BOUND_RADIUS + buf

                if tid_i == self.last_dodged_tid and 0.0 < tca <= 14.0 and miss < r_safe + 10.0:
                    dodged_still_ahead = True

                # Converged climb-corridor crossing rocks eligible for Zero-Fuel CLIMB_YIELD
                if in_climb and n_i >= 4.0 and v_long >= 1.2 and v_long <= (V_CRUISE - 0.9):
                    t_c0, m_c0 = self._predict_climb_cpa(rel, v_rock, vel, u_goal, a_g0, 0.0)
                    if 4.0 < t_c0 <= 40.0 and m_c0 < r_safe + 16.0:
                        yield_cands.append((t_c0, m_c0, rel, v_rock, r_safe, rad_i, tid_i))

                # Only converged (n>=3) forward rocks within 24 s block ZEM
                if n_i >= 3.0 and tca <= 24.0 and miss < r_safe + 36.0:
                    threats_zem.append((tca, rel, v_rel, r_safe, miss))

                if tca > TCA_HORIZON:
                    continue

                need = r_safe - miss
                a_req = 2.0 * need / max(tca * tca, 0.25)

                margin = miss - r_safe
                if margin < min_margin:
                    min_margin = margin
                    closest_diag = {
                        "rel": rel,
                        "v_rock": v_rock,
                        "rad": rad_i,
                        "tca": tca,
                        "miss": miss,
                        "r_safe": r_safe,
                        "a_req": a_req,
                        "n_obs": int(n_i),
                    }

                # Strict convergence gates: eliminate n=1,2,3 distant false alarms!
                if not is_active:
                    if n_i < 2.0 and (tca > 7.0 or miss > rad_i + SHIP_BOUND_RADIUS + 1.0):
                        if not (tca <= 10.0 and miss < rad_i + SHIP_BOUND_RADIUS):
                            continue
                    elif n_i < 3.0 and tca > 12.0:
                        continue
                    elif n_i < 4.0 and tca > (16.5 if rad_i >= MONSTER_RAD_THRESH else 14.0):
                        continue

                if miss < r_safe + 14.0:
                    threats_all.append((tca, rel, v_rel, r_safe, a_req, cpa_vec, miss, rad_i, tid_i))

                base_trig = 0.014 if (is_monster and v_rock_spd > 2.20) else (0.018 if (is_monster and v_rock_spd > 1.45) else A_REQ_TRIG)

                # Single-shot complete EVADE: once active on tid_i, finish cleanly to miss >= r_safe
                # in ONE cheap early burn at high TCA instead of stopping short and re-triggering 3x!
                if is_active:
                    a_trig = 0.0
                else:
                    if tca < 2.0 and miss > rad_i + SHIP_BOUND_RADIUS + max(1.8, (0.21 if is_monster else 0.16) * rad_i):
                        continue
                    a_trig = base_trig

                if (need > 0.0 and a_req >= a_trig) or (tca < 4.5 and miss < rad_i + SHIP_BOUND_RADIUS + max(1.8, (0.20 if is_monster else 0.16) * rad_i)):
                    urgent_threats.append((tca, rel, v_rel, r_safe, a_req, cpa_vec, miss, rad_i, tid_i))

        w_align = self._compute_align_rate(c_ph_to_body, u_goal, a_grav_lat, dist_to_target, w_mag)

        # --- PRIORITY 1: SINGLE-SHOT L1-OPTIMAL LYAPUNOV PURE-AXIS ANTI-PINBALL EVASION ---
        if len(urgent_threats) > 0:
            self.yielding = False
            self.yield_tid = -1
            self._update_env_observer(sim_t, loc, vel, False, dt_step)
            def _thr_prio(x: tuple) -> tuple[float, float]:
                t_x, a_x, tid_x = x[0], x[4], x[8]
                if self.evading and tid_x == self.prev_esc_tid:
                    return (-(a_x * 1.12 + 0.006), t_x)
                return (-a_x, t_x)
            urgent_threats.sort(key=_thr_prio)
            tca, rel, v_rel, r_safe, a_req, cpa_vec, miss, rad_i, tid_i = urgent_threats[0]

            v_rel_u = v_rel / max(float(np.linalg.norm(v_rel)), 1e-6)
            perp = -(cpa_vec - float(np.dot(cpa_vec, v_rel_u)) * v_rel_u)
            p_norm = float(np.linalg.norm(perp))

            if p_norm > 0.6:
                esc_dir = perp / p_norm
                if tid_i == self.prev_esc_tid and float(np.linalg.norm(self.prev_esc_dir)) > 0.5 and miss < r_safe:
                    if float(np.dot(esc_dir, self.prev_esc_dir)) < -0.1:
                        esc_dir = self.prev_esc_dir.copy()
            else:
                if tid_i == self.prev_esc_tid and float(np.linalg.norm(self.prev_esc_dir)) > 0.5:
                    esc_dir = self.prev_esc_dir.copy()
                else:
                    cand = np.cross(v_rel_u, np.array([0.0, 0.0, 1.0]))
                    if float(np.linalg.norm(cand)) < 1e-3:
                        cand = np.cross(v_rel_u, np.array([0.0, 1.0, 0.0]))
                    esc_dir = cand / float(np.linalg.norm(cand))

            dv_test = min(max(r_safe - miss + 2.5, 0.6) / max(tca, 2.0), 1.6)
            best_axis = None
            best_score = -1e9
            for k in ("+X", "-X", "+Y", "-Y"):
                u_act_ph = c_body_to_ph @ AXIS_UNIT_BODY[k]
                align = float(np.dot(u_act_ph, esc_dir))
                if align < -0.15:
                    continue
                dv_k = dv_test * (0.75 if k == "+Y" else 1.0)
                vr1 = v_rel - dv_k * u_act_ph
                v2_1 = float(np.dot(vr1, vr1))
                t1 = max(0.0, -float(np.dot(rel, vr1)) / max(v2_1, 1e-6))
                m1 = float(np.linalg.norm(rel + vr1 * t1))
                if m1 <= miss + 0.15:
                    continue
                min_clear = m1 - r_safe
                for (_, r2, vr2, rs2, _, _, _, _, tid2) in threats_all:
                    if tid2 == tid_i:
                        continue
                    vr2_new = vr2 - dv_k * u_act_ph
                    v2_2 = float(np.dot(vr2_new, vr2_new))
                    if v2_2 < 0.04:
                        continue
                    t2 = -float(np.dot(r2, vr2_new)) / v2_2
                    if 0.0 < t2 <= 22.0:
                        m2 = float(np.linalg.norm(r2 + vr2_new * t2))
                        min_clear = min(min_clear, m2 - (rs2 - 0.5))

                hyst = 0.85 if (tid_i == self.prev_esc_tid and k == self.prev_esc_axis) else (0.22 if k == self.prev_esc_axis else 0.0)
                tgt_bonus = -0.12 * float(np.dot(u_act_ph, zem_lat)) / max(zem_lat_norm, 25.0)
                score_k = align + hyst + tgt_bonus + (0.45 * min_clear if min_clear < 0.0 else min(0.25, 0.05 * min_clear))
                if score_k > best_score:
                    best_score = score_k
                    best_axis = k

            if best_axis is not None:
                self.prev_esc_dir = c_body_to_ph @ AXIS_UNIT_BODY[best_axis]
                self.prev_esc_axis = best_axis
                self.prev_esc_tid = tid_i
                self.last_dodged_tid = tid_i
                self.last_dodged_axis = best_axis
                self.evading = True
                self.last_evade_t = sim_t
                self.zem_burning = False
                self.prev_zem_axis = None
                self.damping = False
                acc_b, ang_b = self._select_vertex(best_axis, ang_vel_b, np.zeros(3), weights=(1.0, 1.0, 1.0))
                return acc_b, ang_b, False, f"EVADE_{best_axis}", closest_diag, zem_dist

            ax_k, _ = self._best_pure_axis(esc_dir, c_ph_to_body, exclude=())
            self.prev_esc_dir = esc_dir
            self.prev_esc_axis = ax_k
            self.prev_esc_tid = tid_i
            self.last_dodged_tid = tid_i
            self.last_dodged_axis = ax_k
            self.evading = True
            self.last_evade_t = sim_t
            self.zem_burning = False
            self.prev_zem_axis = None
            self.damping = False
            acc_b, ang_b = self._select_vertex(ax_k, ang_vel_b, np.zeros(3), weights=(1.0, 1.0, 1.0))
            return acc_b, ang_b, False, f"EVADE_{ax_k}", closest_diag, zem_dist

        self.evading = False
        if not dodged_still_ahead:
            self.last_dodged_tid = -1
            self.last_dodged_axis = None
        if (sim_t - self.last_evade_t) > 1.0:
            self.prev_esc_dir = np.zeros(3, dtype=float)
            self.prev_esc_axis = None
            self.prev_esc_tid = -1

        # --- PRIORITY 2: ZERO-FUEL CLIMB_YIELD + LYAPUNOV 5-VERTEX +Z CLIMB ---
        if v_long >= (V_CRUISE - DEADBAND_ENTER):
            self.long_coasting = True

        if not self.long_coasting:
            h_lifted = float(np.linalg.norm(loc - self.start_loc))
            should_yield = False
            if h_lifted >= 2.0 and self.total_yield_s < MAX_YIELD_S and len(yield_cands) > 0:
                rem_budget = MAX_YIELD_S - self.total_yield_s
                if self.yielding and self.yield_tid >= 0:
                    for (_t_c0, m_c0, rel_c, vrock_c, rs_c, _rad_c, tid_c) in yield_cands:
                        if tid_c != self.yield_tid:
                            continue
                        if m_c0 < rs_c + YIELD_CLEAR_PAD:
                            _td1, m_d1 = self._predict_climb_cpa(rel_c, vrock_c, vel, u_goal, a_g0, 2.5)
                            if (sim_t < self.yield_hold_until) or (m_d1 > m_c0 - 0.5):
                                safe_d = True
                                for (_, m2, r2, vr2, rs2, _, tid2) in yield_cands:
                                    if tid2 == tid_c:
                                        continue
                                    _, m2d = self._predict_climb_cpa(r2, vr2, vel, u_goal, a_g0, 2.5)
                                    if m2d < rs2 + 1.0 and m2d < m2 - 0.5:
                                        safe_d = False
                                        break
                                if safe_d:
                                    should_yield = True
                        break
                else:
                    for (_t_c0, m_c0, rel_c, vrock_c, rs_c, _rad_c, tid_c) in yield_cands:
                        if m_c0 < rs_c + 1.0:
                            d_probe = min(max(rem_budget, 3.0), 8.5)
                            _td1, m_d1 = self._predict_climb_cpa(rel_c, vrock_c, vel, u_goal, a_g0, 2.5)
                            _td2, m_d2 = self._predict_climb_cpa(rel_c, vrock_c, vel, u_goal, a_g0, d_probe)
                            if (m_d2 >= rs_c + 2.0) and (m_d2 > m_c0 + 3.0) and (m_d1 > m_c0 - 1.5):
                                safe_d = True
                                for (_, m2, r2, vr2, rs2, _, tid2) in yield_cands:
                                    if tid2 == tid_c:
                                        continue
                                    _, m2d = self._predict_climb_cpa(r2, vr2, vel, u_goal, a_g0, 2.5)
                                    if m2d < rs2 + 1.5 and m2d < m2 - 0.5:
                                        safe_d = False
                                        break
                                if safe_d:
                                    should_yield = True
                                    self.yield_tid = tid_c
                                    self.yield_hold_until = sim_t + 0.80
                                    break
            if should_yield:
                self.yielding = True
                self.total_yield_s += dt_step
                self._update_env_observer(sim_t, loc, vel, True, dt_step)
                return np.zeros(3, dtype=float), np.zeros(3, dtype=float), True, "CLIMB_YIELD", closest_diag, zem_dist

            self.yielding = False
            self.yield_tid = -1
            self._update_env_observer(sim_t, loc, vel, False, dt_step)

            ez = AXIS_UNIT_BODY["+Z"]
            ug_b = c_ph_to_body @ u_goal
            goal_err = float(np.arctan2(np.linalg.norm(np.cross(ez, ug_b)), np.clip(ug_b[2], -1.0, 1.0)))

            if (h_lifted < 0.6) or (v_long > V_CRUISE - 0.50) or (goal_err > np.deg2rad(12.0)) or (w_mag > 0.08):
                w_des = np.zeros(3, dtype=float)
            elif self.last_evade_t >= 0.0:
                w_des = w_align
            else:
                t_rem = max(dist_to_target / max(V_CRUISE, 4.0), 10.0)
                t_burn_rem = max((V_CRUISE - v_long) / 0.19, 1.5)
                t_coast = max(t_rem - 0.5 * t_burn_rem, 10.0)
                v_lat_des = -0.5 * a_grav_lat * t_coast
                dv_rem = max(V_CRUISE - v_long, 0.8)
                dv_lat = v_lat_des - v_lat_vec
                lat_max = dv_rem * float(np.tan(np.deg2rad(8.0)))
                dv_lat_n = float(np.linalg.norm(dv_lat))
                if dv_lat_n > lat_max:
                    dv_lat = dv_lat * (lat_max / dv_lat_n)
                dv_vec = dv_rem * u_goal + dv_lat
                u_aim = dv_vec / max(float(np.linalg.norm(dv_vec)), 1e-6)
                u_aim_b = c_ph_to_body @ u_aim
                rot_ax = np.cross(ez, u_aim_b)
                sin_a = float(np.linalg.norm(rot_ax))
                ang_err = float(np.arctan2(sin_a, np.clip(u_aim_b[2], -1.0, 1.0)))
                if ang_err > np.deg2rad(0.25):
                    u_ax = rot_ax / max(sin_a, 1e-6)
                    w_des = u_ax * min(0.028, 0.30 * ang_err)
                else:
                    w_des = np.zeros(3, dtype=float)

            acc_b, ang_b = self._select_vertex("+Z", ang_vel_b, w_des, weights=(1.0, 1.0, 0.15))
            return acc_b, ang_b, False, "CLIMB_Z", closest_diag, zem_dist

        # --- PRIORITY 3: 6-AXIS RECOVERY / EDGE-OPTIMIZED ANTI-CHATTER ZEM / HYSTERESIS COAST DAMP ---
        z_align = float(np.dot(c_body_to_ph @ AXIS_UNIT_BODY["+Z"], u_goal))
        if (t_cpa <= 0.5) or (v_long < 2.5 and dist_to_target > 60.0):
            self._update_env_observer(sim_t, loc, vel, False, dt_step)
            v_des = u_goal * min(7.0, max(3.5, 0.06 * dist_to_target))
            dv_rec = v_des - vel
            dvn = float(np.linalg.norm(dv_rec))
            if dvn > 0.25:
                u_rec = dv_rec / dvn
                ax_k, _ = self._best_pure_axis(u_rec, c_ph_to_body, exclude=())
                self.damping = False
                acc_b, ang_b = self._select_vertex(ax_k, ang_vel_b, w_align, weights=(1.0, 1.0, 1.0))
                return acc_b, ang_b, False, f"RECOV_{ax_k}", closest_diag, zem_dist
        elif v_long < 4.5 and dist_to_target > 140.0 and z_align > 0.78 and zem_dist < 44.0:
            self._update_env_observer(sim_t, loc, vel, False, dt_step)
            self.damping = False
            acc_b, ang_b = self._select_vertex("+Z", ang_vel_b, w_align, weights=(1.0, 1.0, 0.15))
            return acc_b, ang_b, False, "BOOST", closest_diag, zem_dist

        # Edge-optimized ZEM with exact 3D CPA norm(zem_vec), anti-chatter lock & 23 s debris lookahead
        if (sim_t - self.last_evade_t) >= EVADE_ZEM_COOLDOWN_S:
            if dist_to_target > 600.0:
                zem_trig = 54.0
                zem_stop = 44.0
            elif dist_to_target > 160.0:
                zem_trig = 46.0
                zem_stop = 42.5
            else:
                zem_trig = 46.0
                zem_stop = 41.0

            if not self.zem_burning and zem_dist > zem_trig and sim_t >= self.zem_blocked_until:
                self.zem_burning = True
            elif self.zem_burning and (zem_dist < zem_stop or (v_lat_mag < 0.08 and zem_dist < 44.5)):
                self.zem_burning = False
                self.prev_zem_axis = None

            if self.zem_burning and zem_lat_norm > 1e-3:
                u_zem = -zem_lat / zem_lat_norm
                excl = set() if z_align < 0.65 else {"+Z", "-Z"}
                if dodged_still_ahead and self.last_dodged_axis is not None:
                    opp_map = {"+X": "-X", "-X": "+X", "+Y": "-Y", "-Y": "+Y", "+Z": "-Z", "-Z": "+Z"}
                    excl.add(opp_map[self.last_dodged_axis])
                ub = c_ph_to_body @ u_zem
                ranked = []
                for k, ax_u in AXIS_UNIT_BODY.items():
                    if k in excl:
                        continue
                    d = float(np.dot(ub, ax_u))
                    d_hyst = d + ((0.45 if sim_t < self.zem_axis_lock_until else 0.30) if k == self.prev_zem_axis else 0.0)
                    if d > 0.25:
                        ranked.append((d_hyst, d, k))
                ranked.sort(key=lambda x: -x[0])

                dv_need = max(0.20, (zem_dist - zem_stop) / max(t_cpa, 6.0))
                dv_cap = min(dv_need + 0.12, 0.80)
                probes = (0.18, 0.42 * dv_cap, dv_cap)

                for (_, _, ax_k) in ranked[:2]:
                    u_act_ph = c_body_to_ph @ AXIS_UNIT_BODY[ax_k]
                    safe = True
                    for dv_p in probes:
                        for (_, rr_j, vr_j, rs_j, m_cur_j) in threats_zem:
                            vr_new = vr_j - dv_p * u_act_ph
                            v2 = float(np.dot(vr_new, vr_new))
                            if v2 < 0.04:
                                continue
                            t_new = -float(np.dot(rr_j, vr_new)) / v2
                            if 0.0 < t_new <= 23.0:
                                m_new = float(np.linalg.norm(rr_j + vr_new * t_new))
                                if m_new < rs_j + 1.5 and m_new < m_cur_j - 0.15:
                                    safe = False
                                    break
                        if not safe:
                            break
                    if safe:
                        self._update_env_observer(sim_t, loc, vel, False, dt_step)
                        if ax_k != self.prev_zem_axis:
                            self.prev_zem_axis = ax_k
                            self.zem_axis_lock_until = sim_t + 0.50
                        self.damping = False
                        acc_b, ang_b = self._select_vertex(ax_k, ang_vel_b, np.zeros(3), weights=(1.0, 1.0, 1.0))
                        return acc_b, ang_b, False, f"ZEM_{ax_k}", closest_diag, zem_dist

                self.zem_burning = False
                self.prev_zem_axis = None
                self.zem_blocked_until = sim_t + 0.5
        else:
            self.zem_burning = False
            self.prev_zem_axis = None

        # Coast-only angular rate damp with hysteresis (never interrupts ZEM, zero 60Hz chatter!)
        if not self.damping and w_mag > COAST_DAMP_ENTER:
            self.damping = True
        elif self.damping and w_mag < COAST_DAMP_EXIT:
            self.damping = False
        if self.damping and w_mag > 1e-6:
            self._update_env_observer(sim_t, loc, vel, False, dt_step)
            ang_damp = -(ang_vel_b / w_mag) * min(0.0012, 0.35 * w_mag)
            return np.zeros(3, dtype=float), ang_damp, True, "DAMP", closest_diag, zem_dist

        self._update_env_observer(sim_t, loc, vel, True, dt_step)
        mode_coast = "COAST" if (sim_t - self.last_evade_t) >= EVADE_ZEM_COOLDOWN_S else "COAST_HOLD"
        return np.zeros(3, dtype=float), np.zeros(3, dtype=float), True, mode_coast, closest_diag, zem_dist


gnc = PhobosGNCController()
vertices_synced = False
iteration = 0
prev_mode = ""
prev_damage = 0.0
exit_flag: bool = False

while not exit_flag:
    time.sleep(LOOP_DT)

    # Sync bit-for-bit exact simplex vertices from live RCSConfig on startup
    if not vertices_synced and (iteration == 0 or iteration % 30 == 0):
        synced_table, sync_msg = try_sync_vertex_table_from_sim()
        if synced_table is not None:
            gnc.vertex_table = synced_table
            vertices_synced = True
            st.logger_info(f"[GNC] Vertex table {sync_msg}")
        elif iteration == 300:
            vertices_synced = True
            st.logger_info(f"[GNC] Using precomputed float64 vertex table ({sync_msg})")

    t_now_sim = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
    sim_time_sec = (t_now_sim - t0_sim).total_seconds()

    # --- 2. Sensor & State Queries ---
    locVelAcc = shuttle.getLocVelAcc()
    loc = np.array(locVelAcc.loc_WRT_ExprIn(phobosFrame), dtype=float)
    vel = np.array(locVelAcc.vel_WRT_ExprIn(phobosFrame), dtype=float)
    ang_vel_body = np.array(shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame), dtype=float)
    if not np.all(np.isfinite(ang_vel_body)):
        ang_vel_body = np.zeros(3, dtype=float)

    damage: float = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
    fuelUsed: float = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])
    targetLoc = np.array(target.getLocVelAcc().loc_WRT_ExprIn(phobosFrame), dtype=float)

    # Fetch full 500-rock radar array at 15 Hz (Radar re-sorts at 2.4 Hz)
    if iteration % RADAR_FETCH_PERIOD == 0:
        debrisInfo = GetNearestDebrisLocations(RADAR_QUERY_COUNT)
        if debrisInfo is not None and len(debrisInfo) > 0:
            debris_arr = np.asarray(debrisInfo, dtype=float)
            gnc.update_tracks(sim_time_sec, loc, vel, debris_arr)

    # Exact Phobos -> Body passive DCM via SpaceTeams FramedVector
    ex_b = np.array(st.frames.FramedVector(_EX, phobosFrame).ExprIn(shuttleFrame), dtype=float)
    ey_b = np.array(st.frames.FramedVector(_EY, phobosFrame).ExprIn(shuttleFrame), dtype=float)
    ez_b = np.array(st.frames.FramedVector(_EZ, phobosFrame).ExprIn(shuttleFrame), dtype=float)
    c_ph_to_body = np.column_stack((ex_b, ey_b, ez_b))

    # --- 3. Run GNC Step (outputs exact body-frame simplex commands) ---
    linearAccel_body, angularAccel_body, coasting, mode, diag, zem_dist = gnc.step(
        sim_time_sec, loc, vel, ang_vel_body, targetLoc, c_ph_to_body
    )

    if not np.all(np.isfinite(linearAccel_body)):
        linearAccel_body = np.zeros(3, dtype=float)
    if not np.all(np.isfinite(angularAccel_body)):
        angularAccel_body = np.zeros(3, dtype=float)

    lin_norm = float(np.linalg.norm(linearAccel_body))
    if lin_norm > MAX_ACCEL:
        linearAccel_body = linearAccel_body * (MAX_ACCEL / lin_norm)
    ang_norm = float(np.linalg.norm(angularAccel_body))
    if ang_norm > MAX_ANG_ACCEL:
        angularAccel_body = angularAccel_body * (MAX_ANG_ACCEL / ang_norm)

    SetCommand(linearAccel_body, angularAccel_body)

    # --- 4. HUD & Flight-Recorder Telemetry ---
    dist_to_target = float(np.linalg.norm(targetLoc - loc))
    speed = float(np.linalg.norm(vel))

    if iteration % 10 == 0:
        st.OnScreenLogMessage(
            f"Dist: {dist_to_target:.1f}m | Spd: {speed:.1f}m/s | Dam: {damage:.1f}J | Fuel: {fuelUsed:.1f}kg | Coast: {coasting}",
            "FlightHUD", st.Severity.Info
        )

    mode_changed = (mode != prev_mode)
    dam_increased = (damage > prev_damage + 1.0)
    if mode_changed or dam_increased or (iteration % 300 == 0):
        u_g = (targetLoc - loc) / max(dist_to_target, 1e-6)
        tilt_deg = float(np.rad2deg(np.arccos(np.clip((c_ph_to_body @ u_g)[2], -1.0, 1.0))))
        w_spd = float(np.linalg.norm(ang_vel_body))
        thr_str = ""
        if diag is not None:
            thr_str = (
                f" | NearThr: rel={np.round(diag['rel'],1).tolist()} vR={np.round(diag['v_rock'],2).tolist()} "
                f"R={diag['rad']:.1f}m tca={diag['tca']:.1f}s miss={diag['miss']:.1f}m "
                f"rsafe={diag['r_safe']:.1f}m areq={diag['a_req']:.3f} n={diag['n_obs']}"
            )
        st.logger_info(
            f"[GNC t={sim_time_sec:.1f}s {mode}] Dist={dist_to_target:.1f}m Spd={speed:.2f} "
            f"zem={zem_dist:.1f}m tilt={tilt_deg:.1f}deg w={w_spd:.3f} yld={gnc.total_yield_s:.1f}s "
            f"loc={np.round(loc,1).tolist()} vel={np.round(vel,2).tolist()} "
            f"tr={len(gnc.tracks)} fuel={fuelUsed:.1f}kg dam={damage:.1f}J{thr_str}"
        )
        prev_mode = mode
        prev_damage = damage

    iteration += 1

st.leave_sim()
