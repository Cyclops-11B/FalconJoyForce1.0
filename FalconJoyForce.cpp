// Falcon - Mouse emulation joystick with Force
// Novint Falcon -> Pico 2 W (CP2102 serial) -> Xbox 360 controller

#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <dhdc.h>
#include <string.h>

#pragma comment(lib, "winmm.lib")


// ── Serial port ────────────────────────────────────────────────────────────
static const char* SERIAL_PORT = "\\\\.\\COM6";
static const DWORD SERIAL_BAUD = 115200;

// ── Packet constants ───────────────────────────────────────────────────────
static const BYTE PACKET_IN_MAGIC = 0xAA;
static const BYTE PACKET_OUT_MAGIC = 0xBB;

// ── Controller config ──────────────────────────────────────────────────────
static const int UPDATE_RATE_MS = 0;  // 0 is no frame wait; 1 is 1-2 frame wait; 8 = ~125Hz
static const bool   INVERT_X = false;
static const bool   INVERT_Y = false;

// ──Velocity settings ──────────────────────────────────────────────────────
static const double VEL_DEADZONE = 0.0001;
static const double VEL_VLOW_SENS = 22.0;    // sensitivity for very slow corrections
static const double VEL_VLOW_CURVE = 0.80;   // closer to 1.0 = more linear, less boosted
static const double VEL_LOW_SENS = 18.0;  // sensitivity at low speeds
static const double VEL_LOW_CURVE = 0.88; // curve for slow zone (lower = more boost)
static const double VEL_HIGH_SENS = 12.0; // sensitivity at high speeds
static const double VEL_HIGH_CURVE = 0.95; // curve for fast zone (1.0 = linear)
static const double VEL_BLEND_VLOW = 0.004;  // below this = pure vlow zone
static const double VEL_BLEND_LOW = 0.025; // below this = low speed
static const double VEL_BLEND_HIGH = 0.15; // above this = high speed
static const double SOFT_RAMP = 0.004;  // m/s width of ramp zone
static const double SHORT_VEL_NOISE = 0.0005;  // higher suppresses more low-speed noise
static const double VEL_VLOW_POS_SCALE = 7.0;  // position error scale
double posBlendMax = VEL_BLEND_LOW * 2.0;
static const double VEL_RELEASE_MULT = 35.0; // decel cutoff multiplier — higher = snappier stop, lower = floatier
static const double VEL_VERTICAL_SCALE = 1.0; // up/down (Z axis) velocity-zone output multiplier; left/right (Y axis) is unaffected. Independent of PUSH_VERTICAL_SCALE

// Flick capture: bank fast motion when saturated
static const double FLICK_VEL_ON = 0.01;   // m/s above which motion is treated as a flick and banked (≈ where the stick saturates)
static const double FLICK_GAIN = 4.0;    // how much banked over-speed becomes extra deflection
static const double FLICK_DECAY = 0.92;   // per-ms decay of the carry tail — lower = shorter tail
static const double FLICK_CARRY_MAX = 0.9;    // clamp on carry deflection so big flicks don't over-throw

// Push zone variables
static const double PUSH_ENTER_RAD = 0.56; // percentage of work radius where push zone kicks in
static const double PUSH_EXIT_RAD = 0.56; // percentage of work radius where push zone stops acting
static const double PUSH_SPEED_BASE = 0.44; // how fast cursor moves when entering push zone
static const double PUSH_SPEED_MAX = 6.0; // maximum push speed at full tilt
static const double PUSH_VERTICAL_SCALE = 0.5; // up/down (Z axis) push-speed multiplier; left/right (Y axis) is unaffected
static const double HALF_RANGE_MAX = 0.06; // how large work radius is (in m)

// Velocity zone stuff
static const double VEL_FC_SPEED_CEIL = 0.06; // velocity ceiling for top smoothing bracket (m/s) — NOT the work area
static const double CALIB_FILL = 0.95;        // fraction of measured reach that maps to full deflection
static const double CALIB_MIN = 0.025;       // reject implausibly small sweeps (m)
static const double CALIB_MAX = 0.120;       // reject implausibly large sweeps (m)

// Push zone re-entry damping
static const double PUSH_DAMP_COEFF = 0.9; // damping strength on direction reversal (0=none, 1=strong)
static const double PUSH_DAMP_DECAY = 3.0; // how quickly damping fades (higher = faster)

// ── Boundary detent: a ridge you push over right before the push zone ─────────
static const double DETENT_WIDTH = 0.12; // fraction of work radius over which the ridge builds
static const double DETENT_PEAK_N = 1.6;  // ridge height in N — resistance grows, then releases at threshold
static const double DETENT_VEL_HYST = 0.003; // m/s direction threshold — detent arms moving out, suppresses moving back in

// ── Exit pump: a brief inward kick right at the border when leaving the push zone ──
static const double PUMP_PEAK_N = 1.3;  // kick strength (N) as you cross back out of push
static const double PUMP_DECAY = 0.90;  // per-ms decay of the kick — lower = snappier/shorter pump

//  Feed forward variables ────────────────────────────────────────────────────────
static const double FRICTION_CANCEL = .14;    // feed forward force in Newtons
static const double FRICTION_VEL_MIN = 0.0006; // engage almost as soon as motion starts
static const double FRICTION_VEL_FULL = 0.004;  // assist reaches full here (fixed window, not tied to MIN)
static const double FRICTION_VEL_MAX = 0.08;   // faded back out by here

static const double STICTION_BREAK_N = 0.09;   // constant breakaway force (N)
static const double STICTION_VEL_ON = 0.0005; // motion floor (= your SHORT_VEL_NOISE)

// ── Button 2 brace / preload ─────────────────────────────────────────────────
static const double BTN2_X_FORCE = 1.6;  // constant X force while button 2 held (N) — preloads mechanism for fine corrections
static const double BTN2_X_SIGN = 1.0;  // +1 or -1 to flip push direction

//  Spring box ─────────────────────────────────────────────────────────────────────
static const double FORCE_SPRING_START = 0.5; // percentage of work radius where spring force starts
static const double FORCE_MAX_RAD = 0.88; // percentage of work radius where max force is achieved
static const double FORCE_MAX_N = 8; // maximum allowable Force (in N)
static const double FORCE_DAMPING = 1.0; // cut down on springiness
static const double FORCE_EXPONENT = 2.3; // how you ramp to max force. lower = force builds earlier and harder

//  Gravity Compensation ────────────────────────────────────────────────────────────
static const double GRAVITY_COMP_SCALE = 1.65;  // 1.0 = normal, 1.2 = 20% more, 0.8 = 20% less

// ──Ambient Rumble settings ──────────────────────────────────────────────────────
static const double AMBIENT_LARGE_SCALE = 5.5;  // random rumble force scale
static const double AMBIENT_SMALL_SCALE = 5.5;  // random rumble force scale
static const double RUMBLE_DECAY = 0.60;
static const double RUMBLE_FORCE_SCALE = 2.0; // overall scale factor of rumble force
static const double AMBIENT_TAIL_QUIET = 0.05;        // ambient is held off through the whole recoil rumble tail; the tail is "over" (ambient allowed again) once incoming rumble decays below this — adapts to large shots' longer tails instead of a fixed time window

// ──Recoil settings ──────────────────────────────────────────────────────
static const double RUMBLE_LARGE_SCALE = 2.4;  // recoil force scale
static const double RUMBLE_SMALL_SCALE = 2.4;  // recoil force scale
static const DWORD  RECOIL_WINDOW_MS = 250; // ms after btn 1 release to still catch trigger recoil
static const double RECOIL_SUSTAIN_THRESHOLD = 0.5;  // liveMag above this = sustaining
static const double RECOIL_CURVE = 0.50; // Recoil compressor -  0.3 boosts small recoils; 1.0 = linear
static const double RECOIL_DECAY = 0.45;
static const double RECOIL_DECAY_MIN = 0.25;  // decay for weak shots (fast cutoff)
static const double RECOIL_DECAY_MAX = 0.75;  // decay for strong shots (long sustain)
static const double RECOIL_MAG_SCALE = 20.0;  // liveMag value that maps to DECAY_MAX
static const double RECOIL_AIM_DAMP = 0.4;  // stick sensitivity multiplier during recoil (0=frozen, 1=no effect)
static const double RECOIL_DAMP_DECAY = 8.0;  // how fast aim damp fades per second
static const double RECOIL_PUSH_SCALE = 20.0;  // sustained push force multiplier

static const double RECOIL_ATTACK_SEC = 0.0;
static const DWORD MIN_EDGE_INTERVAL_MS = 35;   // longer than a single-shot envelope

static const DWORD  RECOIL_DIR_CHANGE_MS = 40;  // how often direction randomizes (same as ambient)
static const double RECOIL_X_SCALE = 20.0;      // backwards kick strength
static const double RECOIL_RELEASE_HZ = 12.0;   // release shaping: low-pass cutoff for the recoil push on the way DOWN only. Higher = sharper release, lower = smoother/longer. (Up-edges pass instantly so the kick stays crisp.)
static const double RECOIL_VERTICAL = 10.0;  // upward force as fraction of recoil (0=none, 1=equal to X)
//static const double RECOIL_Z_RETURN_RATE = 0.2;  // how fast debt bleeds back (higher = snappier return)
static const double EDGE_THRESHOLD = 2.0;  //how much of a rising edge is considered a new impulse

static const double RECOIL_TOPUP_BLEND = 0.6;   // how much of new peak boosts current force
static const double RECOIL_YZ_ONSET_N = 2;     // how many rapid shots before Y/Z texture starts
static const double RECOIL_YZ_DECAY = 0.15;  // how fast Y/Z texture fades between shots
static const double RECOIL_YZ_SCALE = 0.15;   // Y/Z texture strength
static const double RECOIL_YZ_DIR_SLEW_HZ = 6.0;  // how fast lateral texture direction eases toward target while active (lower = smoother)
static const double RECOIL_YZ_GATE_HI = 0.8;      // incoming-rumble level that TURNS ON the lateral texture (strong rumble / the kick only)
static const double RECOIL_YZ_GATE_LO = 0.5;      // level that TURNS OFF (hysteresis) — texture cuts before the weak decay tail to prevent jagged release
static const double RECOIL_YZ_SMOOTH_HZ = 18.0;   // de-jags the active texture (follows packet steps smoothly instead of nudging per packet)
static const double RECOIL_YZ_CUT_HZ = 45.0;      // fast hard-cut once the gate fails
static const DWORD  RUMBLE_FRESH_MS = 40;         // a rumble packet must have arrived within this window to count as live; then clean release


// ── Recoil impulse queue ───────────────────────────────────────────────────
#define RECOIL_QUEUE_SIZE 64
static double g_recoilQueue[RECOIL_QUEUE_SIZE] = {};
static int    g_recoilQHead = 0;
static int    g_recoilQTail = 0;
static DWORD  g_lastRumbleTime = 0;
static CRITICAL_SECTION g_recoilCS;

static void RecoilEnqueue(double peak) {
    EnterCriticalSection(&g_recoilCS);
    int next = (g_recoilQHead + 1) % RECOIL_QUEUE_SIZE;
    if (next != g_recoilQTail) {
        g_recoilQueue[g_recoilQHead] = peak;
        g_recoilQHead = next;
    }
    LeaveCriticalSection(&g_recoilCS);
}

static bool RecoilDequeue(double& peak) {
    EnterCriticalSection(&g_recoilCS);
    if (g_recoilQTail == g_recoilQHead) {
        LeaveCriticalSection(&g_recoilCS);
        return false;
    }
    peak = g_recoilQueue[g_recoilQTail];
    g_recoilQTail = (g_recoilQTail + 1) % RECOIL_QUEUE_SIZE;
    LeaveCriticalSection(&g_recoilCS);
    return true;
}

// ── Runtime state ──────────────────────────────────────────────────────────
static volatile float g_rumbleLarge = 0.0f;
static volatile float g_rumbleSmall = 0.0f;
static double g_recoilForce = 0.0;
static DWORD  g_btn1Released = 0;
static double g_pushDamp = 0.0;
static volatile float g_rumbleLargePeak = 0.0f;  // undecayed, for sustain check
static volatile float g_rumbleSmallPeak = 0.0f;
static double g_recoilPeakOrig = 0.0;  // peak at start of impulse, never decayed
static double g_recoilDampEnv = 0.0;
static double g_recoilPeak = 0.0;
static double g_recoilAttack = 0.0;
static bool   g_recoilFiring = false;
static double g_recoilYZForce = 0.0;  // current Y/Z texture magnitude
static int    g_rapidShotCount = 0;    // consecutive shots that interrupted an impulse
static double g_recoilYZDirY = 1.0;
static double g_recoilYZDirZ = 0.0;
static double g_recoilYZTgtY = 1.0;   // target lateral direction (actual dir slews toward it)
static double g_recoilYZTgtZ = 0.0;
static double g_recoilYZLevel = 0.0;  // smoothed, hysteresis-gated lateral texture magnitude
static bool   g_recoilTailActive = false;  // true while a recoil's rumble tail is still arriving (ambient held off)
static DWORD  g_lastYZDirChange = 0;
static double          g_recoilDecay = 0.55;
static volatile bool   g_running = true;
static volatile uint8_t g_btnMask = 0;
static volatile uint8_t g_dhdButtons = 0;
//static volatile DWORD  g_lastRumbleTime = 0;

// ── Serial ─────────────────────────────────────────────────────────────────
static HANDLE g_hSerial = INVALID_HANDLE_VALUE;

static bool SerialOpen(const char* port, DWORD baud) {
    g_hSerial = CreateFileA(port, GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (g_hSerial == INVALID_HANDLE_VALUE) return false;
    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(g_hSerial, &dcb);
    dcb.BaudRate = baud; dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
    SetCommState(g_hSerial, &dcb);
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutConstant = 2;
    timeouts.WriteTotalTimeoutConstant = 10;
    SetCommTimeouts(g_hSerial, &timeouts);
    return true;
}

static void SerialClose() {
    if (g_hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(g_hSerial);
        g_hSerial = INVALID_HANDLE_VALUE;
    }
}

static void SerialWrite(const BYTE* data, int len) {
    if (g_hSerial == INVALID_HANDLE_VALUE) return;
    DWORD written = 0;
    WriteFile(g_hSerial, data, len, &written, NULL);
}

static DWORD WINAPI SerialReaderThread(LPVOID) {
    BYTE buf[4]; int pos = 0;
    while (g_hSerial != INVALID_HANDLE_VALUE) {
        BYTE b; DWORD n = 0;
        if (!ReadFile(g_hSerial, &b, 1, &n, NULL) || n == 0) { Sleep(1); continue; }
        if (pos == 0 && b != PACKET_OUT_MAGIC) continue;
        buf[pos++] = b;
        if (pos == 4) {
            pos = 0;
            if ((buf[1] ^ buf[2]) == buf[3]) {
                float inL = buf[1] / 255.0f;
                float inS = buf[2] / 255.0f;
                if (inL > g_rumbleLarge) g_rumbleLarge = inL;
                if (inS > g_rumbleSmall) g_rumbleSmall = inS;
                g_rumbleLargePeak = inL;   // ← raw peak, no decay
                g_rumbleSmallPeak = inS;
                double rawMag = (inL * 1.0 + inS * 0.5);
                static DWORD lastRawPrint = 0;
                if (GetTickCount() - lastRawPrint > 100) {
                    lastRawPrint = GetTickCount();
                    printf("\nRAW: L=%.3f S=%.3f mag=%.3f\n", inL, inS, rawMag);
                }
                double newRecoil = pow(rawMag, RECOIL_CURVE) * RUMBLE_FORCE_SCALE * FORCE_MAX_N;
                RecoilEnqueue(newRecoil);
                g_lastRumbleTime = GetTickCount();
            }
        }
    }
    return 0;
}

static void SendControllerPacket(int16_t lx, int16_t ly, uint8_t buttons) {
    BYTE pkt[7];
    pkt[0] = PACKET_IN_MAGIC;
    pkt[1] = (BYTE)((lx >> 8) & 0xFF);
    pkt[2] = (BYTE)(lx & 0xFF);
    pkt[3] = (BYTE)((ly >> 8) & 0xFF);
    pkt[4] = (BYTE)(ly & 0xFF);
    pkt[5] = buttons;
    pkt[6] = pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5];
    SerialWrite(pkt, 7);
}

// ── Per-axis state ─────────────────────────────────────────────────────────
struct AxisState {
    double absMin = 999.0;
    double absMax = -999.0;
    bool   seeded = false;
    double estCenter = 0.0;
    double halfRange = HALF_RANGE_MAX;
    double lastPos = 0.0;
    double smoothVel = 0.0;
    double rawVel = 0.0;
    double flickCarry = 0.0;   // banked fast-motion deflection, paid out as a tail
    int    zeroCount = 0;
    double slowRef = 0.0;
    bool   slowRefSeeded = false;

    static const int VEL_WINDOW = 6;
    double posHistory[8] = {};
    double timeHistory[8] = {};
    int    posIdx = 0;
    bool   posSeeded = false;

    void UpdateReach(double pos) {
        if (pos < absMin) absMin = pos;
        if (pos > absMax) absMax = pos;
        if (absMax > absMin) { seeded = true; estCenter = (absMin + absMax) / 2.0; }
    }

    void UpdateSlowRef(double pos, double dt) {
        if (!slowRefSeeded) { slowRef = pos; slowRefSeeded = true; return; }
        double alpha = 1.0 - exp(-2.0 * 3.14159265 * 1.2 * dt);  // was 2.0Hz, now 1.2Hz
        slowRef += alpha * (pos - slowRef);
    }

    double PosError(double pos) const {
        if (halfRange < 0.001) return 0.0;
        double e = (pos - slowRef) / halfRange;
        return e < -1.0 ? -1.0 : (e > 1.0 ? 1.0 : e);
    }

    void UpdateVelocity(double pos, double dt) {
        if (!posSeeded) {
            for (int i = 0; i < VEL_WINDOW; i++) {
                posHistory[i] = pos;
                timeHistory[i] = dt > 0.0 ? dt : 0.001;
            }
            posSeeded = true;
        }

        posHistory[posIdx] = pos;
        timeHistory[posIdx] = dt > 0.0 ? dt : 0.001;
        posIdx = (posIdx + 1) % VEL_WINDOW;

        double oldPos = posHistory[posIdx];
        double elapsed = 0.0;
        for (int i = 0; i < VEL_WINDOW; i++) elapsed += timeHistory[i];
        double avgVel = (elapsed > 0.0001) ? (pos - oldPos) / elapsed : 0.0;

        int prev2 = (posIdx + VEL_WINDOW - 2) % VEL_WINDOW;
        double t2 = timeHistory[(posIdx + VEL_WINDOW - 1) % VEL_WINDOW]
            + timeHistory[(posIdx + VEL_WINDOW - 2) % VEL_WINDOW];
        double shortVel = (t2 > 0.0001) ? (pos - posHistory[prev2]) / t2 : 0.0;

        if (fabs(shortVel) > SHORT_VEL_NOISE && fabs(shortVel) > fabs(avgVel))
            rawVel = shortVel;
        else
            rawVel = avgVel;

        //Alpha section - smoothing at each speed bracket
        double velSpeed = fabs(rawVel);
        double fc;
        if (velSpeed < VEL_BLEND_VLOW) {
            fc = 9.0;   // was 5 — cuts latency from ~32ms to ~18ms
        }
        else if (velSpeed < VEL_BLEND_LOW) {
            double t = (velSpeed - VEL_BLEND_VLOW) / (VEL_BLEND_LOW - VEL_BLEND_VLOW);
            t = t * t * (3.0 - 2.0 * t);
            fc = 9.0 + (18.0 - 9.0) * t;    // was 3->8
        }
        else if (velSpeed < VEL_BLEND_HIGH) {
            double t = (velSpeed - VEL_BLEND_LOW) / (VEL_BLEND_HIGH - VEL_BLEND_LOW);
            t = t * t * (3.0 - 2.0 * t);
            fc = 18.0 + (30.0 - 18.0) * t;  // was 8->15
        }
        else {
            double t = (velSpeed - VEL_BLEND_HIGH) / (VEL_FC_SPEED_CEIL - VEL_BLEND_HIGH);
            t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
            fc = 30.0 + (55.0 - 30.0) * t;  // was 15->30
        }
        double fcRel = fc;
        if (fabs(rawVel) < fabs(smoothVel)) fcRel = fc * VEL_RELEASE_MULT; // decelerating → collapse faster
        double alpha = 1.0 - exp(-2.0 * 3.14159265 * fcRel * dt);
        smoothVel += alpha * (rawVel - smoothVel);

        if (fabs(rawVel) < VEL_DEADZONE && fabs(smoothVel) < VEL_DEADZONE * 2.0) {
            zeroCount++;
            if (zeroCount > 10) smoothVel = 0.0;
        }
        else {
            zeroCount = 0;
        }

        lastPos = pos;
    }

    // ── Flick capture ────────────────────────────────────────────────────────
    // When the hand moves faster than the stick can express (saturation), bank the
    // over-speed and pay it back as a brief decaying carry, so a quick flick delivers
    // the rotation its displacement deserves. Below the threshold nothing banks, so
    // slow aiming and snappy stops are untouched. A reversal cancels pending carry.
    void UpdateFlick(double dt) {
        double sp = fabs(smoothVel);
        double excess = sp - FLICK_VEL_ON;
        if (excess < 0.0) excess = 0.0;
        double target = (smoothVel >= 0.0 ? 1.0 : -1.0) * excess * FLICK_GAIN;

        if (flickCarry != 0.0 && fabs(smoothVel) > VEL_DEADZONE) {
            double velSign = (smoothVel >= 0.0) ? 1.0 : -1.0;
            if (velSign * flickCarry < 0.0) flickCarry = 0.0;
        }

        if (fabs(target) > fabs(flickCarry)) flickCarry = target;
        else flickCarry *= pow(FLICK_DECAY, dt * 1000.0);
        if (flickCarry > FLICK_CARRY_MAX) flickCarry = FLICK_CARRY_MAX;
        if (flickCarry < -FLICK_CARRY_MAX) flickCarry = -FLICK_CARRY_MAX;
        if (fabs(flickCarry) < 0.001) flickCarry = 0.0;
    }

    double Offset(double pos) const {
        if (halfRange < 0.001) return 0.0;
        double o = (pos - estCenter) / halfRange;
        return o < -1.0 ? -1.0 : (o > 1.0 ? 1.0 : o);
    }
};

// ── Push zone ─────────────────────────────────────────────────────────────
struct PushState2D {
    bool inPushY = false;
    bool inPushZ = false;
    double entryVelY = 0.0;  // velocity captured at moment of entry
    double entryVelZ = 0.0;

    bool Update(double offY, double offZ, double velY, double velZ, double& outX, double& outY) {
        // Y axis
        if (inPushY) {
            if (fabs(offY) < PUSH_EXIT_RAD) { inPushY = false; entryVelY = 0.0; }
        }
        else {
            if (fabs(offY) >= PUSH_ENTER_RAD) {
                inPushY = true;
                entryVelY = outX;  // capture current stick output at entry
            }
        }

        // Z axis
        if (inPushZ) {
            if (fabs(offZ) < PUSH_EXIT_RAD) { inPushZ = false; entryVelZ = 0.0; }
        }
        else {
            if (fabs(offZ) >= PUSH_ENTER_RAD) {
                inPushZ = true;
                entryVelZ = outY;  // capture current stick output at entry
            }
        }

        bool inPush = inPushY || inPushZ;
        outX = outY = 0.0;

        if (inPushY) {
            double excess = (fabs(offY) - PUSH_ENTER_RAD) / (1.0 - PUSH_ENTER_RAD);
            excess = excess < 0.0 ? 0.0 : (excess > 1.0 ? 1.0 : excess);
            // start at entry velocity, scale up toward PUSH_SPEED_MAX with depth
            double sign = offY >= 0.0 ? 1.0 : -1.0;
            double base = fabs(entryVelY) > 0.01 ? fabs(entryVelY) : PUSH_SPEED_BASE;
            double mag = base + excess * (PUSH_SPEED_MAX - base);
            if (mag > PUSH_SPEED_MAX) mag = PUSH_SPEED_MAX;
            outX = sign * mag;
        }
        if (inPushZ) {
            double excess = (fabs(offZ) - PUSH_ENTER_RAD) / (1.0 - PUSH_ENTER_RAD);
            excess = excess < 0.0 ? 0.0 : (excess > 1.0 ? 1.0 : excess);
            double sign = offZ >= 0.0 ? 1.0 : -1.0;
            double base = fabs(entryVelZ) > 0.01 ? fabs(entryVelZ) : PUSH_SPEED_BASE;
            double mag = base + excess * (PUSH_SPEED_MAX - base);
            if (mag > PUSH_SPEED_MAX) mag = PUSH_SPEED_MAX;
            outY = sign * mag * PUSH_VERTICAL_SCALE;   // up/down at reduced speed
        }
        return inPush;
    }
};

static int16_t ToStick(double v) {
    if (v > 1.0) v = 1.0;
    if (v < -1.0) v = -1.0;
    return (int16_t)(v * 32767.0);
}

static void ApplyForces(double y, double z,
    const AxisState& axY, const AxisState& axZ,
    double velY, double velZ,
    double rumX, double rumY, double rumZ,
    double dt)
{
    double offY = axY.Offset(y), offZ = axZ.Offset(z);
    double r = sqrt(offY * offY + offZ * offZ);
    double forceY = 0.0, forceZ = 0.0;

    //Spring force bounding box
    if (r > FORCE_SPRING_START) {
        double dirY = offY / r, dirZ = offZ / r;
        double t = (r - FORCE_SPRING_START) / (FORCE_MAX_RAD - FORCE_SPRING_START);
        t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
        double mag = pow(t, FORCE_EXPONENT) * FORCE_MAX_N;
        double vOut = velY * dirY + velZ * dirZ; // outward velocity component
        if (vOut > 0.0) {
            double proximity = (r - FORCE_SPRING_START) / (PUSH_ENTER_RAD - FORCE_SPRING_START);
            proximity = proximity < 0.0 ? 0.0 : (proximity > 1.0 ? 1.0 : proximity);

            // Gate: only apply enhanced damping above a velocity threshold
            static const double DAMP_VEL_MIN = 0.010; // below this = no enhanced damping
            static const double DAMP_VEL_MAX = 0.020; // above this = full enhanced damping
            double velMag = sqrt(velY * velY + velZ * velZ);
            double velGate = (velMag - DAMP_VEL_MIN) / (DAMP_VEL_MAX - DAMP_VEL_MIN);
            velGate = velGate < 0.0 ? 0.0 : (velGate > 1.0 ? 1.0 : velGate);
            velGate = velGate * velGate * (3.0 - 2.0 * velGate); // smoothstep

            double dynamicDamp = FORCE_DAMPING * (1.0 + proximity * 2.0 * velGate);
            mag += vOut * dynamicDamp;
        }
        forceY = -dirY * mag;
        forceZ = -dirZ * mag;
    }

    // ── Boundary detent — felt approaching the push zone ─────────────────────────
    // Direction-latched: full-strength on the way OUT (crisp pop at the threshold,
    // unchanged from the plain position detent), but it releases the instant you turn
    // back — so coming in it kicks out right at the border instead of lingering inward.
    // This is a direction switch, not a speed-proportional gate, so outward feel and
    // slow-approach crispness are untouched. Re-arms on every outward move, so repeated
    // border crossings stay crisp.
    double detentStart = PUSH_ENTER_RAD - DETENT_WIDTH;        // ridge begins here
    if (r > detentStart && r < PUSH_ENTER_RAD && r > 0.0001) {
        double dirY = offY / r, dirZ = offZ / r;
        double vOut = velY * dirY + velZ * dirZ;               // + = moving outward, - = returning
        static int detentArmed = 1;                            // holds last clear direction (hysteresis)
        if (vOut > DETENT_VEL_HYST) detentArmed = 1;          // moving out → arm
        else if (vOut < -DETENT_VEL_HYST) detentArmed = 0;     // moving back in → suppress (kicks out at border)
        if (detentArmed) {
            double u = (r - detentStart) / DETENT_WIDTH;       // 0 at ridge start, 1 at push threshold
            u = u * u * (3.0 - 2.0 * u);                       // smoothstep — gentle build
            double lip = u * DETENT_PEAK_N;
            forceY += -dirY * lip;                             // resist outward; drops to 0 at threshold = the "pop"
            forceZ += -dirZ * lip;
        }
    }

    // ── Exit pump — a brief inward kick right at the border when leaving push ─────
    // Reproduces the "pump" the plain position detent gave on the way back in, but
    // fired as a self-decaying impulse at the moment you cross out of push, so it
    // lands on the border instead of lingering inward across the band.
    static bool inPushZone = false;
    static double pumpEnv = 0.0;
    bool nowInPush = (r >= PUSH_ENTER_RAD);
    if (inPushZone && !nowInPush) pumpEnv = 1.0;               // just dropped below the border → fire
    inPushZone = nowInPush;
    if (pumpEnv > 0.01 && r > 0.0001) {
        double dirY = offY / r, dirZ = offZ / r;
        double kick = PUMP_PEAK_N * pumpEnv;
        forceY += -dirY * kick;                                // inward kick toward center
        forceZ += -dirZ * kick;
    }
    pumpEnv *= pow(PUMP_DECAY, dt * 1000.0);
    if (pumpEnv < 0.01) pumpEnv = 0.0;

    forceY += rumY;
    forceZ += rumZ;

    // Friction cancellation
    static const double FRICTION_VEL_MIN = 0.003;
    static const double FRICTION_VEL_MAX = 0.08;
    double velMag = sqrt(velY * velY + velZ * velZ);
    if (velMag > FRICTION_VEL_MIN) {
        double fadeIn = fmin((velMag - FRICTION_VEL_MIN) / (FRICTION_VEL_FULL - FRICTION_VEL_MIN), 1.0);
        double fadeOut = fmax(1.0 - (velMag - FRICTION_VEL_FULL) / (FRICTION_VEL_MAX - FRICTION_VEL_FULL), 0.0);
        double blend = fadeIn * fadeOut;
        forceY += (velY / velMag) * FRICTION_CANCEL * blend;
        forceZ += (velZ / velMag) * FRICTION_CANCEL * blend;
    }

    //Coulomb breakaway
    if (velMag > STICTION_VEL_ON) {
        forceY += (velY / velMag) * STICTION_BREAK_N;
        forceZ += (velZ / velMag) * STICTION_BREAK_N;
    }

    // Cap X and YZ independently
    // X = recoil/rumble push, YZ = spring + lateral rumble
    double yzMag = sqrt(forceY * forceY + forceZ * forceZ);
    if (yzMag > 7.8) {
        double s = 7.8 / yzMag;
        forceY *= s;
        forceZ *= s;
    }
    if (rumX > FORCE_MAX_N) rumX = FORCE_MAX_N;
    if (rumX < -FORCE_MAX_N) rumX = -FORCE_MAX_N;

    dhdSetForce(rumX, forceY, forceZ);
}

// ── Main ───────────────────────────────────────────────────────────────────
int main() {
    InitializeCriticalSection(&g_recoilCS);

    printf("FalconJoyForce - Falcon -> Pico 2W (Serial) -> Xbox Controller\n\n");

    printf("Opening %s at %d baud...\n", SERIAL_PORT, SERIAL_BAUD);
    if (!SerialOpen(SERIAL_PORT, SERIAL_BAUD)) {
        printf("ERROR: Cannot open %s\n", SERIAL_PORT);
        printf("Check Device Manager -> Ports for the correct COM number.\n");
        getchar(); return -1;
    }
    printf("Serial port open.\n");

    HANDLE hThread = CreateThread(NULL, 0, SerialReaderThread, NULL, 0, NULL);
    if (!hThread) {
        printf("ERROR: Cannot start reader thread.\n");
        SerialClose(); getchar(); return -1;
    }

    if (dhdOpen() < 0) {
        printf("ERROR: Cannot open Falcon.\n");
        SerialClose(); getchar(); return -1;
    }

    timeBeginPeriod(1);
    // Enable gravity compensation
    dhdSetGravityCompensation(DHD_ON);
    dhdSetStandardGravity(9.81 * GRAVITY_COMP_SCALE);

    printf("Falcon ready. SDK %s\n\n", dhdGetSDKVersionStr());
    printf("Serial: %s   Push zone: %.0f%%   Force max rad: %.2f\n",
        SERIAL_PORT, PUSH_ENTER_RAD * 100.0, FORCE_MAX_RAD);
    printf("F9=quit  F10=debug  F11=recalibrate\n\n");

    AxisState   axY, axZ;
    PushState2D push;

    double rumblePhase = 0.0;
    DWORD  lastFrame = GetTickCount();
    DWORD  lastStats = GetTickCount();
    int    hz = 0;
    bool   debugMode = false;
    bool   btn1WasHeld = false;
    g_btn1Released = GetTickCount();

    double stickX = 0.0, stickY = 0.0;
    bool   recoilActive = false;
    bool   wasRecoilActive = false;
    double stickScale = 1.0;

    while (true) {
        DWORD  now = GetTickCount();
        double dt = (now - lastFrame) / 1000.0;
        if (dt > 0.05) dt = 0.01;
        lastFrame = now;

        double x, y, z;
        if (dhdGetPosition(&x, &y, &z) < 0) { printf("Lost Falcon\n"); break; }

        axY.UpdateVelocity(y, dt);
        axZ.UpdateVelocity(z, dt);
        axY.UpdateSlowRef(y, dt);
        axZ.UpdateSlowRef(z, dt);
        axY.UpdateReach(y);
        axZ.UpdateReach(z);

        double offY = axY.Offset(y), offZ = axZ.Offset(z);
        double stickX = 0.0, stickY = 0.0;
        bool inPush = push.Update(offY, offZ, axY.smoothVel, axZ.smoothVel, stickX, stickY);

        // ── Push zone reversal damping ─────────────────────────────────────
        static bool wasInPush = false;
        if (inPush) {
            double dotY = -(offY * axY.smoothVel);
            double dotZ = -(offZ * axZ.smoothVel);
            double dotVelToCenter = dotY + dotZ;
            if (dotVelToCenter > 0.0) {
                double dampTrigger = dotVelToCenter * PUSH_DAMP_COEFF;
                if (dampTrigger > g_pushDamp) g_pushDamp = dampTrigger;
            }
        }
        else {
            if (wasInPush) g_pushDamp = 1.0;
        }
        wasInPush = inPush;
        g_pushDamp -= g_pushDamp * PUSH_DAMP_DECAY * dt;
        if (g_pushDamp < 0.0) g_pushDamp = 0.0;
        if (g_pushDamp > 1.0) g_pushDamp = 1.0;

        // ── Recoil velocity bleed ──────────────────────────────────────────────
        // Prevents physical Falcon settling during recoil tail from reaching stick output
        if (g_recoilFiring) {
            double bleedStrength = (g_recoilForce / RECOIL_MAG_SCALE);
            bleedStrength = bleedStrength < 0.0 ? 0.0 : (bleedStrength > 1.0 ? 1.0 : bleedStrength);
            // Bias: specifically gate downward (negative Z) velocity harder than upward
            // since gravity is the primary cause of the tail drift
            double bleedY = 1.0 - bleedStrength * 0.5;
            double bleedZ = (axZ.smoothVel < 0.0)
                ? 1.0 - bleedStrength * 0.8   // stronger bleed on downward motion
                : 1.0 - bleedStrength * 0.4;  // lighter on upward
            axY.smoothVel *= bleedY;
            axZ.smoothVel *= bleedZ;
        }

        axY.UpdateFlick(dt);   // bank/pay-out fast-motion carry (uses post-bleed velocity)
        axZ.UpdateFlick(dt);

        if (!inPush) {
            auto evalZone = [](double v, double sens, double crv) -> double {
                double scaled = fabs(v) * sens;
                if (scaled > 1.0) scaled = 1.0;
                return pow(scaled, crv);
            };
            auto curve = [&](double v) -> double {
                if (fabs(v) < VEL_DEADZONE) return 0.0;
                double sign = v > 0.0 ? 1.0 : -1.0;
                double speed = fabs(v);

                double ramp = (speed - VEL_DEADZONE) / SOFT_RAMP;
                ramp = ramp < 0.0 ? 0.0 : (ramp > 1.0 ? 1.0 : ramp);
                ramp = ramp * ramp * (3.0 - 2.0 * ramp);

                double vlow = evalZone(v, VEL_VLOW_SENS, VEL_VLOW_CURVE);
                double lo = evalZone(v, VEL_LOW_SENS, VEL_LOW_CURVE);
                double hi = pow(fmin(speed * VEL_HIGH_SENS, 1.0), VEL_HIGH_CURVE);
                if (hi > 1.0) hi = 1.0;

                // Blend vlow -> lo
                double tVlow = (speed - VEL_BLEND_VLOW) / (VEL_BLEND_LOW - VEL_BLEND_VLOW);
                tVlow = tVlow < 0.0 ? 0.0 : (tVlow > 1.0 ? 1.0 : tVlow);
                tVlow = tVlow * tVlow * (3.0 - 2.0 * tVlow);
                double lowBlended = vlow + (lo - vlow) * tVlow;

                // Blend lowBlended -> hi
                double tHi = (speed - VEL_BLEND_LOW) / (VEL_BLEND_HIGH - VEL_BLEND_LOW);
                tHi = tHi < 0.0 ? 0.0 : (tHi > 1.0 ? 1.0 : tHi);
                tHi = tHi * tHi * (3.0 - 2.0 * tHi);

                return sign * (lowBlended * (1.0 - tHi) + hi * tHi) * ramp;
            };

            stickX = curve(axY.smoothVel);
            stickY = curve(axZ.smoothVel);

            // Position-error blend — kicks in when velocity signal is unreliable
            double velMag = sqrt(axY.smoothVel * axY.smoothVel + axZ.smoothVel * axZ.smoothVel);
            double posBlendRaw = 1.0 - velMag / posBlendMax;
            posBlendRaw = posBlendRaw < 0.0 ? 0.0 : (posBlendRaw > 1.0 ? 1.0 : posBlendRaw);
            posBlendRaw = posBlendRaw * posBlendRaw * (3.0 - 2.0 * posBlendRaw);

            static double posBlendSm = 0.0;  // slew-limited — a one-frame velMag dip can't snap this
            double aPB = 1.0 - exp(-2.0 * 3.14159265 * 6.0 * dt);
            posBlendSm += aPB * (posBlendRaw - posBlendSm);
            double posBlend = posBlendSm;

            if (posBlend > 0.0) {
                double pcX = axY.PosError(y) * VEL_VLOW_POS_SCALE;
                double pcY = axZ.PosError(z) * VEL_VLOW_POS_SCALE;
                pcX = pcX < -1.0 ? -1.0 : (pcX > 1.0 ? 1.0 : pcX);
                pcY = pcY < -1.0 ? -1.0 : (pcY > 1.0 ? 1.0 : pcY);
                stickX = stickX * (1.0 - posBlend) + pcX * posBlend;
                stickY = stickY * (1.0 - posBlend) + pcY * posBlend;
            }

            stickX += axY.flickCarry;   // pay out banked fast-motion as a short carry tail
            stickY += axZ.flickCarry;   // (ToStick clamps the sum to ±1)
            stickY *= VEL_VERTICAL_SCALE;   // up/down at reduced speed (velocity zone)
        }

        rumblePhase += dt * 80.0 * 2.0 * 3.14159265;
        float rL = g_rumbleLarge, rS = g_rumbleSmall;

        // ── Button 1 recoil logic ──────────────────────────────────
        bool btn1held = (dhdGetButton(0) > 0);
        if (btn1WasHeld && !btn1held) g_btn1Released = now;
        btn1WasHeld = btn1held;
        recoilActive = btn1held || (now - g_btn1Released < RECOIL_WINDOW_MS);

        if (recoilActive && !wasRecoilActive) g_recoilDampEnv = 1.0;
        wasRecoilActive = recoilActive;

        g_recoilDampEnv -= g_recoilDampEnv * RECOIL_DAMP_DECAY * dt;
        if (g_recoilDampEnv < 0.0) g_recoilDampEnv = 0.0;

        double rumX = 0.0, rumY = 0.0, rumZ = 0.0;

        if (recoilActive) {
            double liveMag = (g_rumbleLargePeak * RUMBLE_LARGE_SCALE + g_rumbleSmallPeak * RUMBLE_SMALL_SCALE);
            // Stale held-peak guard: g_rumbleLargePeak/SmallPeak are raw and never
            // self-decay, so after firing stops a latched peak keeps sustainingRumble true
            // and re-fires the recoil (the ratchet on release). Zero it ONLY while the
            // trigger is released — while the trigger is HELD we must never zero liveMag,
            // or a momentary packet gap would drop the sustained-fire hold.
            if (!btn1held && (now - g_lastRumbleTime) >= RUMBLE_FRESH_MS) liveMag = 0.0;
            double queuePeak = 0.0, nextPeak = 0.0;
            while (RecoilDequeue(nextPeak))
                if (nextPeak > queuePeak) queuePeak = nextPeak;

            if (queuePeak > 0.0) {
                if (!g_recoilFiring) {
                    g_recoilForce = liveMag;
                    g_recoilPeak = liveMag;
                    g_recoilAttack = (RECOIL_ATTACK_SEC > 0.0) ? 0.0 : 1.0;
                    g_recoilFiring = true;
                    g_rapidShotCount = 0;
                    g_recoilYZForce = 0.0;
                    double magT = liveMag / RECOIL_MAG_SCALE;
                    magT = magT < 0.0 ? 0.0 : (magT > 1.0 ? 1.0 : magT);
                    g_recoilDecay = RECOIL_DECAY_MIN + magT * (RECOIL_DECAY_MAX - RECOIL_DECAY_MIN);
                }
                else {
                    g_rapidShotCount++;
                    if (liveMag > g_recoilForce) g_recoilForce = liveMag;
                    g_recoilPeak = g_recoilForce;
                    g_recoilYZForce = liveMag * RECOIL_YZ_SCALE;
                    double magT = liveMag / RECOIL_MAG_SCALE;
                    magT = magT < 0.0 ? 0.0 : (magT > 1.0 ? 1.0 : magT);
                    g_recoilDecay = RECOIL_DECAY_MIN + magT * (RECOIL_DECAY_MAX - RECOIL_DECAY_MIN);
                }
            }

            bool sustainingRumble = (liveMag > RECOIL_SUSTAIN_THRESHOLD);
            if (sustainingRumble) {
                if (liveMag > g_recoilForce) { g_recoilForce = liveMag; g_recoilPeak = liveMag; }
                if (!g_recoilFiring && liveMag > 0.01) {
                    g_recoilForce = liveMag; g_recoilPeak = liveMag;
                    g_recoilFiring = true;    g_recoilAttack = 1.0;
                }
            }

            double liveYZMag = 0.0;
            if (g_rapidShotCount > 0 || sustainingRumble)
                liveYZMag = liveMag * RECOIL_YZ_SCALE;
            if (liveYZMag > g_recoilYZForce) g_recoilYZForce = liveYZMag;
            g_recoilYZForce *= (float)RUMBLE_DECAY;
            if (g_recoilYZForce < 0.01) g_recoilYZForce = 0.0;

            if (g_recoilYZForce > 0.01 && (now - g_lastYZDirChange) > RECOIL_DIR_CHANGE_MS) {
                g_lastYZDirChange = now;
                double angle = ((rand() % 1000) / 1000.0) * 2.0 * 3.14159265;
                g_recoilYZTgtY = cos(angle);   // set target; actual dir slews toward it below
                g_recoilYZTgtZ = sin(angle);
            }

            if (g_recoilFiring) {
                double envelope = 1.0;
                if (RECOIL_ATTACK_SEC > 0.0) {
                    g_recoilAttack += dt / RECOIL_ATTACK_SEC;
                    if (g_recoilAttack > 1.0) g_recoilAttack = 1.0;
                    envelope = g_recoilAttack;
                }
                rumX = g_recoilPeak * envelope * RECOIL_X_SCALE * 2.0;
                // lateral texture (rumY/rumZ) rendered by the consolidated slewed block below

                if (!sustainingRumble && g_recoilAttack >= 1.0) {
                    double frameDecay = pow(g_recoilDecay, dt * 1000.0);
                    g_recoilForce *= frameDecay;
                    g_recoilPeak *= frameDecay;
                    if (g_recoilForce < 0.01) {
                        g_recoilForce = 0.0;
                        g_recoilFiring = false;
                        g_rapidShotCount = 0;
                    }
                }
            }
        }
        else {
            if (!g_recoilFiring) {
                g_recoilForce = 0.0;
                g_recoilPeak = 0.0;
                g_recoilAttack = 0.0;
            }
            g_recoilYZForce = 0.0;
            g_rapidShotCount = 0;
            double tmp;
            while (RecoilDequeue(tmp)) {}
        }

        if (!recoilActive && g_recoilFiring) {
            double frameDecay = pow(g_recoilDecay, dt * 1000.0);
            g_recoilForce *= frameDecay;
            g_recoilPeak *= frameDecay;
            rumX = g_recoilPeak * RECOIL_X_SCALE * 2.0;
            if (g_recoilForce < 0.01) { g_recoilForce = 0.0; g_recoilFiring = false; }
        }

        // ── Recoil push release shaping ──────────────────────────────────────────────
        // The raw recoil force sawtooths as it ramps down: each incoming tail packet
        // re-bumps g_recoilForce, which then collapses fast before the next arrives — that
        // sawtooth is the buzz. Snap UP instantly so the kick and sustained hold stay
        // sharp, but low-pass on the way DOWN so the release tracks the rumble's decay
        // envelope as one clean ramp. RECOIL_RELEASE_HZ sets sharper (higher) vs smoother.
        static double g_recoilXOut = 0.0;
        if (rumX >= g_recoilXOut) {
            g_recoilXOut = rumX;                                   // rising → keep the sharp kick / hold
        }
        else {
            double aRel = 1.0 - exp(-2.0 * 3.14159265 * RECOIL_RELEASE_HZ * dt);
            g_recoilXOut += aRel * (rumX - g_recoilXOut);         // falling → smooth, buzz-free ramp down
        }
        if (fabs(g_recoilXOut) < 0.01) g_recoilXOut = 0.0;        // reach true 0 so ambient can resume
        rumX = g_recoilXOut;

        // Hold ambient off through the ENTIRE recoil rumble tail, however long it runs.
        // A recoil opens a "tail session" that stays open until the incoming rumble has
        // decayed below AMBIENT_TAIL_QUIET or stopped arriving. This adapts to shot size —
        // larger shots have bigger/longer tails — which a fixed time window did not, which
        // is why the buzz survived on larger shots.
        {
            double liveInNow = (g_rumbleLargePeak * RUMBLE_LARGE_SCALE + g_rumbleSmallPeak * RUMBLE_SMALL_SCALE);
            bool   rumbleFresh = (now - g_lastRumbleTime) < RUMBLE_FRESH_MS;
            if (recoilActive || g_recoilFiring)                      g_recoilTailActive = true;
            else if (!rumbleFresh || liveInNow < AMBIENT_TAIL_QUIET) g_recoilTailActive = false;
        }

        if (rumX == 0.0) {
            static double rumDirY = 1.0, rumDirZ = 0.0;   // current (slewed) direction
            static double tgtDirY = 1.0, tgtDirZ = 0.0;   // target direction
            static double rumMagSm = 0.0;                 // smoothed magnitude
            static DWORD  lastDirChange = 0;

            static const DWORD  DIR_CHANGE = 140;     // ms between new target dirs (was 40)
            static const double DIR_SLEW_HZ = 5.0;     // how fast dir eases toward target
            static const double MAG_ATTACK_HZ = 7.0;     // ramp-up cutoff
            static const double MAG_RELEASE_HZ = 3.0;     // ramp-down cutoff (slower = softer fade)

            double magTarget = (rL * AMBIENT_LARGE_SCALE + rS * AMBIENT_SMALL_SCALE);

            // Hold ambient off for the whole recoil tail session — the smoother below then
            // keeps ambient faded out instead of buzzing on the tail. Genuine ambient with
            // no recent firing is unaffected.
            if (g_recoilTailActive) magTarget = 0.0;

            // pick a new target direction occasionally
            if (now - lastDirChange > DIR_CHANGE) {
                lastDirChange = now;
                double angle = ((rand() % 1000) / 1000.0) * 2.0 * 3.14159265;
                tgtDirY = cos(angle);
                tgtDirZ = sin(angle);
            }

            // slew the actual direction toward the target, then renormalize
            double aDir = 1.0 - exp(-2.0 * 3.14159265 * DIR_SLEW_HZ * dt);
            rumDirY += aDir * (tgtDirY - rumDirY);
            rumDirZ += aDir * (tgtDirZ - rumDirZ);
            double dlen = sqrt(rumDirY * rumDirY + rumDirZ * rumDirZ);
            if (dlen > 1e-6) { rumDirY /= dlen; rumDirZ /= dlen; }

            // attack/release low-pass on magnitude
            double mhz = (magTarget > rumMagSm) ? MAG_ATTACK_HZ : MAG_RELEASE_HZ;
            double aMag = 1.0 - exp(-2.0 * 3.14159265 * mhz * dt);
            rumMagSm += aMag * (magTarget - rumMagSm);

            if (rumMagSm > 0.001) {
                rumY = rumDirY * rumMagSm;
                rumZ = rumDirZ * rumMagSm;
            }
        }

        // ── Recoil lateral texture: only during the STRONG part of the rumble ───────
        // Incoming rumble per shot is a sharp spike then a long weak decaying tail of
        // packets. Rendering the tail directly turned every little packet into a separate
        // lateral nudge = the jagged rumble "coming down." A hysteresis gate keeps the
        // texture on only while the rumble is genuinely strong (HI to turn on, LO to turn
        // off), so it bursts with the kick and cuts before the weak tail. The level is
        // smoothed while active (de-jag) and hard-cut once the gate fails (clean release).
        {
            double liveIn = (g_rumbleLargePeak * RUMBLE_LARGE_SCALE + g_rumbleSmallPeak * RUMBLE_SMALL_SCALE);
            bool   rumbleIncoming = (now - g_lastRumbleTime) < RUMBLE_FRESH_MS;

            static bool yzOn = false;
            if (!recoilActive || !rumbleIncoming)   yzOn = false;            // outside fire / no fresh rumble → off
            else if (liveIn > RECOIL_YZ_GATE_HI)    yzOn = true;             // strong rumble → on
            else if (liveIn < RECOIL_YZ_GATE_LO)    yzOn = false;            // dropped into the weak tail → off

            double yzTarget = yzOn ? liveIn * RECOIL_YZ_SCALE : 0.0;
            double yzHz = (yzTarget > g_recoilYZLevel) ? RECOIL_YZ_SMOOTH_HZ : RECOIL_YZ_CUT_HZ;
            double aMag = 1.0 - exp(-2.0 * 3.14159265 * yzHz * dt);
            g_recoilYZLevel += aMag * (yzTarget - g_recoilYZLevel);
            if (g_recoilYZLevel < 0.001) g_recoilYZLevel = 0.0;

            if (g_recoilYZLevel > 0.0) {
                double aDir = 1.0 - exp(-2.0 * 3.14159265 * RECOIL_YZ_DIR_SLEW_HZ * dt);
                g_recoilYZDirY += aDir * (g_recoilYZTgtY - g_recoilYZDirY);
                g_recoilYZDirZ += aDir * (g_recoilYZTgtZ - g_recoilYZDirZ);
                double yzlen = sqrt(g_recoilYZDirY * g_recoilYZDirY + g_recoilYZDirZ * g_recoilYZDirZ);
                if (yzlen > 1e-6) { g_recoilYZDirY /= yzlen; g_recoilYZDirZ /= yzlen; }
                rumY += g_recoilYZDirY * g_recoilYZLevel;
                rumZ += g_recoilYZDirZ * g_recoilYZLevel;
            }
        }

        g_rumbleLarge = rL * (float)RUMBLE_DECAY;
        g_rumbleSmall = rS * (float)RUMBLE_DECAY;

        // ── Button 2: constant X preload to ease small corrections ───────────────────
        if (dhdGetButton(1) > 0) rumX += BTN2_X_SIGN * BTN2_X_FORCE;

        ApplyForces(y, z, axY, axZ, axY.smoothVel, axZ.smoothVel, rumX, rumY, rumZ, dt);


        // ── Button mask ────────────────────────────────────────────────────
        uint8_t btnMask = 0;
        for (int i = 0; i < 4; i++)
            if (dhdGetButton(i) > 0) btnMask |= (1u << i);

        // ── Send controller packet ─────────────────────────────────────────
        double stickScale = 1.0 - g_pushDamp;
        if (stickScale < 0.0) stickScale = 0.0;
        if (recoilActive)
            stickScale *= 1.0 - (1.0 - RECOIL_AIM_DAMP) * g_recoilDampEnv;

        int16_t lx = ToStick(stickX * stickScale * (INVERT_X ? -1.0 : 1.0));
        int16_t ly = ToStick(stickY * stickScale * (INVERT_Y ? -1.0 : 1.0));
        SendControllerPacket(lx, ly, btnMask);

        // ── Status display ─────────────────────────────────────────────────
        hz++;
        if (now - lastStats >= 1000) {
            int qcount = (g_recoilQHead - g_recoilQTail + RECOIL_QUEUE_SIZE) % RECOIL_QUEUE_SIZE;
            double r = sqrt(offY * offY + offZ * offZ);
            if (debugMode) {
                printf("\nY: ctr=%+.4f half=%.4f off=%+.2f vel=%+.4f\n",
                    axY.estCenter, axY.halfRange, offY, axY.smoothVel);
                printf("Z: ctr=%+.4f half=%.4f off=%+.2f vel=%+.4f\n",
                    axZ.estCenter, axZ.halfRange, offZ, axZ.smoothVel);
                printf("r=%.3f  push=%s  damp=%.2f  stk=(%.2f,%.2f)  rbl=%.2f/%.2f  rcl=%.2f  q=%d  btn=%X\n",
                    r, inPush ? "YES" : "no", g_pushDamp, stickX, stickY,
                    rL, rS, g_recoilForce, qcount, btnMask);
            }
            else {
                int col = (int)((offY + 1.0) / 2.0 * 8.0 + 0.5);
                col = col < 0 ? 0 : (col > 8 ? 8 : col);
                printf("\r%4d Hz  r=%.2f %s  damp=%.2f  [", hz, r, inPush ? "PUSH" : "    ", g_pushDamp);
                for (int c = 0; c <= 8; c++) printf(c == col ? "O" : ".");
                printf("]  stk=(%.2f,%.2f)  vel=%.3f/%.3f  rbl=%.2f/%.2f  rcl=%.2f  q=%d  btn=%X   ",
                    stickX, stickY, axY.smoothVel, axZ.smoothVel, rL, rS, g_recoilForce, qcount, btnMask);
                fflush(stdout);
            }
            hz = 0; lastStats = now;
        }

        if (GetAsyncKeyState(VK_F9) & 0x8000) break;
        if (GetAsyncKeyState(VK_F10) & 0x8000) {
            debugMode = !debugMode;
            printf("\nDebug %s\n", debugMode ? "ON" : "OFF");
            Sleep(200);
        }
        if (GetAsyncKeyState(VK_F11) & 0x8000) {
            printf("\nRecalibrating - move to ALL extremes for 4 seconds...\n");
            axY = {}; axZ = {};
            DWORD t1 = GetTickCount();
            while (GetTickCount() - t1 < 4000) {
                double rx, ry, rz;
                if (dhdGetPosition(&rx, &ry, &rz) >= 0) { axY.UpdateReach(ry); axZ.UpdateReach(rz); }
                Sleep(1);
            }
            double rawHalfY = (axY.absMax - axY.absMin) * 0.5;
            double rawHalfZ = (axZ.absMax - axZ.absMin) * 0.5;
            double fitY = rawHalfY * CALIB_FILL, fitZ = rawHalfZ * CALIB_FILL;
            if (fitY >= CALIB_MIN && fitY <= CALIB_MAX) axY.halfRange = fitY;  // guard against a partial sweep
            if (fitZ >= CALIB_MIN && fitZ <= CALIB_MAX) axZ.halfRange = fitZ;
            double recommend = (axY.halfRange < axZ.halfRange) ? axY.halfRange : axZ.halfRange;
            printf("Measured half-extent:  Y=%.4f m  Z=%.4f m\n", rawHalfY, rawHalfZ);
            printf("Work area now:  Y half=%.4f  Z half=%.4f\n", axY.halfRange, axZ.halfRange);
            printf(">> Permanent fit: set HALF_RANGE_MAX = %.4f\n", recommend);
            Sleep(200);
        }
        Sleep(UPDATE_RATE_MS);
    }

    dhdSetForce(0.0, 0.0, 0.0);
    dhdClose();
    SerialClose();
    DeleteCriticalSection(&g_recoilCS);
    timeEndPeriod(1);
    printf("\nDone.\n");
    return 0;
}