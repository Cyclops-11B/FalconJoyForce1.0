// Falcon - Mouse emulation joystick with Force
// Novint Falcon -> Pico 2 W (CP2102 serial) -> Xbox 360 controller

#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <dhdc.h>
#include <string.h>

// ── Serial port ────────────────────────────────────────────────────────────
static const char* SERIAL_PORT = "\\\\.\\COM6";
static const DWORD SERIAL_BAUD = 115200;

// ── Packet constants ───────────────────────────────────────────────────────
static const BYTE PACKET_IN_MAGIC = 0xAA;
static const BYTE PACKET_OUT_MAGIC = 0xBB;

// ── Controller config ──────────────────────────────────────────────────────
static const int    UPDATE_RATE_MS = 1;
static const bool   INVERT_X = false;
static const bool   INVERT_Y = false;

// ──Velocity settings ──────────────────────────────────────────────────────
static const double VEL_DEADZONE = 0.0004;
static const double VEL_LOW_SENS = 3.0;  // sensitivity at low speeds
static const double VEL_LOW_CURVE = 0.45; // curve for slow zone (lower = more boost)
static const double VEL_HIGH_SENS = 12.0; // sensitivity at high speeds
static const double VEL_HIGH_CURVE = 0.85; // curve for fast zone (1.0 = linear)
static const double VEL_BLEND_LOW = 0.02; // below this = pure low speed
static const double VEL_BLEND_HIGH = 0.10; // above this = pure high speed
static const double VEL_ALPHA = 0.7;  // 0.8 = very responsive, lower = smoother

static const double PUSH_ENTER_RAD = 0.56; // percentage of work radius where push zone kicks in
static const double PUSH_EXIT_RAD = 0.56; // percentage of work radius where push zone stops acting
static const double PUSH_SPEED_BASE = 0.34; // how fast cursor moves when entering push zone
static const double PUSH_SPEED_MAX = 4.0; // maximum push speed at full tilt
static const double HALF_RANGE_MAX = 0.06; // how large work radius is (in m)

static const double PUSH_DAMP_COEFF = 0.7; // damping strength on direction reversal (0=none, 1=strong)
static const double PUSH_DAMP_DECAY = 3.0; // how quickly damping fades (higher = faster)

static const double FORCE_SPRING_START = 0.3; // percentage of work radius where spring force starts
static const double FORCE_MAX_RAD = 0.84; // percentage of work radius where max force is achieved
static const double FORCE_MAX_N = 7.5; // maximum allowable Force (in N)
static const double FORCE_DAMPING = 3.0; // cut down on springiness
static const double FORCE_EXPONENT = 2.2; // how you ramp to max force. lower = force builds earlier and harder

// ──Ambient Rumble settings ──────────────────────────────────────────────────────
static const double RUMBLE_LARGE_SCALE = 3.0; // scaling of xbox large rumble signal
static const double RUMBLE_SMALL_SCALE = 3.0; // scaling of xbox small rumble signal
static const double RUMBLE_DECAY = 0.35;
static const double RUMBLE_FORCE_SCALE = 12.0; // overall scale factor of rumbe force

// ──Recoil settings ──────────────────────────────────────────────────────
static const double RECOIL_DECAY = 0.30; // lower = snappier
static const DWORD  RECOIL_WINDOW_MS = 150; // ms after btn 1 release to still catch trigger recoil
static const double RECOIL_CURVE = 0.5; // Recoil compressor -  0.3 boosts small recoils; 1.0 = linear
static const double RECOIL_AIM_DAMP = 0.40;  // stick sensitivity multiplier during recoil (0=frozen, 1=no effect)
static const double RECOIL_VERTICAL = 0.1;  // upward force as fraction of recoil (0=none, 1=equal to X)
// Attack time: how long (in seconds) recoil ramps from 0 to peak before decaying
// 0.0 = instant peak (original behavior), 0.02 = 20ms ramp, 0.05 = 50ms ramp
static const double RECOIL_ATTACK_SEC = 0.0;
static const DWORD  IMPULSE_GAP_MS = 35;  //Gap between force generation and next force - too small and forces blend, wider for more distinct pushes

static const double RUMBLE_SUSTAIN_THRESHOLD = 0.15; // min rumble mag to auto-requeue a sustain impulse
static const double RUMBLE_SUSTAIN_SCALE = 0.85;     // fraction of current rumble to use as sustain peak

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
static double g_recoilPeak = 0.0;
static double g_recoilAttack = 0.0;
static bool   g_recoilFiring = false;
static DWORD  g_btn1Released = 0;
static double g_pushDamp = 0.0;
static volatile float g_rumbleLargePeak = 0.0f;  // undecayed, for sustain check
static volatile float g_rumbleSmallPeak = 0.0f;
static double g_stickXSmooth = 0.0;
static double g_stickYSmooth = 0.0;
static const double STICK_SMOOTH = 0.3; // lower = smoother, higher = more responsive

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

    // Ring buffer
    static const int VEL_WINDOW = 3;
    double posHistory[6] = {};
    double timeHistory[6] = {};
    int    posIdx = 0;
    bool   posSeeded = false;

    void UpdateReach(double pos) {
        if (pos < absMin) absMin = pos;
        if (pos > absMax) absMax = pos;
        if (absMax > absMin) { seeded = true; estCenter = (absMin + absMax) / 2.0; }
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
        timeHistory[posIdx] = dt;
        posIdx = (posIdx + 1) % VEL_WINDOW;

        double oldPos = posHistory[posIdx];
        double elapsed = 0.0;
        for (int i = 0; i < VEL_WINDOW; i++) elapsed += timeHistory[i];

        rawVel = 0.0;
        int prev = (posIdx + VEL_WINDOW - 1) % VEL_WINDOW;
        for (int i = 0; i < VEL_WINDOW - 1; i++) {
            int cur = (posIdx + VEL_WINDOW - 1 - i) % VEL_WINDOW;
            int prv = (posIdx + VEL_WINDOW - 2 - i) % VEL_WINDOW;
            double t = timeHistory[cur] > 0.0001 ? timeHistory[cur] : 0.001;
            double v = (posHistory[cur] - posHistory[prv]) / t;
            if (fabs(v) > fabs(rawVel)) rawVel = v;
        }
        smoothVel += VEL_ALPHA * (rawVel - smoothVel);

        // Snap to zero only when clearly stopped
        if (fabs(rawVel) < VEL_DEADZONE && fabs(smoothVel) < VEL_DEADZONE)
            smoothVel = 0.0;

        lastPos = pos;
    }

    double Offset(double pos) const {
        if (halfRange < 0.001) return 0.0;
        double o = (pos - estCenter) / halfRange;
        return o < -1.0 ? -1.0 : (o > 1.0 ? 1.0 : o);
    }
};
//const double AxisState::VEL_ALPHA = 0.25; // unused but kept to avoid linker error

// ── Push zone ─────────────────────────────────────────────────────────────
struct PushState2D {
    bool inPushY = false;
    bool inPushZ = false;

    bool Update(double offY, double offZ, double& outX, double& outY) {
        if (inPushY) { if (fabs(offY) < PUSH_EXIT_RAD)  inPushY = false; }
        else { if (fabs(offY) >= PUSH_ENTER_RAD) inPushY = true; }
        if (inPushZ) { if (fabs(offZ) < PUSH_EXIT_RAD)  inPushZ = false; }
        else { if (fabs(offZ) >= PUSH_ENTER_RAD) inPushZ = true; }

        bool inPush = inPushY || inPushZ;
        outX = outY = 0.0;

        if (inPushY) {
            double excess = (fabs(offY) - PUSH_ENTER_RAD) / (1.0 - PUSH_ENTER_RAD);
            excess = excess < 0.0 ? 0.0 : (excess > 1.0 ? 1.0 : excess);
            double mag = PUSH_SPEED_BASE + excess * (PUSH_SPEED_MAX - PUSH_SPEED_BASE);
            if (mag > 1.0) mag = 1.0;
            outX = (offY >= 0.0 ? 1.0 : -1.0) * mag;
        }
        if (inPushZ) {
            double excess = (fabs(offZ) - PUSH_ENTER_RAD) / (1.0 - PUSH_ENTER_RAD);
            excess = excess < 0.0 ? 0.0 : (excess > 1.0 ? 1.0 : excess);
            double mag = PUSH_SPEED_BASE + excess * (PUSH_SPEED_MAX - PUSH_SPEED_BASE);
            if (mag > 1.0) mag = 1.0;
            outY = (offZ >= 0.0 ? 1.0 : -1.0) * mag;
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
    double rumX, double rumY, double rumZ)
{
    double offY = axY.Offset(y), offZ = axZ.Offset(z);
    double r = sqrt(offY * offY + offZ * offZ);
    double forceY = 0.0, forceZ = 0.0;

    if (r > FORCE_SPRING_START) {
        double dirY = offY / r, dirZ = offZ / r;
        double t = (r - FORCE_SPRING_START) / (FORCE_MAX_RAD - FORCE_SPRING_START);
        t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
        double mag = pow(t, FORCE_EXPONENT) * FORCE_MAX_N;
        double vOut = velY * dirY + velZ * dirZ;
        if (vOut > 0.0) mag += vOut * FORCE_DAMPING;
        if (mag > 7.8) mag = 7.8;
        forceY = -dirY * mag;
        forceZ = -dirZ * mag;
    }

    forceY += rumY;
    forceZ += rumZ;
    double totalMag = sqrt(rumX * rumX + forceY * forceY + forceZ * forceZ);
    if (totalMag > 7.8) {
        double s = 7.8 / totalMag;
        rumX *= s; forceY *= s; forceZ *= s;
    }
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

    while (true) {
        DWORD  now = GetTickCount();
        double dt = (now - lastFrame) / 1000.0;
        if (dt > 0.05) dt = 0.01;
        lastFrame = now;

        double x, y, z;
        if (dhdGetPosition(&x, &y, &z) < 0) { printf("Lost Falcon\n"); break; }

        axY.UpdateVelocity(y, dt);
        axZ.UpdateVelocity(z, dt);
        axY.UpdateReach(y);
        axZ.UpdateReach(z);

        double offY = axY.Offset(y), offZ = axZ.Offset(z);
        double stickX = 0.0, stickY = 0.0;
        bool inPush = push.Update(offY, offZ, stickX, stickY);

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

                double lo = evalZone(v, VEL_LOW_SENS, VEL_LOW_CURVE);
                double hi = evalZone(v, VEL_HIGH_SENS, VEL_HIGH_CURVE);

                // Blend factor: 0 = pure low, 1 = pure high
                double t = (speed - VEL_BLEND_LOW) / (VEL_BLEND_HIGH - VEL_BLEND_LOW);
                t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
                // Smooth S-curve blend
                t = t * t * (3.0 - 2.0 * t);

                return sign * (lo * (1.0 - t) + hi * t);
            };

            stickX = curve(axY.smoothVel);
            stickY = curve(axZ.smoothVel);
        }

        rumblePhase += dt * 80.0 * 2.0 * 3.14159265;
        float rL = g_rumbleLarge, rS = g_rumbleSmall;

        // ── Button 1 recoil logic ──────────────────────────────────────────
        bool btn1held = (dhdGetButton(0) > 0);
        if (btn1WasHeld && !btn1held) g_btn1Released = now;
        btn1WasHeld = btn1held;
        bool recoilActive = btn1held || (now - g_btn1Released < RECOIL_WINDOW_MS);

        double rumX = 0.0, rumY = 0.0, rumZ = 0.0;

        if (recoilActive) {
            static DWORD lastImpulseEnd = 0;
            static const DWORD IMPULSE_GAP_MS = 20;

            if (!g_recoilFiring && (now - lastImpulseEnd >= IMPULSE_GAP_MS)) {
                double nextPeak = 0.0;
                if (RecoilDequeue(nextPeak)) {
                    // ── Normal path: impulse arrived via serial packet ─────
                    g_recoilPeak = nextPeak;
                    g_recoilForce = nextPeak;
                    g_recoilAttack = (RECOIL_ATTACK_SEC > 0.0) ? 0.0 : 1.0;
                    g_recoilFiring = true;
                }
                else {
                    // ── Sustain path: use undecayed peak, not the display-decayed rL/rS ──
                    double sustainMag = (g_rumbleLargePeak * 1.0 + g_rumbleSmallPeak * 0.5);
                    if (sustainMag >= RUMBLE_SUSTAIN_THRESHOLD) {
                        nextPeak = pow(sustainMag, RECOIL_CURVE)
                            * RUMBLE_FORCE_SCALE * FORCE_MAX_N
                            * RUMBLE_SUSTAIN_SCALE;
                        g_recoilPeak = nextPeak;
                        g_recoilForce = nextPeak;
                        g_recoilAttack = (RECOIL_ATTACK_SEC > 0.0) ? 0.0 : 1.0;
                        g_recoilFiring = true;
                        g_lastRumbleTime = now;
                    }
                }
            }
            if (g_recoilFiring && g_recoilForce < 0.05)
                lastImpulseEnd = now;

            if (g_recoilFiring) {
                double envelope = 1.0;
                if (RECOIL_ATTACK_SEC > 0.0) {
                    g_recoilAttack += dt / RECOIL_ATTACK_SEC;
                    if (g_recoilAttack > 1.0) g_recoilAttack = 1.0;
                    envelope = g_recoilAttack;
                }
                rumX = (g_recoilPeak * envelope) *2;
                rumZ = (g_recoilPeak * envelope * RECOIL_VERTICAL);
                static DWORD lastPrint = 0;
                if (now - lastPrint > 50) {
                    lastPrint = now;
                    printf("\nFIRING: force=%.3f peak=%.3f firing=%d q=%d\n",
                        g_recoilForce, g_recoilPeak, (int)g_recoilFiring,
                        (g_recoilQHead - g_recoilQTail + RECOIL_QUEUE_SIZE) % RECOIL_QUEUE_SIZE);
                }

                if (g_recoilAttack >= 1.0) {
                    double frameDecay = pow(RECOIL_DECAY, dt * 1000.0);
                    g_recoilForce *= frameDecay;
                    g_recoilPeak *= frameDecay;
                    if (g_recoilForce < 0.05) {
                        g_recoilForce = 0.0;
                        g_recoilFiring = false;
                    }
                }
            }
        }
        else {
            g_recoilForce = 0.0;
            g_recoilPeak = 0.0;
            g_recoilAttack = 0.0;
            g_recoilFiring = false;
            double tmp;
            while (RecoilDequeue(tmp)) {}
        }

        if (now - g_lastRumbleTime > 100) {
            double liveMag = (g_rumbleLarge * 1.0 + g_rumbleSmall * 0.5);
            if (liveMag < RUMBLE_SUSTAIN_THRESHOLD) {
                double tmp;
                while (RecoilDequeue(tmp)) {}
                g_recoilForce = 0.0;
                g_recoilFiring = false;
            }
        }

        // Normal random rumble when not recoiling
        if (rumX == 0.0) {
            static double rumDirY = 1.0, rumDirZ = 0.0, rumDirX = 0.0;
            static DWORD  lastDirChange = 0;
            static const DWORD DIR_CHANGE_MS = 40;
            double mag = (rL * RUMBLE_LARGE_SCALE + rS * RUMBLE_SMALL_SCALE);
            if (mag > 0.01) {
                if (now - lastDirChange > DIR_CHANGE_MS) {
                    lastDirChange = now;
                    double angleYZ = ((rand() % 1000) / 1000.0) * 2.0 * 3.14159265;
                    double angleX = (((rand() % 1000) / 1000.0) - 0.5) * 3.14159265;
                    double cosX = cos(angleX);
                    rumDirY = cos(angleYZ) * cosX;
                    rumDirZ = sin(angleYZ) * cosX;
                    rumDirX = sin(angleX);
                }
                rumX = rumDirX * mag;
                rumY = rumDirY * mag;
                rumZ = rumDirZ * mag;
            }
        }

        g_rumbleLarge = rL * (float)RUMBLE_DECAY;
        g_rumbleSmall = rS * (float)RUMBLE_DECAY;

        ApplyForces(y, z, axY, axZ, axY.smoothVel, axZ.smoothVel, rumX, rumY, rumZ);

        // ── Button mask ────────────────────────────────────────────────────
        uint8_t btnMask = 0;
        for (int i = 0; i < 4; i++)
            if (dhdGetButton(i) > 0) btnMask |= (1u << i);

        // ── Send controller packet ─────────────────────────────────────────
        double stickScale = 1.0 - g_pushDamp;
        if (stickScale < 0.0) stickScale = 0.0;
        if (recoilActive && g_recoilFiring)
            stickScale *= RECOIL_AIM_DAMP;

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
            printf("\nRecalibrating - move to all extremes for 4 seconds...\n");
            axY = {}; axZ = {};
            DWORD t1 = GetTickCount();
            while (GetTickCount() - t1 < 4000) {
                double rx, ry, rz;
                if (dhdGetPosition(&rx, &ry, &rz) >= 0) {
                    axY.UpdateReach(ry);
                    axZ.UpdateReach(rz);
                }
                Sleep(1);
            }
            printf("Done. Y ctr=%.4f half=%.4f  Z ctr=%.4f half=%.4f\n",
                axY.estCenter, axY.halfRange, axZ.estCenter, axZ.halfRange);
            Sleep(200);
        }
        Sleep(UPDATE_RATE_MS);
    }

    dhdSetForce(0.0, 0.0, 0.0);
    dhdClose();
    SerialClose();
    DeleteCriticalSection(&g_recoilCS);
    printf("\nDone.\n");
    return 0;
}