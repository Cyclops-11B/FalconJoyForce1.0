// FalconToMouse.cpp - Falcon to Mouse with Hybrid Control
#include <windows.h>
#include <stdio.h>
#include <math.h>

// Force Dimension SDK
#include <dhdc.h>

// Configuration
const int UPDATE_RATE_MS = 1;           // 1ms = 1000Hz update
const bool INVERT_Y = false;              // Invert Y axis if needed
const bool INVERT_X = false;              // Invert X axis if needed

// Mouse sensitivity in normal mode (pixels per unit of physical movement)
const double NORMAL_SENSITIVITY_X = 500.0;
const double NORMAL_SENSITIVITY_Y = 500.0;

// Mouse speed in "push" mode (pixels per second when at boundary)
const double PUSH_SPEED_X = 1000.0;       // Pixels per second
const double PUSH_SPEED_Y = 1000.0;

// Threshold for entering push mode (percentage of physical range)
const double PUSH_THRESHOLD = 0.85;        // 85% of physical range

// Deadzone (physical units)
const double DEADZONE = 0.005;

// ============ DRIFT COMPENSATION ============
struct DriftCompensator {
    double physicalMin, physicalMax, physicalCenter;
    double observedMin, observedMax;
    double lastValue;
    int samplesAtLimit;
    int axisId;

    DriftCompensator(int axis = 0) {
        physicalMin = -0.06;
        physicalMax = 0.06;
        physicalCenter = 0.0;
        observedMin = 999.0;
        observedMax = -999.0;
        lastValue = 0.0;
        samplesAtLimit = 0;
        axisId = axis;
    }

    void AddSample(double value) {
        if (value < observedMin) observedMin = value;
        if (value > observedMax) observedMax = value;

        bool isAtMax = (value >= physicalMax - 0.001 && value > lastValue);
        bool isAtMin = (value <= physicalMin + 0.001 && value < lastValue);

        if (isAtMax) {
            samplesAtLimit++;
            if (samplesAtLimit > 10) {
                physicalMax = value;
                double range = physicalMax - physicalMin;
                physicalMin = physicalMax - range;
                samplesAtLimit = 0;
                printf(".");
            }
        }
        else if (isAtMin) {
            samplesAtLimit++;
            if (samplesAtLimit > 10) {
                physicalMin = value;
                double range = physicalMax - physicalMin;
                physicalMax = physicalMin + range;
                samplesAtLimit = 0;
                printf(".");
            }
        }
        else {
            samplesAtLimit = 0;
        }

        physicalCenter = (physicalMin + physicalMax) / 2.0;
        lastValue = value;
    }

    // Get normalized position (-1.0 to 1.0)
    double GetNormalizedPosition(double value) {
        double range = physicalMax - physicalMin;
        if (range < 0.001) range = 0.1;

        // Calculate position from min to max (0 to 1)
        double normalized = (value - physicalMin) / range;

        // Convert to -1 to 1 range (centered)
        return (normalized * 2.0) - 1.0;
    }

    // Get position as percentage from center (0 to 1, with sign)
    double GetOffsetFromCenter(double value) {
        double range = physicalMax - physicalMin;
        if (range < 0.001) range = 0.1;

        double offset = value - physicalCenter;
        return offset / (range / 2.0);  // -1 to 1
    }

    // Check if we're in push mode (near the edge)
    bool IsInPushMode(double value) {
        double offset = fabs(GetOffsetFromCenter(value));
        return offset >= PUSH_THRESHOLD;
    }

    // Get push direction (-1, 0, or 1)
    int GetPushDirection(double value) {
        if (!IsInPushMode(value)) return 0;
        return (GetOffsetFromCenter(value) > 0) ? 1 : -1;
    }

    void Calibrate() {
        printf("\nCalibrating axis %d... Move to all extremes\n", axisId);

        double val;
        double tempMin = 999.0, tempMax = -999.0;
        DWORD startTime = GetTickCount();

        while (GetTickCount() - startTime < 3000) {
            double x, y, z;
            if (dhdGetPosition(&x, &y, &z) >= 0) {
                if (axisId == 0) val = x;
                else if (axisId == 1) val = y;
                else val = z;

                if (val < tempMin) tempMin = val;
                if (val > tempMax) tempMax = val;
            }
            Sleep(1);
        }

        if (tempMin < 999.0 && tempMax > -999.0) {
            physicalMin = tempMin;
            physicalMax = tempMax;
            physicalCenter = (physicalMin + physicalMax) / 2.0;
        }

        printf("Axis %d calibrated: [%+.3f to %+.3f] center %+.3f\n",
            axisId, physicalMin, physicalMax, physicalCenter);
    }

    void PrintStats(const char* axisName, double value) {
        double norm = GetOffsetFromCenter(value);
        double pos = GetNormalizedPosition(value);
        bool pushMode = IsInPushMode(value);
        int pushDir = GetPushDirection(value);

        printf("%s: val[%+.4f] center[%+.3f] offset[%+.2f] pos[%+.2f] %s %s\n",
            axisName, value, physicalCenter, norm, pos,
            pushMode ? "PUSH" : "    ",
            pushDir == 1 ? ">>" : (pushDir == -1 ? "<<" : "  "));
    }
};

// Mouse control with hybrid mode
void MoveMouseHybrid(double offsetX, double offsetY,
    bool pushModeX, bool pushModeY,
    int pushDirX, int pushDirY,
    double deltaTime) {

    int pixelsX = 0, pixelsY = 0;

    // Handle X axis
    if (pushModeX && pushDirX != 0) {
        // Push mode - continuous movement in direction
        pixelsX = (int)(pushDirX * PUSH_SPEED_X * deltaTime);
    }
    else if (fabs(offsetX) > DEADZONE) {
        // Normal mode - position-based movement
        pixelsX = (int)(offsetX * NORMAL_SENSITIVITY_X);
    }

    // Handle Y axis
    if (pushModeY && pushDirY != 0) {
        // Push mode - continuous movement in direction
        pixelsY = (int)(pushDirY * PUSH_SPEED_Y * deltaTime);
    }
    else if (fabs(offsetY) > DEADZONE) {
        // Normal mode - position-based movement
        pixelsY = (int)(offsetY * NORMAL_SENSITIVITY_Y);
    }

    // Apply inversion
    if (INVERT_X) pixelsX = -pixelsX;
    if (INVERT_Y) pixelsY = -pixelsY;

    // Move mouse
    if (pixelsX != 0 || pixelsY != 0) {
        POINT currentPos;
        GetCursorPos(&currentPos);
        SetCursorPos(currentPos.x + pixelsX, currentPos.y + pixelsY);
    }
}

// Mouse click simulation
void SimulateMouseClick(int button, bool down) {
    INPUT input = { 0 };
    input.type = INPUT_MOUSE;

    switch (button) {
    case 0: // Left button
        input.mi.dwFlags = down ? MOUSEEVENTF_LEFTDOWN : MOUSEEVENTF_LEFTUP;
        break;
    case 1: // Right button
        input.mi.dwFlags = down ? MOUSEEVENTF_RIGHTDOWN : MOUSEEVENTF_RIGHTUP;
        break;
    case 2: // Middle button
        input.mi.dwFlags = down ? MOUSEEVENTF_MIDDLEDOWN : MOUSEEVENTF_MIDDLEUP;
        break;
    }

    SendInput(1, &input, sizeof(INPUT));
}

int main() {
    printf("========================================\n");
    printf("Falcon to Mouse - HYBRID CONTROL\n");
    printf("========================================\n\n");
    printf("Center region: Precise relative control\n");
    printf("Edge region: Continuous push mode\n");
    printf("Push threshold: %.0f%% of travel\n\n", PUSH_THRESHOLD * 100);

    // Initialize Falcon
    printf("Initializing Falcon...\n");
    if (dhdOpen() < 0) {
        printf("ERROR: Failed to open Falcon\n");
        printf("Press Enter to exit...");
        getchar();
        return -1;
    }
    printf("Falcon opened successfully\n");
    printf("SDK Version: %s\n", dhdGetSDKVersionStr());

    // Create compensators for each axis
    DriftCompensator compX(0), compY(1), compZ(2);

    // Initial calibration
    printf("\n");
    compX.Calibrate();
    compY.Calibrate();
    compZ.Calibrate();

    printf("\n=== HYBRID MOUSE CONTROL ACTIVE ===\n");
    printf("Move the Falcon to control the mouse cursor\n");
    printf("Push to the edge to continue moving in that direction\n");
    printf("Buttons: Button0 = Left Click, Button1 = Right Click\n");
    printf("Dots appear when drift compensation adjusts\n");
    printf("Press 'D' for debug, 'C' to recalibrate, 'Q' to quit\n\n");

    // Button state tracking
    bool lastButtonState[4] = { false, false, false, false };
    bool currentButtonState[4] = { false, false, false, false };

    // Timing for push mode
    DWORD lastFrameTime = GetTickCount();

    // Main loop
    int frameCount = 0;
    DWORD lastTime = GetTickCount();
    DWORD lastDebugTime = lastTime;
    bool running = true;
    bool debugMode = false;

    while (running) {
        // Calculate delta time for smooth push movement
        DWORD currentTime = GetTickCount();
        double deltaTime = (currentTime - lastFrameTime) / 1000.0; // Convert to seconds
        if (deltaTime > 0.1) deltaTime = 0.01; // Cap at 100ms to avoid jumps
        lastFrameTime = currentTime;

        // Read Falcon position
        double x, y, z;
        if (dhdGetPosition(&x, &y, &z) < 0) {
            printf("Lost connection to Falcon\n");
            break;
        }

        // Update compensators
        compX.AddSample(x);
        compY.AddSample(y);
        compZ.AddSample(z);

        // Get offset from center and push mode status
        double offsetX = compX.GetOffsetFromCenter(x);
        double offsetY = compY.GetOffsetFromCenter(y);

        bool pushModeX = compX.IsInPushMode(x);
        bool pushModeY = compY.IsInPushMode(y);

        int pushDirX = compX.GetPushDirection(x);
        int pushDirY = compY.GetPushDirection(y);

        // Control mouse with hybrid method
        MoveMouseHybrid(offsetX, offsetY,
            pushModeX, pushModeY,
            pushDirX, pushDirY,
            deltaTime);

        // Read and handle buttons
        for (int i = 0; i < 4; i++) {
            currentButtonState[i] = (dhdGetButton(i) != 0);

            if (currentButtonState[i] != lastButtonState[i]) {
                if (i == 0) SimulateMouseClick(0, currentButtonState[i]);
                else if (i == 1) SimulateMouseClick(1, currentButtonState[i]);
                lastButtonState[i] = currentButtonState[i];
            }
        }

        // Frame rate counter
        frameCount++;
        DWORD now = GetTickCount();
        if (now - lastTime >= 1000) {
            if (!debugMode) {
                char modeX = pushModeX ? (pushDirX > 0 ? '>' : '<') : ' ';
                char modeY = pushModeY ? (pushDirY > 0 ? 'v' : '^') : ' ';

                printf("\r%d Hz | Buttons:%d%d%d%d | Mode: %c %c | Offset: X%+.2f Y%+.2f  ",
                    frameCount,
                    currentButtonState[0], currentButtonState[1],
                    currentButtonState[2], currentButtonState[3],
                    modeX, modeY, offsetX, offsetY);
                fflush(stdout);
            }
            frameCount = 0;
            lastTime = now;
        }

        // Debug output
        if (debugMode && (now - lastDebugTime >= 1000)) {
            printf("\n=== DRIFT COMPENSATION ===\n");
            compX.PrintStats("X", x);
            compY.PrintStats("Y", y);
            compZ.PrintStats("Z", z);
            printf("Delta time: %.3fs\n", deltaTime);
            printf("Push mode: X=%s (%d) Y=%s (%d)\n",
                pushModeX ? "YES" : "NO", pushDirX,
                pushModeY ? "YES" : "NO", pushDirY);
            printf("==========================\n\n");
            lastDebugTime = now;
        }

        // Check for keys
        if (GetAsyncKeyState('Q') & 0x8000) {
            running = false;
        }
        if (GetAsyncKeyState('D') & 0x8000) {
            debugMode = !debugMode;
            printf("\nDebug mode %s\n", debugMode ? "ON" : "OFF");
            Sleep(200);
        }
        if (GetAsyncKeyState('C') & 0x8000) {
            printf("\nRecalibrating...\n");
            compX.Calibrate();
            compY.Calibrate();
            compZ.Calibrate();
            Sleep(200);
        }

        Sleep(UPDATE_RATE_MS);
    }

    // Cleanup
    printf("\n\nShutting down...\n");
    dhdClose();
    printf("Done.\n");

    return 0;
}