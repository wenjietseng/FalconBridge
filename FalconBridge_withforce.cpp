#include "dhdc.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

// ------------------------------------------------------------
// Export declarations
// ------------------------------------------------------------

#if defined(_WIN32)
    #define FALCON_API __declspec(dllexport)
#else
    #define FALCON_API __attribute__((visibility("default")))
#endif

// ------------------------------------------------------------
// Vector utilities
// ------------------------------------------------------------

struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

static Vec3 operator+(const Vec3& a, const Vec3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 operator-(const Vec3& a, const Vec3& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 operator*(const Vec3& a, double s)
{
    return {a.x * s, a.y * s, a.z * s};
}

struct Quaternion
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

static Vec3 rotateVector(const Quaternion& q, const Vec3& v)
{
    // q * v * conjugate(q), assuming q is normalized.
    const Vec3 u{q.x, q.y, q.z};

    const double dotUV = u.x * v.x + u.y * v.y + u.z * v.z;

    const double dotUU = u.x * u.x + u.y * u.y + u.z * u.z;

    const Vec3 crossUV{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};

    return u * (2.0 * dotUV) + v * (q.w * q.w - dotUU) + crossUV * (2.0 * q.w);
}

static Vec3 mapDhdToTask(double x, double y, double z, const Vec3& framePosition, const Quaternion& frameRotation)
{
    // Existing validated axis mapping:
    // Task X = DHD Y
    // Task Y = DHD Z
    // Task Z = DHD X
    const Vec3 mapped{y, z, x};
    return framePosition + rotateVector(frameRotation, mapped);
}

static double magnitude(const Vec3& v)
{
    const double mag2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if (!std::isfinite(mag2)) return 0.0;
    else return std::sqrt(mag2);
}

// ------------------------------------------------------------
// Simulation parameters and shared state
// ------------------------------------------------------------

struct DeviceFrame
{
    Vec3 position;
    Quaternion rotation;
};

struct SimulationParameters
{
    double stiffness = 50.0;  // N/m
    double damping = 0.5;     // Ns/m
    double ringMass = 0.2;    // kg

    double graspThreshold = 1.0; // N
    double releaseThreshold = 0.5; // N
};

struct SimulationState
{
    Vec3 cursor1;
    Vec3 cursor2;

    Vec3 force1;
    Vec3 force2;

    Vec3 ringPosition;
    Vec3 ringVelocity;

    int participant1Pressing = 0;
    int participant2Pressing = 0;
    int ringGrasped = 0;
};

struct PositionSnapshot // unused, can removed afterwards
{
    Vec3 cursor1_pos;
    Vec3 cursor2_pos;
    Vec3 force;
};

static char deviceID1 = -1;
static char deviceID2 = -1;

static std::atomic<bool> simulationRunning{false};


// This implementation intentionally does not permit physical force output.
static std::atomic<bool> forceOutputEnabled{false}; // unused, can be removed afterwards
static constexpr bool PHYSICAL_FORCE_OUTPUT_ALLOWED = false;


static std::thread hapticThread;

static std::mutex configurationMutex;
static DeviceFrame deviceFrame1;
static DeviceFrame deviceFrame2;
static SimulationParameters parameters;

static Vec3 initialRingPosition;
static Vec3 initialContactOffset1;
static Vec3 initialContactOffset2;

static std::mutex stateMutex;
static SimulationState publishedState;

static Vec3 ringPosition;
static Vec3 ringVelocity;
static bool ringGrasped = false;

// Software limit on the interaction force.
// This is NOT a certified safety limit.
constexpr double MAX_DEVICE_FORCE_N = 2.0; // N
// Limits how quickly the commanded force can change.
constexpr double FORCE_SLEW_RATE_N_PER_SEC = 2.0;


// Velocity filter time constant.
static constexpr double VELOCITY_FILTER_TIME_CONSTANT = 0.01;
static constexpr double TARGET_PERIOD_SECONDS = 0.001;


// DHD button index used as the physical dead-man switch.
// Verify this button is physically convenient to hold on your Falcons.
constexpr int DEADMAN_BUTTON_INDEX = 0;
std::atomic<bool> g_forceEnabled[2] = {false, false};
std::atomic<bool> g_deadmanPressed[2] = {false, false};
std::atomic<bool> g_forceActive[2] = {false, false};
std::atomic<int> g_forceError[2] = {0, 0};
// Latched fault: a device read or force-command failure disables
// force output until the bridge is restarted.
std::atomic<bool> g_forceFault{ false };
Vec3 g_lastCommandedForceDhd[2] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

constexpr int FORCE_ERROR_NONE          = 0;
constexpr int FORCE_ERROR_POSITION_READ = 1;
constexpr int FORCE_ERROR_BUTTON_READ   = 2;
constexpr int FORCE_ERROR_FORCE_COMMAND = 3;
constexpr int FORCE_ERROR_INVALID_DATA  = 4;

// ------------------------------------------------------------
// Helper functions
// ------------------------------------------------------------
static bool finiteVec(const Vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

static bool validQuaternion(const Quaternion& q)
{
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) return false;

    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);

    return norm > 1e-8;
}

static Quaternion normalizedQuaternion(const Quaternion& q)
{
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);

    return {q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

static void publishState(const SimulationState& state)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    publishedState = state;
}

static void commandZeroForce()
{
    if (deviceID1 >= 0) dhdSetForce(0.0, 0.0, 0.0, deviceID1);
    if (deviceID2 >= 0) dhdSetForce(0.0, 0.0, 0.0, deviceID2);
}

static bool sendZeroForce(int deviceIndex)
{
    const int id = (deviceIndex == 0) ? deviceID1 : deviceID2;

    if (id < 0) return false;

    const int result = dhdSetForce(0.0, 0.0, 0.0, static_cast<char>(id));

    g_lastCommandedForceDhd[deviceIndex] = {0.0, 0.0, 0.0};

    g_forceActive[deviceIndex] = false;

    return result >= 0;
}

static void latchForceFault(int deviceIndex, int errorCode)
{
    g_forceError[deviceIndex] = errorCode;
    g_forceFault = true;

    // Immediately attempt zero on both devices.
    sendZeroForce(0);
    sendZeroForce(1);

    g_forceEnabled[0] = false;
    g_forceEnabled[1] = false;
}

extern "C" FALCON_API
void Falcon_SetDeviceForceEnabled(
    int deviceIndex,
    int enabled)
{
    if (deviceIndex < 0 || deviceIndex > 1)
        return;

    if (g_forceFault.load())
        return;

    // This function may be called from Unity's main thread.
    // Only update the atomic request here. The haptic thread is
    // the sole owner of dhdSetForce() while the simulation runs.
    g_forceEnabled[deviceIndex] = (enabled != 0);
}

// static Vec3 limitMagnitude(const Vec3& v, double maximum)
// {
//     const double m = magnitude(v);
//     if (m > maximum && m > 0.0) return v * (maximum / m);
//     return v;
// }

static Vec3 clampMagnitude(const Vec3& v, double maxMagnitude)
{
    const double mag = magnitude(v);
    if (mag <= maxMagnitude || mag < 1e-12) return v;
    const double scale = maxMagnitude / mag;
    return {v.x * scale, v.y * scale, v.z * scale};
}

static Vec3 taskForceToDhd(const Vec3& taskForce, const DeviceFrame& frame)
{
    // First undo the device-frame rotation: R^T * F_task.
    Quaternion inverseRotation = {
        -frame.rotation.x,
        -frame.rotation.y,
        -frame.rotation.z,
         frame.rotation.w
    };

    const Vec3 unrotated = rotateVector(inverseRotation, taskForce);
    
    // Undo the DHD -> task axis permutation:
    // task = (DHD_y, DHD_z, DHD_x)
    // Then undo the axis permutation: P^T * R^T * F_task.
    return {unrotated.z, unrotated.x, unrotated.y};
}

static Vec3 calculateDeviceReactionDhd(const Vec3& springForceTask, const DeviceFrame& frame)
{
    const Vec3 reactionTask = {
        -springForceTask.x,
        -springForceTask.y,
        -springForceTask.z
    };

    return taskForceToDhd(reactionTask, frame);
}

static Vec3 applySlewRateLimit(const Vec3& target, const Vec3& previous, double dt)
{
    if (!std::isfinite(dt) || dt <= 0.0) return {0.0, 0.0, 0.0};

    const Vec3 delta = {
        target.x - previous.x,
        target.y - previous.y,
        target.z - previous.z
    };

    const double maxChange = FORCE_SLEW_RATE_N_PER_SEC * dt;

    const Vec3 limitedDelta = clampMagnitude(delta, maxChange);

    return {
        previous.x + limitedDelta.x,
        previous.y + limitedDelta.y,
        previous.z + limitedDelta.z
    };
}

// ------------------------------------------------------------
// Haptic simulation thread
// ------------------------------------------------------------

static void hapticLoop()
{
    using Clock = std::chrono::steady_clock;

    Vec3 previous_cursor1_pos{0.0, 0.0, 0.0};
    Vec3 previous_cursor2_pos{0.0, 0.0, 0.0};

    bool firstSample = true;

    auto previousTime = Clock::now();
    auto nextTick = previousTime;

    const auto targetPeriod = std::chrono::microseconds(1000);

    while (simulationRunning.load())
    {
        nextTick += targetPeriod;

        // ----------------------------------------------------
        // Read Falcon position
        // ----------------------------------------------------

        double px1 = 0.0, py1 = 0.0, pz1 = 0.0;
        double px2 = 0.0, py2 = 0.0, pz2 = 0.0;

        const int position_result1 = dhdGetPosition(&px1, &py1, &pz1, deviceID1);
        const int position_result2 = dhdGetPosition(&px2, &py2, &pz2, deviceID2);

        if (position_result1 < 0)
        {
            latchForceFault(0, FORCE_ERROR_POSITION_READ);
            break;
        }

        if (position_result2 < 0)
        {
            latchForceFault(1, FORCE_ERROR_POSITION_READ);
            break;
        }

        if (!std::isfinite(px1) || !std::isfinite(py1) || !std::isfinite(pz1) ||
            !std::isfinite(px2) || !std::isfinite(py2) || !std::isfinite(pz2))
        {
            latchForceFault(0, FORCE_ERROR_INVALID_DATA);
            break;
        }

        const int button1 = dhdGetButton(DEADMAN_BUTTON_INDEX, deviceID1);
        const int button2 = dhdGetButton(DEADMAN_BUTTON_INDEX, deviceID2);

        if (button1 < 0)
        {
            g_deadmanPressed[0] = false;
            latchForceFault(0, FORCE_ERROR_BUTTON_READ);
            break;
        }

        if (button2 < 0)
        {
            g_deadmanPressed[1] = false;
            latchForceFault(1, FORCE_ERROR_BUTTON_READ);
            break;
        }

        g_deadmanPressed[0] = (button1 != 0);
        g_deadmanPressed[1] = (button2 != 0);

        DeviceFrame frame1;
        DeviceFrame frame2;
        SimulationParameters p;
        Vec3 contactOffset1;
        Vec3 contactOffset2;

        {
            std::lock_guard<std::mutex> lock(configurationMutex);
            frame1 = deviceFrame1;
            frame2 = deviceFrame2;
            p = parameters;
            contactOffset1 = initialContactOffset1;
            contactOffset2 = initialContactOffset2;
        }

        const Vec3 cursor1 = mapDhdToTask(px1, py1, pz1, frame1.position, frame1.rotation);
        const Vec3 cursor2 = mapDhdToTask(px2, py2, pz2, frame2.position, frame2.rotation);
        // const Vec3 current_cursor1_pos{px1, py1, pz1};
        // const Vec3 current_cursor2_pos{px2, py2, pz2};

        // ----------------------------------------------------
        // Calculate timestep and cursor velocity
        // ----------------------------------------------------
        const auto currentTime = Clock::now();

        double dt = std::chrono::duration<double>(currentTime - previousTime).count();

        previousTime = currentTime;

        // Avoid integrating with an invalid or excessively
        // large timestep after a scheduling interruption.
        if (dt <= 0.0 || dt > 0.01)
        {
            dt = TARGET_PERIOD_SECONDS;
            // filtered_cursor1_velocity = {0.0, 0.0, 0.0};
            // filtered_cursor2_velocity = {0.0, 0.0, 0.0};
            firstSample = true;
        }

        Vec3 cursor1_velocity{0.0, 0.0, 0.0};
        Vec3 cursor2_velocity{0.0, 0.0, 0.0};

        if (!firstSample)
        {
            cursor1_velocity = (cursor1 - previous_cursor1_pos) * (1.0 / dt);
            cursor2_velocity = (cursor2 - previous_cursor2_pos) * (1.0 / dt);
        }

        previous_cursor1_pos = cursor1;
        previous_cursor2_pos = cursor2;
        firstSample = false;

        // ----------------------------------------------------
        // Contact points follow ring translation.
        // Ring rotation is fixed.
        // ----------------------------------------------------
        const Vec3 contact1 = ringPosition + contactOffset1;
        const Vec3 contact2 = ringPosition + contactOffset2;

        const Vec3 contactVelocity1 = ringVelocity;
        const Vec3 contactVelocity2 = ringVelocity;

        const Vec3 displacement1 = cursor1 - contact1;
        const Vec3 displacement2 = cursor2 - contact2;

        const Vec3 relativeVelocity1 = cursor1_velocity - contactVelocity1;
        const Vec3 relativeVelocity2 = cursor2_velocity - contactVelocity2;

        const Vec3 force1 = displacement1 * p.stiffness + relativeVelocity1 * p.damping;
        const Vec3 force2 = displacement2 * p.stiffness + relativeVelocity2 * p.damping;

        if (!finiteVec(force1) || !finiteVec(force2) ||
            !validQuaternion(frame1.rotation) || !validQuaternion(frame2.rotation))
        {
            latchForceFault(0, FORCE_ERROR_INVALID_DATA);
            break;
        }

        const bool participant1Pressing = force1.z >= p.graspThreshold;
        const bool participant2Pressing = force2.z >= p.graspThreshold;

        if (!ringGrasped)
        {
            if (participant1Pressing && participant2Pressing)
            {
                ringGrasped = true;
                ringVelocity = {0.0, 0.0, 0.0};
            }
        }
        else
        {
            if (force1.z < p.releaseThreshold || force2.z < p.releaseThreshold)
            {
                ringGrasped = false;
                ringVelocity = {0.0, 0.0, 0.0};
            }
        }

        // ----------------------------------------------------
        // Ring lateral dynamics
        // Semi-implicit Euler integration
        // ----------------------------------------------------

        if (ringGrasped)
        {
            const Vec3 lateralForce{force1.x + force2.x, force1.y + force2.y, 0.0};

            const Vec3 acceleration = lateralForce * (1.0 / p.ringMass);

            ringVelocity = ringVelocity + acceleration * dt;
            ringVelocity.z = 0.0;

            ringPosition = ringPosition + ringVelocity * dt;
        }
        else
        {
            ringVelocity = {0.0, 0.0, 0.0};
        }

        // Ring Z remains fixed at its initial value.
        // The initial Z is stored in initialRingPosition.
        ringPosition.z = initialRingPosition.z;

        //----------------------------------------------------
        // render force to each device
        // ---------------------------------------------------
        // commandZeroForce();
        const bool globalForceAllowed = !g_forceFault.load();
        const bool device1Allowed = globalForceAllowed && g_forceEnabled[0].load() && g_deadmanPressed[0].load();
        const bool device2Allowed = globalForceAllowed && g_forceEnabled[1].load() && g_deadmanPressed[1].load();

        const Vec3 zero = {0.0, 0.0, 0.0};

        Vec3 target1 = zero;
        Vec3 target2 = zero;

        if (device1Allowed) target1 = calculateDeviceReactionDhd(force1, frame1);
        if (device2Allowed) target2 = calculateDeviceReactionDhd(force2, frame2);

        // Limit the magnitude of each device's command independently.
        target1 = clampMagnitude(target1, MAX_DEVICE_FORCE_N);
        target2 = clampMagnitude(target2, MAX_DEVICE_FORCE_N);

        // Apply a slew-rate limit to avoid abrupt force transitions.
        target1 = applySlewRateLimit(target1, g_lastCommandedForceDhd[0], dt);
        target2 = applySlewRateLimit(target2, g_lastCommandedForceDhd[1], dt);

        // Releasing the dead-man switch or disabling a device should
        // command zero immediately, rather than ramping down.
        if (!device1Allowed) target1 = zero;
        if (!device2Allowed) target2 = zero;

        if (!finiteVec(target1) || !finiteVec(target2))
        {
            latchForceFault(0, FORCE_ERROR_INVALID_DATA);
            break;
        }

        const int result1 = dhdSetForce(target1.x, target1.y, target1.z, static_cast<char>(deviceID1));
        if (result1 < 0)
        {
            latchForceFault(0, FORCE_ERROR_FORCE_COMMAND);
            break;
        }
        g_lastCommandedForceDhd[0] = target1;
        g_forceActive[0] = device1Allowed && (target1.x != 0.0 || target1.y != 0.0 || target1.z != 0.0);

        const int result2 = dhdSetForce(target2.x, target2.y, target2.z, static_cast<char>(deviceID2));
        if (result2 < 0)
        {
            latchForceFault(1, FORCE_ERROR_FORCE_COMMAND);
            break;
        }
        g_lastCommandedForceDhd[1] = target2;
        g_forceActive[1] = device2Allowed && (target2.x != 0.0 || target2.y != 0.0 || target2.z != 0.0);

        SimulationState state;

        state.cursor1 = cursor1;
        state.cursor2 = cursor2;

        state.force1 = force1;
        state.force2 = force2;

        state.ringPosition = ringPosition;
        state.ringVelocity = ringVelocity;

        state.participant1Pressing = participant1Pressing ? 1 : 0;
        state.participant2Pressing = participant2Pressing ? 1 : 0;

        state.ringGrasped = ringGrasped ? 1 : 0;
        publishState(state);

        // Avoid a catch-up burst if the thread was interrupted.
        const auto now = Clock::now();

        if (nextTick < now) nextTick = now;

        std::this_thread::sleep_until(nextTick);
    }

    // commandZeroForce();
    sendZeroForce(0);
    sendZeroForce(1);
    g_forceActive[0] = false;
    g_forceActive[1] = false;
    simulationRunning.store(false);
}

// ------------------------------------------------------------
// Public C API
// ------------------------------------------------------------

extern "C"
{

FALCON_API int Falcon_Init()
{
    if (deviceID1 >= 0 && deviceID2 >= 0) return 0;

    if (deviceID1 >= 0)
    {
        dhdClose(deviceID1);
        deviceID1 = -1;
    }

    if (deviceID2 >= 0)
    {
        dhdClose(deviceID2);
        deviceID2 = -1;
    }

    int count = dhdGetDeviceCount();

    if (count < 2) return -1;

    int id1 = dhdOpenID(0);
    if (id1 < 0) return -2;

    int id2 = dhdOpenID(1);
    if (id2 < 0) 
    {
        dhdClose(static_cast<char>(id1));
        return -3;
    }

    if (id1 == id2)
    {
        dhdClose(static_cast<char>(id1));
        dhdClose(static_cast<char>(id2));
        return -4;
    }

    deviceID1 = static_cast<char>(id1);
    deviceID2 = static_cast<char>(id2);

    return 0;
}

FALCON_API int Falcon_GetDeviceIDs(int* id1, int* id2)
{
    if (id1 == nullptr || id2 == nullptr) return -1;

    if (deviceID1 < 0 || deviceID2 < 0) return -2;

    *id1 = static_cast<int>(deviceID1);
    *id2 = static_cast<int>(deviceID2);

    return 0;
}

// deviceIndex: 1 or 2
// Translation is in metres.
// Quaternion uses Unity's x,y,z,w convention.
FALCON_API int Falcon_SetDeviceFrame(int deviceIndex, double tx, double ty, double tz, double qx, double qy, double qz, double qw)
{
    if (simulationRunning.load()) return -1;

    const Vec3 position{tx, ty, tz};
    Quaternion rotation{qx, qy, qz, qw};

    if (!finiteVec(position) || !validQuaternion(rotation)) return -2;

    rotation = normalizedQuaternion(rotation);

    std::lock_guard<std::mutex> lock(configurationMutex);

    if (deviceIndex == 1)
    {
        deviceFrame1.position = position;
        deviceFrame1.rotation = rotation;
    }
    else if (deviceIndex == 2)
    {
        deviceFrame2.position = position;
        deviceFrame2.rotation = rotation;
    }
    else
    {
        return -3;
    }

    return 0;
}

FALCON_API int Falcon_SetSimulationParameters(double stiffness, double damping, double mass, double graspThreshold, double releaseThreshold)
{
    if (simulationRunning.load()) return -1;

    if (!std::isfinite(stiffness) || !std::isfinite(damping) || !std::isfinite(mass) || !std::isfinite(graspThreshold) || !std::isfinite(releaseThreshold)) return -2;

    if (stiffness < 0.0 || damping < 0.0 || mass <= 0.0 || releaseThreshold < 0.0 || graspThreshold <= releaseThreshold) return -3;

    std::lock_guard<std::mutex> lock(configurationMutex);

    parameters.stiffness = stiffness;
    parameters.damping = damping;
    parameters.ringMass = mass;
    parameters.graspThreshold = graspThreshold;
    parameters.releaseThreshold = releaseThreshold;

    return 0;
}

// FALCON_API int Falcon_SetParameters(
//     double stiffness,
//     double damping,
//     double mass)
// {
//     if (stiffness < 0.0 ||
//         damping < 0.0 ||
//         mass <= 0.0)
//     {
//         return -1;
//     }

//     std::lock_guard<std::mutex> lock(parameterMutex);

//     parameters.stiffness = stiffness;
//     parameters.damping = damping;
//     parameters.ringMass = mass;

//     return 0;
// }

// Ring position and contact offsets are in task-frame metres.
// Contact offsets are relative to the ring centre.
FALCON_API int Falcon_SetInitialRingState(double rx, double ry, double rz, double c1x, double c1y, double c1z, double c2x, double c2y, double c2z)
{
    if (simulationRunning.load()) return -1;

    const Vec3 r{rx, ry, rz};
    const Vec3 c1{c1x, c1y, c1z};
    const Vec3 c2{c2x, c2y, c2z};

    if (!finiteVec(r) || !finiteVec(c1) || !finiteVec(c2)) return -2;

    std::lock_guard<std::mutex> lock(configurationMutex);

    initialRingPosition = r;
    initialContactOffset1 = c1;
    initialContactOffset2 = c2;

    ringPosition = r;
    ringVelocity = {0.0, 0.0, 0.0};
    ringGrasped = false;

    return 0;
}

FALCON_API int Falcon_ResetRing()
{
    if (simulationRunning.load()) return -1;

    std::lock_guard<std::mutex> lock(configurationMutex);

    ringPosition = initialRingPosition;
    ringVelocity = {0.0, 0.0, 0.0};
    ringGrasped = false;

    return 0;
}

FALCON_API int Falcon_SetForceOutputEnabled(int enabled)
{
    // Legacy all-devices safety control retained for compatibility.
    // Physical output is now controlled independently per device.
    // Never call dhdSetForce() from Unity's main thread.
    if (enabled != 0) return -1;

    g_forceEnabled[0] = false;
    g_forceEnabled[1] = false;
    return 0;
}


FALCON_API int Falcon_StartSimulation()
{
    if (deviceID1 < 0 || deviceID2 < 0) return -1;

    if (simulationRunning.load()) return 0;

    // Join a previous thread if it exited after an error.
    if (hapticThread.joinable()) hapticThread.join();

    // Reset the force state before launching the haptic thread.
    // This prevents the thread from overwriting an early Unity arm request.
    g_forceFault = false;
    for (int i = 0; i < 2; ++i)
    {
        g_forceEnabled[i] = false;
        g_deadmanPressed[i] = false;
        g_forceActive[i] = false;
        g_forceError[i] = FORCE_ERROR_NONE;
        g_lastCommandedForceDhd[i] = {0.0, 0.0, 0.0};
    }

    simulationRunning.store(true);

    hapticThread = std::thread(hapticLoop);

    return 0;
}

// 18 double outputs:
// cursor1 xyz, cursor2 xyz,
// force1 xyz, force2 xyz,
// ringPosition xyz, ringVelocity xyz.
// Three integer outputs:
// participant1Pressing, participant2Pressing, ringGrasped.
FALCON_API int Falcon_GetSimulationState(
    double* c1x, double* c1y, double* c1z,
    double* c2x, double* c2y, double* c2z,
    double* f1x, double* f1y, double* f1z,
    double* f2x, double* f2y, double* f2z,
    double* rx, double* ry, double* rz,
    double* vx, double* vy, double* vz,
    int* pressing1,
    int* pressing2,
    int* grasped)
{
    if (!c1x || !c1y || !c1z ||
        !c2x || !c2y || !c2z ||
        !f1x || !f1y || !f1z ||
        !f2x || !f2y || !f2z ||
        !rx || !ry || !rz ||
        !vx || !vy || !vz ||
        !pressing1 || !pressing2 || !grasped)
        return -1;

    SimulationState s;

    {
        std::lock_guard<std::mutex> lock(stateMutex);
        s = publishedState;
    }

    *c1x = s.cursor1.x;
    *c1y = s.cursor1.y;
    *c1z = s.cursor1.z;

    *c2x = s.cursor2.x;
    *c2y = s.cursor2.y;
    *c2z = s.cursor2.z;

    *f1x = s.force1.x;
    *f1y = s.force1.y;
    *f1z = s.force1.z;

    *f2x = s.force2.x;
    *f2y = s.force2.y;
    *f2z = s.force2.z;

    *rx = s.ringPosition.x;
    *ry = s.ringPosition.y;
    *rz = s.ringPosition.z;

    *vx = s.ringVelocity.x;
    *vy = s.ringVelocity.y;
    *vz = s.ringVelocity.z;

    *pressing1 = s.participant1Pressing;
    *pressing2 = s.participant2Pressing;
    *grasped = s.ringGrasped;

    return 0;
}

FALCON_API void Falcon_GetForceStatus(
    int* enabled1,
    int* enabled2,
    int* deadman1,
    int* deadman2,
    int* active1,
    int* active2,
    int* error1,
    int* error2,
    int* fault)
{
    if (enabled1) *enabled1 = g_forceEnabled[0] ? 1 : 0;
    if (enabled2) *enabled2 = g_forceEnabled[1] ? 1 : 0;

    if (deadman1) *deadman1 = g_deadmanPressed[0] ? 1 : 0;
    if (deadman2) *deadman2 = g_deadmanPressed[1] ? 1 : 0;

    if (active1) *active1 = g_forceActive[0] ? 1 : 0;
    if (active2) *active2 = g_forceActive[1] ? 1 : 0;

    if (error1) *error1 = g_forceError[0].load();
    if (error2) *error2 = g_forceError[1].load();

    if (fault) *fault = g_forceFault ? 1 : 0;
}

// FALCON_API int Falcon_GetDev1Position(double* x, double* y, double* z)
// {
//     if (!x || !y || !z) return -1;

//     std::lock_guard<std::mutex> lock(snapshotMutex);

//     *x = snapshot.cursor1_pos.x;
//     *y = snapshot.cursor1_pos.y;
//     *z = snapshot.cursor1_pos.z;

//     return 0;
// }

// FALCON_API int Falcon_GetDev2Position(double* x, double* y, double* z)
// {
//     if (!x || !y || !z) return -1;

//     std::lock_guard<std::mutex> lock(snapshotMutex);

//     *x = snapshot.cursor2_pos.x;
//     *y = snapshot.cursor2_pos.y;
//     *z = snapshot.cursor2_pos.z;

//     return 0;
// }

// FALCON_API int Falcon_GetRingPosition(double* x, double* y, double* z)
// {
//     if (!x || !y || !z) return -1;

//     std::lock_guard<std::mutex> lock(snapshotMutex);

//     *x = snapshot.ring_pos.x;
//     *y = snapshot.ring_pos.y;
//     *z = snapshot.ring_pos.z;

//     return 0;
// }


FALCON_API int Falcon_StopSimulation()
{
    g_forceEnabled[0] = false;
    g_forceEnabled[1] = false;
    simulationRunning.store(false);

    if (hapticThread.joinable()) hapticThread.join();

    // The haptic thread has exited; it is now safe to issue final zeros here.
    commandZeroForce();

    return 0;
}


FALCON_API int Falcon_Close()
{
    Falcon_StopSimulation();

    if (deviceID1 >= 0)
    {
        dhdClose(deviceID1);
        deviceID1 = -1;
    }

    if (deviceID2 >= 0)
    {
        dhdClose(deviceID2);
        deviceID2 = -1;
    }

    return 0;
}

} // extern "C"