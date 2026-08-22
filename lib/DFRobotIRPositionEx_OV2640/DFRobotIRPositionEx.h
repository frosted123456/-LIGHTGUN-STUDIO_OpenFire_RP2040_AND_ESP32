/*!
 * DFRobotIRPositionEx.h — OV2640 backend for the OpenFIRE camera interface.
 *
 * Drop-in replacement for the DFRobot/PAJ7025 camera class: same public
 * interface, but served from the latest frame published by the OV2640 capture
 * stack through ov2640_bridge.h rather than over I2C.
 *
 * Coordinate space: the bridge carries native x16 subpixel units; this shim
 * scales them into the interface's output space (see the OUT_MAX defines).
 * Sensitivity_e is accepted and stored but maps to nothing.
 */
#ifndef DFRobotIRPositionEx_h
#define DFRobotIRPositionEx_h

#include <stdint.h>
#include "ov2640_bridge.h"
#if defined(ESP_PLATFORM)
#include "ov2640_capture.h"   // firmware build: begin() boots the camera
#include "esp_timer.h"

// Latency probe counters, DEFINED in ov2640_capture.cpp and only declared here
// (C++ linkage must match there; an `extern "C"` block would not link).
//   ov2640_pub_t_us         — esp_timer stamp taken at publish (capture side)
//   ov2640_shim_lat_last_us — age of the frame OpenFIRE just consumed, us
//   ov2640_shim_lat_us      — worst age since the last LAT/s report, us
extern volatile uint32_t ov2640_pub_t_us;
extern volatile uint32_t ov2640_shim_lat_us;
extern volatile uint32_t ov2640_shim_lat_last_us;
#endif

// Output coordinate space handed to OpenFIRE's solvers. Raise both if a larger
// space is ever verified against OpenFIREConst.h's CamResX/CamResY.
#define OV2640_IRPOS_OUT_MAX_X 1023
#define OV2640_IRPOS_OUT_MAX_Y 767

// Pre-mirror X so OpenFIRE's internal un-mirror (`CamMaxX - px[i]`) lands the
// right way round. Set to 0 if a module/mounting lands mirrored the other way
// (symptom: left/right swapped).
#define OV2640_IRPOS_MIRROR_X 1

class TwoWire;   // fwd-decl so the I2C-style ctor signature stays available
class SPIClass;  // (and the wrapper-style one) — both ignored internally.

// OpenFIRE camera interface served from the OV2640 bridge instead of I2C.
class DFRobotIRPositionEx {
public:
    // The quad in NATIVE camera px, corner order as the resolver emits it
    // (TL TR BL BR). NOT a duplicate of positionX/Y, which are OpenFIRE's
    // 0..1023 space with X mirrored; aim_core's calibration is fitted in
    // native px and must be fed this.
    float    nativeX[4] = {0,0,0,0};
    float    nativeY[4] = {0,0,0,0};
    uint16_t nativeW = 0, nativeH = 0;
    int      nativeCount = 0;

    enum DataFormat_e {
        DataFormat_Basic    = 0,
        DataFormat_Extended = 1
    };
    enum Sensitivity_e {
        Sensitivity_Min     = 0,
        Sensitivity_Default = 0,
        Sensitivity_High    = 1,
        Sensitivity_Max     = 2
    };
    enum Errors_e {
        Error_SuccessMismatch = 1,
        Error_Success         = 0,
        Error_IICerror        = -1,
        Error_DataMismatch    = -2
    };
    enum Retry_e {
        Retry_0  = 0, Retry_0s = 1,
        Retry_1  = 2, Retry_1s = 3,
        Retry_2  = 4, Retry_2s = 5
    };

    // All ctor shapes accepted so any OpenFIRE instantiation compiles unchanged.
    DFRobotIRPositionEx() { init(); }
    explicit DFRobotIRPositionEx(TwoWire&) { init(); }
    DFRobotIRPositionEx(SPIClass*, int8_t = -1) { init(); }
    ~DFRobotIRPositionEx() {}

    // Starts the capture stack; clock/format are accepted for compatibility.
    bool begin(uint32_t clock = 400000,
               DataFormat_e format = DataFormat_Basic,
               Sensitivity_e sensitivity = Sensitivity_Default) {
        (void)clock;
        current_format = format;
        current_sens   = sensitivity;
#if defined(ESP_PLATFORM)
        if (ov2640_capture_start() != 0) return false;
#endif
        ov2640_bridge_frame_t f;
        last_seq = ov2640_bridge_read(&f);
        return true;   // capture may still be warming up; first frames follow
    }

    void dataFormat(DataFormat_e format)          { current_format = format; }
    void sensitivityLevel(Sensitivity_e s)        { current_sens = s; }

    // ---- the per-frame calls OpenFIRE actually makes ----
    int basicAtomic(Retry_e retry = Retry_1s)     { (void)retry; return fetch(); }
    int extendedAtomic(Retry_e retry = Retry_1s)  { (void)retry; return fetch(); }

    // request/available pairs kept for interface completeness
    void requestPositionBasic()                   { pending = true; }
    void requestPositionExtended()                { pending = true; }
    bool availableBasic()          { fetch(); return true; }
    bool availableBasicNoSeen()    { fetchNoSeen(); return true; }
    bool availableExtended()       { fetch(); return true; }
    bool availableExtendedNoSeen() { fetchNoSeen(); return true; }

    // ---- accessors (exact signatures of the original) ----
    int x(int index) const                { return positionX[index]; }
    int y(int index) const                { return positionY[index]; }
    int size(int index) const             { return unpackedSizes[index]; }
    const int* xPositions() const         { return positionX; }
    const int* yPositions() const         { return positionY; }
    unsigned int seen() const             { return seenFlags; }

private:
    int positionX[4];
    int positionY[4];
    int unpackedSizes[4];
    unsigned int seenFlags;
    DataFormat_e  current_format;
    Sensitivity_e current_sens;
    uint32_t last_seq;
    bool pending;

    // Resets the accessors to the "nothing seen" state.
    void init() {
        for (int i = 0; i < 4; ++i) {
            positionX[i] = 1023; positionY[i] = 1023;  // DFRobot "not seen" idiom
            unpackedSizes[i] = 0;
        }
        seenFlags = 0;
        current_format = DataFormat_Basic;
        current_sens   = Sensitivity_Default;
        last_seq = 0;
        pending  = false;
    }

    // Reads the bridge and returns Error_Success ONLY when a NEW frame exists.
    // Repeat calls return Error_DataMismatch, which OpenFIRE already treats as a
    // silent "nothing to do" -- that is what paces its filter to one sample per
    // camera frame. Accessors keep the last good values.
    int fetch() {
        ov2640_bridge_frame_t f;
        uint32_t seq = ov2640_bridge_read(&f);
        if (seq == 0) { seenFlags = 0; return Error_DataMismatch; }  // not warm yet
        if (seq == last_seq) return Error_DataMismatch;              // no NEW frame
        // Capture-to-consumption latency, measured where OpenFIRE takes the data.
        // Guarded: esp_timer and these counters only exist on-device.
#if defined(ESP_PLATFORM)
        {
            const uint32_t age = (uint32_t)esp_timer_get_time() - ov2640_pub_t_us;
            if (age < 1000000u) {                       // ignore boot/wrap
                ov2640_shim_lat_last_us = age;
                if (age > ov2640_shim_lat_us) ov2640_shim_lat_us = age;
            }
        }
#endif
        // Publish the quad in native camera px for the aim pipeline.
        nativeW = f.frame_w; nativeH = f.frame_h; nativeCount = f.count;
        for (int i = 0; i < 4; ++i) {
            nativeX[i] = (i < f.count) ? (float)f.x16[i] * (1.0f/16.0f) : 0.0f;
            nativeY[i] = (i < f.count) ? (float)f.y16[i] * (1.0f/16.0f) : 0.0f;
        }
        unsigned int flags = 0;
        for (int i = 0; i < 4; ++i) {
            if (i < f.count) {
                // native x16 -> output space. x16 may be negative or past the
                // frame edge (a reconstructed corner outside the sensor is real
                // geometry) and an out-of-range output is fine for OpenFIRE.
                //
                // ALL-SIGNED ARITHMETIC, deliberately: the divisor is cast to
                // int32_t too, or the expression promotes to unsigned and turns
                // a small negative into a huge positive.
                const int32_t denx = (int32_t)f.frame_w * 16 - 1;
                const int32_t deny = (int32_t)f.frame_h * 16 - 1;
                const int sx = (denx > 0)
                    ? (int)((int32_t)f.x16[i] * OV2640_IRPOS_OUT_MAX_X / denx) : 0;
#if OV2640_IRPOS_MIRROR_X
                positionX[i] = OV2640_IRPOS_OUT_MAX_X - sx;
#else
                positionX[i] = sx;
#endif
                positionY[i] = (deny > 0)
                    ? (int)((int32_t)f.y16[i] * OV2640_IRPOS_OUT_MAX_Y / deny) : 0;
                int a = f.area4[i];
                unpackedSizes[i] = a > 15 ? 15 : a;   // DFRobot size range 0..15
                flags |= (1u << i);
            } else {
                positionX[i] = 1023; positionY[i] = 1023;
                unpackedSizes[i] = 0;
            }
        }
        seenFlags = flags;
        last_seq  = seq;
        pending   = false;
        return Error_Success;
    }

    void fetchNoSeen() {   // fetch() variant that leaves seenFlags untouched
        unsigned int keep = seenFlags;
        fetch();
        seenFlags = keep;
    }
};

#endif
