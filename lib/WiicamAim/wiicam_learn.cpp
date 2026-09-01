// wiicam_learn.cpp -- histograms of what a confirmed LED looks like, and of
// what the gate threw away. See the header for why the label comes from the
// quad resolver and not from the gate's own decisions.
//
// Histograms rather than a running median and MAD, for two reasons. The first
// is that they are exact and cost nothing: the features are small integers, so
// 2 classes x 6 features x 32 bins of uint16 is 768 bytes and one increment
// per blob, with no floating point and no ordering. The second matters more --
// the question this module exists to answer is not "what is the median size"
// but "do these two distributions overlap", and only the shape answers that. A
// median would say an LED is 1 and a window is 4 and hide that a fifth of the
// windows are also 1.
#include "wiicam_learn.h"
#include <string.h>

static uint16_t s_hist[WL_CLASSES][WL_NFEAT][WL_BINS];
static uint32_t s_blobs[WL_CLASSES];
static uint32_t s_frames;
static uint8_t  s_on;

static const char* const s_names[WL_NFEAT] = {
    "sz", "bw", "bh", "aspect", "area", "irel"
};

const char* wl_feat_name(int feat)
{
    return (feat >= 0 && feat < WL_NFEAT) ? s_names[feat] : "?";
}

void wl_reset(void)
{
    memset(s_hist, 0, sizeof(s_hist));
    memset(s_blobs, 0, sizeof(s_blobs));
    s_frames = 0;
}

void wl_enable(int on)
{
    // The off->on EDGE clears. A distribution accumulated across a change of
    // sensitivity, of lens, or of the LED bar itself is two rigs averaged
    // together, which is worse than no measurement: it looks like one wide
    // spread rather than two narrow ones, and the conclusion drawn from it
    // would be that the rig cannot be separated when neither was measured.
    //
    // The edge, and not every call, because the tools re-send a value on every
    // keypress and a repeated on:1 that wiped the capture would make a held
    // button silently destroy a long measurement. Changing sensitivity
    // mid-capture and re-arming therefore does NOT clear -- stop it first, or
    // use camlearn=reset, which is there for exactly that.
    if (on && !s_on) wl_reset();
    s_on = on ? 1 : 0;
}

int wl_enabled(void) { return s_on; }

uint32_t wl_blobs(int cls)
{
    return (cls >= 0 && cls < WL_CLASSES) ? s_blobs[cls] : 0;
}

uint32_t wl_frames(void) { return s_frames; }

void wl_note_frame(void) { if (s_on) ++s_frames; }

const uint16_t* wl_hist(int cls, int feat)
{
    if (cls < 0 || cls >= WL_CLASSES || feat < 0 || feat >= WL_NFEAT) return 0;
    return s_hist[cls][feat];
}

// Saturating, because a bin that wraps to zero after 65535 samples turns the
// tallest peak in the distribution into the shortest and inverts the very
// conclusion the histogram exists to support. A capture long enough to
// saturate has already made its point.
static void bump(int cls, int feat, int bin)
{
    if (bin < 0) bin = 0;
    if (bin >= WL_BINS) bin = WL_BINS - 1;
    uint16_t* h = &s_hist[cls][feat][bin];
    if (*h != 0xFFFFu) ++*h;
}

void wl_note(int cls, int sz, int bw, int bh, int intens, int imed, int flags)
{
    if (!s_on || cls < 0 || cls >= WL_CLASSES) return;
    ++s_blobs[cls];

    // Size is the only feature the extended format carries. A negative size
    // means the sensor did not report one at all (basic format), and a
    // histogram that counted those as zero would show a large, sharp peak at
    // the smallest possible blob that no LED ever produced.
    if (sz >= 0) bump(cls, WL_SZ, sz);

    if (!(flags & WL_HAS_BOX)) return;

    bump(cls, WL_BW, bw);
    bump(cls, WL_BH, bh);
    bump(cls, WL_AREA, bw * bh);

    // Aspect needs both sides. A box with a zero side is not a flat blob, it
    // is a blob one pixel across in that axis after the sensor's 7-bit corners
    // were subtracted -- there is no ratio to take, and forcing one would put
    // every small round LED in the "extremely elongated" bin, which is exactly
    // the wrong end.
    if (bw > 0 && bh > 0) {
        const int lo = (bw < bh) ? bw : bh;
        const int hi = (bw < bh) ? bh : bw;
        bump(cls, WL_ASPECT, (8 * hi) / lo - 8);
    }

    // Relative to the frame's own median, never absolute: distance changes
    // intensity by about 1/r^2, so an absolute threshold learned at one
    // standing position is wrong at every other one. This is the same reason
    // the odd-one-out gate compares a blob against its neighbours instead of
    // against a number.
    if (imed > 0) bump(cls, WL_IREL, (16 * intens) / imed);
}
