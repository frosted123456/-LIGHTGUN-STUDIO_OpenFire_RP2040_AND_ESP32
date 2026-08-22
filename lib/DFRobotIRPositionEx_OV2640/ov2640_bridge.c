// ov2640_bridge.c — 4-slot ring publish/read. See header for the contract.
//
// Safety argument: the writer never writes the slot a reader could be copying
// unless it has lapped the reader by 3 whole publishes, which the reader
// detects and retries. Slot k&3 is safe while (writer_count - k) < 3.
//
// Uses GCC __atomic builtins so the same file compiles as C (xtensa firmware)
// and C++ (host tests).
#include "ov2640_bridge.h"
#include <string.h>

static ov2640_bridge_frame_t s_slot[4];
static uint32_t s_count;      // completed publishes; slot of publish n = n&3

// Publishes one frame into the next ring slot.
void ov2640_bridge_publish(const ov2640_bridge_frame_t* f)
{
    uint32_t n = __atomic_load_n(&s_count, __ATOMIC_RELAXED);
    memcpy(&s_slot[(n + 1) & 3u], f, sizeof(*f));      /* next slot, private */
    __atomic_store_n(&s_count, n + 1, __ATOMIC_RELEASE); /* data before count */
}

// Copies the latest published frame; returns its frame_seq, or 0 if none yet.
uint32_t ov2640_bridge_read(ov2640_bridge_frame_t* out)
{
    for (;;) {
        uint32_t n = __atomic_load_n(&s_count, __ATOMIC_ACQUIRE);
        if (n == 0) return 0;                     /* nothing ever published */
        memcpy(out, &s_slot[n & 3u], sizeof(*out));
        __atomic_thread_fence(__ATOMIC_ACQUIRE);  /* copy before the recheck */
        uint32_t m = __atomic_load_n(&s_count, __ATOMIC_RELAXED);
        if (m - n < 3u) return out->frame_seq;    /* writer never reached our
                                                     slot: copy is intact */
    }
}
