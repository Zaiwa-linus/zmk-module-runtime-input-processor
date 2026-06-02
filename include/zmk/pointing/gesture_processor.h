#pragma once

#include <stdint.h>

#define GESTURE_DIR_UP    0
#define GESTURE_DIR_DOWN  1
#define GESTURE_DIR_LEFT  2
#define GESTURE_DIR_RIGHT 3
#define GESTURE_DIRS      4
#define GESTURE_BINDINGS  12 /* 4×3: same-direction pairs excluded */

/* Map (dir1, dir2) where dir1 ≠ dir2 → index 0..11
 * U-D=0 U-L=1 U-R=2 / D-U=3 D-L=4 D-R=5 / L-U=6 L-D=7 L-R=8 / R-U=9 R-D=10 R-L=11 */
static inline int gesture_binding_idx(int d1, int d2)
{
    if (d1 == d2 || d1 < 0 || d1 >= GESTURE_DIRS || d2 < 0 || d2 >= GESTURE_DIRS) {
        return -1;
    }
    return d1 * 3 + (d2 > d1 ? d2 - 1 : d2);
}

struct linea40_gesture_binding {
    int32_t behavior_id; /* -1 = no binding */
    uint32_t param1;
    uint32_t param2;
};

/* Called by behavior_gesture_mod when the gesture key is pressed/released */
void linea40_gesture_arm(void);
void linea40_gesture_disarm(void);

/* RPC interface */
int linea40_gesture_get_all_bindings(struct linea40_gesture_binding *out, size_t count);
int linea40_gesture_set_binding(uint8_t dir1, uint8_t dir2,
                                const struct linea40_gesture_binding *binding);
