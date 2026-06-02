/*
 * Gesture input processor for ZMK.
 *
 * While the gesture key is held, intercepts REL_X/Y events and detects two
 * consecutive directional strokes (4 dirs × 3 remaining = 12 gestures).
 * Fires the configured ZMK behavior binding on completion.
 */

#define DT_DRV_COMPAT zmk_input_processor_gesture

#include <drivers/input_processor.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#include <zmk/behavior.h>
#include <zmk/pointing/gesture_processor.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

typedef enum {
    GESTURE_STATE_IDLE,
    GESTURE_STATE_ARMED,
    GESTURE_STATE_DIR2_WAITING,
} gesture_state_t;

struct gesture_proc_config {
    uint32_t threshold;
    uint32_t timeout_ms;
};

struct gesture_proc_data {
    gesture_state_t state;
    int dir1;
    int32_t accum_x;
    int32_t accum_y;
    struct k_work_delayable timeout_work;
    struct k_work fire_work;
    int pending_dir1;
    int pending_dir2;
};

static struct linea40_gesture_binding g_bindings[GESTURE_BINDINGS];
static bool g_bindings_loaded;

static int determine_dir(int32_t x, int32_t y)
{
    if (abs(x) >= abs(y)) {
        return x > 0 ? GESTURE_DIR_RIGHT : GESTURE_DIR_LEFT;
    }
    return y > 0 ? GESTURE_DIR_DOWN : GESTURE_DIR_UP;
}

/* ── NVS persistence ─────────────────────────────────────────────────────── */

#if IS_ENABLED(CONFIG_SETTINGS)
static K_WORK_DELAYABLE_DEFINE(save_work, NULL);

static void save_work_handler(struct k_work *work)
{
    int ret = settings_save_one("linea40/gesture/bindings", g_bindings, sizeof(g_bindings));
    if (ret < 0) {
        LOG_ERR("Failed to save gesture bindings: %d", ret);
    } else {
        LOG_DBG("Gesture bindings saved");
    }
}

/* Re-define the work item with handler (K_WORK_DELAYABLE_DEFINE requires handler at definition) */
static struct k_work_delayable gesture_save_work;

static void schedule_save(void)
{
    k_work_reschedule(&gesture_save_work, K_MSEC(500));
}

static int gesture_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                void *cb_arg)
{
    const char *next;
    if (!settings_name_steq(name, "bindings", &next) || next) {
        return -ENOENT;
    }
    if (len != sizeof(g_bindings)) {
        return -EINVAL;
    }
    int ret = read_cb(cb_arg, g_bindings, sizeof(g_bindings));
    if (ret >= 0) {
        LOG_DBG("Gesture bindings loaded");
    }
    return ret;
}

SETTINGS_STATIC_HANDLER_DEFINE(linea40_gesture, "linea40/gesture", NULL, gesture_settings_set,
                                NULL, NULL);
#else
static void schedule_save(void) {}
#endif /* CONFIG_SETTINGS */

/* ── Behavior firing ─────────────────────────────────────────────────────── */

static void fire_gesture_work_handler(struct k_work *work)
{
    struct gesture_proc_data *data = CONTAINER_OF(work, struct gesture_proc_data, fire_work);
    int idx = gesture_binding_idx(data->pending_dir1, data->pending_dir2);
    if (idx < 0) {
        return;
    }

    const struct linea40_gesture_binding *entry = &g_bindings[idx];
    if (entry->behavior_id < 0) {
        LOG_DBG("Gesture %d-%d: no binding assigned", data->pending_dir1, data->pending_dir2);
        return;
    }

    const char *name = zmk_behavior_find_behavior_name_from_local_id(entry->behavior_id);
    if (!name) {
        LOG_WRN("Gesture: unknown behavior_id=%d", entry->behavior_id);
        return;
    }

    struct zmk_behavior_binding binding = {
        .behavior_dev = name,
        .param1 = entry->param1,
        .param2 = entry->param2,
    };
    struct zmk_behavior_binding_event event = {
        .position = INT32_MAX,
        .timestamp = k_uptime_get(),
    };

    zmk_behavior_invoke_binding(&binding, event, true);
    zmk_behavior_invoke_binding(&binding, event, false);
    LOG_DBG("Gesture fired: dir1=%d dir2=%d idx=%d behavior_id=%d",
            data->pending_dir1, data->pending_dir2, idx, entry->behavior_id);
}

/* ── Timeout: reset dir2 accumulator ────────────────────────────────────── */

static void timeout_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct gesture_proc_data *data =
        CONTAINER_OF(dwork, struct gesture_proc_data, timeout_work);

    if (data->state == GESTURE_STATE_DIR2_WAITING) {
        data->accum_x = 0;
        data->accum_y = 0;
        LOG_DBG("Gesture timeout: dir2 accumulator reset");
    }
}

/* ── Input processor handle_event ───────────────────────────────────────── */

static int gesture_handle_event(const struct device *dev, struct input_event *event,
                                uint32_t param1, uint32_t param2,
                                struct zmk_input_processor_state *state)
{
    struct gesture_proc_data *data = dev->data;
    const struct gesture_proc_config *cfg = dev->config;

    if (data->state == GESTURE_STATE_IDLE) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_X) {
        data->accum_x += event->value;
    } else {
        data->accum_y += event->value;
    }

    /* Only evaluate direction on sync (final event in batch) */
    if (!event->sync) {
        return 1; /* suppress, accumulate more */
    }

    int32_t mag_x = abs(data->accum_x);
    int32_t mag_y = abs(data->accum_y);
    int32_t mag = mag_x > mag_y ? mag_x : mag_y;

    if (mag < (int32_t)cfg->threshold) {
        return 1; /* suppress, not enough movement yet */
    }

    int dir = determine_dir(data->accum_x, data->accum_y);
    data->accum_x = 0;
    data->accum_y = 0;

    if (data->state == GESTURE_STATE_ARMED) {
        data->dir1 = dir;
        data->state = GESTURE_STATE_DIR2_WAITING;
        k_work_reschedule(&data->timeout_work, K_MSEC(cfg->timeout_ms));
        LOG_DBG("Gesture dir1=%d captured", dir);
    } else {
        /* DIR2_WAITING */
        k_work_cancel_delayable(&data->timeout_work);
        if (dir != data->dir1) {
            data->pending_dir1 = data->dir1;
            data->pending_dir2 = dir;
            k_work_submit(&data->fire_work);
            /* Stay ARMED for potential chaining */
            data->state = GESTURE_STATE_ARMED;
            LOG_DBG("Gesture complete: %d → %d", data->dir1, dir);
        } else {
            /* Same direction: ignore, restart timeout */
            k_work_reschedule(&data->timeout_work, K_MSEC(cfg->timeout_ms));
            LOG_DBG("Gesture: same dir=%d, waiting for different dir2", dir);
        }
    }

    return 1; /* always suppress while armed */
}

static const struct zmk_input_processor_driver_api gesture_driver_api = {
    .handle_event = gesture_handle_event,
};

/* ── Public API (called by behavior_gesture_mod) ────────────────────────── */

void linea40_gesture_arm(void)
{
    const struct device *dev = DEVICE_DT_INST_GET(0);
    if (!device_is_ready(dev)) {
        return;
    }
    struct gesture_proc_data *data = dev->data;
    data->state = GESTURE_STATE_ARMED;
    data->accum_x = 0;
    data->accum_y = 0;
    LOG_DBG("Gesture armed");
}

void linea40_gesture_disarm(void)
{
    const struct device *dev = DEVICE_DT_INST_GET(0);
    if (!device_is_ready(dev)) {
        return;
    }
    struct gesture_proc_data *data = dev->data;
    k_work_cancel_delayable(&data->timeout_work);
    data->state = GESTURE_STATE_IDLE;
    data->accum_x = 0;
    data->accum_y = 0;
    LOG_DBG("Gesture disarmed");
}

int linea40_gesture_get_all_bindings(struct linea40_gesture_binding *out, size_t count)
{
    if (count < GESTURE_BINDINGS) {
        return -EINVAL;
    }
    memcpy(out, g_bindings, sizeof(g_bindings));
    return 0;
}

int linea40_gesture_set_binding(uint8_t dir1, uint8_t dir2,
                                const struct linea40_gesture_binding *binding)
{
    int idx = gesture_binding_idx(dir1, dir2);
    if (idx < 0) {
        return -EINVAL;
    }
    g_bindings[idx] = *binding;
    schedule_save();
    return 0;
}

/* ── Device init ─────────────────────────────────────────────────────────── */

static int gesture_proc_init(const struct device *dev)
{
    struct gesture_proc_data *data = dev->data;

    /* Default: all bindings unset */
    for (int i = 0; i < GESTURE_BINDINGS; i++) {
        g_bindings[i].behavior_id = -1;
        g_bindings[i].param1 = 0;
        g_bindings[i].param2 = 0;
    }

    data->state = GESTURE_STATE_IDLE;
    data->accum_x = 0;
    data->accum_y = 0;
    k_work_init_delayable(&data->timeout_work, timeout_work_handler);
    k_work_init(&data->fire_work, fire_gesture_work_handler);

#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_init_delayable(&gesture_save_work, save_work_handler);
#endif

    return 0;
}

#define GESTURE_PROC_INST(n)                                                                       \
    static struct gesture_proc_data gesture_proc_data_##n;                                         \
    static const struct gesture_proc_config gesture_proc_config_##n = {                            \
        .threshold = DT_INST_PROP(n, threshold),                                                   \
        .timeout_ms = DT_INST_PROP(n, timeout_ms),                                                 \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, gesture_proc_init, NULL, &gesture_proc_data_##n,                      \
                          &gesture_proc_config_##n, POST_KERNEL,                                   \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &gesture_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GESTURE_PROC_INST)
