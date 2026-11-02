#include "managers/views/jammer_view.h"
#include "sdkconfig.h"

#if CONFIG_JAMMER_ENABLE

#include "lvgl.h"
#include "jammer.h"
#include "esp_log.h"

static const char *TAG_VIEW = "JammerView";

static lv_obj_t *s_btn = NULL;
static lv_obj_t *s_label = NULL;

static void update_label(void)
{
    if (!s_label) return;
    lv_label_set_text(s_label, jammer_is_running() ? "STOP" : "START");
}

static void btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED) return;

    if (jammer_is_running()) {
        jammer_stop();
        ESP_LOGI(TAG_VIEW, "stopped");
    } else {
        esp_err_t err = jammer_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG_VIEW, "start failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG_VIEW, "started");
        }
    }
    update_label();
}

static void jammer_view_create(lv_obj_t *parent)
{
    lv_obj_t *scr = parent ? parent : lv_scr_act();

    s_btn = lv_btn_create(scr);
    lv_obj_set_size(s_btn, 160, 80);
    lv_obj_align(s_btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(s_btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    s_label = lv_label_create(s_btn);
    lv_obj_center(s_label);

    update_label();
}

static void jammer_view_destroy(void)
{
    s_btn = NULL;
    s_label = NULL;
}

const View jammer_view = {
    .name = "jammer",
    .create = jammer_view_create,
    .destroy = jammer_view_destroy,
};

#endif /* CONFIG_JAMMER_ENABLE */
