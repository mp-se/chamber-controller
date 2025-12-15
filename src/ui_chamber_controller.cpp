/*
MIT License

Copyright (c) 2024-2025 Magnus

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#if defined(ENABLE_LVGL)

#include <ui_chamber_controller.hpp>
#include <log.hpp>
#include <stdio.h>
#include <math.h>
#include <string.h>

/**
 * UI state structure - matches hardware layout exactly
 */
typedef struct {
    // Left column labels (state, mode, temps)
    lv_obj_t* lbl_state;
    lv_obj_t* lbl_mode;
    lv_obj_t* lbl_beer_label;      // Static "Beer" text
    lv_obj_t* lbl_beer_value;      // Beer temperature value
    lv_obj_t* lbl_chamber_label;   // Static "Chamber" text
    lv_obj_t* lbl_chamber_value;   // Chamber temperature value
    lv_obj_t* lbl_target;          // Target temperature (centered)
    lv_obj_t* lbl_status;          // Status bar at bottom
    
    // Right side buttons (Beer, Chamber, Off)
    lv_obj_t* btn_beer;
    lv_obj_t* btn_chamber;
    lv_obj_t* btn_off;
    
    // Bottom buttons (-, +)
    lv_obj_t* btn_down;
    lv_obj_t* btn_up;
    
    // Button callbacks
    lv_event_cb_t cb_beer;
    lv_event_cb_t cb_chamber;
    lv_event_cb_t cb_off;
    lv_event_cb_t cb_up;
    lv_event_cb_t cb_down;
    
    // Data buffers
    char str_beer[32];
    char str_chamber[32];
    char str_target[32];
    char str_status[64];
    char str_mode[32];
    char str_state[32];
    
    // Button visibility state
    bool beer_visible;
    bool chamber_visible;
    
    // Theme state
    bool darkmode;
} chamber_controller_state_t;

static chamber_controller_state_t g_state = {0};
static lv_disp_t* g_disp = NULL;

/**
 * Button callbacks - forward to stored application callbacks
 */
static void btn_beer_callback(lv_event_t* e) {
    if (g_state.cb_beer) g_state.cb_beer(e);
}

static void btn_chamber_callback(lv_event_t* e) {
    if (g_state.cb_chamber) g_state.cb_chamber(e);
}

static void btn_off_callback(lv_event_t* e) {
    if (g_state.cb_off) g_state.cb_off(e);
}

static void btn_up_callback(lv_event_t* e) {
    if (g_state.cb_up) g_state.cb_up(e);
}

static void btn_down_callback(lv_event_t* e) {
    if (g_state.cb_down) g_state.cb_down(e);
}

/**
 * Initialize chamber controller UI - exact layout from display.cpp
 * Display: 320x240 (or 240x320 scaled)
 */
void chamber_controller_init(lv_disp_t* disp, bool darkmode,
                             lv_event_cb_t cb_beer,
                             lv_event_cb_t cb_chamber,
                             lv_event_cb_t cb_off,
                             lv_event_cb_t cb_up,
                             lv_event_cb_t cb_down) {
    if (!disp) {
        Log.error(F("UI: chamber_controller_init: NULL display" CR));
        return;
    }
    
    g_disp = disp;
    memset(&g_state, 0, sizeof(g_state));
    g_state.darkmode = darkmode;
    
    // Store button callbacks
    g_state.cb_beer = cb_beer;
    g_state.cb_chamber = cb_chamber;
    g_state.cb_off = cb_off;
    g_state.cb_up = cb_up;
    g_state.cb_down = cb_down;
    
    // Get active screen
    lv_obj_t* scr = lv_scr_act();
    if (!scr) {
        Log.error(F("UI: No active screen for chamber controller" CR));
        return;
    }
    
    // Get theme colors
    ui_theme_colors_t theme_colors = ui_get_theme_colors(darkmode ? UI_THEME_DARK : UI_THEME_LIGHT);
    
    // Apply background
    lv_obj_set_style_bg_color(scr, theme_colors.bg, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    
    // ===== LEFT COLUMN - Labels and Values =====
    
    // State label: (5, 10, 195, 26)
    g_state.lbl_state = ui_create_label(scr, "", 5, 10, 195, 26,
                                        LV_TEXT_ALIGN_LEFT, theme_colors.text, &lv_font_montserrat_18);
    
    // Mode label: (5, 44, 195, 26)
    g_state.lbl_mode = ui_create_label(scr, "", 5, 44, 195, 26,
                                       LV_TEXT_ALIGN_LEFT, theme_colors.text, &lv_font_montserrat_18);
    
    // Beer label (static): (5, 82)
    g_state.lbl_beer_label = ui_create_label(scr, "Beer:", 5, 82, 90, 26,
                                             LV_TEXT_ALIGN_LEFT, theme_colors.text, &lv_font_montserrat_18);
    
    // Beer temp value: (110, 82, 90, 26)
    g_state.lbl_beer_value = ui_create_label(scr, "", 110, 82, 90, 26,
                                             LV_TEXT_ALIGN_LEFT, theme_colors.text, &lv_font_montserrat_18);
    
    // Chamber label (static): (5, 119)
    g_state.lbl_chamber_label = ui_create_label(scr, "Chamber:", 5, 119, 90, 26,
                                                LV_TEXT_ALIGN_LEFT, theme_colors.text, &lv_font_montserrat_18);
    
    // Chamber temp value: (110, 119, 90, 26)
    g_state.lbl_chamber_value = ui_create_label(scr, "", 110, 119, 90, 26,
                                                LV_TEXT_ALIGN_LEFT, theme_colors.text, &lv_font_montserrat_18);
    
    // Target temp (centered): (110, 171, 90, 26)
    g_state.lbl_target = ui_create_label(scr, "", 110, 171, 90, 26,
                                         LV_TEXT_ALIGN_CENTER, theme_colors.text, &lv_font_montserrat_18);
    
    // Status bar (centered): (5, 214, 305, 16)
    g_state.lbl_status = ui_create_status_label(scr, "", 5, 214, 305, 16,
                                                LV_TEXT_ALIGN_CENTER, theme_colors.text, &lv_font_montserrat_12);
    
    // ===== RIGHT SIDE BUTTONS =====
    
    // Beer button: (205, 10, 100, 44)
    g_state.btn_beer = ui_create_button(scr, "Beer", 205, 10, 100, 44,
                                        btn_beer_callback, theme_colors.button_bg, theme_colors.button_text, &lv_font_montserrat_18);
    
    // Chamber button: (205, 60, 100, 44)
    g_state.btn_chamber = ui_create_button(scr, "Chamber", 205, 60, 100, 44,
                                           btn_chamber_callback, theme_colors.button_bg, theme_colors.button_text, &lv_font_montserrat_18);
    
    // Off button: (205, 110, 100, 44)
    g_state.btn_off = ui_create_button(scr, "Off", 205, 110, 100, 44,
                                       btn_off_callback, theme_colors.button_bg, theme_colors.button_text, &lv_font_montserrat_18);
    
    // ===== BOTTOM BUTTONS =====
    
    // Down (-) button: (30, 161, 44, 44)
    g_state.btn_down = ui_create_button(scr, "-", 30, 161, 44, 44,
                                        btn_down_callback, theme_colors.button_bg, theme_colors.button_text, &lv_font_montserrat_18);
    
    // Up (+) button: (230, 161, 44, 44)
    g_state.btn_up = ui_create_button(scr, "+", 230, 161, 44, 44,
                                      btn_up_callback, theme_colors.button_bg, theme_colors.button_text, &lv_font_montserrat_18);
}

/**
 * Update beer temperature display
 */
void chamber_controller_set_beer_temp(float temp, const char unit) {
    if (isnan(temp)) {
        strncpy(g_state.str_beer, "--", sizeof(g_state.str_beer) - 1);
    } else {
        snprintf(g_state.str_beer, sizeof(g_state.str_beer), "%.1f°%c", temp, unit);
    }
}

/**
 * Update chamber temperature display
 */
void chamber_controller_set_chamber_temp(float temp, const char unit) {
    if (isnan(temp)) {
        strncpy(g_state.str_chamber, "--", sizeof(g_state.str_chamber) - 1);
    } else {
        snprintf(g_state.str_chamber, sizeof(g_state.str_chamber), "%.1f°%c", temp, unit);
    }
}

/**
 * Update target temperature display
 */
void chamber_controller_set_target_temp(float temp, const char unit) {
    snprintf(g_state.str_target, sizeof(g_state.str_target), "%.1f°%c", temp, unit);
}

/**
 * Update status display
 */
void chamber_controller_set_status(const char* status) {
    if (!status) return;
    strncpy(g_state.str_status, status, sizeof(g_state.str_status) - 1);
}

/**
 * Set display mode (beer/chamber)
 */
void chamber_controller_set_mode(const char* mode) {
    if (!mode) return;
    strncpy(g_state.str_mode, mode, sizeof(g_state.str_mode) - 1);
}

/**
 * Set state message
 */
void chamber_controller_set_state(const char* state) {
    if (!state) return;
    strncpy(g_state.str_state, state, sizeof(g_state.str_state) - 1);
}

/**
 * Show/hide beer button
 */
void chamber_controller_set_beer_button_visible(bool visible) {
    g_state.beer_visible = visible;
}

/**
 * Show/hide chamber button
 */
void chamber_controller_set_chamber_button_visible(bool visible) {
    g_state.chamber_visible = visible;
}

/**
 * Set theme (alternate name)
 */
void chamber_controller_set_theme(bool darkmode) {
    g_state.darkmode = darkmode;
}

/**
 * Chamber controller UI loop handler
 * Call from LVGL background thread to apply UI state
 * Thread-safe: main thread updates state via setters, this applies to LVGL objects
 */
void chamber_controller_loop(void) {
#if defined(ENABLE_LVGL)
    // Always apply current state to LVGL objects
    
    if (g_state.lbl_beer_value) {
        lv_label_set_text(g_state.lbl_beer_value, g_state.str_beer);
    }
    
    if (g_state.lbl_chamber_value) {
        lv_label_set_text(g_state.lbl_chamber_value, g_state.str_chamber);
    }
    
    if (g_state.lbl_target) {
        lv_label_set_text(g_state.lbl_target, g_state.str_target);
    }
    
    if (g_state.lbl_status) {
        lv_label_set_text(g_state.lbl_status, g_state.str_status);
    }
    
    if (g_state.lbl_mode) {
        lv_label_set_text(g_state.lbl_mode, g_state.str_mode);
    }
    
    if (g_state.lbl_state) {
        lv_label_set_text(g_state.lbl_state, g_state.str_state);
    }
    
    if (g_state.btn_beer) {
        if (g_state.beer_visible) {
            lv_obj_clear_flag(g_state.btn_beer, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(g_state.btn_beer, LV_OBJ_FLAG_HIDDEN);
        }
    }
    
    if (g_state.btn_chamber) {
        if (g_state.chamber_visible) {
            lv_obj_clear_flag(g_state.btn_chamber, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(g_state.btn_chamber, LV_OBJ_FLAG_HIDDEN);
        }
    }
    
    lv_obj_t* scr = lv_scr_act();
    if (scr) {
        ui_theme_colors_t theme_colors = ui_get_theme_colors(g_state.darkmode ? UI_THEME_DARK : UI_THEME_LIGHT);
        ui_apply_theme_to_screen(scr, &theme_colors);
    }
#endif
}

/**
 * Cleanup UI
 */
void chamber_controller_cleanup(void) {
    // LVGL handles cleanup automatically
    memset(&g_state, 0, sizeof(g_state));
    g_disp = NULL;
}

#endif  // ENABLE_LVGL

// EOF
