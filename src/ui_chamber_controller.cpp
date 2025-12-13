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
#include "ui_chamber_controller.hpp"
#include "ui_helpers.hpp"
#include "ui_helpers.hpp"
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
    
    // Data buffers
    char str_beer[32];
    char str_chamber[32];
    char str_target[32];
    char str_status[64];
} chamber_controller_state_t;

static chamber_controller_state_t g_state = {0};
static lv_disp_t* g_disp = NULL;
static bool g_darkmode = false;

/**
 * Button callbacks
 */
static void btn_beer_callback(lv_event_t* e) {
    (void)e;
    printf("UI: Beer button pressed\n");
}

static void btn_chamber_callback(lv_event_t* e) {
    (void)e;
    printf("UI: Chamber button pressed\n");
}

static void btn_off_callback(lv_event_t* e) {
    (void)e;
    printf("UI: Off button pressed\n");
}

static void btn_up_callback(lv_event_t* e) {
    (void)e;
    printf("UI: Up (+) button pressed\n");
}

static void btn_down_callback(lv_event_t* e) {
    (void)e;
    printf("UI: Down (-) button pressed\n");
}

/**
 * Initialize chamber controller UI - exact layout from display.cpp
 * Display: 320x240 (or 240x320 scaled)
 */
void chamber_controller_init(lv_disp_t* disp, bool darkmode) {
    if (!disp) {
        fprintf(stderr, "chamber_controller_init: NULL display\n");
        return;
    }
    
    g_disp = disp;
    g_darkmode = darkmode;
    memset(&g_state, 0, sizeof(g_state));
    
    // Get active screen
    lv_obj_t* scr = lv_scr_act();
    if (!scr) {
        fprintf(stderr, "ERROR: No active screen\n");
        return;
    }
    
    // Get theme colors
    ui_theme_colors_t theme_colors = ui_get_theme_colors(darkmode ? UI_THEME_DARK : UI_THEME_LIGHT);
    
    // Apply background
    lv_obj_set_style_bg_color(scr, theme_colors.bg, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    
    // ===== LEFT COLUMN - Labels and Values =====
    
    // State label: (5, 10, 195, 26)
    g_state.lbl_state = ui_create_label(scr, "Idle", 5, 10, 195, 26,
                                        LV_TEXT_ALIGN_LEFT, theme_colors.text);
    
    // Mode label: (5, 44, 195, 26)
    g_state.lbl_mode = ui_create_label(scr, "Mode: Beer", 5, 44, 195, 26,
                                       LV_TEXT_ALIGN_LEFT, theme_colors.text);
    
    // Beer label (static): (5, 82)
    g_state.lbl_beer_label = ui_create_label(scr, "Beer", 5, 82, 90, 26,
                                             LV_TEXT_ALIGN_LEFT, theme_colors.text);
    
    // Beer temp value: (110, 82, 90, 26)
    g_state.lbl_beer_value = ui_create_label(scr, "20.5", 110, 82, 90, 26,
                                             LV_TEXT_ALIGN_LEFT, theme_colors.text);
    
    // Chamber label (static): (5, 119)
    g_state.lbl_chamber_label = ui_create_label(scr, "Chamber", 5, 119, 90, 26,
                                                LV_TEXT_ALIGN_LEFT, theme_colors.text);
    
    // Chamber temp value: (110, 119, 90, 26)
    g_state.lbl_chamber_value = ui_create_label(scr, "20.5", 110, 119, 90, 26,
                                                LV_TEXT_ALIGN_LEFT, theme_colors.text);
    
    // Target temp (centered): (110, 171, 90, 26)
    g_state.lbl_target = ui_create_label(scr, "20.0", 110, 171, 90, 26,
                                         LV_TEXT_ALIGN_CENTER, theme_colors.text);
    
    // Status bar (centered): (5, 214, 305, 16)
    g_state.lbl_status = ui_create_status_label(scr, "Ready", 5, 214, 305, 16,
                                                LV_TEXT_ALIGN_CENTER, theme_colors.text);
    
    // ===== RIGHT SIDE BUTTONS =====
    
    // Beer button: (205, 10, 100, 44)
    g_state.btn_beer = ui_create_button(scr, "Beer", 205, 10, 100, 44,
                                        btn_beer_callback, theme_colors.button_bg, theme_colors.text);
    
    // Chamber button: (205, 60, 100, 44)
    g_state.btn_chamber = ui_create_button(scr, "Chamber", 205, 60, 100, 44,
                                           btn_chamber_callback, theme_colors.button_bg, theme_colors.text);
    
    // Off button: (205, 110, 100, 44)
    g_state.btn_off = ui_create_button(scr, "Off", 205, 110, 100, 44,
                                       btn_off_callback, theme_colors.button_bg, theme_colors.text);
    
    // ===== BOTTOM BUTTONS =====
    
    // Down (-) button: (30, 161, 44, 44)
    g_state.btn_down = ui_create_button(scr, "-", 30, 161, 44, 44,
                                        btn_down_callback, theme_colors.button_bg, theme_colors.text);
    
    // Up (+) button: (230, 161, 44, 44)
    g_state.btn_up = ui_create_button(scr, "+", 230, 161, 44, 44,
                                      btn_up_callback, theme_colors.button_bg, theme_colors.text);
    
    printf("Chamber Controller UI initialized (320x240, %s mode) - layout matches display.cpp\n", 
           darkmode ? "dark" : "light");
}

/**
 * Update beer temperature display
 */
void chamber_controller_set_beer_temp(float temp) {
    if (!g_state.lbl_beer_value) return;
    
    if (isnan(temp)) {
        lv_label_set_text(g_state.lbl_beer_value, "--");
    } else {
        snprintf(g_state.str_beer, sizeof(g_state.str_beer), "%.1f", temp);
        lv_label_set_text(g_state.lbl_beer_value, g_state.str_beer);
    }
}

/**
 * Update chamber temperature display
 */
void chamber_controller_set_chamber_temp(float temp) {
    if (!g_state.lbl_chamber_value) return;
    
    if (isnan(temp)) {
        lv_label_set_text(g_state.lbl_chamber_value, "--");
    } else {
        snprintf(g_state.str_chamber, sizeof(g_state.str_chamber), "%.1f", temp);
        lv_label_set_text(g_state.lbl_chamber_value, g_state.str_chamber);
    }
}

/**
 * Update target temperature display
 */
void chamber_controller_set_target_temp(float temp) {
    if (!g_state.lbl_target) return;
    
    snprintf(g_state.str_target, sizeof(g_state.str_target), "%.1f", temp);
    lv_label_set_text(g_state.lbl_target, g_state.str_target);
}

/**
 * Update status display
 */
void chamber_controller_set_status(const char* status) {
    if (!g_state.lbl_status || !status) return;
    lv_label_set_text(g_state.lbl_status, status);
}

/**
 * Set display mode (beer/chamber)
 */
void chamber_controller_set_mode(const char* mode) {
    if (!g_state.lbl_mode || !mode) return;
    
    snprintf(g_state.str_status, sizeof(g_state.str_status), "Mode: %s", mode);
    lv_label_set_text(g_state.lbl_mode, g_state.str_status);
}

/**
 * Set state message
 */
void chamber_controller_set_state(const char* state) {
    if (!g_state.lbl_state || !state) return;
    lv_label_set_text(g_state.lbl_state, state);
}

/**
 * Show/hide beer button
 */
void chamber_controller_set_beer_button_visible(bool visible) {
    if (!g_state.btn_beer) return;
    
    if (visible) {
        lv_obj_clear_flag(g_state.btn_beer, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(g_state.btn_beer, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * Show/hide chamber button
 */
void chamber_controller_set_chamber_button_visible(bool visible) {
    if (!g_state.btn_chamber) return;
    
    if (visible) {
        lv_obj_clear_flag(g_state.btn_chamber, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(g_state.btn_chamber, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * Toggle display theme
 */
void chamber_controller_toggle_theme(bool darkmode) {
    g_darkmode = darkmode;
    lv_obj_t* scr = lv_scr_act();
    if (!scr) return;
    
    ui_theme_colors_t theme_colors = ui_get_theme_colors(darkmode ? UI_THEME_DARK : UI_THEME_LIGHT);
    ui_apply_theme_to_screen(scr, &theme_colors);
}

/**
 * Set theme (alternate name)
 */
void chamber_controller_set_theme(bool darkmode) {
    chamber_controller_toggle_theme(darkmode);
}

/**
 * Cleanup UI
 */
void chamber_controller_cleanup(void) {
    // LVGL handles cleanup automatically
    memset(&g_state, 0, sizeof(g_state));
    g_disp = NULL;
}
