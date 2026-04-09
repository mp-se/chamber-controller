/*
 * Chamber Controller
 * Copyright (c) 2024-2026 Magnus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifndef SRC_UI_CHAMBER_CONTROLLER_HPP_
#define SRC_UI_CHAMBER_CONTROLLER_HPP_

#if defined(ENABLE_LVGL)

#include <lvgl.h>

#include <ui_helpers.hpp>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize chamber controller UI
 * Must be called after hw_display_init()
 *
 * @param disp LVGL display object
 * @param darkmode Initial theme (true=dark, false=light)
 * @param cb_beer Beer button event callback
 * @param cb_chamber Chamber button event callback
 * @param cb_off Off button event callback
 * @param cb_up Up button event callback
 * @param cb_down Down button event callback
 */
void chamber_controller_init(lv_disp_t* disp, bool darkmode,
                             lv_event_cb_t cb_beer, lv_event_cb_t cb_chamber,
                             lv_event_cb_t cb_off, lv_event_cb_t cb_up,
                             lv_event_cb_t cb_down);

/**
 * Update beer temperature display
 * Handles NaN values appropriately
 *
 * @param temp Temperature in Celsius (-99.9 to 99.9), or NaN for unavailable
 */
void chamber_controller_set_beer_temp(float temp, const char unit);

/**
 * Update chamber temperature display
 * Handles NaN values appropriately
 *
 * @param temp Temperature in Celsius, or NaN for unavailable
 */
void chamber_controller_set_chamber_temp(float temp, const char unit);

/**
 * Update target temperature display
 *
 * @param temp Target temperature in Celsius
 */
void chamber_controller_set_target_temp(float temp, const char unit);

/**
 * Update display mode (beer/chamber/off)
 *
 * @param mode Mode string (e.g., "Beer", "Chamber", "Off")
 */
void chamber_controller_set_mode(const char* mode);

/**
 * Update state message
 *
 * @param state State string (e.g., "Heating", "Cooling", "Idle")
 */
void chamber_controller_set_state(const char* state);

/**
 * Update status bar message
 *
 * @param status Status string
 */
void chamber_controller_set_status(const char* status);

/**
 * Show/hide beer button
 *
 * @param visible true to show, false to hide
 */
void chamber_controller_set_beer_button_visible(bool visible);

/**
 * Show/hide chamber button
 *
 * @param visible true to show, false to hide
 */
void chamber_controller_set_chamber_button_visible(bool visible);

/**
 * Set UI theme (light or dark mode)
 *
 * @param darkmode true for dark theme, false for light theme
 */
void chamber_controller_set_theme(bool darkmode);

/**
 * Chamber controller UI loop handler
 * Call from LVGL background thread (lvgl_loop_handler) to apply pending UI
 * updates This is thread-safe: main thread queues updates via setter functions,
 * this function applies them from the LVGL thread
 */
void chamber_controller_loop(void);

/**
 * Cleanup and destroy UI
 * Call when shutting down
 */
void chamber_controller_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif  // ENABLE_LVGL

#endif  // SRC_UI_CHAMBER_CONTROLLER_HPP_

// EOF
