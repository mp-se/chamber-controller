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
#ifndef SRC_UI_CHAMBER_CONTROLLER_HPP_
#define SRC_UI_CHAMBER_CONTROLLER_HPP_

#if defined(ENABLE_LVGL)

#include <ui_helpers.hpp>

/**
 * Initialize chamber controller UI
 * Must be called after hw_display_init()
 * 
 * @param disp LVGL display object
 * @param darkmode Initial theme (true=dark, false=light)
 */
void chamber_controller_init(lv_disp_t* disp, bool darkmode);

/**
 * Update beer temperature display
 * Handles NaN values appropriately
 * 
 * @param temp Temperature in Celsius (-99.9 to 99.9), or NaN for unavailable
 */
void chamber_controller_set_beer_temp(float temp);

/**
 * Update chamber temperature display
 * Handles NaN values appropriately
 * 
 * @param temp Temperature in Celsius, or NaN for unavailable
 */
void chamber_controller_set_chamber_temp(float temp);

/**
 * Update target temperature display
 * 
 * @param temp Target temperature in Celsius
 */
void chamber_controller_set_target_temp(float temp);

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
 * Cleanup and destroy UI
 * Call when shutting down
 */
void chamber_controller_cleanup(void);

#endif  // ENABLE_LVGL

#endif  // SRC_UI_CHAMBER_CONTROLLER_HPP_

// EOF
