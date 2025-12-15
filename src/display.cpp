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
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <Wire.h>

#include <cstdio>
#include <display.hpp>
#include <fonts.hpp>
#include <functional>
#include <log.hpp>
#include <looptimer.hpp>
#include <main.hpp>
#include <pidconfig.hpp>

constexpr auto TTF_CALIBRATION_FILENAME = "/tft.dat";

#if defined(ENABLE_LVGL)
TaskHandle_t lvglTaskHandler;
#endif

void Display::setup() {
#if defined(ENABLE_TFT)
  Log.notice(
      F("DISP: TFT Config: MISO=%d, MOSI=%d, SCLK=%d, CS=%d, DC=%d, RST=%d, "
        "TOUCH_CS=%d" CR),
      TFT_MISO, TFT_MOSI, TFT_SCLK, TFT_CS, TFT_DC, TFT_RST, TOUCH_CS);

  _tft = new TFT_eSPI();

  if (!_tft) {
    Log.warning(F("DISP: No TFT_eSPI driver is created!" CR));
    return;
  }

  _tft->init();
  _tft->setRotation(_rotation);
  clear();
  setFont(FontSize::FONT_9);
#else
  Log.warning(F("DISP: TFT driver support is not included in this build!" CR));
#endif
}

void Display::setRotation(Rotation r) {
#if defined(ENABLE_TFT)
  if (!_tft) {
    return;
  }

  _rotation = r;
  _tft->setRotation(_rotation);
#endif
}

void Display::setFont(FontSize f) {
#if defined(ENABLE_TFT)
  if (!_tft) return;

  switch (f) {
    default:
    case FontSize::FONT_9:
      _tft->setFreeFont(FF17);
      break;
    case FontSize::FONT_12:
      _tft->setFreeFont(FF18);
      break;
    case FontSize::FONT_18:
      _tft->setFreeFont(FF19);
      break;
    case FontSize::FONT_24:
      _tft->setFreeFont(FF20);
      break;
  }
#endif
}

void Display::printLine(int l, const String &text) {
#if defined(ENABLE_TFT)
  if (!_tft) return;

  uint16_t h = _tft->fontHeight();
  _tft->fillRect(0, l * h, _tft->width(), h, _backgroundColor);
  _tft->drawString(text.c_str(), 0, l * h, GFXFF);
#endif
}

void Display::printLineCentered(int l, const String &text) {
#if defined(ENABLE_TFT)
  if (!_tft) return;

  uint16_t h = _tft->fontHeight();
  uint16_t w = _tft->textWidth(text);
  _tft->fillRect(0, l * h, _tft->width(), h, _backgroundColor);
  _tft->drawString(text.c_str(), (_tft->width() - w) / 2, l * h, GFXFF);
#endif
}

void Display::clear(uint32_t color) {
#if defined(ENABLE_TFT)
  if (!_tft) return;

  _backgroundColor = color;
  _tft->fillScreen(_backgroundColor);
  delay(1);
#endif
}

void Display::updateButtons(bool beerEnabled, bool chamberEnabled) {
#if defined(ENABLE_LVGL)
  // Update button visibility using new API
  Log.notice(F("DISP: Updating button visibility: Beer=%d, Chamber=%d" CR), beerEnabled, chamberEnabled);
  chamber_controller_set_beer_button_visible(beerEnabled);
  chamber_controller_set_chamber_button_visible(chamberEnabled);
#endif
}

void Display::updateTemperatures(const char *mode, const char *state,
                                 const char *statusBar, float beerTemp,
                                 float chamberTemp, char tempFormat,
                                 bool darkmode) {
#if defined(ENABLE_LVGL)
  _tempFormat = tempFormat;
  _darkmode = darkmode;

  // Update the new UI using chamber_controller API
  chamber_controller_set_mode(mode);
  chamber_controller_set_state(state);
  chamber_controller_set_beer_temp(beerTemp, myConfig.getTempFormat());
  chamber_controller_set_chamber_temp(chamberTemp, myConfig.getTempFormat()); 
  chamber_controller_set_status(statusBar);
  
  // Update target temp
  chamber_controller_set_target_temp(_targetTemperature, myConfig.getTempFormat());
  
  // Update theme if changed
  chamber_controller_set_theme(darkmode);
#endif
}

void Display::calibrateTouch() {
#if defined(ENABLE_LVGL)
  if (!_tft) return;

  uint16_t x, y, pressed, i = 0;

  myDisplay.printLineCentered(4, "Press screen to calibrate");

  do {
    delay(300);
    pressed = _tft->getTouch(&x, &y, 600);
    // Log.info(F("DISP: Screen touched %d." CR), pressed);
  } while (!pressed && ++i < 10);

  if (pressed) {
    clear(TFT_GREEN);
    myDisplay.printLineCentered(4, "Touch detected");
    Log.info(F("DISP: Touch screen pressed, force calibration." CR));
    delay(3000);
  }

  File file = LittleFS.open(TTF_CALIBRATION_FILENAME, "r");

  if (file) {
    Log.info(F("DISP: Loading touch calibration data from file." CR));
    file.read(reinterpret_cast<uint8_t *>(&this->_touchCalibrationlData),
              sizeof(_touchCalibrationlData));
    file.close();
  } else {
    _touchCalibrationlData[0] = 0;
  }

  if (pressed || (_touchCalibrationlData[0] == 0)) {
    Log.info(F("DISP: Running calibration sequence." CR));

    clear();
    myDisplay.printLineCentered(4, "Calibration started");
    _tft->calibrateTouch(_touchCalibrationlData, TFT_GREEN, TFT_BLACK, 15);

    file = LittleFS.open(TTF_CALIBRATION_FILENAME, "w");

    if (file) {
      file.write(reinterpret_cast<uint8_t *>(&this->_touchCalibrationlData),
                 sizeof(_touchCalibrationlData));
      file.close();
    } else {
      Log.warning(F("DISP: Failed to write calibration data to file." CR));
    }

    myDisplay.printLineCentered(4, "Touch calibration completed");
    delay(3000);
  }

  myDisplay.printLineCentered(4, "");
#endif
}

bool Display::getTouch(uint16_t *x, uint16_t *y) {
#if defined(ENABLE_TFT)
  uint16_t xt, yt;
  uint8_t b = _tft->getTouch(&xt, &yt);

  if (b) {
    if (xt < 0) xt = 0;
    if (yt < 0) yt = 0;

    if (_rotation == Rotation::ROTATION_90) {
      *x = yt;
      *y = TFT_HEIGHT - xt;
      Log.verbose(F("DISP: Touch ROTATION_90: raw(%u,%u) -> display(%u,%u)" CR), xt, yt, *x, *y);
    } else {  // Rotation::ROTATION_270
      *x = TFT_WIDTH - yt;
      *y = xt;
      Log.verbose(F("DISP: Touch ROTATION_270: raw(%u,%u) -> display(%u,%u)" CR), xt, yt, *x, *y);
    }
  }

  return b;
#else
  return false;
#endif
}

void Display::createUI() {
#if defined(ENABLE_LVGL)
  if (!_tft) return;

  Log.notice(F("DISP: Using LVGL v%d.%d.%d." CR), lv_version_major(),
             lv_version_minor(), lv_version_patch());

  lv_init();
  lv_log_register_print_cb(log_print);

  Log.notice(F("DISP: Display dimensions: %d x %d, Rotation=%d (1=90°, 3=270°)" CR), TFT_WIDTH, TFT_HEIGHT, _rotation);
  Log.notice(F("DISP: Button positions - Beer(205,10,100,44) Chamber(205,60,100,44) Off(205,110,100,44) Down(30,161,44,44) Up(230,161,44,44)" CR));

  Log.notice(F("DISP: Display dimensions: %d x %d, Rotation=%d (1=90°, 3=270°)" CR), TFT_WIDTH, TFT_HEIGHT, _rotation);
  Log.notice(F("DISP: Button positions - Beer(205,10,100,44) Chamber(205,60,100,44) Off(205,110,100,44) Down(30,161,44,44) Up(230,161,44,44)" CR));

  Log.notice(F("DISP: Display configured - Width=%d Height=%d Rotation=%d" CR), TFT_WIDTH, TFT_HEIGHT, _rotation);
  Log.notice(F("DISP: Expected layout: 320x240 with buttons at: Beer(205,10) Chamber(205,60) Off(205,110) Down(30,161) Up(230,161)" CR));

#define DRAW_BUF_SIZE (TFT_WIDTH * TFT_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))

  void *draw_buf = ps_malloc(DRAW_BUF_SIZE);

  if (!draw_buf) {
    Log.error(
        F("DISP: Failed to allocate ps ram for display buffer, size=%d" CR),
        DRAW_BUF_SIZE);
  }

  _display = lv_tft_espi_create(TFT_WIDTH, TFT_HEIGHT, draw_buf, DRAW_BUF_SIZE);

  if (_rotation == Rotation::ROTATION_90) {
    lv_display_set_rotation(_display, LV_DISPLAY_ROTATION_90);
  } else {  // Rotation::ROTATION_270
    lv_display_set_rotation(_display, LV_DISPLAY_ROTATION_270);
  }

  // Initialize an LVGL input device object (Touchscreen)
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touchscreenHandler);

  // Initialize theme colors from ui_helpers
  _theme = UI_THEME_LIGHT;
  _colors = ui_get_theme_colors(_theme);

  Log.notice(F("DISP: Creating UI components." CR));

  // Initialize the new chamber controller UI with button callbacks
  Log.notice(F("DISP: Initializing chamber controller UI" CR));
  chamber_controller_init(_display, _darkmode,
                         btnBeerEventHandler,
                         btnChamberEventHandler,
                         btnOffEventHandler,
                         btnUpEventHandler,
                         btnDownEventHandler);
  Log.notice(F("DISP: Chamber controller UI initialized" CR));

  xTaskCreatePinnedToCore(lvgl_loop_handler,  // Function to implement the task
                          "LVGL_Handler",     // Name of the task
                          10000,              // Stack size in words
                          NULL,               // Task input parameter
                          0,                  // Priority of the task
                          &lvglTaskHandler,   // Task handle.
                          0);                 // Core where the task should run
#endif
}

void Display::handleButtonEvent(char btn) {
#if defined(ENABLE_LVGL)
  Log.info(F("DISP: Button pressed, char=%c" CR), btn);

  switch (btn) {
    case 'o':
    case 'b':
    case 'f':
      _mode = btn;
      setNewControllerMode(_mode, _targetTemperature);
      break;

    case '+':
      _targetTemperature += 0.5;
      break;

    case '-':
      _targetTemperature -= 0.5;
      break;
  }
#endif
}

// LVGL Wrappers and Handlers
// **************************************************************************************************

#if defined(ENABLE_LVGL)
void log_print(lv_log_level_t level, const char *buf) {
  LV_UNUSED(level);
  Log.notice(F("LVGL: %s." CR), buf);
}

void touchscreenHandler(lv_indev_t *indev, lv_indev_data_t *data) {
  uint16_t x = 0, y = 0;

  if (myDisplay.getTouch(&x, &y)) {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
    Log.verbose(F("LVGL: Touch point (%u,%u) sent to LVGL" CR), x, y);
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void btnBeerEventHandler(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
    myDisplay.handleButtonEvent('b');
  }
}

void btnChamberEventHandler(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
    myDisplay.handleButtonEvent('f');
  }
}

void btnOffEventHandler(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
    myDisplay.handleButtonEvent('o');
  }
}

void btnUpEventHandler(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
    myDisplay.handleButtonEvent('+');
  }
}

void btnDownEventHandler(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
    myDisplay.handleButtonEvent('-');
  }
}

void lvgl_loop_handler(void *parameter) {
  LoopTimer taskLoop(500);

  for (;;) {
    if (taskLoop.hasExpired()) {
      taskLoop.reset();
      // UI updates are handled by Display::updateTemperatures() calls
    }

    lv_task_handler();
    
    // Apply queued UI updates (thread-safe state application)
    chamber_controller_loop();
    
    lv_tick_inc(5);
    delay(5);
  }
}
#endif

// EOF
