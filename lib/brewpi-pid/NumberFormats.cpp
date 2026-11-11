/*
 * Copyright 2012-2013 BrewPi/Elco Jacobs.
 * Copyright 2013 Matthew McGowan.
 *
 * This file is part of BrewPi.
 *
 * BrewPi is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BrewPi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BrewPi.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include <NumberFormats.hpp>

static int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Parse a hex string (upper or lower case) into exactly len 8-bit bytes. The
// string comes from the configuration, so it can have any length or content:
// never write more than len bytes, and if the string is not exactly 2 * len
// hex digits clear the whole buffer (all zeros is not a valid device address).
bool parseBytes(uint8_t *data, const char *s, uint8_t len) {
  memset(data, 0, len);
  if (s == NULL) return false;

  for (uint8_t i = 0; i < len; i++) {
    int high = hexDigit(s[2 * i]);
    // Only look at the next character if the string did not end
    int low = high < 0 ? -1 : hexDigit(s[2 * i + 1]);

    if (high < 0 || low < 0) {
      memset(data, 0, len);
      return false;
    }
    data[i] = (high << 4) | low;
  }

  if (s[2 * len] != '\0') {  // trailing characters
    memset(data, 0, len);
    return false;
  }
  return true;
}

// Render 8-bit data as hex
void printBytes(const uint8_t *data, uint8_t len, char *buf) {
  for (int i = 0; i < len; i++) {
    uint8_t b = (data[i] >> 4) & 0x0f;
    *buf++ = (b > 9 ? b - 10 + 'A' : b + '0');
    b = data[i] & 0x0f;
    *buf++ = (b > 9 ? b - 10 + 'A' : b + '0');
  }
  *buf = 0;
}

// EOF
