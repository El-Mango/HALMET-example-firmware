#ifndef __SRC_HALMET_DISPLAY_H__
#define __SRC_HALMET_DISPLAY_H__

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <utility>
#include <vector>

#include "sensesp_base_app.h"

namespace halmet {

bool InitializeSSD1306(const std::shared_ptr<sensesp::SensESPBaseApp> sensesp_app,
                       Adafruit_SSD1306** display, TwoWire* i2c);

void ClearRow(Adafruit_SSD1306* display, int row);

void PrintValue(Adafruit_SSD1306* display, int row, String title, float value);
void PrintValue(Adafruit_SSD1306* display, int row, String title, String value);

/**
 * @brief Shows one value at a time, full screen, cycling through pages.
 *
 * The 128x64 OLED is too small to read several values in the 6x8 font at a
 * glance. Each page is a title (large) over a value (larger), switching every
 * `interval_ms`. Add pages with add_page() and update them with set_value()
 * from wherever the value is produced; drawing happens on the timer only.
 */
class DisplayCarousel {
 public:
  DisplayCarousel(Adafruit_SSD1306* display, unsigned int interval_ms = 2000);

  /// Adds a page and returns its index. Pages are shown in the order added.
  int add_page(const String& title, const String& value = "--");
  void set_title(int page, const String& title);
  void set_value(int page, const String& value);

 private:
  void show_next();
  void draw(const String& title, const String& value);

  Adafruit_SSD1306* display_;
  std::vector<std::pair<String, String>> pages_;  // (title, value)
  size_t current_ = 0;
};

}  // namespace halmet

#endif
