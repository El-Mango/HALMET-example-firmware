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
 * glance. Each page is a left-aligned title (size 2) over a large, centred
 * value (size 3). A value
 * too wide for the screen scrolls horizontally, and its page stays up until
 * the scroll has finished. Add pages with add_page() and update them with
 * set_title()/set_value() from wherever the value is produced; the display is
 * redrawn by the carousel's own timer, and only when something changed.
 */
class DisplayCarousel {
 public:
  DisplayCarousel(Adafruit_SSD1306* display, unsigned int page_ms = 4000);

  /// Adds a page and returns its index. Pages are shown in the order added.
  int add_page(const String& title, const String& value = "--");
  void set_title(int page, const String& title);
  void set_value(int page, const String& value);

 private:
  void tick();
  void draw(const String& title, const String& value, int scroll_offset);

  Adafruit_SSD1306* display_;
  unsigned int page_ms_;
  std::vector<std::pair<String, String>> pages_;  // (title, value)
  size_t current_ = 0;
  unsigned long page_start_ = 0;
  // What is on screen now, to skip redundant redraws over I2C.
  String drawn_title_, drawn_value_;
  int drawn_offset_ = -1;
};

}  // namespace halmet

#endif
