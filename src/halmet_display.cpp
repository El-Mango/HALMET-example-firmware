
#include "halmet_display.h"

#include <WiFi.h>

namespace halmet {

// OLED display width and height, in pixels
const int kScreenWidth = 128;
const int kScreenHeight = 64;

bool InitializeSSD1306(const std::shared_ptr<sensesp::SensESPBaseApp> sensesp_app,
                       Adafruit_SSD1306** display, TwoWire* i2c) {
  *display = new Adafruit_SSD1306(kScreenWidth, kScreenHeight, i2c, -1);
  bool init_successful = (*display)->begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!init_successful) {
    debugD("SSD1306 allocation failed");
    return false;
  }
  delay(100);
  (*display)->setRotation(2);
  (*display)->clearDisplay();
  (*display)->setTextSize(1);
  (*display)->setTextColor(SSD1306_WHITE);
  (*display)->setCursor(0, 0);
  (*display)->printf("Host: %s\n", sensesp_app->get_hostname().c_str());
  (*display)->display();

  return true;
}

/// Clear a text row on an Adafruit graphics display
void ClearRow(Adafruit_SSD1306* display, int row) {
  display->fillRect(0, 8 * row, kScreenWidth, 8, 0);
}

void PrintValue(Adafruit_SSD1306* display, int row, String title, float value) {
  ClearRow(display, row);
  display->setCursor(0, 8 * row);
  display->printf("%s: %.1f", title.c_str(), value);
  display->display();
}

void PrintValue(Adafruit_SSD1306* display, int row, String title,
                String value) {
  ClearRow(display, row);
  display->setCursor(0, 8 * row);
  display->printf("%s: %s", title.c_str(), value.c_str());
  display->display();
}

// The built-in font is 6x8 pixels per character (5x7 glyph plus spacing),
// scaled by the text size.
static int TextWidth(const String& text, int size) {
  return text.length() == 0 ? 0 : text.length() * 6 * size - size;
}

static void DrawCentered(Adafruit_SSD1306* display, const String& text,
                         int size, int y) {
  int x = (kScreenWidth - TextWidth(text, size)) / 2;
  display->setTextSize(size);
  display->setCursor(x < 0 ? 0 : x, y);
  display->print(text);
}

const int kTitleSize = 2;
const int kValueSize = 3;
const int kTitleY = 4;
const int kValueY = 32;
// Scrolling: move kScrollStepPx every kScrollStepMs, holding still for
// kScrollPauseMs at the start and at the end. Each redraw pushes the whole
// 1 KB frame over I2C, so the step interval is kept modest.
const unsigned int kScrollStepMs = 100;
const int kScrollStepPx = 6;
const unsigned int kScrollPauseMs = 1000;

DisplayCarousel::DisplayCarousel(Adafruit_SSD1306* display,
                                 unsigned int page_ms)
    : display_{display}, page_ms_{page_ms} {
  page_start_ = millis();
  sensesp::event_loop()->onRepeat(kScrollStepMs, [this]() { tick(); });
}

int DisplayCarousel::add_page(const String& title, const String& value) {
  pages_.emplace_back(title, value);
  return pages_.size() - 1;
}

void DisplayCarousel::set_title(int page, const String& title) {
  if (page >= 0 && page < (int)pages_.size()) pages_[page].first = title;
}

void DisplayCarousel::set_value(int page, const String& value) {
  if (page >= 0 && page < (int)pages_.size()) pages_[page].second = value;
}

void DisplayCarousel::tick() {
  if (pages_.empty()) return;
  unsigned long elapsed = millis() - page_start_;

  // A value wider than the screen scrolls; its page lasts at least as long
  // as the scroll plus both pauses.
  int overflow = TextWidth(pages_[current_].second, kValueSize) - kScreenWidth;
  unsigned long duration = page_ms_;
  if (overflow > 0) {
    unsigned long scroll_ms =
        ((overflow + kScrollStepPx - 1) / kScrollStepPx) * kScrollStepMs;
    unsigned long needed = 2 * kScrollPauseMs + scroll_ms;
    if (needed > duration) duration = needed;
  }
  if (elapsed >= duration) {
    current_ = (current_ + 1) % pages_.size();
    page_start_ = millis();
    elapsed = 0;
    overflow =
        TextWidth(pages_[current_].second, kValueSize) - kScreenWidth;
  }

  int offset = 0;
  if (overflow > 0 && elapsed > kScrollPauseMs) {
    offset = ((elapsed - kScrollPauseMs) / kScrollStepMs) * kScrollStepPx;
    if (offset > overflow) offset = overflow;
  }

  const String& title = pages_[current_].first;
  const String& value = pages_[current_].second;
  if (title == drawn_title_ && value == drawn_value_ &&
      offset == drawn_offset_) {
    return;
  }
  draw(title, value, offset);
  drawn_title_ = title;
  drawn_value_ = value;
  drawn_offset_ = offset;
}

void DisplayCarousel::draw(const String& title, const String& value,
                           int scroll_offset) {
  display_->clearDisplay();
  display_->setTextWrap(false);
  display_->setTextColor(SSD1306_WHITE);

  int title_size = TextWidth(title, kTitleSize) <= kScreenWidth ? kTitleSize : 1;
  DrawCentered(display_, title, title_size, kTitleY);

  if (TextWidth(value, kValueSize) <= kScreenWidth) {
    DrawCentered(display_, value, kValueSize, kValueY);
  } else {
    // Characters that fall off the left edge are clipped by Adafruit_GFX.
    display_->setTextSize(kValueSize);
    display_->setCursor(-scroll_offset, kValueY);
    display_->print(value);
  }
  display_->display();
}

}  // namespace halmet
