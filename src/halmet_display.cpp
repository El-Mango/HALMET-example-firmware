
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

static bool Fits(const String& text, int size) {
  return TextWidth(text, size) <= kScreenWidth;
}

static void DrawCentered(Adafruit_SSD1306* display, const String& text,
                         int size, int y) {
  int x = (kScreenWidth - TextWidth(text, size)) / 2;
  display->setTextSize(size);
  display->setCursor(x < 0 ? 0 : x, y);
  display->print(text);
}

// Split `text` into two lines that each fit at `size`, breaking at a '.', ' '
// or ':' and choosing the most even split ("192." / "168.4.1" rather than
// "192.168.4." / "1"). Returns false if no such split exists.
static bool SplitInTwo(const String& text, int size, String* first,
                       String* second) {
  int best_longest = -1;
  for (int i = 1; i < (int)text.length() - 1; i++) {
    char c = text[i];
    if (c != '.' && c != ' ' && c != ':') continue;
    String a = text.substring(0, c == ' ' ? i : i + 1);
    String b = text.substring(i + 1);
    if (!Fits(a, size) || !Fits(b, size)) continue;
    int longest = a.length() > b.length() ? a.length() : b.length();
    if (best_longest < 0 || longest < best_longest) {
      best_longest = longest;
      *first = a;
      *second = b;
    }
  }
  return best_longest >= 0;
}

DisplayCarousel::DisplayCarousel(Adafruit_SSD1306* display,
                                 unsigned int interval_ms)
    : display_{display} {
  sensesp::event_loop()->onRepeat(interval_ms, [this]() { show_next(); });
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

void DisplayCarousel::show_next() {
  if (pages_.empty()) return;
  current_ = (current_ + 1) % pages_.size();
  draw(pages_[current_].first, pages_[current_].second);
}

// Layout on 128x64: title at size 2 (16 px) on top, value at size 3 (24 px,
// up to 7 characters) or size 2 (up to 10) below. A longer value, such as an
// IP address, takes two lines at size 2 under a small (size 1) title.
void DisplayCarousel::draw(const String& title, const String& value) {
  display_->clearDisplay();
  display_->setTextWrap(false);
  display_->setTextColor(SSD1306_WHITE);

  int title_size = Fits(title, 2) ? 2 : 1;
  String first, second;
  if (Fits(value, 3)) {
    DrawCentered(display_, title, title_size, 4);
    DrawCentered(display_, value, 3, 32);
  } else if (Fits(value, 2)) {
    DrawCentered(display_, title, title_size, 4);
    DrawCentered(display_, value, 2, 36);
  } else if (SplitInTwo(value, 2, &first, &second)) {
    DrawCentered(display_, title, 1, 0);
    DrawCentered(display_, first, 2, 18);
    DrawCentered(display_, second, 2, 42);
  } else {
    DrawCentered(display_, title, title_size, 4);
    DrawCentered(display_, value, 1, 40);
  }
  display_->display();
}

}  // namespace halmet
