// include guard
#ifndef TRACK_LONG_STRIP_HPP
#define TRACK_LONG_STRIP_HPP
#include "Adafruit_NeoPixel.h"
#ifdef ESP_LED_STRIP
#include "led_strip.h"
#endif

namespace strip {
// Just a declaration
// I don't use C++ inheritance and too lazy to use any polymorphism library
// maybe some day
class IStrip {
public:
  /**
   * @brief expect to call `show` inside the function
   * @param start
   * @param count
   * @param color
   * @return if the operation is successful
   */
  virtual bool fill_and_show_forward(size_t start, size_t count, uint32_t color,uint8_t offset) {
    auto res = clear();
    if (!res) {
      return false;
    }
    auto begin = start + offset - count;
    // if(start < 20 or start > 480) {
    //   ESP_LOGI("show","FOR fill begin= %d,head=%d",begin,start);
    // }
    res = fill(begin, count, color);
    if (!res) {
      return false;
    }
    return show();
  };
  /**
   * @note expect to call `show` inside the function
   * @param start
   * @param count
   * @param color
   * @return if the operation is successful
   */
  virtual bool fill_and_show_backward(size_t start, size_t count, uint32_t color,uint8_t offset, uint32_t total) {
    auto res   = clear();
    if (!res) {
      return false;
    }
    auto begin = total - start + offset;
    // if(start < 30 or start > 480) {
    //   ESP_LOGI("show","BACK fill begin= %d,head=%d",begin,start);
    // }
    res = fill(begin, count, color);
    if (!res) {
      return false;
    }
    return show();
  };
  virtual bool clear()                                          = 0;
  virtual bool fill(size_t start, size_t count, uint32_t color) = 0;
  virtual bool show()                                           = 0;
  virtual bool begin()                                          = 0;
  virtual bool set_max_LEDs(size_t new_max_LEDs)                = 0;
  [[nodiscard]] virtual size_t get_max_LEDs() const             = 0;
  // resolve some  complain in destructor
  virtual ~IStrip() = default;
};

#ifdef ESP_LED_STRIP
class LedStripEsp : public IStrip {
private:
#if SOC_RMT_SUPPORT_DMA
  bool is_dma = true;
#else
  bool is_dma = false;
#endif
  // the resolution is the clock frequency instead of strip frequency
  const uint32_t LED_STRIP_RMT_RES_HZ = (10 * 1000 * 1000); // 10MHz
  led_strip_config_t strip_config     = {
          .strip_gpio_num   = GPIO_NUM_NC,          // The GPIO that connected to the LED strip's data line
          .max_leds         = 0,                    // The number of LEDs in the strip,
          .led_pixel_format = LED_PIXEL_FORMAT_RGB, // Pixel format of your LED strip
          .led_model        = LED_MODEL_WS2812,     // LED strip model
          .flags            = {
                         .invert_out = false // whether to invert the output signal
      },
  };
  led_strip_rmt_config_t rmt_config = {
      .clk_src           = RMT_CLK_SRC_DEFAULT,  // different clock source can lead to different power consumption
      .resolution_hz     = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
      .mem_block_symbols = SOC_RMT_TX_CANDIDATES_PER_GROUP * 64,
      .flags             = {
                      .with_dma = is_dma // DMA feature is available on ESP target like ESP32-S3
      },
  };
  led_strip_handle_t strip_handle = nullptr;

  /**
   * @param handle strip handle
   * @param start
   * @param count
   * @param color from MSB first byte is nothing. second byte is red. third byte is green. fourth byte is blue.
   * @return
   */
  static esp_err_t led_strip_set_many_pixels(led_strip_handle_t handle, int start, int count, uint32_t color) {
    const auto TAG = "led_strip";
    auto r         = (color >> 16) & 0xff;
    auto g         = (color >> 8) & 0xff;
    auto b         = color & 0xff;
    for (std::integral auto i : std::ranges::iota_view(start, start + count)) {
      // theoretically, I should be unsigned and never be negative.
      // However, I can safely ignore the negative value and continue the iteration.
      if (i < 0) {
        ESP_LOGW(TAG, "Invalid index %d", i);
        continue;
      }
      ESP_RETURN_ON_ERROR(led_strip_set_pixel(handle, i, r, g, b), TAG, "Failed to set pixel %d", i);
    }
    return ESP_OK;
  }

public:
  LedStripEsp() = default;
  LedStripEsp(LedStripEsp &&rhs) noexcept {
    rhs.strip_handle   = strip_handle;
    rhs.strip_config   = strip_config;
    rhs.rmt_config     = rmt_config;
    this->strip_handle = nullptr;
  };
  LedStripEsp(const LedStripEsp &rhs)    = delete;
  void operator=(const LedStripEsp &rhs) = delete;
  LedStripEsp(led_strip_config_t strip_config, led_strip_rmt_config_t rmt_config) : strip_config(strip_config), rmt_config(rmt_config) {}
  ~LedStripEsp() {
    if (strip_handle != nullptr) {
      led_strip_del(strip_handle);
    }
  }

  bool begin() override {
    return led_strip_new_rmt_device(&strip_config, &rmt_config, &strip_handle) == ESP_OK;
  }

  bool fill(size_t start, size_t count, uint32_t color) override {
    if (strip_handle == nullptr) {
      return false;
    }
    auto res = ESP_ERROR_CHECK_WITHOUT_ABORT(led_strip_set_many_pixels(strip_handle, start, count, color));
    return res == ESP_OK;
  }

  bool clear() override {
    if (strip_handle == nullptr) {
      return false;
    }
    return led_strip_clean_clear(strip_handle) == ESP_OK;
  }

  bool show() override {
    if (strip_handle == nullptr) {
      return false;
    }
    return led_strip_refresh(strip_handle) == ESP_OK;
  }

  bool set_max_LEDs(size_t new_max_LEDs) override {
    if (strip_handle != nullptr) {
      led_strip_del(strip_handle);
      strip_handle = nullptr;
    }

    strip_config.max_leds = new_max_LEDs;
    return led_strip_new_rmt_device(&strip_config, &rmt_config, &strip_handle) == ESP_OK;
  }
  size_t get_max_LEDs() const override {
    return strip_config.max_leds;
  }
};
#endif

class AdafruitPixel : public IStrip {
private:
  size_t max_LEDs = 0;
  uint8_t pin     = GPIO_NUM_NC;
  std::unique_ptr<Adafruit_NeoPixel> pixel{};
  neoPixelType pixelType;
  bool has_begun = false;

public:
  explicit AdafruitPixel(size_t max_LEDs, uint8_t pin, neoPixelType pixel_type) : max_LEDs(max_LEDs), pin(pin), pixelType(pixel_type) {
    pixel = std::make_unique<Adafruit_NeoPixel>(max_LEDs, pin, pixel_type);
    pixel->setBrightness(255);
  }

  /// move constructor
  AdafruitPixel(AdafruitPixel &&rhs) noexcept {
    this->max_LEDs  = rhs.max_LEDs;
    this->pin       = rhs.pin;
    this->pixelType = rhs.pixelType;
    this->pixel     = std::move(rhs.pixel);
    rhs.pixel       = nullptr;
  };
  AdafruitPixel(AdafruitPixel const &rhs)  = delete;
  void operator=(AdafruitPixel const &rhs) = delete;

  bool begin() override {
    if (pixel == nullptr) {
      return false;
    }
    pixel->begin();
    has_begun = true;
    return true;
  }

  bool fill(size_t start, size_t count, uint32_t color) override {
    if (pixel == nullptr) {
      return false;
    }
    pixel->fill(color, start, count);
    return true;
  }

  bool set_max_LEDs(size_t new_max_LEDs) override {
    if (pixel == nullptr) {
      return false;
    }
    this->max_LEDs = new_max_LEDs;
    pixel          = std::make_unique<Adafruit_NeoPixel>(max_LEDs, pin, pixelType);
    pixel->setBrightness(255);
    if (has_begun) {
      pixel->begin();
    }
    return true;
  }

  [[nodiscard]] size_t get_max_LEDs() const override {
    return max_LEDs;
  }

  bool clear() override {
    if (pixel == nullptr) {
      return false;
    }
    pixel->clear();
    return true;
  }
  bool show() override {
    if (pixel == nullptr) {
      return false;
    }
    pixel->show();
    return true;
  }
};
}
#endif // TRACK_LONG_STRIP_HPP