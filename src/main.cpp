// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

// Comment out this line to disable NMEA 2000 output.
#define ENABLE_NMEA2000_OUTPUT

// Comment out this line to disable Signal K support. At the moment, disabling
// Signal K support also disables all WiFi functionality.
#define ENABLE_SIGNALK

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <memory>

#include "counting_nmea2000.h"

#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp/ui/ui_controls.h"
#ifdef ENABLE_SIGNALK
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder
#else
#include "sensesp/net/web/app_command_handler.h"
#include "sensesp/net/web/base_command_handler.h"
#include "sensesp/net/web/config_handler.h"
#include "sensesp/net/web/static_file_handler.h"
#include "sensesp_minimal_app_builder.h"
#define BUILDER_CLASS SensESPMinimalAppBuilder
#endif

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"

using namespace sensesp;
using namespace halmet;

#ifndef ENABLE_SIGNALK
// SensESPMinimalApp creates none of these; the web UI needs them.
std::shared_ptr<SensESPMinimalApp> sensesp_app;
std::shared_ptr<Networking> networking;
std::shared_ptr<MDNSDiscovery> mdns_discovery;
std::shared_ptr<HTTPServer> http_server;
std::shared_ptr<SystemStatusLed> system_status_led;
#endif

/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

#ifdef ENABLE_NMEA2000_OUTPUT
CountingNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
// Received-message counter shown on the status page.
ObservableValue<int> n2k_rx_counter{0};
#endif

TwoWire* i2c;
Adafruit_SSD1306* display;
// Cycles one value at a time over the whole display; null if no display.
DisplayCarousel* display_carousel = nullptr;

// Alarm states for local display output.
// EDIT: Names shown on the display when an alarm is active. Add an entry (and
// an index before kNumAlarms) for every alarm you connect.
enum AlarmIndex {
  kAlarmOilPressure,
  kAlarmEngineTemp,
  kAlarmFuelLow,
  kAlarmWaterLow,
  kNumAlarms
};
const char* const kAlarmNames[kNumAlarms] = {"Oil pressure", "Engine temp",
                                             "Fuel low", "Water low"};
bool alarm_states[kNumAlarms] = {};

// Raise `alarm` when the tank level drops below a threshold set in the web UI
// (default 25 %), and clear it only once the level is kHysteresis above it, so
// fuel sloshing around the threshold does not make the alarm flicker.
void ConnectTankLowAlarm(sensesp::FloatProducer* tank_level, const String& name,
                         int sort_order, AlarmIndex alarm) {
  const float kHysteresis = 2;  // percentage points
  float default_percent = 25;
  String config_path = "/Tanks/" + name + "/Low Level Alarm";
  auto threshold = new sensesp::NumberConfig(default_percent, config_path);
  ConfigItem(threshold)
      ->set_title(name + " Low Level Alarm (%)")
      ->set_description("Show a low-level alarm on the display when the " +
                        name + " tank is below this level")
      ->set_sort_order(sort_order);

  tank_level->connect_to(
      new sensesp::LambdaConsumer<float>([threshold, alarm, kHysteresis](float level) {
        float percent = 100 * level;
        float limit = threshold->get_value();
        if (percent < limit) {
          alarm_states[alarm] = true;
        } else if (percent > limit + kHysteresis) {
          alarm_states[alarm] = false;
        }
      }));
}

// Set the ADS1115 GAIN to adjust the analog input voltage range.
// On HALMET, this refers to the voltage range of the ADS1115 input
// AFTER the 33.3/3.3 voltage divider.

// GAIN_TWOTHIRDS: 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// GAIN_ONE:       1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// GAIN_TWO:       2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// GAIN_FOUR:      4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// GAIN_EIGHT:     8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// GAIN_SIXTEEN:   16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

const adsGain_t kADS1115Gain = GAIN_ONE;

/////////////////////////////////////////////////////////////////////
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
const int kTestOutputFrequency = 380;
#endif

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // These calls can be used for fine-grained control over the logging level.
  // esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

  Serial.begin(115200);

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    // EDIT: Set a custom hostname for the app.
                    ->set_hostname("halmet")
                    // EDIT: Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed. These three calls
                    // exist only on SensESPAppBuilder and do not compile when
                    // ENABLE_SIGNALK is commented out (SensESPMinimalAppBuilder).
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    // EDIT: Enable OTA updates with a password.
                    //->enable_ota("my_ota_password")
                    ->get_app();

#ifndef ENABLE_SIGNALK
  // Initialize components that would normally be present in SensESPApp
  networking = std::make_shared<Networking>("/System/WiFi Settings", "", "");
  mdns_discovery = std::make_shared<MDNSDiscovery>();
  http_server = std::make_shared<HTTPServer>();
  // Nothing drives the LED here; SensESPApp is what connects it to the
  // system status controller.
  system_status_led = std::make_shared<SystemStatusLed>(LED_BUILTIN);
  add_static_file_handlers(http_server);
  add_base_app_http_command_handlers(http_server);
  add_app_http_command_handlers(http_server, networking);
  add_config_handlers(http_server);
#endif

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();

  ads1115->setGain(kADS1115Gain);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcAttach(kTestOutputPin, kTestOutputFrequency, 13);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(kTestOutputPin, 4096);
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  nmea2000 = new CountingNMEA2000(kCANTxPin, kCANRxPin);

  // The send buffer was reduced from this example's earlier 250 frames to 96
  // (about 2.5 KB) to reclaim contiguous heap for the mbedTLS record
  // allocation -- raising it back can reintroduce MBEDTLS_ERR_SSL_ALLOC_FAILED;
  // re-check the largest free block on the status page before doing so. The
  // receive side is not set: NMEA2000_twai uses fixed 40-entry TWAI queues and
  // ignores SetN2kCANReceiveFrameBufSize.
  nmea2000->SetN2kCANSendFrameBufSize(96);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);

  // Count received N2K messages and feed the watchdog.
  nmea2000->SetMsgHandler([](const tN2kMsg&) {
    n2k_rx_counter.set(n2k_rx_counter.get() + 1);
    n2k_time_since_rx = 0;
  });

  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  // NMEA 2000 message counters on the status page. TX is tallied by
  // CountingNMEA2000 on each accepted SendMsg; RX by the handler above.
  // connect_to() copies these shared_ptrs into the producer's observer list, so
  // they keep themselves alive.
  auto n2k_rx_status = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Received Messages", 0, "NMEA 2000", 300);
  n2k_rx_counter.connect_to(n2k_rx_status);
  auto n2k_tx_status = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Transmitted Messages", 0, "NMEA 2000", 310);
  nmea2000->tx_count_.connect_to(n2k_tx_status);
#endif  // ENABLE_NMEA2000_OUTPUT

  // Initialize the OLED display
  bool display_present = InitializeSSD1306(sensesp_app->get(), &display, i2c);
  if (display_present) {
    // EDIT: Change the time each page stays on screen (ms).
    display_carousel = new DisplayCarousel(display, 4000);
  }

  ///////////////////////////////////////////////////////////////////
  // Analog inputs

#ifdef ENABLE_SIGNALK
  bool enable_signalk_output = true;
#else
  bool enable_signalk_output = false;
#endif

  // Connect the tank senders.
  // EDIT: To enable more tanks, uncomment the lines below.
  auto tank_a1_volume = ConnectTankSender(ads1115, 0, "Fuel", "fuel.main", 3000,
                                          enable_signalk_output);
  // A2 is used as a plain voltage input below; the water tank is on A3.
  auto tank_a3_volume = ConnectTankSender(ads1115, 2, "Water",
                                          "freshWater.main", 3100,
                                          enable_signalk_output);
  // auto tank_a4_volume = ConnectTankSender(ads1115, 3, "A4");

  ConnectTankLowAlarm(tank_a1_volume, "Fuel", 3008, kAlarmFuelLow);
  ConnectTankLowAlarm(tank_a3_volume, "Water", 3108, kAlarmWaterLow);

#ifdef ENABLE_NMEA2000_OUTPUT
  // Tank 1, instance 0. Capacity 200 liters. You can change the capacity
  // in the web UI as well.
  // EDIT: Make sure this matches your tank configuration above.
  N2kFluidLevelSender* tank_a1_sender = new N2kFluidLevelSender(
      "/Tanks/Fuel/NMEA 2000", 0, N2kft_Fuel, 200, nmea2000);

  ConfigItem(tank_a1_sender)
      ->set_title("Tank A1 NMEA 2000")
      ->set_description("NMEA 2000 tank sender for tank A1")
      ->set_sort_order(3005);

  tank_a1_volume->connect_to(&(tank_a1_sender->tank_level_));

  // Water tank on A3, instance 0 of the fresh water type. Capacity 100 liters.
  N2kFluidLevelSender* tank_a3_sender = new N2kFluidLevelSender(
      "/Tanks/Water/NMEA 2000", 0, N2kft_Water, 100, nmea2000);

  ConfigItem(tank_a3_sender)
      ->set_title("Tank A3 NMEA 2000")
      ->set_description("NMEA 2000 tank sender for tank A3 (water)")
      ->set_sort_order(3105);

  tank_a3_volume->connect_to(&(tank_a3_sender->tank_level_));
#endif  // ENABLE_NMEA2000_OUTPUT

  if (display_carousel) {
    // EDIT: Duplicate the lines below to make the display show all your tanks.
    int fuel_page = display_carousel->add_page("Fuel");
    tank_a1_volume->connect_to(
        new LambdaConsumer<float>([fuel_page](float value) {
          display_carousel->set_value(fuel_page, String(100 * value, 0) + " %");
        }));
    int water_page = display_carousel->add_page("Water");
    tank_a3_volume->connect_to(
        new LambdaConsumer<float>([water_page](float value) {
          display_carousel->set_value(water_page, String(100 * value, 0) + " %");
        }));
  }

  // Read the voltage level of analog input A2
  auto a2_voltage = new ADS1115VoltageInput(ads1115, 1, "/Voltage A2");

  ConfigItem(a2_voltage)
      ->set_title("Analog Voltage A2")
      ->set_description("Voltage level of analog input A2")
      ->set_sort_order(3000);

  a2_voltage->connect_to(new LambdaConsumer<float>(
      [](float value) { debugD("Voltage A2: %f", value); }));

  // If you want to output something else than the voltage value,
  // you can insert a suitable transform here.
  // For example, to convert the voltage to a distance with a conversion
  // factor of 0.17 m/V, you could use the following code:
  // auto a2_distance = new Linear(0.17, 0.0);
  // a2_voltage->connect_to(a2_distance);

#ifdef ENABLE_SIGNALK
  a2_voltage->connect_to(
      new SKOutputFloat("sensors.a2.voltage", "Analog Voltage A2",
                        new SKMetadata("V", "Analog Voltage A2")));
  // Example of how to output the distance value to Signal K.
  // a2_distance->connect_to(
  //     new SKOutputFloat("sensors.a2.distance", "Analog Distance A2",
  //                       new SKMetadata("m", "Analog Distance A2")));
#endif

  ///////////////////////////////////////////////////////////////////
  // Digital alarm inputs

  // EDIT: More alarm inputs can be defined by duplicating the lines below.
  // Make sure to not define a pin for both a tacho and an alarm.
  auto alarm_d2_input =
      ConnectAlarmSender(kDigitalInputPin2, "D2", enable_signalk_output);
  auto alarm_d3_input =
      ConnectAlarmSender(kDigitalInputPin3, "D3", enable_signalk_output);
  // auto alarm_d4_input =
  //     ConnectAlarmSender(kDigitalInputPin4, "D4", enable_signalk_output);

  // Update the alarm states based on the input value changes.
  // EDIT: If you added more alarm inputs, uncomment the respective lines below.
  alarm_d2_input->connect_to(
      new LambdaConsumer<bool>(
          [](bool value) { alarm_states[kAlarmOilPressure] = value; }));
  // In this example, alarm_d3_input is active low, so invert the value.
  auto alarm_d3_inverted = alarm_d3_input->connect_to(
      new LambdaTransform<bool, bool>([](bool value) { return !value; }));
  alarm_d3_inverted->connect_to(
      new LambdaConsumer<bool>(
          [](bool value) { alarm_states[kAlarmEngineTemp] = value; }));
  // alarm_d4_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[kAlarmX] = value; }));

#ifdef ENABLE_NMEA2000_OUTPUT
  // EDIT: This example connects the D2 alarm input to the low oil pressure
  // warning. Modify according to your needs.
  N2kEngineParameterDynamicSender* engine_dynamic_sender =
      new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0,
                                          nmea2000);

  ConfigItem(engine_dynamic_sender)
      ->set_title("Engine 1 Dynamic")
      ->set_description("NMEA 2000 dynamic engine parameters for engine 1")
      ->set_sort_order(3010);

  alarm_d2_input->connect_to(engine_dynamic_sender->low_oil_pressure_);

  // This is just an example -- normally temperature alarms would not be
  // active-low (inverted).
  alarm_d3_inverted->connect_to(engine_dynamic_sender->over_temperature_);
#endif  // ENABLE_NMEA2000_OUTPUT

  // FIXME: Transmit the alarms over SK as well.

  ///////////////////////////////////////////////////////////////////
  // Digital tacho inputs

  // Connect the tacho senders. Engine name is "main".
  // EDIT: More tacho inputs can be defined by duplicating the line below.
  auto tacho_d1_frequency =
      ConnectTachoSender(kDigitalInputPin1, "main", enable_signalk_output);

#ifdef ENABLE_NMEA2000_OUTPUT
  // Connect outputs to the N2k senders.
  // EDIT: Make sure this matches your tacho configuration above.
  //       Duplicate the lines below to connect more tachos, but be sure to
  //       use different engine instances.
  N2kEngineParameterRapidSender* engine_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 1 Rapid Update", 0,
                                        nmea2000);  // Engine 1, instance 0

  ConfigItem(engine_rapid_sender)
      ->set_title("Engine 1 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 1")
      ->set_sort_order(3015);

  tacho_d1_frequency->connect_to(&(engine_rapid_sender->engine_speed_));
#endif  // ENABLE_NMEA2000_OUTPUT

  if (display_carousel) {
    int rpm_d1_page = display_carousel->add_page("RPM");
    tacho_d1_frequency->connect_to(
        new LambdaConsumer<float>([rpm_d1_page](float value) {
          display_carousel->set_value(rpm_d1_page, String(60 * value, 0));
        }));
  }

#ifdef ENABLE_NMEA2000_OUTPUT
  ///////////////////////////////////////////////////////////////////
  // Configure the NMEA 2000 watchdog

  CheckboxConfig* enable_n2k_watchdog_config = new CheckboxConfig(
      false, "Enable NMEA 2000 Watchdog", "/NMEA 2000/Enable Watchdog");

  ConfigItem(enable_n2k_watchdog_config)
      ->set_title("Enable NMEA 2000 Watchdog")
      ->set_description(
          "Enable the NMEA 2000 watchdog. If enabled, the device will reboot "
          "after two minutes if no NMEA 2000 messages are received. This "
          "setting requires a restart to take effect.")
      ->set_sort_order(100)
      ->set_requires_restart(true);

  if (enable_n2k_watchdog_config->get_value()) {
    event_loop()->onRepeat(1000, []() {
      if (n2k_time_since_rx > 120000) {
        ESP_LOGE("NMEA2000", "No messages received in 2 minutes. Restarting.");
        // All hope is lost; it doesn't matter if we delay for a bit to ensure
        // the log message is sent.
        delay(10);
        ESP.restart();
      }
    });
  }
#endif  // ENABLE_NMEA2000_OUTPUT

  ///////////////////////////////////////////////////////////////////
  // Display setup

  // Connect the outputs to the display
  if (display_carousel) {
    // Active alarms by name ("Oil pressure, Fuel low"; long lists scroll), or
    // "OK" when none is active.
    int alarms_page = display_carousel->add_page("Alarms");
    event_loop()->onRepeat(1000, [alarms_page]() {
      String active;
      for (int i = 0; i < kNumAlarms; i++) {
        if (alarm_states[i]) {
          if (active.length()) active += ", ";
          active += kAlarmNames[i];
        }
      }
      display_carousel->set_title(alarms_page, active.length() ? "ALARM" : "Alarms");
      display_carousel->set_value(alarms_page, active.length() ? active : "OK");
    });

    // Station IP once joined to a network; the setup access point's IP while
    // the device is still being provisioned (localIP() is 0.0.0.0 then).
    int ip_page = display_carousel->add_page("IP");
    event_loop()->onRepeat(1000, [ip_page]() {
      if (WiFi.isConnected()) {
        display_carousel->set_title(ip_page, "IP");
        display_carousel->set_value(ip_page, WiFi.localIP().toString());
      } else if (WiFi.getMode() & WIFI_AP) {
        display_carousel->set_title(ip_page, "Setup AP");
        display_carousel->set_value(ip_page, WiFi.softAPIP().toString());
      } else {
        display_carousel->set_title(ip_page, "IP");
        display_carousel->set_value(ip_page, "no network");
      }
    });
  }

  // Heap and main-loop-stack diagnostics on the status page. The largest free
  // block is what the TLS handshake needs as one contiguous allocation.
  auto largest_block_status = std::make_shared<StatusPageItem<int>>(
      "Largest free block (bytes)", 0, "System", 250);
  event_loop()->onRepeat(2000, [largest_block_status]() {
    largest_block_status->set(static_cast<int>(ESP.getMaxAllocHeap()));
  });

  auto main_loop_stack_status = std::make_shared<StatusPageItem<int>>(
      "Main loop min free stack (bytes)", 0, "System", 260);
  event_loop()->onRepeat(2000, [main_loop_stack_status]() {
    main_loop_stack_status->set(
        static_cast<int>(uxTaskGetStackHighWaterMark(nullptr)));
  });

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
