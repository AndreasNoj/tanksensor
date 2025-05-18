

#include <memory>

#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
// Sensor-specific #includes:
#include <Adafruit_BMP280.h>
#include <Wire.h>

// #define SERIAL_DEBUG_DISABLED = true

using namespace sensesp;
using namespace reactesp;
using namespace sensesp::onewire;
reactesp::ReactESP app;

void setup() {
  SetupLogging();

  // Set up sensesp
  SetupLogging(ESP_LOG_DEBUG);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("refrigerator-monitor")
                    ->enable_ota("LilleMyOTA")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("Obelix", "obelix2idefix")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    //->enable_uptime_sensor()
                    ->get_app();

  /*
     Find all the sensors and their unique addresses. Then, each new instance
     of OneWireTemperature will use one of those addresses. You can't specify
     which address will initially be assigned to a particular sensor, so if you
     have more than one sensor, you may have to swap the addresses around on
     the configuration page for the device. (You get to the configuration page
     by entering the IP address of the device into a browser.)
  */

  /*
     Tell SensESP where the sensor is connected to the board
     ESP32 pins are specified as just the X in GPIOX
  */
  uint8_t pin = 4;

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(pin);

  // Define how often SensESP should read the sensor(s) in milliseconds
  uint read_delay = 500;

  // Below are temperatures sampled and sent to Signal K server
  // To find valid Signal K Paths that fits your need you look at this link:
  // https://signalk.org/specification/1.7.0/doc/vesselsBranch.html

  // Create OneWire temperature sensors for refrigerator monitoring
  auto* fridge_inside_temp =
      new OneWireTemperature(dts, read_delay, "/fridge/inside/oneWire");
  auto* fridge_compressor_temp =
      new OneWireTemperature(dts, read_delay, "/fridge/compressor/oneWire");
  auto* fridge_locker_temp =
      new OneWireTemperature(dts, read_delay, "/fridge/locker/oneWire");

  // Calibration transforms
  auto* fridge_inside_cal = new Linear(1.0, 0.0, "/fridge/inside/calibration");
  auto* fridge_compressor_cal =
      new Linear(1.0, 0.0, "/fridge/compressor/calibration");
  auto* fridge_locker_cal = new Linear(1.0, 0.0, "/fridge/locker/calibration");

  // Signal K outputs with configurable SK paths
  auto* fridge_inside_sk = new SKOutputFloat(
      "environment.refrigerator.inside.temperature", "/fridge/inside/skPath");
  auto* fridge_compressor_sk =
      new SKOutputFloat("environment.refrigerator.compressor.temperature",
                        "/fridge/compressor/skPath");
  auto* fridge_locker_sk = new SKOutputFloat(
      "environment.refrigerator.locker.temperature", "/fridge/locker/skPath");

  // Optionally label in Config UI
  ConfigItem(fridge_inside_temp)
      ->set_title("Fridge Interior Temp")
      ->set_description("Temperature inside the refrigerator");

  ConfigItem(fridge_compressor_temp)
      ->set_title("Fridge Compressor Temp")
      ->set_description("Temperature on the compressor");

  ConfigItem(fridge_locker_temp)
      ->set_title("Fridge Locker Temp")
      ->set_description(
          "Ambient temperature where the fridge compressor is installed");

  ConfigItem(fridge_inside_cal)->set_title("Fridge Interior Calibration");
  ConfigItem(fridge_compressor_cal)->set_title("Compressor Calibration");
  ConfigItem(fridge_locker_cal)->set_title("Locker Calibration");

  ConfigItem(fridge_inside_sk)->set_title("Signal K Path - Fridge Inside");
  ConfigItem(fridge_compressor_sk)
      ->set_title("Signal K Path - Fridge Compressor");
  ConfigItem(fridge_locker_sk)->set_title("Signal K Path - Fridge Locker");

  // Connect pipelines
  fridge_inside_temp->connect_to(fridge_inside_cal)
      ->connect_to(fridge_inside_sk);
  fridge_compressor_temp->connect_to(fridge_compressor_cal)
      ->connect_to(fridge_compressor_sk);
  fridge_locker_temp->connect_to(fridge_locker_cal)
      ->connect_to(fridge_locker_sk);

  // Constants
  const float adc_voltage_reference = 3.3;  // ESP32 ADC ref voltage
  const int adc_max = 4095;                 // 12-bit ADC
  const float fixed_resistor = 220.0;       // Your fixed resistor in ohms

  // Voltage divider equation: Vout = Vin * (sensor / (sensor + fixed))
  // Reverse to get sensor resistance from voltage:
  // R_sensor = R_fixed * (V / (Vin - V))

//   // Analog input pin
//   const int analog_pin = 34;

//   // Create analog sensor
//   auto* analog_input =
//       new AnalogInput(analog_pin, 1000);  // sample every 1000 ms

//   // Convert raw voltage to resistance using a lambda
//   auto* resistance_transform = new LambdaTransform<float, float>(
//       [fixed_resistor, adc_voltage_reference, adc_max](float voltage) -> float {
//         if (voltage <= 0.0 || voltage >= adc_voltage_reference) return 9999.0;
//         return fixed_resistor * voltage / (adc_voltage_reference - voltage);
//       });

//   // Convert resistance (0–190Ω) to percentage
//   auto* percent_transform =
//       new Linear(1.0 / 190.0, 0.0);  // 0Ω -> 0%, 190Ω -> 100%

//   // Connect the pipeline
//   analog_input->connect_to(resistance_transform)
//       ->connect_to(percent_transform)
//       ->connect_to(new SKOutputFloat("tanks.diesel.level"));
}

void loop() { event_loop()->tick(); }