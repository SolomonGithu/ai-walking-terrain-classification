/* Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// This codes runs a terrain classification model using IMU data and send results via BLE

/* Includes ---------------------------------------------------------------- */
#include <Walk_terrain_classification_inferencing.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h>

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f
#define MAX_ACCEPTED_RANGE 2.0f  // starting 03/2022, models are generated setting range to +-2, but this example use Arudino library which set range to +-4g. If you are using an older model, ignore this value and use 4.0f instead

BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");  // UART Service UUID

BLECharacteristic txCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
                                   BLERead | BLENotify, 20);

unsigned long currentMillis;
unsigned long previousMillis = 0;
const long interval = 1000;    // 1 second
const int ble_upload_ms = 60000;  // frequency to send step counts via BLE, in seconds
int countdownTime = ble_upload_ms;

int inference_time, flat_surface_steps, uphill_steps, downhill_steps = 0;

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;  // Set this to true to see e.g. features generated from the raw signal
LSM6DS3 myIMU(I2C_MODE, 0x6A);
/**
* @brief      Arduino setup function
*/

const int RED_ledPin = 11;
const int BLUE_ledPin = 12;
const int GREEN_ledPin = 13;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //while (!Serial);
  Serial.println("Edge Impulse Inferencing Demo");

  if (myIMU.begin() != 0) {
  //if (!myIMU.begin()) {
    ei_printf("Failed to initialize IMU!\r\n");
  } else {
    ei_printf("IMU initialized\r\n");
  }

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
    return;
  }

  if (!BLE.begin()) {
    Serial.println("BLE init failed!");
    while (1)
      ;
  }
  BLE.setLocalName("XIAO-nRF52840-Sense");
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(txCharacteristic);
  BLE.addService(uartService);
  BLE.advertise();
  Serial.println("BLE UART-like Service Started");
}

/**
 * @brief Return the sign of the number
 * 
 * @param number 
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number) {
  return (number >= 0.0) ? 1.0 : -1.0;
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop() {
  currentMillis = millis();

  Serial.println("==============================");

  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
  }

  uint8_t buf1[64] = "idle";
  uint8_t buf2[64] = "left&right";
  uint8_t buf3[64] = "up&down";

  ei_printf("Sampling...\n");

  // Allocate a buffer here for the values we'll read from the IMU
  float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

  for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
    // Determine the next tick (and then sleep later)
    uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

    buffer[ix] = myIMU.readFloatAccelX();
    buffer[ix + 1] = myIMU.readFloatAccelY();
    buffer[ix + 2] = myIMU.readFloatAccelZ();

    //buffer[ix] = myIMU.readFloatGyroX();
    //buffer[ix+1] = myIMU.readFloatGyroY();
    //buffer[ix+2] = myIMU.readFloatGyroZ();

    for (int i = 0; i < 3; i++) {
      if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
        buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
      }
    }

    buffer[ix + 0] *= CONVERT_G_TO_MS2;
    buffer[ix + 1] *= CONVERT_G_TO_MS2;
    buffer[ix + 2] *= CONVERT_G_TO_MS2;
    /*
    Serial.print(buffer[ix + 0], 4);
    Serial.print('\t');
    Serial.print(buffer[ix + 1], 4);
    Serial.print('\t');
    Serial.println(buffer[ix + 2], 4);
    */

    delayMicroseconds(next_tick - micros());
  }

  // Turn the raw buffer in a signal which we can the classify
  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
    ei_printf("Failed to create signal from buffer (%d)\n", err);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = { 0 };

  err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // print the predictions
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  ei_printf(": \n");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

// "downhill_surface", "flat_surface", "no_motion", "uphill_surface"
  if (result.classification[1].value > 0.5) {
    digitalWrite(RED_ledPin, LOW);
    digitalWrite(BLUE_ledPin, HIGH);
    digitalWrite(GREEN_ledPin, HIGH);
    flat_surface_steps += 1;
  }
  if (result.classification[3].value > 0.5) {
    digitalWrite(RED_ledPin, HIGH);
    digitalWrite(BLUE_ledPin, LOW);
    digitalWrite(GREEN_ledPin, HIGH);
    uphill_steps += 1;
  }
  if (result.classification[0].value > 0.5) {
    digitalWrite(RED_ledPin, HIGH);
    digitalWrite(BLUE_ledPin, HIGH);
    digitalWrite(GREEN_ledPin, LOW);
    downhill_steps += 1;
  }

  Serial.print("Flat surface steps = "); Serial.println(flat_surface_steps);
  Serial.print("Uphill surface steps = "); Serial.println(uphill_steps);
  Serial.print("Downhill surface steps = "); Serial.println(downhill_steps);

  // Send steps classification after the set duration
  if (countdownTime > 0) {
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      countdownTime--;
      /*
      Serial.print("Time left: ");
      Serial.print(countdownTime);
      Serial.println(" seconds");
      */
    }
  } else {
    //Serial.println("Countdown finished! Restarting...");
    if (central.connected()) {
      inference_time = result.timing.dsp + result.timing.classification;
      String message = String(inference_time) + ',' + String(flat_surface_steps) + ',' + String(uphill_steps) + ',' + String(downhill_steps);

      txCharacteristic.writeValue((const uint8_t*)message.c_str(), message.length());  // Send data via BLE
      Serial.print("Data sent via BLE: ");
      Serial.println(message);
    }
    countdownTime = ble_upload_ms;   // Reset countdown
    previousMillis = currentMillis;  // Reset timing
  }
}
