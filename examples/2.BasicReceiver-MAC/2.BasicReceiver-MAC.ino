/*
 * MIT License
 * 
 * Copyright (c) 2018 Michele Biondi, Andrea Salvatori
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file BasicReceiver.ino
 * Use this to test simple sender/receiver functionality with two
 * DW1000:: Complements the "BasicSender" example sketch.
 * 
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  
 */

#include <DW1000Jang.hpp>

#if defined(ESP8266)
//const uint8_t PIN_RST = 5; // reset pin
//const uint8_t PIN_IRQ = 4; // irq pin
const uint8_t PIN_SS = 15; // spi select pin
#else
//const uint8_t PIN_RST = 7; // reset pin
//const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = 10; // spi select pin
#endif

int16_t numReceived = 0; // todo check int type
String message;



device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_128,
    PreambleCode::CODE_3
};

frame_filtering_configuration_t FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  Serial.println(F("### DW1000Jang-arduino-receiver-test ###"));
  // initialize the driver
  DW1000Jang::initializeNoInterrupt(PIN_SS);
  Serial.println(F("DW1000Jang initialized ..."));

  DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
  DW1000Jang::enableFrameFiltering(FRAME_FILTER_CONFIG);

  DW1000Jang::setDeviceAddress(6);
  DW1000Jang::setNetworkId(10);

  DW1000Jang::setAntennaDelay(16436);
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Jang::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000Jang::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Jang::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Jang::getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
}

void loop() {
  DW1000Jang::startReceive();
  while(!DW1000Jang::isReceiveDone()) {
    #if defined(ESP8266)
    yield();
    #endif
  }
  DW1000Jang::clearReceiveStatus();
  numReceived++;
  // get data as string
  DW1000Jang::getReceivedData(message);

  Serial.println("************************************");
  Serial.print("Received message ... #"); Serial.println(numReceived);


  Serial.print("Receivied Data : "); 
  for(int i=11; i<16; i++)
    Serial.print(char(message[i]));
  
  Serial.println("\n************************************");
  
}
