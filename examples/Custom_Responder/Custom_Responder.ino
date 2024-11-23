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
 * @file RangingAnchor.ino
 * Use this to test two-way ranging functionality with two
 * DW1000Jang:: This is the anchor component's code which computes range after
 * exchanging some messages. Addressing and frame filtering is currently done
 * in a custom way, as no MAC features are implemented yet.
 *
 * Complements the "RangingTag" example sketch.
 *
 * @todo
 *  - weighted average of ranging results based on signal quality
 *  - use enum instead of define
 *  - move strings to flash (less RAM consumption)
 */

#include <DW1000Jang.hpp>
#include <DW1000JangUtils.hpp>
#include <DW1000JangTime.hpp>
#include <DW1000JangConstants.hpp>
#include <DW1000JangRanging.hpp>
#include <DW1000JangRTLS.hpp>

// connection pins
const uint8_t PIN_RST = 7; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = 10; // spi select pin

// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;

uint64_t timeComputedRange;
// last computed range/time
// data buffer
#define LEN_DATA 18
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_3,
    DataRate::RATE_6800KBPS,
    PulseFrequency::FREQ_64MHZ,
    PreambleLength::LEN_128,
    PreambleCode::CODE_10
};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false /* This allows blink frames */
};

void setup() {
    // DEBUG monitoring
    Serial.begin(1000000);
    delay(1000);
    //Serial.println(F("### DW1000Jang-arduino-ranging-responder-2 ###"));
    // initialize the driver
    DW1000Jang::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    //Serial.println(F("DW1000Jang initialized ..."));
    // general configuration
    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
    DW1000Jang::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);

    DW1000Jang::setPreambleDetectionTimeout(64);
    DW1000Jang::setSfdDetectionTimeout(273);
    DW1000Jang::setReceiveFrameWaitTimeoutPeriod(400);

    DW1000Jang::setDeviceAddress(5);
    DW1000Jang::setNetworkId(16);
   
    DW1000Jang::setAntennaDelay(16436);
    
    //Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Jang::getPrintableNetworkIdAndShortAddress(msg);
    //Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Jang::getPrintableDeviceMode(msg);
    //Serial.print("Device mode: "); Serial.println(msg);
}

void loop() {

    double range;
    if(!DW1000JangRTLS::receiveFrame()) {
        return;
    }
    else 
    {
        size_t poll_len = DW1000Jang::getReceivedDataLength();
        byte poll_data[poll_len];
        DW1000Jang::getReceivedData(poll_data, poll_len);

        if(poll_len > 9 && poll_data[9] == RANGING_TAG_POLL) 
        { 
            uint64_t timePollReceived = DW1000Jang::getReceiveTimestamp();
            double phasePoll = DW1000Jang::getReceivedPhase();

            uint64_t timeResponseToPoll = DW1000JangRTLS::transmitResponseToPoll_v2(&poll_data[7], 1400);
            DW1000JangRTLS::waitForTransmission();
            
            if(!DW1000JangRTLS::receiveFrame_v2(1250)) {
                return;
            }
            else 
            {
                size_t rfinal_len = DW1000Jang::getReceivedDataLength();
                byte rfinal_data[rfinal_len];
                DW1000Jang::getReceivedData(rfinal_data, rfinal_len);
                
                if(rfinal_len > 22 && rfinal_data[9] == RANGING_TAG_FINAL_RESPONSE_EMBEDDED) {
                    uint64_t timeFinalMessageReceive = DW1000Jang::getReceiveTimestamp();
                    double phaseFinal = DW1000Jang::getReceivedPhase();
                    double phaseResponse = static_cast<double>(DW1000JangUtils::bytesAsValue(&rfinal_data[22], 4) / 1000.0);
                    uint64_t timePollSent = DW1000JangUtils::bytesAsValue(&rfinal_data[10], 4);
                    uint64_t timeResponseToPollReceived = DW1000JangUtils::bytesAsValue(&rfinal_data[14], 4);
                    uint64_t timeFinalMessageSent = DW1000JangUtils::bytesAsValue(&rfinal_data[18], 4);

                    if (!DW1000JangRTLS::receiveFrame_v3(1250)) {
                        return;
                    }
                    else 
                    {
                        size_t rpostfinal_len = DW1000Jang::getReceivedDataLength();
                        byte rpostfinal_data[rpostfinal_len];
                        DW1000Jang::getReceivedData(rpostfinal_data, rpostfinal_len);

                        if (rpostfinal_len > 9 && rpostfinal_data[9] == RANGING_TAG_POST_FINAL_RESPONSE_EMBEDDED) {
                            double phasePostFinal = DW1000Jang::getReceivedPhase(); 
                      
                            range = DW1000JangRanging::computeRangeAsymmetric(
                                timePollSent, // Poll send time
                                timePollReceived, 
                                timeResponseToPoll, // Response to poll sent time
                                timeResponseToPollReceived, // Response to Poll Received
                                timeFinalMessageSent, // Final Message send time
                                timeFinalMessageReceive // Final message receive time
                            );

                            /* In case of wrong read due to bad device calibration */
                            // if(range <= 0)       
                            //     range = 0.000001;

                            double sumPollResp = fmod((phasePoll + phaseResponse), 2 * PI);
                            if (sumPollResp < 0) {
                              sumPollResp += 2 * PI;
                            }

                            double diffFinalPost = fmod((phaseFinal - phasePostFinal), 2 * PI);
                            if (diffFinalPost < 0) {
                              diffFinalPost += 2 * PI;
                            }
                            
                            double recPhase = fmod(sumPollResp - diffFinalPost, 2 * PI);
                            if (recPhase < 0) {
                               recPhase += 2 * PI;
                            }

                            // Serial.print(sumPollResp);
                            // Serial.print("|");
                            // Serial.print(diffFinalPost);
                            // Serial.print("|");
                            Serial.print(recPhase);
                            Serial.print("|");
                            //Serial.println("|");

                            //Serial.print("  current distance : ");
                            Serial.println(range);
                            //Serial.println(" [m]");
                            return;
                        }
                    }
                }
            }
        }
    }
}
    

