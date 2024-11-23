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
 * @file RangingTag.ino
 * Use this to test two-way ranging functionality with two DW1000Jang:: This is
 * the tag component's code which polls for range computation. Addressing and
 * frame filtering is currently done in a custom way, as no MAC features are
 * implemented yet.
 *
 * Complements the "RangingAnchor" example sketch.
 *
 * @todo
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

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255



// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
uint64_t timePollSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
// data buffer
#define LEN_DATA 18
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;

uint16_t replyDelayTimeUS = 3000;

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

frame_filtering_configuration_t TAG_FRAME_FILTER_CONFIG = {
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
    Serial.begin(1000000);
    Serial.println(F("### DW1000Jang-arduino-ranging-initiator-2 ###"));
    // initialize the driver
    DW1000Jang::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println("DW1000Jang initialized ...");
    // general configuration
    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
	DW1000Jang::enableFrameFiltering(TAG_FRAME_FILTER_CONFIG);

    DW1000Jang::setNetworkId(10);
    DW1000Jang::setDeviceAddress(4);
    
    DW1000Jang::setAntennaDelay(16436);
    DW1000Jang::setPreambleDetectionTimeout(64);
    DW1000Jang::setSfdDetectionTimeout(273);
    DW1000Jang::setReceiveFrameWaitTimeoutPeriod(4000);
    
    // Serial.println(F("Committed configuration ..."));
    // // DEBUG chip info and registers pretty printed
    // char msg[128];
    // DW1000Jang::getPrintableDeviceIdentifier(msg);
    // Serial.print("Device ID: "); Serial.println(msg);
    // DW1000Jang::getPrintableExtendedUniqueIdentifier(msg);
    // Serial.print("Unique ID: "); Serial.println(msg);
    // DW1000Jang::getPrintableNetworkIdAndShortAddress(msg);
    // Serial.print("Network ID & Device Address: "); Serial.println(msg);
    // DW1000Jang::getPrintableDeviceMode(msg);
    // Serial.print("Device mode: "); Serial.println(msg);
}

byte target_anchor[2] = {0x05, 0x00};

void loop() {

    DW1000JangRTLS::transmitPoll(target_anchor);
    if(!DW1000JangRTLS::waitForNextRangingStep()) {
        return;
    } 
    else 
    {
        size_t cont_len = DW1000Jang::getReceivedDataLength();
        byte cont_recv[cont_len];
        DW1000Jang::getReceivedData(cont_recv, cont_len);

        if (cont_len > 10 && cont_recv[9] == ACTIVITY_CONTROL && cont_recv[10] == RANGING_CONTINUE) 
        {
            /* Received Response to poll */
            uint64_t timePollTransmitted = DW1000Jang::getTransmitTimestamp();
            uint64_t timeResponseReceived = DW1000Jang::getReceiveTimestamp();

            DW1000JangRTLS::transmitFinalMessage(
                &cont_recv[7], 
                700, 
                timePollTransmitted, // Poll transmit time
                timeResponseReceived  // Response to poll receive time
            );
            DW1000JangRTLS::waitForTransmission();
            
            if(!DW1000JangRTLS::receiveFrame()) {
                return;
            }
            else 
            {
                size_t act_len = DW1000Jang::getReceivedDataLength();
                char act_recv[act_len];
                DW1000Jang::getReceivedData(act_recv, act_len);

                if(act_len > 10 && act_recv[9] == ACTIVITY_CONTROL) 
                {
                    if (act_len > 12 && act_recv[10] == RANGING_CONFIRM) 
                    {
                        uint64_t timeReportReceived = DW1000Jang::getReceiveTimestamp();
                        double tmp = static_cast<double>(DW1000JangUtils::bytesAsValue(&act_recv[13],2) / 1000.0);
                        if(tmp > 65)
                            tmp = 0;
                        Serial.print("current distance : ");
                        Serial.print(tmp);
                        Serial.println(" [m]");
                        double t =  timeReportReceived - timePollTransmitted;
                        Serial.print("total Range Time : ");
                        Serial.print(t);
                        Serial.println(" [usec]");
                        return;
                    } 
                    
                } 
                else 
                {
                    return;
                }
            }
        } 
        else 
        {
            return;
        } 
    }
}