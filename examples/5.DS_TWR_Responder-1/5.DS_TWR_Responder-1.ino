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
#include <DW1000JangRanging.hpp>

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
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
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

uint16_t replyDelayTimeUS = 3000;


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
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
    true,
    true,
    false,
    false,
    false
};

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    Serial.println(F("### DW1000Jang-arduino-ranging-responder-1 ###"));
    // initialize the driver
    DW1000Jang::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println(F("DW1000Jang initialized ..."));
    // general configuration
    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
   DW1000Jang::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

    DW1000Jang::setDeviceAddress(1);
   
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
    // attach callback for (successfully) sent and received messages
    DW1000Jang::attachSentHandler(handleSent);
    DW1000Jang::attachReceivedHandler(handleReceived);
    // anchor starts in receiving mode, awaiting a ranging poll message
   
    receiver();
    noteActivity();
    // for first time ranging frequency computation
    rangingCountPeriod = millis();
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // anchor listens for POLL
    expectedMsgId = POLL;
    receiver();
    noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
}

void transmitPollAck() {
    data[0] = POLL_ACK;

    DW1000Jang::setTransmitData(data, LEN_DATA);
    DW1000Jang::startTransmit();
}

void transmitRangeReport(double distance) {
    data[0] = RANGE_REPORT;
    DW1000JangUtils::writeValueToBytes(&data[1], static_cast<uint16_t>((distance*1000)), 2);
    // write final ranging result
    
    DW1000Jang::setTransmitData(data, LEN_DATA);
    DW1000Jang::startTransmit();
}

void transmitRangeFailed() {
    data[0] = RANGE_FAILED;
    DW1000Jang::setTransmitData(data, LEN_DATA);
    DW1000Jang::startTransmit();
}

void receiver() {
    DW1000Jang::forceTRxOff();
    // so we don't need to restart the receiver manually
    DW1000Jang::startReceive();
}

void loop() {
  
    int32_t curMillis = millis();
    if (!sentAck && !receivedAck) {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {
            resetInactive();
        }
        return;
    }
    // continue on any success confirmation
    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL_ACK) {
            timePollAckSent = DW1000Jang::getTransmitTimestamp();
            noteActivity();
        }
        DW1000Jang::startReceive();
    }
    if (receivedAck) {
        
        receivedAck = false;
        // get message and parse
        DW1000Jang::getReceivedData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message, start over again (except if already POLL)
            protocolFailed = true;
        }
        if (msgId == POLL) {
            //Serial.println("poll receive");
            // on POLL we (re-)start, so no protocol failure
            protocolFailed = false;
            timePollReceived = DW1000Jang::getReceiveTimestamp();
            expectedMsgId = RANGE;
            transmitPollAck();
            noteActivity();
        }
        else if (msgId == RANGE) {
            timeRangeReceived = DW1000Jang::getReceiveTimestamp();
            expectedMsgId = POLL;
            if (!protocolFailed) {
                timePollSent = DW1000JangUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
                timePollAckReceived = DW1000JangUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
                timeRangeSent = DW1000JangUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
                // (re-)compute range as two-way ranging is done
                double distance = DW1000JangRanging::computeRangeAsymmetric(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);
                /* Apply simple bias correction */
                distance = DW1000JangRanging::correctRange(distance);                
                transmitRangeReport(distance);
               
                }
            }
            else {
                transmitRangeFailed();
            }

            noteActivity();
    }

}

