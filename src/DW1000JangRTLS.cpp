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

#include <Arduino.h>
#include "DW1000JangRTLS.hpp"
#include "DW1000Jang.hpp"
#include "DW1000JangUtils.hpp"
#include "DW1000JangTime.hpp"
#include "DW1000JangRanging.hpp"

static byte SEQ_NUMBER = 0;

namespace DW1000JangRTLS 
{

    byte increaseSequenceNumber(){
        return ++SEQ_NUMBER;
    }

    void transmitTwrShortBlink() {
        byte Blink[] = {BLINK, SEQ_NUMBER++, 0,0,0,0,0,0,0,0, NO_BATTERY_STATUS | NO_EX_ID, TAG_LISTENING_NOW};
        DW1000Jang::getEUI(&Blink[2]);
        DW1000Jang::setTransmitData(Blink, sizeof(Blink));
        DW1000Jang::startTransmit();
    }

    void transmitRangingInitiation(byte tag_eui[], byte tag_short_address[]) {
        byte RangingInitiation[] = {DATA, SHORT_SRC_LONG_DEST, SEQ_NUMBER++, 0,0, 0,0,0,0,0,0,0,0,  0,0, RANGING_INITIATION, 0,0};
        DW1000Jang::getNetworkId(&RangingInitiation[3]);
        memcpy(&RangingInitiation[5], tag_eui, 8);
        DW1000Jang::getDeviceAddress(&RangingInitiation[13]);
        memcpy(&RangingInitiation[16], tag_short_address, 2);
        DW1000Jang::setTransmitData(RangingInitiation, sizeof(RangingInitiation));
        DW1000Jang::startTransmit();
    }

    void transmitPoll(byte anchor_address[]){
        byte Poll[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0 , RANGING_TAG_POLL,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

        DW1000Jang::getNetworkId(&Poll[3]);
        memcpy(&Poll[5], anchor_address, 2);
        DW1000Jang::getDeviceAddress(&Poll[7]);
        DW1000Jang::setTransmitData(Poll, sizeof(Poll));
        DW1000Jang::startTransmit();
    }

    void transmitResponseToPoll(byte tag_short_address[]) {
        byte pollAck[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0, ACTIVITY_CONTROL, RANGING_CONTINUE, 0, 0};

        DW1000Jang::getNetworkId(&pollAck[3]);
        memcpy(&pollAck[5], tag_short_address, 2);
        DW1000Jang::getDeviceAddress(&pollAck[7]);
        DW1000Jang::setTransmitData(pollAck, sizeof(pollAck));
        DW1000Jang::startTransmit();
    }

    uint64_t transmitResponseToPoll_v2(byte anchor_address[], uint16_t reply_delay) {
        /* Calculation of future time */
        byte futureTimeBytes[LENGTH_TIMESTAMP];

	    uint64_t timeFinalMessageSent = DW1000Jang::getReceiveTimestamp();
	    timeFinalMessageSent += DW1000JangTime::microsecondsToUWBTime(reply_delay);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, timeFinalMessageSent, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);
        timeFinalMessageSent += DW1000Jang::getTxAntennaDelay();

        byte pollAck[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0, ACTIVITY_CONTROL, RANGING_CONTINUE,
            0,0,0,0,0,0,0,0,0,0,0,0,0 };

        DW1000Jang::getNetworkId(&pollAck[3]);
        memcpy(&pollAck[5], anchor_address, 2);
        DW1000Jang::getDeviceAddress(&pollAck[7]);
        DW1000Jang::setTransmitData(pollAck, sizeof(pollAck));
        DW1000Jang::startTransmit(TransmitMode::DELAYED);

        return timeFinalMessageSent;
    }

    void transmitResponseToPoll_v3(byte anchor_address[], uint16_t reply_delay) {
        /* Calculation of future time */
        byte futureTimeBytes[LENGTH_TIMESTAMP];

	    uint64_t timeFinalMessageSent = DW1000Jang::getReceiveTimestamp();
	    timeFinalMessageSent += DW1000JangTime::microsecondsToUWBTime(reply_delay);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, timeFinalMessageSent, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);
        timeFinalMessageSent += DW1000Jang::getTxAntennaDelay();

        byte pollAck[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0, ACTIVITY_CONTROL, RANGING_CONTINUE,
            0,0,0,0,0,0,0,0,0,0,0,0,0 };

        DW1000Jang::getNetworkId(&pollAck[3]);
        memcpy(&pollAck[5], anchor_address, 2);
        DW1000Jang::getDeviceAddress(&pollAck[7]);
        DW1000Jang::setTransmitData(pollAck, sizeof(pollAck));
        DW1000Jang::startTransmit(TransmitMode::DELAYED);
    }

    void transmitFinalMessage(byte anchor_address[], uint16_t reply_delay, uint64_t timePollSent, uint64_t timeResponseToPollReceived) {
        /* Calculation of future time */
        byte futureTimeBytes[LENGTH_TIMESTAMP];

	    uint64_t timeFinalMessageSent = DW1000Jang::getSystemTimestamp();
	    timeFinalMessageSent += DW1000JangTime::microsecondsToUWBTime(reply_delay);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, timeFinalMessageSent, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);
        timeFinalMessageSent += DW1000Jang::getTxAntennaDelay();

        byte finalMessage[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0, RANGING_TAG_FINAL_RESPONSE_EMBEDDED, 
            0,0,0,0,0,0,0,0,0,0,0,0 };

        DW1000Jang::getNetworkId(&finalMessage[3]);
        memcpy(&finalMessage[5], anchor_address, 2);
        DW1000Jang::getDeviceAddress(&finalMessage[7]);

        DW1000JangUtils::writeValueToBytes(finalMessage + 10, (uint32_t) timePollSent, 4);
        DW1000JangUtils::writeValueToBytes(finalMessage + 14, (uint32_t) timeResponseToPollReceived, 4);
        DW1000JangUtils::writeValueToBytes(finalMessage + 18, (uint32_t) timeFinalMessageSent, 4);
        DW1000Jang::setTransmitData(finalMessage, sizeof(finalMessage));
        DW1000Jang::startTransmit(TransmitMode::DELAYED);
    }

    // Final Message를 보낼 때 Response Message를 수신한 timestamp를 기준으로 delay를 잡는 함수
    void transmitFinalMessage_v2(byte anchor_address[], uint16_t reply_delay, uint64_t timePollSent, uint64_t timeResponseToPollReceived, double phaseResponse) {
        /* Calculation of future time */
        byte futureTimeBytes[LENGTH_TIMESTAMP];

	    uint64_t timeFinalMessageSent = DW1000Jang::getReceiveTimestamp();
	    timeFinalMessageSent += DW1000JangTime::microsecondsToUWBTime(reply_delay);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, timeFinalMessageSent, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);
        timeFinalMessageSent += DW1000Jang::getTxAntennaDelay();

        byte finalMessage[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0, RANGING_TAG_FINAL_RESPONSE_EMBEDDED, 
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

        DW1000Jang::getNetworkId(&finalMessage[3]);
        memcpy(&finalMessage[5], anchor_address, 2);
        DW1000Jang::getDeviceAddress(&finalMessage[7]);

        DW1000JangUtils::writeValueToBytes(finalMessage + 10, (uint32_t) timePollSent, 4);
        DW1000JangUtils::writeValueToBytes(finalMessage + 14, (uint32_t) timeResponseToPollReceived, 4);
        DW1000JangUtils::writeValueToBytes(finalMessage + 18, (uint32_t) timeFinalMessageSent, 4);
        DW1000JangUtils::writeValueToBytes(finalMessage + 22, static_cast<uint32_t>(phaseResponse * 1000), 4);
        DW1000Jang::setTransmitData(finalMessage, sizeof(finalMessage));
        DW1000Jang::startTransmit(TransmitMode::DELAYED);
    }

    // Final Message를 보낼 때 현재 timestamp를 기준으로 delay를 잡는 함수
    void transmitFinalMessage_v3(byte anchor_address[], uint16_t reply_delay, uint64_t timePollSent, uint64_t timeResponseToPollReceived, double phaseResponse) {
        /* Calculation of future time */
        byte futureTimeBytes[LENGTH_TIMESTAMP];

	    uint64_t timeFinalMessageSent = DW1000Jang::getSystemTimestamp();
	    timeFinalMessageSent += DW1000JangTime::microsecondsToUWBTime(reply_delay);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, timeFinalMessageSent, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);
        timeFinalMessageSent += DW1000Jang::getTxAntennaDelay();

        byte finalMessage[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0, RANGING_TAG_FINAL_RESPONSE_EMBEDDED, 
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

        DW1000Jang::getNetworkId(&finalMessage[3]);
        memcpy(&finalMessage[5], anchor_address, 2);
        DW1000Jang::getDeviceAddress(&finalMessage[7]);

        DW1000JangUtils::writeValueToBytes(finalMessage + 10, (uint32_t) timePollSent, 4);
        DW1000JangUtils::writeValueToBytes(finalMessage + 14, (uint32_t) timeResponseToPollReceived, 4);
        DW1000JangUtils::writeValueToBytes(finalMessage + 18, (uint32_t) timeFinalMessageSent, 4);
        DW1000JangUtils::writeValueToBytes(finalMessage + 22, static_cast<uint32_t>(phaseResponse * 1000), 4);
        DW1000Jang::setTransmitData(finalMessage, sizeof(finalMessage));
        DW1000Jang::startTransmit(TransmitMode::DELAYED);
    }

    // PostFinal Message를 보낼 때 Final Message를 송신한 timestamp를 기준으로 delay를 잡는 함수
    void transmitPostFinalMessage(byte anchor_address[], uint16_t reply_delay) {
        /* Calculation of future time */
        byte futureTimeBytes[LENGTH_TIMESTAMP];

	    uint64_t timeFinalMessageSent = DW1000Jang::getTransmitTimestamp();
	    timeFinalMessageSent += DW1000JangTime::microsecondsToUWBTime(reply_delay);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, timeFinalMessageSent, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);
        timeFinalMessageSent += DW1000Jang::getTxAntennaDelay();

        byte finalMessage[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0, RANGING_TAG_POST_FINAL_RESPONSE_EMBEDDED, 
            0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

        DW1000Jang::getNetworkId(&finalMessage[3]);
        memcpy(&finalMessage[5], anchor_address, 2);
        DW1000Jang::getDeviceAddress(&finalMessage[7]);
        DW1000Jang::setTransmitData(finalMessage, sizeof(finalMessage));
        DW1000Jang::startTransmit(TransmitMode::DELAYED);
    }

    // PostFinal Message를 보낼 때 현재 timestamp를 기준으로 delay를 잡는 함수
    void transmitPostFinalMessage_v2(byte anchor_address[], uint16_t reply_delay) {
        /* Calculation of future time */
        byte futureTimeBytes[LENGTH_TIMESTAMP];

	    uint64_t timeFinalMessageSent = DW1000Jang::getSystemTimestamp();
	    timeFinalMessageSent += DW1000JangTime::microsecondsToUWBTime(reply_delay);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, timeFinalMessageSent, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);
        timeFinalMessageSent += DW1000Jang::getTxAntennaDelay();

        byte finalMessage[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0, RANGING_TAG_POST_FINAL_RESPONSE_EMBEDDED, 
            0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

        DW1000Jang::getNetworkId(&finalMessage[3]);
        memcpy(&finalMessage[5], anchor_address, 2);
        DW1000Jang::getDeviceAddress(&finalMessage[7]);
        DW1000Jang::setTransmitData(finalMessage, sizeof(finalMessage));
        DW1000Jang::startTransmit(TransmitMode::DELAYED);
    }

    void transmitRangingConfirm(byte tag_short_address[], byte next_anchor[]) {
        byte rangingConfirm[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 
        0,0, ACTIVITY_CONTROL, RANGING_CONFIRM, next_anchor[0], next_anchor[1]};
        
        DW1000Jang::getNetworkId(&rangingConfirm[3]);
        memcpy(&rangingConfirm[5], tag_short_address, 2);
        DW1000Jang::getDeviceAddress(&rangingConfirm[7]);
        DW1000Jang::setTransmitData(rangingConfirm, sizeof(rangingConfirm));
        DW1000Jang::startTransmit();
    }

    void transmitRangingConfirm_v1(byte tag_short_address[], double distance) {
        byte rangingConfirm[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 0,0, ACTIVITY_CONTROL, RANGING_CONFIRM, 0, 0, 0, 0};
        DW1000Jang::getNetworkId(&rangingConfirm[3]);
        memcpy(&rangingConfirm[5], tag_short_address, 2);
        DW1000Jang::getDeviceAddress(&rangingConfirm[7]);
        DW1000JangUtils::writeValueToBytes(&rangingConfirm[13], static_cast<uint16_t>((distance*1000)), 2);
        DW1000Jang::setTransmitData(rangingConfirm, sizeof(rangingConfirm));
        DW1000Jang::startTransmit();
    }

    void transmitRangingConfirm_v2(byte tag_short_address[], byte next_anchor[], double distance) {
        byte rangingConfirm[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 0,0, ACTIVITY_CONTROL, RANGING_CONFIRM, next_anchor[0], next_anchor[1],0,0};
        DW1000Jang::getNetworkId(&rangingConfirm[3]);
        memcpy(&rangingConfirm[5], tag_short_address, 2);
        DW1000Jang::getDeviceAddress(&rangingConfirm[7]);
        DW1000JangUtils::writeValueToBytes(&rangingConfirm[13], static_cast<uint16_t>((distance*1000)), 2);
        DW1000Jang::setTransmitData(rangingConfirm, sizeof(rangingConfirm));
        DW1000Jang::startTransmit();
    }

    void transmitRangingConfirm_v3(byte tag_short_address[], double distance) {

        byte futureTimeBytes[LENGTH_TIMESTAMP];

	    uint64_t timeFinalMessageSent = DW1000Jang::getSystemTimestamp();
	    timeFinalMessageSent += DW1000JangTime::microsecondsToUWBTime(3000);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, timeFinalMessageSent, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);
        byte rangingConfirm[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 0,0, 0,0, ACTIVITY_CONTROL, RANGING_CONFIRM,0,0,0,0};
        DW1000Jang::getNetworkId(&rangingConfirm[3]);
        memcpy(&rangingConfirm[5], tag_short_address, 2);
        DW1000Jang::getDeviceAddress(&rangingConfirm[7]);
        DW1000JangUtils::writeValueToBytes(&rangingConfirm[13], static_cast<uint16_t>((distance*1000)), 2);
        DW1000Jang::setTransmitData(rangingConfirm, sizeof(rangingConfirm));
        DW1000Jang::startTransmit(TransmitMode::DELAYED);
    }


    void transmitActivityFinished(byte tag_short_address[], byte blink_rate[]) {
        /* I send the new blink rate to the tag */
        byte rangingConfirm[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 0,0, ACTIVITY_CONTROL, ACTIVITY_FINISHED, blink_rate[0], blink_rate[1],0,0};
        DW1000Jang::getNetworkId(&rangingConfirm[3]);
        memcpy(&rangingConfirm[5], tag_short_address, 2);
        DW1000Jang::getDeviceAddress(&rangingConfirm[7]);
        DW1000Jang::setTransmitData(rangingConfirm, sizeof(rangingConfirm));
        DW1000Jang::startTransmit();
    }

    void transmitActivityFinished_v2(byte tag_short_address[], byte blink_rate[], double distance) {
        /* I send the new blink rate to the tag */
        byte rangingConfirm[] = {DATA, SHORT_SRC_AND_DEST, SEQ_NUMBER++, 0,0, 0,0, 0,0, ACTIVITY_CONTROL, ACTIVITY_FINISHED, blink_rate[0], blink_rate[1],0,0};
        DW1000Jang::getNetworkId(&rangingConfirm[3]);
        memcpy(&rangingConfirm[5], tag_short_address, 2);
        DW1000Jang::getDeviceAddress(&rangingConfirm[7]);
        DW1000JangUtils::writeValueToBytes(&rangingConfirm[13], static_cast<uint16_t>((distance*1000)), 2);
        DW1000Jang::setTransmitData(rangingConfirm, sizeof(rangingConfirm));
        DW1000Jang::startTransmit();
    }

    static uint32_t calculateNewBlinkRate(byte frame[]) {
        uint32_t blinkRate = frame[11] + static_cast<uint32_t>(((frame[12] & 0x3F) << 8));
        byte multiplier = ((frame[12] & 0xC0) >> 6);
        if(multiplier  == 0x01) {
            blinkRate *= 25;
        } else if(multiplier == 0x02) {
            blinkRate *= 1000;
        }

        return blinkRate;
    }

    void waitForTransmission() {
        while(!DW1000Jang::isTransmitDone()) {
            #if defined(ESP8266)
            yield();
            #endif
        }
        DW1000Jang::clearTransmitStatus();
    }

    boolean receiveFrame() {
        DW1000Jang::startReceive(ReceiveMode::IMMEDIATE);
        while(!DW1000Jang::isReceiveDone()) {
            if(DW1000Jang::isReceiveTimeout() ) {
                DW1000Jang::clearReceiveTimeoutStatus();
                return false;
            }
            #if defined(ESP8266)
            yield();
            #endif
        }
        DW1000Jang::clearReceiveStatus();
        return true;
    }

    boolean receiveFrame_v2(uint64_t timeDelay) {
        byte futureTimeBytes[LENGTH_TIMESTAMP];

        uint64_t time = DW1000Jang::getTransmitTimestamp();
        time += DW1000JangTime::microsecondsToUWBTime(timeDelay);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, time, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);

        DW1000Jang::startReceive(ReceiveMode::DELAYED);
        while(!DW1000Jang::isReceiveDone()) {
            if(DW1000Jang::isReceiveTimeout() ) {
                DW1000Jang::clearReceiveTimeoutStatus();
                return false;
            }
            #if defined(ESP8266)
            yield();
            #endif
        }
        DW1000Jang::clearReceiveStatus();
        return true;
    }

    boolean receiveFrame_v3(uint64_t timeDelay) {
        byte futureTimeBytes[LENGTH_TIMESTAMP];

        uint64_t time = DW1000Jang::getReceiveTimestamp();
        time += DW1000JangTime::microsecondsToUWBTime(timeDelay);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, time, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);

        DW1000Jang::startReceive(ReceiveMode::DELAYED);
        while(!DW1000Jang::isReceiveDone()) {
            if(DW1000Jang::isReceiveTimeout() ) {
                DW1000Jang::clearReceiveTimeoutStatus();
                return false;
            }
            #if defined(ESP8266)
            yield();
            #endif
        }
        DW1000Jang::clearReceiveStatus();
        return true;
    }

    boolean receiveFrame2() {

        byte futureTimeBytes[LENGTH_TIMESTAMP];

	    uint64_t time = DW1000Jang::getSystemTimestamp();
	    time += DW1000JangTime::microsecondsToUWBTime(2000);
        DW1000JangUtils::writeValueToBytes(futureTimeBytes, time, LENGTH_TIMESTAMP);
        DW1000Jang::setDelayedTRX(futureTimeBytes);

        DW1000Jang::startReceive(ReceiveMode::DELAYED);
        while(!DW1000Jang::isReceiveDone()) {
            if(DW1000Jang::isReceiveTimeout() ) {
                DW1000Jang::clearReceiveTimeoutStatus();
                return false;
            }
            #if defined(ESP8266)
            yield();
            #endif
        }
        DW1000Jang::clearReceiveStatus();
        return true;
    }

    static boolean waitForNextRangingStep() {
        DW1000JangRTLS::waitForTransmission();
        if(!DW1000JangRTLS::receiveFrame()) return false;
        return true;
    }

    static boolean waitForNextRangingStep_v2(uint64_t timeDelay) {
        DW1000JangRTLS::waitForTransmission();
        if(!DW1000JangRTLS::receiveFrame_v2(timeDelay)) return false;
        return true;
    }

    static boolean waitForNextRangingStep2() {
        DW1000JangRTLS::waitForTransmission();
        if(!DW1000JangRTLS::receiveFrame2()) return false;
        return true;
    }

    RangeRequestResult tagRangeRequest() {
        DW1000JangRTLS::transmitTwrShortBlink();
        
        if(!DW1000JangRTLS::waitForNextRangingStep()) return {false, 0};

        size_t init_len = DW1000Jang::getReceivedDataLength();
        byte init_recv[init_len];
        DW1000Jang::getReceivedData(init_recv, init_len);

        if(!(init_len > 17 && init_recv[15] == RANGING_INITIATION)) {
            return { false, 0};
        }

        DW1000Jang::setDeviceAddress(DW1000JangUtils::bytesAsValue(&init_recv[16], 2));
        return { true, static_cast<uint16_t>(DW1000JangUtils::bytesAsValue(&init_recv[13], 2)) };
    }

    RangeAcceptResult anchorRangeAccept(NextActivity next, uint16_t value)
    {
        RangeAcceptResult returnValue;

        double range;
        if(!DW1000JangRTLS::receiveFrame()) {
            returnValue = {false, 0};
        } else {

            size_t poll_len = DW1000Jang::getReceivedDataLength();
            byte poll_data[poll_len];
            DW1000Jang::getReceivedData(poll_data, poll_len);

            if(poll_len > 9 && poll_data[9] == RANGING_TAG_POLL) {
                uint64_t timePollReceived = DW1000Jang::getReceiveTimestamp();
                DW1000JangRTLS::transmitResponseToPoll(&poll_data[7]);
                DW1000JangRTLS::waitForTransmission();
                uint64_t timeResponseToPoll = DW1000Jang::getTransmitTimestamp();
                delayMicroseconds(1500);

                if(!DW1000JangRTLS::receiveFrame()) {
                    returnValue = {false, 0};
                } else {

                    size_t rfinal_len = DW1000Jang::getReceivedDataLength();
                    byte rfinal_data[rfinal_len];
                    DW1000Jang::getReceivedData(rfinal_data, rfinal_len);
                    if(rfinal_len > 18 && rfinal_data[9] == RANGING_TAG_FINAL_RESPONSE_EMBEDDED) {
                        uint64_t timeFinalMessageReceive = DW1000Jang::getReceiveTimestamp();

                        byte finishValue[2];
                        DW1000JangUtils::writeValueToBytes(finishValue, value, 2);

                        if(next == NextActivity::RANGING_CONFIRM) {
                            DW1000JangRTLS::transmitRangingConfirm(&rfinal_data[7], finishValue);
                        } else {
                            DW1000JangRTLS::transmitActivityFinished(&rfinal_data[7], finishValue);
                        }
                        
                        DW1000JangRTLS::waitForTransmission();

                        range = DW1000JangRanging::computeRangeAsymmetric(
                            DW1000JangUtils::bytesAsValue(rfinal_data + 10, LENGTH_TIMESTAMP), // Poll send time
                            timePollReceived, 
                            timeResponseToPoll, // Response to poll sent time
                            DW1000JangUtils::bytesAsValue(rfinal_data + 14, LENGTH_TIMESTAMP), // Response to Poll Received
                            DW1000JangUtils::bytesAsValue(rfinal_data + 18, LENGTH_TIMESTAMP), // Final Message send time
                            timeFinalMessageReceive // Final message receive time
                        );

                        range = DW1000JangRanging::correctRange(range);

                        /* In case of wrong read due to bad device calibration */
                        if(range <= 0) 
                            range = 0.000001;

                        returnValue = {true, range};
                    }
                }
            }
        }

        return returnValue;
    }

    static RangeResult tagFinishRange(uint16_t anchor, uint16_t replyDelayUs) 
    {
        RangeResult returnValue;

        byte target_anchor[2];
        DW1000JangUtils::writeValueToBytes(target_anchor, anchor, 2);
        DW1000JangRTLS::transmitPoll(target_anchor);
        /* Start of poll control for range */
        if(!DW1000JangRTLS::waitForNextRangingStep()) 
        {   returnValue = {false, false, 0, 0}; } 
        else 
        {
            size_t cont_len = DW1000Jang::getReceivedDataLength();
            byte cont_recv[cont_len];
            DW1000Jang::getReceivedData(cont_recv, cont_len);

            if (cont_len > 10 && cont_recv[9] == ACTIVITY_CONTROL && cont_recv[10] == RANGING_CONTINUE) {
                /* Received Response to poll */
                DW1000JangRTLS::transmitFinalMessage(
                    &cont_recv[7], 
                    replyDelayUs, 
                    DW1000Jang::getTransmitTimestamp(), // Poll transmit time
                    DW1000Jang::getReceiveTimestamp()  // Response to poll receive time
                );

                if(!DW1000JangRTLS::waitForNextRangingStep()) 
                {   returnValue = {false, false, 0, 0}; } 
                else 
                {
                    size_t act_len = DW1000Jang::getReceivedDataLength();
                    byte act_recv[act_len];
                    DW1000Jang::getReceivedData(act_recv, act_len);

                    if(act_len > 10 && act_recv[9] == ACTIVITY_CONTROL) {
                        if (act_len > 12 && act_recv[10] == RANGING_CONFIRM) {
                            returnValue = {true, true, static_cast<uint16_t>(DW1000JangUtils::bytesAsValue(&act_recv[11], 2)), 0};
                        } 
                        else if(act_len > 12 && act_recv[10] == ACTIVITY_FINISHED) {
                            returnValue = {true, false, 0, calculateNewBlinkRate(act_recv)};
                        }
                    } 
                    else {
                        returnValue = {false, false, 0, 0};
                    }
                }
            } 
            else 
            {   returnValue = {false, false, 0, 0}; }
        }
        return returnValue;
    }

    RangeInfrastructureResult tagRangeInfrastructure(uint16_t target_anchor, uint16_t finalMessageDelay) 
    {
        RangeInfrastructureResult returnValue;

        RangeResult result = tagFinishRange(target_anchor, finalMessageDelay);
        byte keep_going = 1;

        if(!result.success) 
        {
            keep_going = 0;
            returnValue = {false , 0};
        } 
        else 
        {
            while(result.success && result.next) {
                result = tagFinishRange(result.next_anchor, finalMessageDelay);
                if(!result.success) {
                    keep_going = 0;
                    returnValue = {false , 0};
                    break;
                    }
                }
                #if defined(ESP8266)
                if (keep_going == 1) {
                    yield();
                }
                #endif
        }
        if (keep_going == 1) {

            if(result.success && result.new_blink_rate != 0) 
            {
                keep_going = 0;
                returnValue = { true, static_cast<uint16_t>(result.new_blink_rate)};
            } 
            else 
            {
                if(!result.success) {
                    keep_going = 0;
                    returnValue = { false , 0};
                } 
                else {
                    // TODO. Handle this condition?
                }
            }
        }
        return returnValue;
    }


    RangeInfrastructureResult tagTwrLocalize(uint16_t finalMessageDelay) {
        RangeRequestResult request_result = DW1000JangRTLS::tagRangeRequest();

        if(request_result.success) {
            
            RangeInfrastructureResult result = DW1000JangRTLS::tagRangeInfrastructure(request_result.target_anchor, finalMessageDelay);

            if(result.success)
                return result;
        }
        return {false, 0};
    }


    //------------------------------------------ tuning function ---------------------------------------------------    

    RangeAcceptResult anchorRangeAccept_v2(NextActivity next, uint16_t value)
    {
        RangeAcceptResult returnValue;

        double range;
        if(!DW1000JangRTLS::receiveFrame()) {
            returnValue = {false, 0};
        } else {

            size_t poll_len = DW1000Jang::getReceivedDataLength();
            byte poll_data[poll_len];
            DW1000Jang::getReceivedData(poll_data, poll_len);

            if(poll_len > 9 && poll_data[9] == RANGING_TAG_POLL) {
                uint64_t timePollReceived = DW1000Jang::getReceiveTimestamp();
                DW1000JangRTLS::transmitResponseToPoll(&poll_data[7]);
                DW1000JangRTLS::waitForTransmission();
                uint64_t timeResponseToPoll = DW1000Jang::getTransmitTimestamp();
                delayMicroseconds(1500);

                if(!DW1000JangRTLS::receiveFrame()) {
                    returnValue = {false, 0};
                } else {

                    size_t rfinal_len = DW1000Jang::getReceivedDataLength();
                    byte rfinal_data[rfinal_len];
                    DW1000Jang::getReceivedData(rfinal_data, rfinal_len);
                    if(rfinal_len > 18 && rfinal_data[9] == RANGING_TAG_FINAL_RESPONSE_EMBEDDED) {
                        uint64_t timeFinalMessageReceive = DW1000Jang::getReceiveTimestamp();

                        range = DW1000JangRanging::computeRangeAsymmetric(
                            DW1000JangUtils::bytesAsValue(rfinal_data + 10, LENGTH_TIMESTAMP), // Poll send time
                            timePollReceived, 
                            timeResponseToPoll, // Response to poll sent time
                            DW1000JangUtils::bytesAsValue(rfinal_data + 14, LENGTH_TIMESTAMP), // Response to Poll Received
                            DW1000JangUtils::bytesAsValue(rfinal_data + 18, LENGTH_TIMESTAMP), // Final Message send time
                            timeFinalMessageReceive // Final message receive time
                        );

                        range = DW1000JangRanging::correctRange(range);


                        byte finishValue[2];
                        DW1000JangUtils::writeValueToBytes(finishValue, value, 2);

                        if(next == NextActivity::RANGING_CONFIRM) {
                            DW1000JangRTLS::transmitRangingConfirm_v2(&rfinal_data[7], finishValue, range);
                        } else {
                            DW1000JangRTLS::transmitActivityFinished_v2(&rfinal_data[7], finishValue, range);
                        }
                        
                        DW1000JangRTLS::waitForTransmission();

                        /* In case of wrong read due to bad device calibration */
                        if(range <= 0) 
                            range = 0.000001;

                        returnValue = {true, range};
                    }
                }
            }
        }
        return returnValue;
    }

    static RangeResult_v2 tagFinishRange_v2(uint16_t anchor, uint16_t replyDelayUs) {
        RangeResult_v2 returnValue;

        byte target_anchor[2];
        DW1000JangUtils::writeValueToBytes(target_anchor, anchor, 2);
        DW1000JangRTLS::transmitPoll(target_anchor);
        /* Start of poll control for range */
        if(!DW1000JangRTLS::waitForNextRangingStep()) {
            returnValue = {false, false, 0, 0, 0};
        } else {

            size_t cont_len = DW1000Jang::getReceivedDataLength();
            byte cont_recv[cont_len];
            DW1000Jang::getReceivedData(cont_recv, cont_len);

            if (cont_len > 10 && cont_recv[9] == ACTIVITY_CONTROL && cont_recv[10] == RANGING_CONTINUE) {
                /* Received Response to poll */
                DW1000JangRTLS::transmitFinalMessage(
                    &cont_recv[7], 
                    replyDelayUs, 
                    DW1000Jang::getTransmitTimestamp(), // Poll transmit time
                    DW1000Jang::getReceiveTimestamp()  // Response to poll receive time
                );

                if(!DW1000JangRTLS::waitForNextRangingStep()) {
                    returnValue = {false, false, 0, 0, 0};
                } else {

                    size_t act_len = DW1000Jang::getReceivedDataLength();
                    byte act_recv[act_len];
                    DW1000Jang::getReceivedData(act_recv, act_len);


                    if(act_len > 10 && act_recv[9] == ACTIVITY_CONTROL) {
                        if (act_len > 12 && act_recv[10] == RANGING_CONFIRM) {
                            returnValue = {true, true, static_cast<uint16_t>(DW1000JangUtils::bytesAsValue(&act_recv[11], 2)), 0, static_cast<double>(DW1000JangUtils::bytesAsValue(&act_recv[13],2) / 1000.0)};
                        } else if(act_len > 12 && act_recv[10] == ACTIVITY_FINISHED) {
                            returnValue = {true, false, 0, calculateNewBlinkRate(act_recv), static_cast<double>(DW1000JangUtils::bytesAsValue(&act_recv[13],2) / 1000.0)};
                        }
                    } else {
                        returnValue = {false, false, 0, 0, 0};
                    }
                }
            } else {
                returnValue = {false, false, 0, 0, 0};
            }
            
        }
        return returnValue;
    }

    RangeInfrastructureResult_v2 tagRangeInfrastructure_v2(uint16_t target_anchor, uint16_t finalMessageDelay)
    {
        RangeInfrastructureResult_v2 returnValue;

        double main_dist;
        double b_dist;
        double c_dist;

        RangeResult_v2 result = tagFinishRange_v2(target_anchor, finalMessageDelay);
        byte keep_going = 1;

        if(result.success)
        {
            main_dist = result.distance;

            // Serial.print("---------------------------1-----------------------------\n");

            // Serial.print("success ? : ");
            // Serial.print(result.success);
            // Serial.println("\n");

            // Serial.print("next ? : ");
            // Serial.print(result.next);
            // Serial.println("\n");

            // Serial.print("next anchor address : ");
            // Serial.print(result.next_anchor);
            // Serial.println("\n");

            // Serial.print("blink_rate : ");
            // Serial.print(result.new_blink_rate);
            // Serial.println("\n");

            // Serial.print("main_dist : ");
            // Serial.print(main_dist);
            // Serial.println("\n");
        }


        if(!result.success) {
            keep_going = 0;
            returnValue = {false , 0,0,0,0};
        } else {
            while(result.success && result.next) {
                result = tagFinishRange_v2(result.next_anchor, finalMessageDelay);
                if(!result.success) {
                    keep_going = 0;
                    returnValue = {false , 0,0,0,0};
                    break;
                }
                else {

                    if(result.next_anchor == 3)            
                    {
                        b_dist = result.distance;

                        // Serial.print("---------------------------2-----------------------------\n");

                        // Serial.print("success ? : ");
                        // Serial.print(result.success);
                        // Serial.println("\n");

                        // Serial.print("next ? : ");
                        // Serial.print(result.next);
                        // Serial.println("\n");

                        // Serial.print("next anchor address : ");
                        // Serial.print(result.next_anchor);
                        // Serial.println("\n");

                        // Serial.print("blink_rate : ");
                        // Serial.print(result.new_blink_rate);
                        // Serial.println("\n");

                        // Serial.print("b_dist : ");
                        // Serial.print(b_dist);
                        // Serial.println("\n");
                    }

                    else
                    {
                        
                        c_dist = result.distance;

                        // Serial.print("---------------------------3-----------------------------\n");

                        // Serial.print("success ? : ");
                        // Serial.print(result.success);
                        // Serial.println("\n");

                        // Serial.print("next ? : ");
                        // Serial.print(result.next);
                        // Serial.println("\n");

                        // Serial.print("next anchor address : ");
                        // Serial.print(result.next_anchor);
                        // Serial.println("\n");

                        // Serial.print("blink_rate : ");
                        // Serial.print(result.new_blink_rate);
                        // Serial.println("\n");

                        // Serial.print("c_dist : ");
                        // Serial.print(c_dist);
                        // Serial.println("\n");
                    
                    }

                }

                #if defined(ESP8266)
                if (keep_going == 1) {
                    yield();
                }
                #endif
            }

            if (keep_going == 1) {

                if(result.success && result.new_blink_rate != 0) {
                    keep_going = 0;
                    returnValue = { true, static_cast<uint16_t>(result.new_blink_rate), main_dist, b_dist, c_dist };
                } else {
                    if(!result.success) {
                        keep_going = 0;
                        returnValue = { false , 0 ,0,0,0};
                    } else {
                        // TODO. Handle this condition?
                    }
                }
            }
        }
        return returnValue;
    }


    RangeInfrastructureResult_v2 tagTwrLocalize_v2(uint16_t finalMessageDelay) {
        RangeRequestResult request_result = DW1000JangRTLS::tagRangeRequest();

        if(request_result.success) {
            
            RangeInfrastructureResult_v2 result = DW1000JangRTLS::tagRangeInfrastructure_v2(request_result.target_anchor, finalMessageDelay);

            if(result.success)
                return result;
        }
        return {false, 0,0,0,0};
    }


// -----------------------------------------------NEW VERSION-----------------------------------------------

// typdef struct New_structure {
//     boolean success;
//     double distance;
// }New_structure;

    New_structure Tag_Distance_Request(uint16_t anchor_address, uint16_t finalMessageDelay)
    {
        New_structure returnValue;

        byte target_anchor[2];
        DW1000JangUtils::writeValueToBytes(target_anchor, anchor_address, 2);
        DW1000JangRTLS::transmitPoll(target_anchor);
        /* Start of poll control for range */
        if(!DW1000JangRTLS::waitForNextRangingStep()) {

            // Serial.println("response fail");
            returnValue = {false, 0};
        } 

        else 
        {
            // Serial.println("response success");
            size_t cont_len = DW1000Jang::getReceivedDataLength();
            byte cont_recv[cont_len];
            DW1000Jang::getReceivedData(cont_recv, cont_len);

            if (cont_len > 10 && cont_recv[9] == ACTIVITY_CONTROL && cont_recv[10] == RANGING_CONTINUE) 
            {
                /* Received Response to poll */
                DW1000JangRTLS::transmitFinalMessage(
                    &cont_recv[7], 
                    finalMessageDelay, 
                    DW1000Jang::getTransmitTimestamp(), // Poll transmit time
                    DW1000Jang::getReceiveTimestamp()  // Response to poll receive time
                );
                DW1000JangRTLS::waitForTransmission();
                // Serial.println("Final msg trans");
                

                if(!DW1000JangRTLS::receiveFrame()) {
                    // Serial.println("RANGE_CONFIRM_receive_fail");
                    returnValue = {false, 0};
                }
                else 
                {
                    
                    size_t act_len = DW1000Jang::getReceivedDataLength();
                    char act_recv[act_len];
                    DW1000Jang::getReceivedData(act_recv, act_len);

                    // Serial.println(act_len);
                    

                    if(act_len > 10 && act_recv[9] == ACTIVITY_CONTROL) 
                    {
                        // Serial.println("RANGE_CONFIRM_receive_ok1");
                        // Serial.println(act_recv[10]);
                        if (act_len > 12 && act_recv[10] == RANGING_CONFIRM) 
                        {
                            // Serial.println("RANGE_CONFIRM_receive_ok2");
                            double tmp = static_cast<double>(DW1000JangUtils::bytesAsValue(&act_recv[13],2) / 1000.0);
                            if(tmp > 65)
                                tmp = 0;
                            
                            returnValue = {true, tmp};
                        } 
                        
                    } 
                    else 
                    {
                        // Serial.println("RANGE_CONFIRM_receive_nono");
                        returnValue = {false, 0};
                    }
                }
            } 
            else 
            {
                returnValue = {false, 0};
            } 
        }
    
        return returnValue;
    }



    RangeAcceptResult Anchor_Distance_Response()
    {
        RangeAcceptResult returnValue;

        double range;
        if(!DW1000JangRTLS::receiveFrame()) {
            Serial.println("poll_receive_fail");
            returnValue = {false, 0};
        }
        else 
        {

            size_t poll_len = DW1000Jang::getReceivedDataLength();
            byte poll_data[poll_len];
            DW1000Jang::getReceivedData(poll_data, poll_len);

            if(poll_len > 9 && poll_data[9] == RANGING_TAG_POLL) 
            {
                uint64_t timePollReceived = DW1000Jang::getReceiveTimestamp();
                DW1000JangRTLS::transmitResponseToPoll(&poll_data[7]);
                DW1000JangRTLS::waitForTransmission();
                uint64_t timeResponseToPoll = DW1000Jang::getTransmitTimestamp();
                delayMicroseconds(1500);

                if(!DW1000JangRTLS::receiveFrame()) {
                    Serial.println("final_receive_fail");
                    returnValue = {false, 0};
                }
                else 
                {

                    size_t rfinal_len = DW1000Jang::getReceivedDataLength();
                    byte rfinal_data[rfinal_len];
                    DW1000Jang::getReceivedData(rfinal_data, rfinal_len);
                    if(rfinal_len > 18 && rfinal_data[9] == RANGING_TAG_FINAL_RESPONSE_EMBEDDED) {
                        uint64_t timeFinalMessageReceive = DW1000Jang::getReceiveTimestamp();

                        range = DW1000JangRanging::computeRangeAsymmetric(
                            DW1000JangUtils::bytesAsValue(rfinal_data + 10, LENGTH_TIMESTAMP), // Poll send time
                            timePollReceived, 
                            timeResponseToPoll, // Response to poll sent time
                            DW1000JangUtils::bytesAsValue(rfinal_data + 14, LENGTH_TIMESTAMP), // Response to Poll Received
                            DW1000JangUtils::bytesAsValue(rfinal_data + 18, LENGTH_TIMESTAMP), // Final Message send time
                            timeFinalMessageReceive // Final message receive time
                        );

                        range = DW1000JangRanging::correctRange(range);


                        byte target_anchor[] = {0x00, 0x00};
                        DW1000JangRTLS::transmitRangingConfirm_v2(&rfinal_data[7], target_anchor, range);
                        DW1000JangRTLS::waitForTransmission();
                        Serial.println("range_confirm_sent");

                        /* In case of wrong read due to bad device calibration */
                        if(range <= 0) 
                            range = 0.000001;

                        returnValue = {true, range};
                    }
                }
            }
        }
        return returnValue;
    }
}