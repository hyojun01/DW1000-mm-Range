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

#pragma once

#include <Arduino.h>

/* Frame control */
constexpr byte BLINK = 0xC5;
constexpr byte DATA = 0x41;
constexpr byte SHORT_SRC_AND_DEST = 0x88;
constexpr byte LONG_SRC_AND_DEST = 0xCC;
constexpr byte SHORT_SRC_LONG_DEST = 0x8C;
constexpr byte LONG_SRC_SHORT_DEST = 0xC8;

/* Application ID */
constexpr byte RTLS_APP_ID_LOW = 0x9A;
constexpr byte RTLS_APP_ID_HIGH = 0x60;
constexpr uint16_t RTLS_APP_ID = RTLS_APP_ID_LOW | ((uint16_t) (RTLS_APP_ID_HIGH << 8));

/* Function code */
constexpr byte ACTIVITY_CONTROL = 0x10;
constexpr byte RANGING_INITIATION = 0x20;
constexpr byte RANGING_TAG_POLL = 0x21;
constexpr byte RANGING_TAG_FINAL_RESPONSE_EMBEDDED = 0x23;
constexpr byte RANGING_TAG_FINAL_RESPONSE_NO_EMBEDDED = 0x25;
constexpr byte RANGING_TAG_FINAL_SEND_TIME = 0x27;
constexpr byte RANGING_TAG_POST_FINAL_RESPONSE_EMBEDDED = 0x29;

/* Activity code */
constexpr byte ACTIVITY_FINISHED = 0x00;
constexpr byte RANGING_CONFIRM = 0x01;
constexpr byte RANGING_CONTINUE = 0x02;

/* BLINK Encoding Header */
constexpr byte BATTERY_GOOD = 0x00;
constexpr byte BATTERY_10_30_PERCENT = 0x02;
constexpr byte BATTERY_0_10_PERCENT = 0x01;
constexpr byte NO_BATTERY_STATUS = 0x03;

constexpr byte TEMPERATURE_DATA = 0x20;

constexpr byte EX_ID = 0x80;
constexpr byte NO_EX_ID = 0x40;

/* BLINK Ext Header */
constexpr byte BLINK_RATE_AND_LISTENING = 0x01;
constexpr byte TAG_LISTENING_NOW = 0x02;

enum class NextActivity {
    ACTIVITY_FINISHED,
    RANGING_CONFIRM
};

typedef struct New_structure {
    boolean success;
    double distance;
} New_structure;

typedef struct RangeRequestResult {
    boolean success;
    uint16_t target_anchor;
} RangeRequestResult;

typedef struct RangeResult {
    boolean success;
    boolean next;
    uint16_t next_anchor;
    uint32_t new_blink_rate;
} RangeResult;

typedef struct RangeResult_v2 {
    boolean success;
    boolean next;
    uint16_t next_anchor;
    uint32_t new_blink_rate;
    double distance;
} RangeResult_v2;

typedef struct RangeInfrastructureResult {
    boolean success;
    uint16_t new_blink_rate;
} RangeInfrastructureResult;

typedef struct RangeInfrastructureResult_v2 {
    boolean success;
    uint16_t new_blink_rate;
    double main_dist;
    double b_dist;
    double c_dist;
} RangeInfrastructureResult_v2;

typedef struct RangeAcceptResult {
    boolean success;
    double range;
} RangeAcceptResult;

namespace DW1000JangRTLS {
    /*** TWR functions used in ISO/IEC 24730-62:2013, refer to the standard or the decawave manual for details about TWR ***/
    byte increaseSequenceNumber();
    void transmitTwrShortBlink();
    void transmitRangingInitiation(byte tag_eui[], byte tag_short_address[]);
    void transmitPoll(byte anchor_address[]);
    void transmitResponseToPoll(byte tag_short_address[]);
    uint64_t transmitResponseToPoll_v2(byte anchor_address[], uint16_t reply_delay);
    void transmitResponseToPoll_v3(byte anchor_address[], uint16_t reply_delay);
    void transmitFinalMessage(byte anchor_address[], uint16_t reply_delay, uint64_t timePollSent, uint64_t timeResponseToPollReceived);
    void transmitFinalMessage_v2(byte anchor_address[], uint16_t reply_delay, uint64_t timePollSent, uint64_t timeResponseToPollReceived, double phaseResponse);
    void transmitFinalMessage_v3(byte anchor_address[], uint16_t reply_delay, uint64_t timePollSent, uint64_t timeResponseToPollReceived, double phaseResponse);
    void transmitPostFinalMessage(byte anchor_address[], uint16_t reply_delay);
    void transmitPostFinalMessage_v2(byte anchor_address[], uint16_t reply_delay);
    void transmitRangingConfirm(byte tag_short_address[], byte next_anchor[]);
    void transmitActivityFinished(byte tag_short_address[], byte blink_rate[]);

    void transmitRangingConfirm_v1(byte tag_short_address[], double distance);
    void transmitRangingConfirm_v2(byte tag_short_address[], byte next_anchor[], double distance);
    void transmitRangingConfirm_v3(byte tag_short_address[], double distance);
    void transmitActivityFinished_v2(byte tag_short_address[], byte blink_rate[], double distance);

    boolean receiveFrame();
    boolean receiveFrame_v2(uint64_t timeDelay);
    boolean receiveFrame_v3(uint64_t timeDelay);
    void waitForTransmission();
    boolean waitForNextRangingStep();
    boolean waitForNextRangingStep_v2(uint64_t timeDelay);
    /*** End of TWR functions ***/
    
    /* Send a request range from tag to the rtls infrastructure */
    RangeRequestResult tagRangeRequest();

    /* Used by an anchor to accept an incoming tagRangeRequest by means of the infrastructure
       NextActivity is used to indicate the tag what to do next after the ranging process (Activity finished is to return to blink (range request), 
        Continue range is to tell the tag to range a new anchor)
       value is the value relative to the next activity (Activity finished = new blink rante, continue range = new anchor address)
    */
    RangeAcceptResult anchorRangeAccept(NextActivity next, uint16_t value);

    RangeAcceptResult anchorRangeAccept_v2(NextActivity next, uint16_t value);

    /* Used by tag to range after range request accept of the infrastructure 
       Target anchor is given after a range request success
       Finalmessagedelay is used in the process of TWR, a value of 1500 works on 8mhz-80mhz range devices,
        you could try to decrease it to improve system performance.
    */
    RangeInfrastructureResult tagRangeInfrastructure(uint16_t target_anchor, uint16_t finalMessageDelay);

    RangeInfrastructureResult_v2 tagRangeInfrastructure_v2(uint16_t target_anchor, uint16_t finalMessageDelay);

    /* Can be used as a single function start the localization process from the tag.
        Finalmessagedelay is the same as in function tagRangeInfrastructure
    */
    RangeInfrastructureResult tagTwrLocalize(uint16_t finalMessageDelay);

    RangeInfrastructureResult_v2 tagTwrLocalize_v2(uint16_t finalMessageDelay);


//---------------------------- new function -------------------------------------------
    RangeAcceptResult Anchor_Distance_Response();

    New_structure Tag_Distance_Request(uint16_t target_anchor, uint16_t finalMessageDelay);
}