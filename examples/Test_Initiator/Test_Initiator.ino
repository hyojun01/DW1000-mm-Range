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
}

byte target_anchor[2] = {0x05, 0x00};
const uint16_t RECEIVE_MODE_DELAY = 1550;
const uint16_t RESP_FINAL_DELAY = 1500;
const uint16_t FINAL_POST_DELAY = 1700;

void loop() {

    DW1000JangRTLS::transmitPoll(target_anchor);
    if(!DW1000JangRTLS::waitForNextRangingStep_v2(RECEIVE_MODE_DELAY)) {
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
            double phaseResponse = DW1000Jang::getReceivedPhase(); //Response Message를 받고 위상을 얻는 작업

            DW1000JangRTLS::transmitFinalMessage_v3(
                &cont_recv[7], 
                RESP_FINAL_DELAY, 
                timePollTransmitted, // Poll transmit time
                timeResponseReceived,  // Response to poll receive time
                phaseResponse //위상 정보를 packet에 담기
            ); // Final Message를 보내는 시점을 현재 timestamp 시점을 기준으로 함.
            DW1000JangRTLS::waitForTransmission();

            DW1000JangRTLS::transmitPostFinalMessage(
                &cont_recv[7],
                FINAL_POST_DELAY
            ); //PostFinal Message를 보내는 부분, PostFinal Message를 보내는 시점을 Final Message 송신 시점을 기준으로 함.
            DW1000JangRTLS::waitForTransmission();

            return;
        } 
        else 
        {
            return;
        } 
    }
}