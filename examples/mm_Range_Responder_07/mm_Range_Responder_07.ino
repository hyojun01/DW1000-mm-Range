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
    // initialize the driver
    DW1000Jang::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    // general configuration
    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
    DW1000Jang::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);

    DW1000Jang::setPreambleDetectionTimeout(64);
    DW1000Jang::setSfdDetectionTimeout(273);
    DW1000Jang::setReceiveFrameWaitTimeoutPeriod(8000);

    DW1000Jang::setDeviceAddress(7);
    DW1000Jang::setNetworkId(10);
   
    DW1000Jang::setAntennaDelay(16436);
}

const uint16_t RECEIVE_MODE_DELAY = 1550;
const uint16_t POLL_RESP_DELAY = 1700;

const double LIGHT_VELOCITY = 3 * 10e8;
const double FREQ_CH3 = 2 * (4492.8 * 10e6);
const double LAMBDA = LIGHT_VELOCITY / FREQ_CH3;

void loop() {

    double dist_twr;
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
            double phasePoll = DW1000Jang::getReceivedPhase(); //Poll Message를 받고 위상을 얻는 작업

            DW1000JangRTLS::transmitResponseToPoll_v3(&poll_data[7], POLL_RESP_DELAY); //기본 코드와 동일한 함수 사용, Response Message를 보내는 시점을 Poll Message 수신 시점을 기준으로 함.
            DW1000JangRTLS::waitForTransmission();
            uint64_t timeResponseToPoll = DW1000Jang::getTransmitTimestamp();

            if(!DW1000JangRTLS::receiveFrame_v2(RECEIVE_MODE_DELAY)) {
                return;
            }
            else 
            {
                size_t rfinal_len = DW1000Jang::getReceivedDataLength();
                byte rfinal_data[rfinal_len];
                DW1000Jang::getReceivedData(rfinal_data, rfinal_len);

                if(rfinal_len > 22 && rfinal_data[9] == RANGING_TAG_FINAL_RESPONSE_EMBEDDED) {
                    uint64_t timeFinalMessageReceive = DW1000Jang::getReceiveTimestamp();
                    double phaseFinal = DW1000Jang::getReceivedPhase(); //Final Message를 받고 위상을 얻는 작업
                    // Final Message를 받은 후 packet에 존재하는 정보를 먼저 변수에 저장
                    double phaseResponse = static_cast<double>(DW1000JangUtils::bytesAsValue(&rfinal_data[22], 4) / 1000.0);
                    uint64_t timePollSent = DW1000JangUtils::bytesAsValue(&rfinal_data[10], 4);
                    uint64_t timeResponseToPollReceived = DW1000JangUtils::bytesAsValue(&rfinal_data[14], 4);
                    uint64_t timeFinalMessageSent = DW1000JangUtils::bytesAsValue(&rfinal_data[18], 4);

                    // PostFinal Message를 받는 부분 추가
                    if (!DW1000JangRTLS::receiveFrame_v3(RECEIVE_MODE_DELAY)) {
                      return;
                    }
                    else
                    {
                      size_t rpostfinal_len = DW1000Jang::getReceivedDataLength();
                      byte rpostfinal_data[rpostfinal_len];
                      DW1000Jang::getReceivedData(rpostfinal_data, rpostfinal_len);

                      if (rpostfinal_len > 9 && rpostfinal_data[9] == RANGING_TAG_POST_FINAL_RESPONSE_EMBEDDED) {
                        double phasePostFinal = DW1000Jang::getReceivedPhase(); //PostFinal Message를 받고 위상을 얻는 작업

                        dist_twr = DW1000JangRanging::computeRangeAsymmetric(
                            timePollSent, // Poll send time
                            timePollReceived, 
                            timeResponseToPoll, // Response to poll sent time
                            timeResponseToPollReceived, // Response to Poll Received
                            timeFinalMessageSent, // Final Message send time
                            timeFinalMessageReceive // Final message receive time
                        ); // timestamp를 저장한 변수들을 가지고 거리를 계산하는 작업

                        if (dist_twr <= 0) {
                            dist_twr = 0.000001;
                        }

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
                        
                        // int N = floor(dist_twr / LAMBDA);
                        // double dist = ((recPhase / (2 * PI)) + N) * LAMBDA;
										
                        // Serial.println(dist);

                        Serial.print(recPhase);
                        Serial.print("|");
                        Serial.println(dist_twr);
                      }
                    }
                }
            }
        }
    }
}
    

