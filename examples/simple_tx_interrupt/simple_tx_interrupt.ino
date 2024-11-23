#include <DW1000Jang.hpp>
#include <DW1000JangUtils.hpp>
#include <DW1000JangTime.hpp>
#include <DW1000JangConstants.hpp>

// 연결 핀
const uint8_t PIN_RST = 7; // 리셋 핀
const uint8_t PIN_IRQ = 2; // 인터럽트 요청 핀
const uint8_t PIN_SS = 10; // SPI 선택 핀

byte send_start = true;
byte recv_start = false;

byte DATA = 0x01;
byte SHORT_SRC_AND_DEST = 0x88;
byte SEQ_NUMBER = 1;
byte receiver_address[] = {0x06, 0x00};
// 데이터 버퍼
String input;
#define LEN_DATA 128
byte data[LEN_DATA];
#define MAX_MESSAGE_LENGTH 128
byte transmit_data[MAX_MESSAGE_LENGTH + 11]; // 헤더 길이 11을 더한 최대 길이

// 기본 디바이스 구성
device_configuration_t DEFAULT_CONFIG = {
    false, true, true, true, false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_64MHZ,
    PreambleLength::LEN_2048,
    PreambleCode::CODE_10
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
    false, true, true, false, true
};



void setup() {
    Serial.begin(115200);
    Serial.println(F("### UWB Chat Program ###"));
    // input = 'test\n';
    
    // int headerLength = 11; // 헤더 길이
    // int totalLength = headerLength + 5; // 총 데이터 길이
    // 드라이버 초기화
    DW1000Jang::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
    DW1000Jang::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

    // 네트워크 ID 설정
    DW1000Jang::setNetworkId(10);
    DW1000Jang::setDeviceAddress(5); // Set your device address here
    DW1000Jang::setAntennaDelay(16436);

    // 송수신 핸들러 설정
    Serial.println(F("Chat program started. Type messages and send."));
    // transmit_data[0] = DATA; // DATA
    // transmit_data[1] = (byte)totalLength; // SHORT_SRC_AND_DEST
    // transmit_data[2] = SEQ_NUMBER++;
    
    // for (int i = 3; i < headerLength; i++) {
    //     transmit_data[i] = 0; // 나머지 헤더는 0으로 채움
    // }
    // DW1000Jang::getNetworkId(&transmit_data[3]);
    // memcpy(&transmit_data[5], receiver_address, 2);
    // DW1000Jang::getNetworkId(&transmit_data[7]);
    // DW1000Jang::getDeviceAddress(&transmit_data[9]);

    // // 사용자 입력을 byte 배열에 복사
    // input.getBytes(&transmit_data[headerLength], input.length() + 1); // +1 to include null terminator
}

void loop() 
{
  if (Serial.available() > 0) 
  {
    input = Serial.readStringUntil('\n');
    input.trim(); // 문자열 끝의 공백 또는 개행 문자 제거

    if (input.length() > 0 && input.length() <= MAX_MESSAGE_LENGTH) {
        int headerLength = 11; // 헤더 길이
        int totalLength = headerLength + input.length(); // 총 데이터 길이

        // 헤더 데이터 설정
        transmit_data[0] = DATA; // DATA
        transmit_data[1] = (byte)totalLength; // SHORT_SRC_AND_DEST
        transmit_data[2] = SEQ_NUMBER++;
        
        for (int i = 3; i < headerLength; i++) {
            transmit_data[i] = 0; // 나머지 헤더는 0으로 채움
        }
        DW1000Jang::getNetworkId(&transmit_data[3]);
        memcpy(&transmit_data[5], receiver_address, 2);
        DW1000Jang::getNetworkId(&transmit_data[7]);
        DW1000Jang::getDeviceAddress(&transmit_data[9]);

        // 사용자 입력을 byte 배열에 복사
        input.getBytes(&transmit_data[headerLength], input.length() + 1); // +1 to include null terminator

        // 데이터 전송
        DW1000Jang::setTransmitData(transmit_data, totalLength);
        DW1000Jang::startTransmit(TransmitMode::IMMEDIATE);
        while(!DW1000Jang::isTransmitDone()) {
          #if defined(ESP8266)
          yield();
          #endif
        }
      
        DW1000Jang::clearTransmitStatus();

        // 전송된 메시지 출력
        Serial.print("Sent: ");
        for (int i = headerLength; i < totalLength; i++) {
            Serial.print(char(transmit_data[i])); // 16진수 형태로 출력
        }
        Serial.println(); // 줄 바꿈
    }
  }
  // transmit_data[0] = DATA;
  // DW1000Jang::setTransmitData(transmit_data, 16);
  // DW1000Jang::startTransmit(TransmitMode::IMMEDIATE);kjahsdf
  // while(!DW1000Jang::isTransmitDone()) {
  //   #if defined(ESP8266)
  //   yield();
  //   #endif
    
  // }

  // DW1000Jang::clearTransmitStatus();
  // delay(100);

}