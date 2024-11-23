#include <DW1000Jang.hpp>
#include <DW1000JangUtils.hpp>
#include <DW1000JangTime.hpp>
#include <DW1000JangConstants.hpp>

// 연결 핀
const uint8_t PIN_RST = 7; // 리셋 핀
const uint8_t PIN_IRQ = 2; // 인터럽트 요청 핀
const uint8_t PIN_SS = 10; // SPI 선택 핀


// 데이터 버퍼
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

    // 드라이버 초기화
    DW1000Jang::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
    DW1000Jang::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

    // 네트워크 ID 설정
    DW1000Jang::setNetworkId(10);
    DW1000Jang::setDeviceAddress(5); // Set your device address here
    DW1000Jang::setAntennaDelay(16436);

    // 송수신 핸들러 설정
    DW1000Jang::attachReceivedHandler(receive);
    DW1000Jang::attachReceiveFailedHandler(receive_fail);
    Serial.println(F("Chat program started. Type messages and send."));
    DW1000Jang::startReceive();
}

void loop() 
{
  // DW1000Jang::startReceive();
  // while(!DW1000Jang::isReceiveDone()) {
  //   //Serial.println("1");
  //   #if defined(ESP8266)
  //   yield();
  //   #endif
  // }
}


void receive() {
  // DW1000Jang::startReceive();
  DW1000Jang::getReceivedData(data, LEN_DATA);
  
  
  int len = data[1];

  if(data[0] == 0x01) {
    Serial.print("Receive: ");
    for (int i = 11; i < len ; i++) {
      Serial.print(char(data[i])); // 16진수 형태로 출력
    }
    Serial.println(); // 줄 바꿈
  }
  DW1000Jang::clearReceiveStatus();
  DW1000Jang::startReceive();
}

void receive_fail()
{
  Serial.println("receive fail!");
  DW1000Jang::forceTRxOff();
}