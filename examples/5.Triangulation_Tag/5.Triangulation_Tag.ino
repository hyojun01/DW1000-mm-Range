/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSTag_TWR.ino
 * 
 * This is an example tag in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

#include <DW1000Jang.hpp>
#include <DW1000JangUtils.hpp>
#include <DW1000JangTime.hpp>
#include <DW1000JangConstants.hpp>
#include <DW1000JangRanging.hpp>
#include <DW1000JangRTLS.hpp>

#define Delay 1000
// connection pins
#if defined(ESP8266)
const uint8_t PIN_SS = 15;
#else
const uint8_t PIN_SS = 10; // spi select pin
const uint8_t PIN_RST = 7;
#endif

double range_A;
double range_B;
double range_C;

typedef struct Position {
    double x;
    double y;
} Position;

Position position_A = {0,0};
Position position_B = {1.2,0};
Position position_C = {1.2,1.2};

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
    Serial.begin(115200);
    Serial.println(F("### DW1000Jang-arduino-ranging-tag ###"));
    // initialize the driver
    #if defined(ESP8266)
    DW1000Jang::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Jang::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println("DW1000Jang initialized ...");
    // general configuration
    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
    DW1000Jang::enableFrameFiltering(TAG_FRAME_FILTER_CONFIG);

    DW1000Jang::setDeviceAddress(4); // 1조 Device Address : 4
                                     // 2조 Device Address : 5
                                     // ...
                                     // 10조 Device Address : 13
    DW1000Jang::setNetworkId(RTLS_APP_ID);

    DW1000Jang::setAntennaDelay(16436);


    DW1000Jang::setPreambleDetectionTimeout(64);
    DW1000Jang::setSfdDetectionTimeout(273);
    DW1000Jang::setReceiveFrameWaitTimeoutPeriod(5000);
    
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

void calculatePosition(double &x, double &y) {

    /* This gives for granted that the z plane is the same for anchor and tags */
    double A = ( (-2*position_A.x) + (2*position_B.x) );
    double B = ( (-2*position_A.y) + (2*position_B.y) );
    double C = (range_A*range_A) - (range_B*range_B) - (position_A.x*position_A.x) + (position_B.x*position_B.x) - (position_A.y*position_A.y) + (position_B.y*position_B.y);
    double D = ( (-2*position_B.x) + (2*position_C.x) );
    double E = ( (-2*position_B.y) + (2*position_C.y) );
    double F = (range_B*range_B) - (range_C*range_C) - (position_B.x*position_B.x) + (position_C.x*position_C.x) - (position_B.y*position_B.y) + (position_C.y*position_C.y);

    x = (C*E-F*B) / (E*A-B*D);
    y = (C*D-A*F) / (B*D-A*E);
}



void loop() {

  int rand;
  double x,y;
  
  New_structure result_A_Anchor = DW1000JangRTLS::Tag_Distance_Request(1, 1500);
  if(result_A_Anchor.success)
  {

    range_A = result_A_Anchor.distance;
    randomSeed(analogRead(0));
    rand = random(1,10);
    delay(10+10*rand);

    New_structure result_B_Anchor = DW1000JangRTLS::Tag_Distance_Request(2, 1500);
    if(result_B_Anchor.success)
    {

      range_B = result_B_Anchor.distance;
      randomSeed(analogRead(0));
      rand = random(1,10);
      delay(10+10*rand);

      New_structure result_C_Anchor = DW1000JangRTLS::Tag_Distance_Request(3, 1500);
      if(result_C_Anchor.success)
      {

        range_C = result_C_Anchor.distance;
        randomSeed(analogRead(0));
        rand = random(1,10);
        delay(10+10*rand);

        calculatePosition(x,y);

        Serial.println("------------------------------");
        Serial.print("x : ");
        Serial.println(x);

        Serial.print("y : ");
        Serial.println(y);

        Serial.println("------------------------------");
      }
    }
  }
}


      










    // if(result_A_Anchor.success)
    // {
    //   range_A = result_A_Anchor.distance;
    //   New_structure result_B_Anchor = DW1000JangRTLS::Tag_Distance_Request(2, 1500);

    //   if(result_B_Anchor.success)
    //   {
    //     range_B = result_B_Anchor.distance;
    //     New_structure result_C_Anchor = DW1000JangRTLS::Tag_Distance_Request(3, 1500);

    //     if(result_C_Anchor.success)
    //     {
    //         range_C = result_C_Anchor.distance;
    //         calculatePosition(x,y);
    //         Serial.print("x : ");
    //         Serial.println(x);
    //         Serial.print("y : ");
    //         Serial.println(y);
    //         Serial.println();
    //     }
    //     else
    //     {
    //       Serial.println("C fail");
    //     }
    //   }
    //   else
    //   {
    //     Serial.println("B fail");
    //   }
    // }
    // else
    // {
    //   Serial.println("A fail");
    // }
    //Serial.println(result_A_Anchor.success);
    // Serial.print("A dist : ");
    // Serial.println(A_dist);

    // //Serial.println(result_B_Anchor.success);
    // Serial.print("B dist : ");
    // Serial.println(B_dist);

    // //Serial.println(result_C_Anchor.success);
    // Serial.print("C dist : ");
    // Serial.println(C_dist);

    // Serial.println(result.success);
    // Serial.println(reslt.distance);

    // Serial.println(result.success);
    // Serial.println(reslt.distance);

