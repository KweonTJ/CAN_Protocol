/*
 * 예제 108-1-2 (CAN통신 슬레이브1코드)
 * SPI통신라인에 MCP2515를 연결하고 특별히 CS핀은 10으로 하자
 * 기본 예제에서 보내는 2개의 메시지중에
 * 슬레이브1은 메시지1만을 출력하고 슬레이브2는 메시지2만 출력하도록 해보자
 * (필터링을 하라는 소리)
 */

#include <SPI.h>
#include <mcp2515.h>
#define slaveID 0x0F6
struct can_frame canMsg;
MCP2515 mcp2515(10);


void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(slaveID != canMsg.can_id) return;
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();      
  }
}