/*
 * 예제 108-2-2 (CAN통신 슬레이브1코드)
 * SPI통신라인에 MCP2515를 연결하고 특별히 CS핀은 10으로 하자
 * 슬레이브1과 슬레이브2에 각각 메시지를 전송하되
 * 8바이트중에 첫번째 바이트가 0이면 LED가 꺼지고 1이면 켜지도록 해보자
 * 슬레이브1은 빨간색 LED를 D3에 연결했다
 */

#include <SPI.h>
#include <mcp2515.h>
#define slaveID 0x0F6
#define myLED 3
struct can_frame canMsg;
MCP2515 mcp2515(10);


void setup() {
  Serial.begin(115200);
  pinMode(myLED,OUTPUT);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(slaveID != canMsg.can_id) return;

    if(canMsg.data[0] == 0x00){
      //LED OFF
      digitalWrite(myLED,LOW);
    }else if(canMsg.data[0] == 0x01){
      //LED ON
      digitalWrite(myLED,HIGH);
    }
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