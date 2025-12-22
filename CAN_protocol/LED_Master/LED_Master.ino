/*
 * 예제 108-2-1 (CAN통신 마스터코드)
 * SPI통신라인에 MCP2515를 연결하고 특별히 CS핀은 10으로 하자
 * 슬레이브1과 슬레이브2에 각각 메시지를 전송하되
 * 8바이트중에 첫번째 바이트가 0이면 LED가 꺼지고 1이면 켜지도록 해보자
 */

#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg1;
struct can_frame canMsg2;
MCP2515 mcp2515(10);


void setup() {
  canMsg1.can_id  = 0x0F6; //슬레이브1의 ID
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = 0x00;
  canMsg1.data[3] = 0x00;
  canMsg1.data[4] = 0x00;
  canMsg1.data[5] = 0x00;
  canMsg1.data[6] = 0x00;
  canMsg1.data[7] = 0x00;

  canMsg2.can_id  = 0x036; //슬레이브2의 ID
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0x00;
  canMsg2.data[1] = 0x00;
  canMsg2.data[2] = 0x00;
  canMsg2.data[3] = 0x00;
  canMsg2.data[4] = 0x00;
  canMsg2.data[5] = 0x00;
  canMsg2.data[6] = 0x00;
  canMsg2.data[7] = 0x00;
  
  while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("Example: Write to CAN");
}

void loop() {
  canMsg1.data[0] = 0x01;
  canMsg2.data[0] = 0x00;
  mcp2515.sendMessage(&canMsg1);
  mcp2515.sendMessage(&canMsg2);
  Serial.println("LED ON Messages sent");
  delay(1000);
  canMsg1.data[0] = 0x00;
  canMsg2.data[0] = 0x01;
  mcp2515.sendMessage(&canMsg1);
  mcp2515.sendMessage(&canMsg2);
  Serial.println("LED OFF Messages sent");
  delay(1000);
}