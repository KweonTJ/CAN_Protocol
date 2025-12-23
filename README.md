# 2025 자율주행 CAN 통신 실무 프로젝트 경진대회
## 주제 : CAN 통신을 이용해 차량 통신 시스템을 구축 및 자율주행 시스템 제작
### 내용
- CAN 통신의 이점과 구조를 파악하고, 용도에 맞게 차량 제작
- Arbitration 통해 각 차량 회사들의 우선순위를 파악하여 새로운 CAN Database를 구축한 뒤 Message를 송신한다
- 각종 센서를 이용해 자율주행이 가능하도록 CAN 통신으로 구동하게 하여 Mission을 수행한다
### Database
- 모터 컨트롤 (Motor_Cmd) Master -> Slave
	- Left_Motor_Pwm
	- Right_Motor_Pwm
	- Left_Motor_Dir
	- Right_Motor_Dir
	- Motor_Enable
	- EStop
- 모터 상태 (Motor_States) Slave -> Master
	- Slave_Alive_Counter
	- Timeout_Stop
	- Driver_Fault
- 마스터 상태 (Master_State) Master -> Slave
	- Mode
	- Avoid_Dir
	- UltraSonic_Dist
	- Servo_Angle
	- Line_State
- 장애물 회피 (Dynamnic_Planning) Master DB
	- Ultra_Sonic
	- Servo_Angle
	- Stop_Signal
	- Back_Signal
- 휠 상태 (Wheel_State)
	- Wheel_Front
	- Wheel_Back
	- Wheel_Left
	- Wheel_Right
- Line_tracing
	- Front_Signal
	- Left_Signal
	- Right_Signal


# 각 회사별 DBC 정리

| 회사            | Byte Order       | Message Name      | Message ID |
| ------------- | ---------------- | ----------------- | ---------- |
| Cadillac      | Motorola         | ECMAcceleratorPos | 0xBE       |
| BMW           | Intel & Motorola | SYNC              | 0x80       |
| Hyundai       | Intel            | EMS_DCT1          | 0x80       |
| Mercedes Benz | Motorola         | STEER_SENSOR      | 0x3        |
| Tesla         | Intel            | DI_torque1        | 0x106      |
| Toyota        | Motorola         | TRACK_A_0         | 0x210      |
| Volvo         | Motorola         | SAS0              | 0x10       |
| VW            | Intel            | Airbag_01         | 0x40       |
