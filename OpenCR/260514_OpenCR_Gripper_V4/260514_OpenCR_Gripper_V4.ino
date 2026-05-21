#include <Adafruit_NeoPixel.h>
#include <Dynamixel2Arduino.h>
#include <CAN.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Button.h"


#define OPENCR_CAN


#define USER1_LED   22  // SINK (LOW -> TUNR ON)
#define USER2_LED   23  // SINK
#define USER3_LED   24  // SINK
#define USER4_LED   25  // SINK 
#define ARDUINO_LED 13  // Source (HIGH -> TURN ON)
#define CAN_TX_LED USER1_LED
#define CAN_RX_LED USER2_LED
#define GRIP_LED   USER3_LED
#define PUSH_LED   USER4_LED

#define USER_BUTTON 3

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial

#define DXL_CLOSE_POSITION      2040
#define DXL_OPEN_POSITION       1200

#define DXL_PUSH_POSTION        2100
#define DXL_RELEASE_POSITION    3800

#define DXL_GRIP_VELOCITY       100
#define DXL_PUSH_VELOCITY       200

#define ARRIVE_THRESHOLD        10
#define GRIP_THRESHOLD_RAW      80
#define GOAL_CURRENT            300

#define GRIP_MOVE_TIMEOUT_MS    3000
#define PUSH_MOVE_TIMEOUT_MS    5000

#define ERROR_NONE              0x00
#define ERROR_GRIPPER_TIMEOUT   0x01
#define ERROR_PUSHER_TIMEOUT    0x02
#define ERROR_GRIP_DETECT_FAIL  0x03
#define ERROR_RETURN_TIMEOUT    0x04


unsigned long gripperMoveStartMillis = 0;
unsigned long pusherMoveStartMillis = 0;

uint16_t gripperPrevTarget = DXL_OPEN_POSITION;
uint16_t pusherPrevTarget = DXL_RELEASE_POSITION;

bool pending_error = false;
uint8_t last_error_motor_id = 0;

const uint8_t GRIPPER_ID = 1;
const uint8_t PUSHER_ID = 2;
const unsigned long CAN_RX_TIMEOUT_MS = 200;

// 각 모터의 목표 위치를 저장할 변수 (전역 변수로 선언)
uint16_t gripper_Target = 0;
uint16_t pusher_Target = 0;

bool isOpened = true;
bool isPushed = false;

// 이동 중인지 상태를 관리할 플래그
bool isGripperMoving = false;
bool isPusherMoving = false;

bool isGripperReturning = false;
bool isPusherReturning = false;

bool longPressExecuted = false;
bool can_bus_active = false;       // 현재 CAN 버스가 살아있는가?
bool need_send_heartbeat = false;
unsigned long last_rx_millis = 0;   // 마지막 마스터 패킷 수신 시간

const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

Button myBtn(3, true, 100);
#ifdef OPENCR_CAN
can_message_t tx_msg, rx_msg;
#endif
uint32_t can_rx_count = 0, can_tx_count = 0;

enum
{
    CMD_IDLE        = 0x00,
    CMD_STOP        = 0x01,
    CMD_MOVE        = 0x02,
    CMD_ARRIVED     = 0x03,
    CMD_GET_STATE   = 0x10,
    CMD_TORQUE_ON   = 0x11,
    CMD_TORQUE_OFF  = 0x12,
    CMD_ERROR       = 0xEE,
    CMD_HEARTBEAT   = 0xFF
} cmd_state_t;

enum
{
    DISCONNECT   = 0x00,
    PUSHER_ONLT  = 0x01,
    GRIPPER_ONLY = 0x02,
    CONNECTED    = 0x03

} status_t;


typedef union {
    // 1. 내부 구조체: 각 멤버에 이름으로 접근할 때 사용
    struct __attribute__((packed)) {
        uint8_t ledSwitch;   // 1바이트: 전원 상태
        uint8_t red;         // 1바이트: R
        uint8_t green;       // 1바이트: G
        uint8_t blue;        // 1바이트: B
        uint8_t brightness;  // 1바이트: 밝기
    } fields;

    // 2. 바이트 배열: 전체 데이터를 한꺼번에 다루거나 전송할 때 사용
    uint8_t raw[5];
} led_t;


typedef union {
    struct __attribute__((packed)) {
        uint8_t hallSwitch;   // 1byte : 센서 상태 
        uint8_t high_value;   // 1byte : 상위 8bit 
        uint8_t low_value;    // 1byte : 하위 8bit
        uint8_t distance;     // 1byte : 거리 mm
    } fields;

    uint8_t raw[4];
} sensor_t;


// SIMPLE TEST
typedef struct __attribute__((packed)) {
    uint8_t cmd;        // 명령 종류 (예: 0x01: 위치 이동, 0x02: 상태 요청)
    uint8_t id;         // 대상 모터 ID
    union {
        // 위치 이동 명령일 때 쓰는 데이터
        struct {
            uint16_t target_position;
            uint16_t speed;
            uint8_t  reserved[2]; // 8바이트를 맞추기 위한 빈 공간
        } move_cmd;

        // [Motor -> Master] 현재 상태 응답 시 사용
        struct {
            uint16_t gripper_pos;
            uint16_t pusher_pos;
            uint8_t isConnect;
            uint8_t error_code;  // 에러 상태 등 추가 가능
        } status_res;

        uint8_t raw[6]; // 나머지 6바이트 공간
    } data;
} can_packet_t;

can_packet_t system_Status;


int ledPins[] = {USER1_LED, USER2_LED, USER3_LED, USER4_LED, ARDUINO_LED};
const int ledCount = sizeof(ledPins)/ sizeof(ledPins[0]);
bool ledStatus[ledCount] = {0,};
int ledCurrent = 0;

unsigned long prevMillis = 0;
unsigned long pingMillis = 0;
unsigned long ledTxMillis = 0;
unsigned long ledRxMillis = 0;

using namespace ControlTableItem;

bool canInit(uint32_t id, uint8_t CAN_BAUD);

//////////////////////////////////SETUP/////////////////////////////////////

void setup() {
    DEBUG_SERIAL.begin(115200);
    dxl.begin(57600);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    bool isGripOk = dxl.ping(GRIPPER_ID);
    bool isPushOk = dxl.ping(PUSHER_ID);

    system_Status.data.status_res.isConnect =  isGripOk << 1 | isPushOk;

    if(system_Status.data.status_res.isConnect == CONNECTED)
    {
        DEBUG_SERIAL.println("Motor Connected");
    }
    else
    {
        DEBUG_SERIAL.println("Motor Not Connected");
    }

    DEBUG_SERIAL.println("SET PINOUT");

    pinMode(CAN_TX_LED, OUTPUT);
    pinMode(CAN_RX_LED, OUTPUT);
    pinMode(GRIP_LED, OUTPUT);
    pinMode(PUSH_LED, OUTPUT);
    pinMode(ARDUINO_LED, OUTPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);

    delay(10);
    for(int i = 0; i < 5; i++) {
        myBtn.read();
        delay(5);
    }
    longPressExecuted = false;

    DEBUG_SERIAL.println("Initializing CAN...");
#ifdef OPENCR_CAN
    bool can_ok = canInit(0x123, CAN_BAUD_500K);
    if(can_ok) {
        DEBUG_SERIAL.println("CAN Init Success!");
    } else {
        DEBUG_SERIAL.println("CAN Init Fail! but continue to setup...");
    }
#endif

    dxl.torqueOff(GRIPPER_ID);
    // dxl.setOperatingMode(GRIPPER_ID, OP_CURRENT_BASED_POSITION);
    dxl.setOperatingMode(PUSHER_ID, OP_EXTENDED_POSITION);
    dxl.torqueOn(GRIPPER_ID);

    dxl.writeControlTableItem(PROFILE_VELOCITY, GRIPPER_ID, DXL_GRIP_VELOCITY);

    system_Status.data.status_res.gripper_pos = dxl.getPresentPosition(GRIPPER_ID);
    dxl.writeControlTableItem(GOAL_CURRENT, GRIPPER_ID, 100);

    delay(100);

    dxl.torqueOff(PUSHER_ID);
    dxl.setOperatingMode(PUSHER_ID, OP_EXTENDED_POSITION);
    dxl.torqueOn(PUSHER_ID);

    dxl.writeControlTableItem(PROFILE_VELOCITY, PUSHER_ID, DXL_PUSH_VELOCITY);

    system_Status.data.status_res.pusher_pos = dxl.getPresentPosition(PUSHER_ID);
    
    dxl.ledOn(GRIPPER_ID);
    dxl.ledOn(PUSHER_ID);

    

    DEBUG_SERIAL.println("SETUP DONE");
}


//////////////////////////////////LOOP/////////////////////////////////////

void loop() {
    unsigned long currMillis = millis();

    buttonTask();
    checkArrival();
    checkGripStatus();
    // checkMoveTimeout();

    // 캔 데이터 받으면 모터 동작함
#ifdef OPENCR_CAN
    if(canReceive()){
        digitalWrite(CAN_RX_LED, LOW);
        ledRxMillis = currMillis;

        can_bus_active = true; // 통신 부활!
        last_rx_millis = currMillis;
    }

    if (need_send_heartbeat && isCanRecentlyAlive()) {
        need_send_heartbeat = false;
        sendHeartbeatPacket();
    }

#endif

    

    // HeartBeat
    if(currMillis - pingMillis >= 500){
        pingMillis = currMillis;
        // dxl.ping(GRIPPER_ID);
        
        dxlReconnect();   
    }

   // 4. 통신선 탈락 예외 처리 (3초 동안 마스터 패킷이 안 오면 송신을 잠가서 하드웨어 보호)
    if (can_bus_active && !isCanRecentlyAlive()) {
        can_bus_active = false;
        DEBUG_SERIAL.println("CAN Line Disconnected! Blocking TX to prevent lock.");
    }

    if(currMillis - ledTxMillis >=10){
        digitalWrite(CAN_TX_LED, HIGH);  // HIGH -> OFF
    }
    if(currMillis - ledRxMillis >=10){
        digitalWrite(CAN_RX_LED, HIGH);
    }
   
}



//////////////////////////////////FUNCTION/////////////////////////////////////

bool canInit(uint32_t id, uint8_t CAN_BAUD)
{
    // if(CanBus.begin(CAN_BAUD, CAN_STD_FORMAT) == false)
    if(CanBus.begin(CAN_BAUD, CAN_STD_FORMAT) == false)
    {
        DEBUG_SERIAL.println("CAN open fail");
        return false;
    }
    else
    {
        DEBUG_SERIAL.println("CAN open complete");
        CanBus.configFilter(id, 0, CAN_STD_FORMAT); // ID, MASK, FORMAT
        // can_message_t temp_msg;

        // temp_msg.length = sizeof(can_packet_t);
        // temp_msg.format = CAN_STD_FORMAT;
        // for(int i=0; i< temp_msg.length; i++){
        //     temp_msg.data[i] = 0xFF;
        // }
        // canSend(0x1FF, temp_msg);
        return true;
    }

    return false;
}

bool isCanRecentlyAlive()
{
    return (millis() - last_rx_millis) <= CAN_RX_TIMEOUT_MS;
}


bool canReceive()
{
    if (CanBus.availableMessage())
    {
        if(CanBus.readMessage(&rx_msg))
        {   
            can_rx_count++;
            can_packet_t *rxPkt = (can_packet_t*)&rx_msg.data;

            DEBUG_SERIAL.print("[");
            DEBUG_SERIAL.print(can_rx_count);
            DEBUG_SERIAL.print("] ");
            DEBUG_SERIAL.print("ID : ");
            DEBUG_SERIAL.print(rx_msg.id, HEX);
            DEBUG_SERIAL.print(", Length : ");
            DEBUG_SERIAL.print(rx_msg.length);
            DEBUG_SERIAL.print(", Data : ");
            for (int i = 0; i < rx_msg.length; i++)
            {
                DEBUG_SERIAL.print(rx_msg.data[i], HEX);
                DEBUG_SERIAL.print(" ");
            }
            DEBUG_SERIAL.println();

            switch (rxPkt->cmd)
            {
                case CMD_MOVE:  // 0x02
                {
                    // 엔디언 문제 해결: 0x000F가 3840으로 보인다면 아래 주석을 해제하세요.
                    // rxPkt->data.move_cmd.target_position = (__builtin_bswap16(rxPkt->data.move_cmd.target_position));
                    // rxPkt->data.move_cmd.speed = (__builtin_bswap16(rxPkt->data.move_cmd.speed));

                    uint8_t targetId = rxPkt->id;
                    uint16_t targetPos = rxPkt->data.move_cmd.target_position;
                    uint16_t targetSpd = rxPkt->data.move_cmd.speed;

                    if (targetId != GRIPPER_ID && targetId != PUSHER_ID) {
                        DEBUG_SERIAL.println("Invalid motor ID");
                        break;
                    }

                    // 1. 속도 설정 (속도가 실려온 경우에만)
                    if(targetSpd > 0) {
                        dxl.writeControlTableItem(PROFILE_VELOCITY, targetId, targetSpd);
                        DEBUG_SERIAL.print(targetId == GRIPPER_ID ? "GRIPPER" : "PUSHER");
                        DEBUG_SERIAL.print(" Speed change to: ");
                        DEBUG_SERIAL.println(targetSpd);
                    }

                    // 2. 모터 구동 및 도착 감시 설정
                    if(targetId == GRIPPER_ID) {
                        gripperPrevTarget = dxl.getPresentPosition(GRIPPER_ID);
                        gripper_Target = targetPos; // 전역 변수에 목표가 저장
                        gripperMoveStartMillis = millis();
                        isGripperMoving = true;      // 도착 감시 시작

                        dxl.setGoalPosition(GRIPPER_ID, targetPos);
                        
                        DEBUG_SERIAL.print("GRIPPER Moving to: ");
                        DEBUG_SERIAL.println(targetPos);
                    }
                    else if(targetId == PUSHER_ID) {
                        pusherPrevTarget = dxl.getPresentCurrent(PUSHER_ID);
                        pusher_Target = targetPos;  // 전역 변수에 목표가 저장
                        pusherMoveStartMillis = millis();
                        isPusherMoving = true;       // 도착 감시 시작
                        dxl.setGoalPosition(PUSHER_ID, targetPos);
                        
                        DEBUG_SERIAL.print("PUSHER Moving to: ");
                        DEBUG_SERIAL.println(targetPos);
                        digitalWrite(PUSH_LED, LOW);
                    }
                }
                break;

                case CMD_STOP: // 0x01
                    dxl.torqueOff(GRIPPER_ID);
                    dxl.torqueOff(PUSHER_ID);
                    dxl.torqueOn(GRIPPER_ID); // 토크 오프 후 온 하면 제동 상태가 됨
                    dxl.torqueOn(PUSHER_ID);
                    DEBUG_SERIAL.println("Emergency STOP!");
                    break;

                case CMD_TORQUE_ON: // 0x11
                    dxl.torqueOn(rxPkt->id);
                    DEBUG_SERIAL.print("Motor ");
                    DEBUG_SERIAL.print(rxPkt->id);
                    DEBUG_SERIAL.println(" Torque ON");
                    break;

                case CMD_TORQUE_OFF: // 0x12
                    dxl.torqueOff(rxPkt->id);
                    DEBUG_SERIAL.print("Motor ");
                    DEBUG_SERIAL.print(rxPkt->id);
                    DEBUG_SERIAL.println(" Torque OFF");
                    break;

                case CMD_GET_STATE: // 0x10 (즉시 상태 보고 요청)
                    need_send_heartbeat = true;
                    DEBUG_SERIAL.println("State Request Received");
                    break;
                case CMD_HEARTBEAT:
                    DEBUG_SERIAL.println("State/Heartbeat Request Received");
                    break;

                default:
                    DEBUG_SERIAL.print("Unknown CMD: ");
                    DEBUG_SERIAL.println(rxPkt->cmd, HEX);
                    break;
            }
            return true;
        }
    }
    return false;
}

bool canSendSafe(uint32_t id, can_message_t msg)
{
#ifdef OPENCR_CAN
    if (!isCanRecentlyAlive()) {
        can_bus_active = false;
        return false;
    }

    msg.id = id;
    msg.format = CAN_STD_FORMAT;

    CanBus.writeMessage(&msg);

    can_tx_count++;
    return true;
#else
    return false;
#endif
}

void sendHeartbeatPacket()
{
#ifdef OPENCR_CAN
    if (!isCanRecentlyAlive()) {
        can_bus_active = false;
        return;
    }

    can_packet_t txPkt = {0};
    txPkt.cmd = CMD_HEARTBEAT;
    txPkt.id = 0x00;

    system_Status.data.status_res.gripper_pos = dxl.getPresentPosition(GRIPPER_ID);
    system_Status.data.status_res.pusher_pos = dxl.getPresentPosition(PUSHER_ID);

    txPkt.data.status_res.gripper_pos = system_Status.data.status_res.gripper_pos;
    txPkt.data.status_res.pusher_pos = system_Status.data.status_res.pusher_pos;
    txPkt.data.status_res.isConnect = system_Status.data.status_res.isConnect;

    tx_msg.format = CAN_STD_FORMAT;
    tx_msg.length = sizeof(can_packet_t);
    memcpy(tx_msg.data, &txPkt, tx_msg.length);

    if (canSendSafe(0x124, tx_msg)) {
        digitalWrite(CAN_TX_LED, LOW);
        ledTxMillis = millis();
    }
#endif
}


bool dxlReconnect() {
    bool ret = false;
    // 현재 비트 상태 추출 (bit 1: Gripper, bit 0: Pusher)
    bool wasGripConnected = (system_Status.data.status_res.isConnect & 0x02);
    bool wasPushConnected = (system_Status.data.status_res.isConnect & 0x01);

    // --- 1. Gripper 체크 ---
    bool gripperNow = dxl.ping(GRIPPER_ID);
    
    // 끊겨있다가(bit1 == 0) 방금 연결된(true) 경우
    if (gripperNow == true && wasGripConnected == false) {
        dxl.setOperatingMode(GRIPPER_ID, OP_POSITION);
        dxl.torqueOn(GRIPPER_ID);
        dxl.writeControlTableItem(PROFILE_VELOCITY, GRIPPER_ID, DXL_GRIP_VELOCITY);
        dxl.ledOn(GRIPPER_ID);
        
        // bit 1을 1로 세팅 (OR 연산)
        system_Status.data.status_res.isConnect |= 0x02; 
        DEBUG_SERIAL.println("GRIPPER Reconnected");
        ret = true;
    } 
    // 연결되어 있다가(bit1 == 1) 방금 끊긴(false) 경우
    else if (gripperNow == false && wasGripConnected == true) {
        // Gripper가 끊기면 하위 데이지 체인인 Pusher도 비트를 모두 끎 (AND NOT 연산)
        system_Status.data.status_res.isConnect &= ~0x03; 
        DEBUG_SERIAL.println("GRIPPER Disconnected!");
    }

    // --- 2. Pusher 체크 (Gripper가 물리적으로 연결되어 있을 때만 확인) ---
    if (gripperNow == true) {
        bool pusherNow = dxl.ping(PUSHER_ID);

        // 끊겨있다가(bit0 == 0) 방금 연결된(true) 경우
        if (pusherNow == true && wasPushConnected == false) {
            dxl.setOperatingMode(PUSHER_ID, OP_EXTENDED_POSITION);
            dxl.torqueOn(PUSHER_ID);
            dxl.writeControlTableItem(PROFILE_VELOCITY, PUSHER_ID, DXL_PUSH_VELOCITY);
            dxl.ledOn(PUSHER_ID);

            // bit 0을 1로 세팅
            system_Status.data.status_res.isConnect |= 0x01; 
            DEBUG_SERIAL.println("PUSHER Reconnected");
            ret = true;
        }
        // 연결되어 있다가(bit0 == 1) 방금 끊긴(false) 경우
        else if (pusherNow == false && wasPushConnected == true) {
            // bit 0만 끎
            system_Status.data.status_res.isConnect &= ~0x01; 
            DEBUG_SERIAL.println("PUSHER Disconnected!");
        }
    }

    return ret;
}

void checkArrival() {
    unsigned long currMillis = millis();

    // --- 1. Gripper 도착 확인 ---
    if (isGripperMoving) {
        uint16_t presentPos = dxl.getPresentPosition(GRIPPER_ID);
        // abs() 함수로 차이값(절대값) 계산
        if (abs((long)presentPos - (long)gripper_Target) <= ARRIVE_THRESHOLD) {
            
            isGripperMoving = false; // 도착했으므로 플래그 해제

            // if (abs((long)presentPos - DXL_CLOSE_POSITION) <= 100) { // 오차 범위 200 정도 설정
            //     isOpened = false;
            //     DEBUG_SERIAL.println("Status: Gripper CLOSED");
            // } 
            // // 열림 위치(Open) 근처라면 열린 것으로 간주
            // else if (abs((long)presentPos - DXL_OPEN_POSITION) <= 100) {
            //     isOpened = true;
            //     DEBUG_SERIAL.println("Status: Gripper OPENED");
            // }
            sendArrivedPacket(GRIPPER_ID, presentPos);
            DEBUG_SERIAL.println("GRIPPER Arrived!");           
        }
    }

    // --- 2. Pusher 도착 확인 ---
    if (isPusherMoving) {
        uint16_t presentPos = dxl.getPresentPosition(PUSHER_ID);
        if (abs((long)presentPos - (long)pusher_Target) <= ARRIVE_THRESHOLD) {
            
            isPusherMoving = false;

            // if (abs((long)presentPos - DXL_PUSH_POSTION) <= 100) { 
            //     isPushed = true;
            //     DEBUG_SERIAL.println("Status: Pusher PUSHED");
            // } 
            // // 열림 위치(Open) 근처라면 열린 것으로 간주
            // else if (abs((long)presentPos - DXL_RELEASED_POSITION) <= 100) {
            //     isPushed = false;
            //     DEBUG_SERIAL.println("Status: Pusher RELEASED");
            // }

            sendArrivedPacket(PUSHER_ID, presentPos);
            DEBUG_SERIAL.println("PUSHER Arrived!");
            digitalWrite(PUSH_LED, HIGH);
        }
    }
}


// 도착 패킷 전송 함수 보완
void sendArrivedPacket(uint8_t id, uint16_t pos) {
    can_packet_t txPkt = {0};
    txPkt.cmd = CMD_ARRIVED; 
    txPkt.id = id;
    
    // 최신 정보 채우기
    txPkt.data.status_res.gripper_pos = (id == GRIPPER_ID) ? pos : (uint16_t)dxl.getPresentPosition(GRIPPER_ID);
    txPkt.data.status_res.pusher_pos = (id == PUSHER_ID) ? pos : (uint16_t)dxl.getPresentPosition(PUSHER_ID);
    // txPkt.data.status_res.gripper_cur = dxl.getPresentCurrent(GRIPPER_ID);
    txPkt.data.status_res.isConnect = system_Status.data.status_res.isConnect;

#ifdef OPENCR_CAN
    if (!isCanRecentlyAlive()) {
        return;
    }

    tx_msg.length = sizeof(can_packet_t);
    memcpy(tx_msg.data, &txPkt, tx_msg.length);
    // canSend(0x124, tx_msg);
    canSendSafe(0x124, tx_msg);
#endif
}

void checkGripStatus() {
    if (!isGripperMoving) return; // 그리퍼가 움직일 때만 체크

    int16_t current = dxl.getPresentCurrent(GRIPPER_ID);
    int16_t posDiff = abs((long)dxl.getPresentPosition(GRIPPER_ID) - (long)gripper_Target);

    // XH 시리즈 임계값 판단 (80 raw = 약 215mA)
    if (abs(current) > GRIP_THRESHOLD_RAW && posDiff > 30) {
        DEBUG_SERIAL.print("Object Detected! Current: ");
        DEBUG_SERIAL.println(current);
        
        // [중요] 물체를 잡았으므로 더 이상의 이동 명령 감시를 중단하고 PC에 도착 신호 전송
        uint16_t presentPos = dxl.getPresentPosition(GRIPPER_ID);
        sendArrivedPacket(GRIPPER_ID, presentPos); 
        
        isGripperMoving = false; // 플래그 해제
        
        // LED로 잡기 성공 표시 (선택사항)
        digitalWrite(GRIP_LED, LOW); 
    }
}

void buttonTask(){
    myBtn.read();

    // [방안 1] 2초가 되는 바로 그 순간 즉시 그리퍼(Gripper) 구동
    if (myBtn.pressedFor(2000)) {
        if (!longPressExecuted) {
            DEBUG_SERIAL.println("--- 2 Seconds Reached! Controlling Gripper Immediate ---");
            
            // 1. 사용자 피드백 (내장 LED 켜기)
            digitalWrite(ARDUINO_LED, HIGH); 
            
            // 2. 위치 기반 그리퍼 제어 (열림/닫힘 토글)
            uint16_t currentPos = dxl.getPresentPosition(GRIPPER_ID);
            isGripperMoving = true;
            gripperMoveStartMillis = millis();
            gripperPrevTarget = currentPos;

            if (abs((long)currentPos - DXL_OPEN_POSITION) < abs((long)currentPos - DXL_CLOSE_POSITION)) {
                gripper_Target = DXL_CLOSE_POSITION;
                DEBUG_SERIAL.println("Action: Gripper CLOSE");
            } else {
                gripper_Target = DXL_OPEN_POSITION;
                DEBUG_SERIAL.println("Action: Gripper OPEN");
            }
            
            
            dxl.setGoalPosition(GRIPPER_ID, gripper_Target);
            
            longPressExecuted = true; // 문 걸어잠그기
        }
    }

    // [방안 2] 버튼에서 손을 떼는 순간 (Release)
    if (myBtn.wasReleased()) {
        digitalWrite(ARDUINO_LED, LOW); // 피드백 LED 끄기

        // 이미 2초가 지나서 그리퍼가 작동했다면, 손을 떼는 시점에는 아무것도 안 함
        if (longPressExecuted) {
            longPressExecuted = false; 
            DEBUG_SERIAL.println("Button Released after Long Press (Gripper Done)");
        } 
        // 2초가 되기 전에 손을 뗐다면 -> "짧은 누름"이므로 푸셔(Pusher) 구동
        else {
            DEBUG_SERIAL.println("--- Short Press Detected on Release: Controlling Pusher ---");
            
            uint16_t currentPos = dxl.getPresentPosition(PUSHER_ID);
            isPusherMoving = true;
            pusherMoveStartMillis = millis();
            pusherPrevTarget = currentPos;

            // 푸셔도 위치 기반으로 토글 제어 (전진 위치에 가까우면 후진, 반대면 전진)
            if (abs((long)currentPos - DXL_PUSH_POSTION) < abs((long)currentPos - DXL_RELEASE_POSITION)) {
                pusher_Target = DXL_RELEASE_POSITION;
                DEBUG_SERIAL.println("Action: Pusher RELEASE");
            } else {
                pusher_Target = DXL_PUSH_POSTION;
                DEBUG_SERIAL.println("Action: Pusher PUSH");
                digitalWrite(PUSH_LED, LOW); // 전진 시작할 때 LED 켜기
            }
            
            
            dxl.setGoalPosition(PUSHER_ID, pusher_Target);
        }
    }

}


void checkMoveTimeout() {
    unsigned long currMillis = millis();

    if (isGripperMoving && currMillis - gripperMoveStartMillis > GRIP_MOVE_TIMEOUT_MS) {
        uint16_t presentPos = dxl.getPresentPosition(GRIPPER_ID);

        DEBUG_SERIAL.print("GRIPPER Timeout! Present: ");
        DEBUG_SERIAL.println(presentPos);

        isGripperMoving = false;

        setMotorError(GRIPPER_ID, ERROR_GRIPPER_TIMEOUT);

        // CAN 송신하지 않고, 그리퍼만 안전 위치로 복귀
        gripper_Target = DXL_OPEN_POSITION;
        gripperMoveStartMillis = currMillis;
        isGripperMoving = true;

        dxl.setGoalPosition(GRIPPER_ID, gripper_Target);
    }

    if (isPusherMoving && currMillis - pusherMoveStartMillis > PUSH_MOVE_TIMEOUT_MS) {
        uint16_t presentPos = dxl.getPresentPosition(PUSHER_ID);

        DEBUG_SERIAL.print("PUSHER Timeout! Present: ");
        DEBUG_SERIAL.println(presentPos);

        isPusherMoving = false;

        setMotorError(PUSHER_ID, ERROR_PUSHER_TIMEOUT);

        pusher_Target = DXL_RELEASE_POSITION;
        pusherMoveStartMillis = currMillis;
        isPusherMoving = true;

        dxl.setGoalPosition(PUSHER_ID, pusher_Target);
    }
}


void setMotorError(uint8_t id, uint8_t errorCode) {
    pending_error = true;
    last_error_motor_id = id;

    system_Status.cmd = CMD_ERROR;
    system_Status.id = id;
    system_Status.data.status_res.error_code = errorCode;
    system_Status.data.status_res.gripper_pos = dxl.getPresentPosition(GRIPPER_ID);
    system_Status.data.status_res.pusher_pos = dxl.getPresentPosition(PUSHER_ID);
    system_Status.data.status_res.isConnect = system_Status.data.status_res.isConnect;

    DEBUG_SERIAL.print("Motor Error ID: ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(", Error Code: ");
    DEBUG_SERIAL.println(errorCode, HEX);
}
