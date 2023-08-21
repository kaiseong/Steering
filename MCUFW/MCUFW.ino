#include "fsm.h"
#include "states.h"
#include "rmd_motor_V3_GIGA.h"
#include "mbed.h"

#define FLAG_FSM (1UL << 0)
#define FLAG_SUB (1UL << 1)
#define CMD_FROM_PC_LEN 5
#define CMD_TO_PC_LEN 9


using namespace mbed;
using namespace rtos;

// String idx2str[8] = {String("IDX_INIT"), String("IDX_READY"), String("IDX_L2L"), String("IDX_CONST"), String("IDX_SINE"), String("NUM_STATE"), String("IDX_PAUSE"), String("IDX_SINETERM")};
EventFlags ef;

RMDmotor rm;
motorStateMachine fsm(std::move(rm));
bool cmd_received = false;
int tickcnt = 0;

void taskFSM();
void setFlag();
void setLED(int);
void serialEvent();


// bool r_led_state; 
// bool g_led_state;
// bool b_led_state;

Thread threadFsm(osPriorityRealtime); /* operating fsm and publish information*/
Thread threadSub(osPriorityRealtime); /* subscribe user msg*/

Ticker tick10ms;   /* 10 ms*/
byte cmd_from_pc[CMD_FROM_PC_LEN];
byte do_nothing_cmd[CMD_FROM_PC_LEN];

// BLE Device name
const char *nameOfPeripheral = "MbsStrRbt"; //name
const char *uuidOfService = "57e21176-df0b-11ed-b5ea-0242ac120002";
const char *uuidOfRxChar = "57154a3d-3859-4a9a-a220-d3f7cfa4f2f2";
const char *uuidOfTxChar = "3b9066d2-ed67-4815-9b98-d2d47096556f";



bool ble_connected=false;
//extern bool ble_connected;

void setup() {
  rm.begin();
  rm.zero_offset();
  delay(10);
  rm.restart();
  do_nothing_cmd[0] = STATES_IDX::NOCHANGE;
  String init_cmd = "";

  Serial.begin(115200);
  delay(100);//while(!Serial);

  fsm.getCurrentState().update(do_nothing_cmd);

  delay(100);

  tick10ms.attach(&setFlag, 0.01); // 10ms
  threadFsm.start(taskFSM);
  threadSub.start(taskSub);
}
  

void loop(){
   
}
void taskFSM(){
  while(true){
    ef.wait_any(FLAG_FSM);
    

    if(cmd_received){
      fsm.updateCurrentState(cmd_from_pc);
      cmd_from_pc[0] = STATES_IDX::NOCHANGE;
      cmd_received = false;
    }
    else{
      fsm.updateCurrentState(cmd_from_pc);
    }
    fsm.getCurrentState().run();
    Serial.write(fsm.checkMsgToPC(), CMD_TO_PC_LEN);
  }
}

void taskSub(){
  while(true){
    ef.wait_any(FLAG_SUB);
    serialEvent();
  }
}



void serialEvent(){
  if (Serial.available() >= CMD_FROM_PC_LEN) {
    Serial.readBytes(cmd_from_pc, CMD_FROM_PC_LEN);
    cmd_from_pc[0] = fsm.cmd2idx[cmd_from_pc[0]]; 
    cmd_received = true;
  }
}

// *rxChar notification handler.


void setFlag(){
  ef.set(FLAG_FSM);
  if(tickcnt % 4 == 0) ef.set(FLAG_SUB);
  ++tickcnt;
}

