#include "states.h"
#include "Arduino.h"
#include "math.h"
/*****************************/ 
/******define init state******/
/*****************************/ 
void initState::update(byte* cmd) {
    // Serial.println("init mode");
    stateidx = STATES_IDX::IDX_INIT; 
    rm->offMotor(); 
    //rm->writePIDRAM(); 07.25
}

STATES_IDX initState::getNextState(byte* cmd){ 
    //Serial.println("stopState::getNextState");
    // if(cmd.equals("L2L\n")) {
    if(cmd[0] == STATES_IDX::IDX_L2L){
        return STATES_IDX::IDX_L2L;
    }
    return STATES_IDX::IDX_INIT;
}

/*****************************/ 
/******define l2l state*******/
/*****************************/ 
void l2lState::update(byte* cmd){
    // Serial.println("L2L mode");
    stateidx = STATES_IDX::IDX_L2L; 
    start_moment = millis();
    rm->updateCurrentState2();
    direction = CMD_MOTOR_DIR_CCW;

    cw_done = false;
    ccw_done = false;
    prev_mult_angle = 0;
    
    terminate = false;

    cw_stuck_count = 0;
    ccw_stuck_count = 0;
    
    cmd_target_torque = *((uint16_t*)(cmd + 1)); 
    cmd_stuck_angle = cmd[3]; // (0.1deg) 
    cmd_stuck_time = cmd[4];  // (0.1sec)
    
    ccw_target_torque = cmd_target_torque; // ccw_target_torque = 200;
    cw_target_torque = cmd_target_torque; // cw_target_torque = 200;
    
    threshold_stuck_angle = 10*cmd_stuck_angle; // angle to assum the motion stopped (0.01deg)
    threshold_stuck_count = 10*cmd_stuck_time;  // duration to decide the end of stroke (0.01sec)
    //threshold_stuck_count = 1e6/SAMPLE_CLKus * 
}

STATES_IDX l2lState::getNextState(byte* cmd){
    if(cmd[0] == STATES_IDX::IDX_INIT) {return STATES_IDX::IDX_INIT;}
    if(terminate){
        rm->pauseMotor();
        stateidx = STATES_IDX::IDX_L2LTERM;
        if(cmd[0] == STATES_IDX::IDX_L2L) update(cmd);
        else if(cmd[0] == STATES_IDX::IDX_GOREADY) return STATES_IDX::IDX_GOREADY;
    }
    else if(cmd[0] == STATES_IDX::IDX_PAUSE){
        pause = true; 
        stateidx = STATES_IDX::IDX_PAUSE;
        pause_moment = millis();//  global_moment.
    }
    if(pause){
        if(cmd[0] == STATES_IDX::IDX_RESUME){
            stateidx = STATES_IDX::IDX_L2L;
            ms_t pause_end_moment = millis(); // global_moment.
            pause_time += (pause_end_moment - pause_moment); 
            pause = false;
        }
        else if(cmd[0] == STATES_IDX::IDX_INIT){
            return STATES_IDX::IDX_INIT; //return STATES_IDX::IDX_READY;
        }
    }
    return STATES_IDX::IDX_L2L;
}

void l2lState::run(){
    rm->updateMultAngle();
    if(!ccw_done){
        ccwCalibration();
    }
    else if(!cw_done){
        cwCalibration();
    }
    else {
        rm->setZeroMultAngle();
        rm->pauseMotor();
        //rm->torqueControl1(0); // to smooth 
        
        terminate = true;
    }
    prev_mult_angle = rm->getMultAngleRAW();
}

void l2lState::ccwCalibration()
{
    if(abs(rm->getMultAngleRAW() - prev_mult_angle) < threshold_stuck_angle) ccw_stuck_count++;
    else ccw_stuck_count = 0;
    if(ccw_stuck_count > threshold_stuck_count) {
        ccw_done = true;
        rm->torqueControl2(CMD_MOTOR_DIR_CW, 0); // to smooth 
        rm->setCCWLock(rm->getMultAngleRAW());
        return;
    }
    rm->torqueControl2(CMD_MOTOR_DIR_CCW, ccw_target_torque);
}

void l2lState::cwCalibration()
{
    if(abs(rm->getMultAngleRAW() - prev_mult_angle) < threshold_stuck_angle) cw_stuck_count++;
    else cw_stuck_count = 0;
    if(cw_stuck_count > threshold_stuck_count) {
        cw_done = true;
        rm->torqueControl2(CMD_MOTOR_DIR_CW, 0); // to smooth 
        rm->setCWLock(rm->getMultAngleRAW());
        return;
    }
    rm->torqueControl2(CMD_MOTOR_DIR_CW, cw_target_torque);
}

/*****************************/ 
/*****define reday state*****/
/*****************************/ 
void readyState::update(byte* cmd) {
    terminate = false;
    check_cnt=0;
    rm->restart();
    stateidx = STATES_IDX::IDX_GOREADY;
    
}

void readyState::run(){
    rm->updateMultAngle();

    if(!terminate){
        if((rm->getZeroMultAngle() - rm->getMultAngleRAW() > -ANGLE_TOLERANCE)&&
        (rm->getZeroMultAngle() - rm->getMultAngleRAW() < ANGLE_TOLERANCE)) {
            check_cnt++;
            if(check_cnt > 100){
                rm->pauseMotor();
                //rm->torqueControl2(CMD_MOTOR_DIR_CW, 0);
                terminate = true;
            }
        }
        else{
            rm->absposControl(BASIC_SPEED, rm->getZeroMultAngle());
        }
    }
    else {
        rm->pauseMotor();
    }
}
STATES_IDX readyState::getNextState(byte* cmd){
    if(cmd[0] == STATES_IDX::IDX_INIT) {return STATES_IDX::IDX_INIT;}
    if(terminate){
        stateidx = STATES_IDX::IDX_READY;
        if(cmd[0] == STATES_IDX::IDX_CONST) {return STATES_IDX::IDX_CONST;}
        else if(cmd[0] == STATES_IDX::IDX_SINE) {return STATES_IDX::IDX_SINE;}
    }
    return STATES_IDX::IDX_GOREADY;
}
/*****************************/ 
/*****define const state******/
/*****************************/ 
void constState::update(byte* cmd){
    // Serial.println("const mode");
    stateidx = STATES_IDX::IDX_CONST;  
    direction = CMD_MOTOR_DIR_CW;
    terminate = false;
    pause = false;
    start_moment = millis();
    pause_time = 0; // 생성시 초기화하는데 한번 더 함.
    cycle = 0;
    center_flag = false;

    cmd_rps = cmd[1];  // 0.01rps
    cmd_cycle = cmd[2];
    cmd_range = cmd[3];// cmd_amp = *((uint16_t*)(cmd + 3)); half range

    // target_speed = (cmd_rps*360 * 13/10 + 18000)*GEAR_RATIO;
    // target_speed = (int32_t)(cmd_rps*3600*GEAR_RATIO);
    target_speed = (int32_t)(cmd_rps*3.60*GEAR_RATIO);
    target_cycle = cmd_cycle;
    target_amp = ((float)(rm->getCWLockAngle() - rm->getCCWLockAngle())) * ((float)cmd_range / (float)200.);
    uint32_t st_time=millis();
    check_cnt=0;
    check_cnt2=0;
}


void constState::run(){
    if(pause){  
        rm->pauseMotor();
    }
    else{
        rm->updateMultAngle();
        if (center_flag){ // if center reached
            if(cycle < target_cycle){
                if(direction == CMD_MOTOR_DIR_CW){
                    rm->speedControl1(target_speed*100);
                    if(rm->getMultAngleRAW() > rm->getZeroMultAngle() + target_amp)
                        {
                            rm->pauseMotor();
                            //rm->torqueControl1(0);
                            direction = CMD_MOTOR_DIR_CCW;
                        }
                    }
                else if(direction == CMD_MOTOR_DIR_CCW){
                    rm->speedControl1(target_speed*(-100));
                    if(rm->getMultAngleRAW() < rm->getZeroMultAngle() - target_amp){
                        rm->pauseMotor();
                        //rm->torqueControl1(0);
                        direction = CMD_MOTOR_DIR_CW;
                        cycle++;
                        stop_flag=false;
                    }
                }
                // rm->speedControl2(direction, target_speed);
            }
            else{
              if(!stop_flag)
                rm->speedControl1(target_speed*100);
                if((rm->getZeroMultAngle() - rm->getMultAngleRAW() > -ANGLE_TOLERANCE)&&
                (rm->getZeroMultAngle() - rm->getMultAngleRAW() < ANGLE_TOLERANCE)) {
                     stop_flag=true;
                    
                     rm->absposControl(BASIC_SPEED, rm->getZeroMultAngle() );
                    check_cnt2++;
                    if(check_cnt2 > 80){
                      rm->torqueControl2(CMD_MOTOR_DIR_CW, 0);
                      terminate = true;
                    }
                }
                // else{
                //     rm->speedControl2(direction, target_speed);
                //     rm->posControl2(500,(int32_t)rm->getZeroMultAngle());
                // }
            }
        }
        else{ // go to the center position
            rm->absposControl(BASIC_SPEED, rm->getZeroMultAngle() );
            if((rm->getZeroMultAngle() - rm->getMultAngleRAW() > -ANGLE_TOLERANCE)&&
            (rm->getZeroMultAngle() - rm->getMultAngleRAW() < ANGLE_TOLERANCE)){
                check_cnt++;
                if(check_cnt >100)
                    center_flag = true;
            }
        }
    }
}
STATES_IDX constState::getNextState(byte* cmd){
    if(cmd[0] == STATES_IDX::IDX_INIT) {
        return STATES_IDX::IDX_INIT;
    }

    if(terminate) return STATES_IDX::IDX_GOREADY;
    else if(cmd[0] == STATES_IDX::IDX_PAUSE){
        pause = true; 
        pause_moment = millis();// 
        stateidx = STATES_IDX::IDX_PAUSE;
    }

    if(pause){
        if(cmd[0] == STATES_IDX::IDX_RESUME){
            stateidx = STATES_IDX::IDX_CONST;
            ms_t pause_end_moment = millis(); // 
            pause_time += (pause_end_moment - pause_moment); // 
            pause = false;
        }
        else if(cmd[0] == STATES_IDX::IDX_GOREADY){
            return STATES_IDX::IDX_GOREADY;
        }
    }
    return STATES_IDX::IDX_CONST;
}

/*****************************/ 
/******define sine state******/
/*****************************/ 
void sineState::update(byte * cmd){
    
    // Serial.println("sine mode");
    stateidx = STATES_IDX::IDX_SINE;
    // direction = CMD_MOTOR_DIR_CW;
    terminate = false;
    pause = false;

    
    start_moment = millis();
    offset_flag = false;
    pause_time = 0; // 생성자에서 초기화 한 번하고 다시 한번.
    // cycle = 0;

    cmd_fq      = cmd[1];
    cmd_cycle   = cmd[2];
    cmd_offset  = cmd[3];
    cmd_amp     = cmd[4];
    
    target_fq = (uint16_t)cmd_fq * 10;
    target_cycle = cmd_cycle;
    /*0.5%/LSB*/
    target_offset = rm->getCCWLockAngle()+((rm->getCWLockAngle() - rm->getCCWLockAngle()))*((float)cmd_offset / (float)200.);
    //target_amp = ((rm->getCWLockAngle() - rm->getCCWLockAngle()))*((float)cmd_amp / (float)200);
    /*360/256deg /LSB*/
    target_amp = cmd_amp*GEAR_RATIO;
    check_cnt=0;
    check_cnt2=0;
    once=false;
    uint32_t st_time1=millis();
    rm->restart();
    while((millis()-st_time1)<10);
}

void sineState::run(){
  if(pause){
      rm->pauseMotor();
  }
  else{
      rm->updateMultAngle();
      if (offset_flag){ // offset position reached
          cur_time = millis()-start_moment-pause_time;
          if((cur_time+10)*target_fq < (uint32_t)target_cycle*1000000){ 
              double tmp_target= target_offset/100.+(target_amp*sin(2.0*M_PI*(float)target_fq/1000.*(float)cur_time/1000.));
              double velocity = (2.0*M_PI*(float)target_fq/1000.*target_amp*cos(2.0*M_PI*(float)target_fq/1000.*(float)cur_time/1000.));
              rm->motioncontrol(tmp_target,velocity,(float)target_fq/1000.*(-214.)+357,(float)target_fq/1000.*180-80);
          }
          else{
              terminate = true;
              //offset_flag = true;
          }
      }
      else{ // go to the offset position
          //rm->absposControl(BASIC_SPEED,target_offset);
          rm->checking(rm->getMultAngleRAW(),target_offset );
          rm->motioncontrol(target_offset/100,0,57,310);
          if((target_offset - rm->getMultAngleRAW() > -ANGLE_TOLERANCE)&&
          (target_offset - rm->getMultAngleRAW() < ANGLE_TOLERANCE)){
              check_cnt++;
              if(check_cnt>100){
                offset_flag = true;
                start_moment = millis(); //start_moment = millis()+10;
              }
          }
      }
  }
}


STATES_IDX sineState::getNextState(byte* cmd){
    if(cmd[0] == STATES_IDX::IDX_INIT){
        return STATES_IDX::IDX_INIT;
        }
    if(terminate){
        //rm->pauseMotor();
        stateidx = STATES_IDX::IDX_SINETERM;
        if(cmd[0] == STATES_IDX::IDX_SINE) update(cmd);
        else if(cmd[0] == STATES_IDX::IDX_GOREADY){
            if(!once){
                rm->restart();
                once=true;
            }
            else{
              check_cnt2++;
              if(check_cnt2>100){
                  rm->absposControl(BASIC_SPEED, rm->getZeroMultAngle());
                  //rm->motioncontrol(rm->getZeroMultAngle(),0,57,310);
                  return STATES_IDX::IDX_GOREADY;
              }
            }
        }
    }
    else if(cmd[0] == STATES_IDX::IDX_PAUSE){
        pause = true; 
        stateidx = STATES_IDX::IDX_PAUSE;
        pause_moment = millis();// 
    }
    if(pause){
        if(cmd[0] == STATES_IDX::IDX_RESUME){
            stateidx = STATES_IDX::IDX_SINE;
            ms_t pause_end_moment = millis(); // global_moment.
            pause_time += (pause_end_moment - pause_moment); // 
            pause = false;
        }
        else if(cmd[0] == STATES_IDX::IDX_GOREADY){
            return STATES_IDX::IDX_GOREADY;
        }
    }
    return STATES_IDX::IDX_SINE;
}