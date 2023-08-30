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
        //rm->pauseMotor();
        rm->torqueControl2(CMD_MOTOR_DIR_CW, 0); // to smooth 
        
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
    stateidx = STATES_IDX::IDX_GOREADY;
    
}

void readyState::run(){
    rm->updateMultAngle();

    if(!terminate){
        if((rm->getZeroMultAngle() - rm->getMultAngleRAW() > -ANGLE_TOLERANCE)&&
        (rm->getZeroMultAngle() - rm->getMultAngleRAW() < ANGLE_TOLERANCE)) {
            //rm->torqueControl2(CMD_MOTOR_DIR_CW, 0);
            terminate = true;
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
    rm->restart();
    while((millis()-st_time)<100);
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
                    rm->absposControl(target_speed,(int32_t)rm->getZeroMultAngle() + target_amp+ANGLE_TOLERANCE);
                    if(rm->getMultAngleRAW() > rm->getZeroMultAngle() + target_amp){
                        // rm->torqueControl2(CMD_MOTOR_DIR_CW, 0); // to smooth the reversion
                        direction = CMD_MOTOR_DIR_CCW;
                        // return;
                    }
                }
                else if(direction == CMD_MOTOR_DIR_CCW){
                    rm->absposControl(target_speed,(int32_t)rm->getZeroMultAngle() - target_amp-ANGLE_TOLERANCE);
                    if(rm->getMultAngleRAW() < rm->getZeroMultAngle() - target_amp){
                        // rm->torqueControl2(CMD_MOTOR_DIR_CW, 0); // to smooth the reversion
                        direction = CMD_MOTOR_DIR_CW;
                        cycle++;
                        // return;
                    }
                }
                // rm->speedControl2(direction, target_speed);
            }
            else{
                rm->absposControl(target_speed,(int32_t)rm->getZeroMultAngle());
                if((rm->getZeroMultAngle() - rm->getMultAngleRAW() > -ANGLE_TOLERANCE)&&
                (rm->getZeroMultAngle() - rm->getMultAngleRAW() < ANGLE_TOLERANCE)) {
                    // rm->posControl2(500,(int32_t)rm->getZeroMultAngle());
                    terminate = true;
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
    target_offset = rm->getCCWLockAngle()+((rm->getCWLockAngle() - rm->getCCWLockAngle()))*((float)cmd_offset / (float)200);
    //target_amp = ((rm->getCWLockAngle() - rm->getCCWLockAngle()))*((float)cmd_amp / (float)200);
    /*360/256deg /LSB*/
    target_amp = cmd_amp*GEAR_RATIO;
    uint32_t st_time=millis();
    rm->restart();
    while((millis()-st_time)<100);
    // init_mult_angle = rm->getCCWLockAngle() + target_offset;
    // target_speed = 36000 * ((float)cmd_fq / (float)100);
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
                double tmp_target= target_offset+(target_amp*sin(2.0*M_PI*(float)target_fq/1000.*(float)cur_time/1000.));
                rm->motioncontrol(tmp_target,(2.0*M_PI*(float)target_fq/1000.*target_amp*cos(2.0*M_PI*(float)target_fq/1000.*(float)cur_time/1000.)),
                3./1000.*(double)target_fq+1.,14./100.*(double)target_fq+40);
            }
            else{
                terminate = true;
                //offset_flag = true;
            }
        }
        else{ // go to the offset position
            rm->absposControl(BASIC_SPEED,target_offset);
            //rm->motioncontrol(target_offset,1,9,320);
            if((target_offset - rm->getMultAngleRAW() > -ANGLE_TOLERANCE)&&
            (target_offset - rm->getMultAngleRAW() < ANGLE_TOLERANCE)){
                offset_flag = true;
                start_moment = millis(); //start_moment = millis()+10;
                uint32_t st_time=millis();
                rm->restart();
                while((millis()-st_time)<100);
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
            uint32_t st_time=millis();
            rm->restart();
            while((millis()-st_time)<100);
            rm->absposControl(BASIC_SPEED, rm->getZeroMultAngle());
            //rm->motioncontrol(0,1,9,320);
            return STATES_IDX::IDX_GOREADY;
        
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