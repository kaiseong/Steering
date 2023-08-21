#ifndef STATES_H
#define STATES_H
#include "utils.h"
#include "rmd_motor_V3_GIGA.h"
// #include "fsm.h"

#define GEAR_RATIO                          1
#define ANGLE_TOLERANCE                     500

class motorStateMachine; /*forward declaration to set friend*/
class State {
    friend class motorStateMachine;
public:
    State(){ }
    State(RMDmotor&& rm_, STATES_IDX idx){
        rm = &rm_;
        start_moment = 0;
        terminate = false;
        stateidx = idx;
    }
    inline STATES_IDX getStateIdx() {return stateidx;}
    //virtual void enter() {}
    virtual void update(byte* cmd) {}
    //virtual void exit() {}
    virtual void run() {}
    virtual STATES_IDX getNextState(byte* cmd) = 0;
protected:
    ms_t start_moment;
    bool terminate;
    RMDmotor *rm;
    STATES_IDX stateidx;
};


// 보통 상속받은 클래스에서는 virtual대신 override를 사용한다.(c++11 이후에 override 도입) virtual 그대로 써도 무방
// 가상함수는 State의 더블 포인터 배열을 활용하기 위해 사용

/*****************************/ 
/*****declare init state******/
/*****************************/ 
class initState:public State{
public:
    initState(RMDmotor&& rm, STATES_IDX idx): State(std::move(rm), idx){}
    virtual void update(byte* cmd); 
    virtual STATES_IDX getNextState(byte* cmd);
};

/*****************************/ 
/*****declare ready state*****/
/*****************************/ 
class readyState:public State{
    
    // bool ready_flag;
public:
    readyState(RMDmotor&& rm, STATES_IDX idx): State(std::move(rm), idx){}
    virtual void update(byte* cmd);
    virtual STATES_IDX getNextState(byte* cmd);
    virtual void run();
};

/*****************************/ 
/******declare l2l state******/
/*****************************/ 
class l2lState:public State{
    uint16_t cw_target_torque;
    uint16_t ccw_target_torque;


    uint16_t cw_stuck_count;
    uint16_t ccw_stuck_count;
    int32_t prev_mult_angle;

    uint8_t direction;
    // uint16_t step_torque;
    bool ccw_done;
    bool cw_done;

    bool pause;
    ms_t pause_time;
    ms_t pause_moment;
    ms_t pause_end_time;

    uint16_t cmd_target_torque;
    uint8_t cmd_stuck_angle;
    uint8_t cmd_stuck_time;

    int32_t threshold_stuck_angle;
    uint16_t threshold_stuck_count; /*5 sec*/
    
    void ccwCalibration();
    void cwCalibration();

public:
    l2lState(RMDmotor&& rm, STATES_IDX idx): 
        State(std::move(rm), idx),
        cw_target_torque(0), 
        ccw_target_torque(0), 
        // step_torque(10),
        cw_done(false),
        ccw_done(false),
        direction(CMD_MOTOR_DIR_CCW),
        pause_time(0)
    {}

    virtual void update(byte* cmd); 
    virtual void run();
    STATES_IDX getNextState(byte* cmd);
};

/*****************************/ 
/*****declare const state*****/
/*****************************/ 
class constState:public State{
    uint8_t direction;
    bool pause;
    ms_t pause_time;
    ms_t pause_moment;
    ms_t pause_end_time;
    uint8_t cycle;

    uint8_t cmd_rps;
    uint8_t cmd_cycle;
    uint8_t cmd_range;

    uint8_t target_cycle;
    int32_t target_speed;
    int32_t target_amp;

    bool center_flag;

public:
    constState(RMDmotor&& rm, STATES_IDX idx):
        State(std::move(rm), idx),
        target_speed(0),
        pause_time(0),
        cmd_rps(0),
        cmd_cycle(0),
        cmd_range(0),
        direction(CMD_MOTOR_DIR_CW)
    {}
    virtual void update(byte* cmd);
    virtual void run();
    STATES_IDX getNextState(byte* cmd);
};

/*****************************/ 
/*****declare sine state******/
/*****************************/ 
class sineState:public State{
    // int32_t target_speed;
    // uint8_t direction;
    bool pause;
    ms_t pause_time;
    ms_t pause_moment;
    ms_t pause_end_time;
    ms_t cur_time;
    
    // uint8_t cycle;
    uint8_t cmd_fq;
    uint8_t cmd_cycle;
    uint8_t cmd_offset;
    uint8_t cmd_amp;

    uint16_t target_fq; //mHz
    uint8_t target_cycle;
    uint8_t target_offset;
    uint8_t target_amp;
    // int64_t init_mult_angle;

    bool offset_flag;
    
public:
    sineState(RMDmotor&& rm, STATES_IDX idx):
        State(std::move(rm), idx),
        // target_speed(0),
        pause_time(0)
        // direction(CMD_MOTOR_DIR_CW),
        // offset_flag(false)
    {}
    virtual void update(byte* cmd);
    virtual void run();
    STATES_IDX getNextState(byte* cmd);
};
#endif
