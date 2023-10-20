#ifndef RMD_MOTOR_V3_GIGA_H
#define RMD_MOTOR_V3_GIGA_H

#include "mbed.h"

using namespace mbed;

/* result communication */
#define CAN_FAILED                          0
#define CAN_OK                              1



/* pin set */
#define CAN_RX                              PB_5
#define CAN_TX                              PB_13


#define CAN_LEN                            8

#define BASIC_SPEED                         100

#define CMD_READ_PID                        0x30
#define CMD_WRITE_RAM_PID                   0x31
#define CMD_WRITE_ROM_PID                   0x32
#define CMD_READ_ACC                        0x42
#define CMD_WRITE_RAM_ACC                   0x43
#define CMD_WRITE_ENCODER_OFFSET            0x63
#define CMD_WRITE_ROM_CURRENT_POS_AS_ZERO   0x64
#define CMD_SYSTEM_RESET                    0x76
#define CMD_READ_SINGLE_TURN_ENCODER        0x90
#define CMD_READ_MULTI_TURNS_ANGLE          0x92
#define CMD_READ_SINGLE_TURN_ANGLE          0x94
#define CMD_READ_MOTOR_STATUS1_ERRORFLAG    0x9A
#define CMD_READ_MOTOR_STATUS2              0x9C
#define CMD_READ_MOTOR_STATUS3              0x9D
#define CMD_MOTOR_OFF                       0x80
#define CMD_MOTOR_PAUSE                     0x81
#define CMD_TORQUE_CL                       0xA1
#define CMD_SPEED_CL                        0xA2
#define CMD_ABS_POSITION_CL                 0xA4
#define CMD_SINGLE_TURN_POSITION_CL         0xA6
#define CMD_INCRE_POSITION_CL               0xA8

#define CMD_MOTOR_DIR_CCW                   0x01
#define CMD_MOTOR_DIR_CW                    0x00

#define CW_ENCODER_SIGN                     (1 )
#define CCW_ENCODER_SIGN                    (-1)
#define CW_ANGLE_SIGN                       (1 )
#define CCW_ANGLE_SIGN                      (-1)

#define ENCODER_MAX                         16383

typedef struct motorState2{
    uint8_t cmd;
    int8_t temperature;
    int16_t iq;
    int16_t speed;
    int16_t degree;
} motor_state2;

typedef struct {
    uint8_t trash;
    uint16_t angle;
    uint32_t vel_tor;
    uint8_t blink;  
} pre_motionState;

typedef struct {
    double deg;
    double vel;
    double tor;
} motionState;

class RMDmotor{
private:
    uint16_t addr;
    uint32_t rxId;
    uint8_t rxLen;
    CAN can;
    CANMessage rxmsg;
    CANMessage txmsg;
    bool state_update_flag;

    byte frame_write_RAMPID[8]      = {CMD_WRITE_RAM_PID,                0x00, 100, 100, 70, 20, 50, 20}; 
    byte frame_pause[8]             = {CMD_MOTOR_PAUSE,                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
    byte frame_off[8]               = {CMD_MOTOR_OFF,               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_read_mult_angle[8]   = {CMD_READ_MULTI_TURNS_ANGLE,       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_read_single_angle[8] = {CMD_READ_SINGLE_TURN_ANGLE,       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_read_status1[8]      = {CMD_READ_MOTOR_STATUS1_ERRORFLAG, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_status2[8]      = {CMD_READ_MOTOR_STATUS2,           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    byte frame_torque_control[8]    = {CMD_TORQUE_CL,                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_speed_control[8]     = {CMD_SPEED_CL,                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_abs_pos_control[8]       = {CMD_ABS_POSITION_CL,              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_single_pos_control[8]    = {CMD_SINGLE_TURN_POSITION_CL,      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_incre_pos_control[8]     = {CMD_INCRE_POSITION_CL,            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_motion_control[8]        = {0x00,};

    byte frame_res_mult_angle[8];
    byte frame_res_single_angle[8];

    byte frame_system_reset[8]      = {CMD_SYSTEM_RESET,                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_zero_offset[8]       = {CMD_WRITE_ROM_CURRENT_POS_AS_ZERO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    byte pid_value[8];
    byte current_state1[8];
    motor_state2 current_state2;
    pre_motionState pre_state;
    motionState motion_state;
    uint16_t single_angle;
    int32_t mult_angle;
    int32_t ccw_lock_mult_angle;
    int32_t cw_lock_mult_angle;
    int32_t zero_mult_angle;
    uint8_t sendCMD(byte req[]);
    uint8_t sendCMD(byte req[], byte res[]);
    uint8_t RAW_motionControl(const uint16_t& p_des, const uint16_t& v_des, const uint16_t& Kp,
      const uint16_t& Kd, const uint16_t& t_ff, byte res[]);

public:
    RMDmotor(uint8_t id);
    RMDmotor();
    void begin();

/* motor initialize */
    bool offMotor();
    bool pauseMotor();
    bool writePIDRAM();
    bool writePIDRAM(byte set[]);
    bool restart();
    bool zero_offset();

/* set lock to lock angle()*/
    inline void setCCWLock(int32_t angle)  {ccw_lock_mult_angle = angle;}
    inline void setCWLock(int32_t angle)   {cw_lock_mult_angle  = angle;}
    inline void setZeroMultAngle()          {zero_mult_angle     = (ccw_lock_mult_angle + cw_lock_mult_angle) / 2;}
    inline void setnewAngles(){
        ccw_lock_mult_angle = (ccw_lock_mult_angle - cw_lock_mult_angle)/2;
        cw_lock_mult_angle = (cw_lock_mult_angle - ccw_lock_mult_angle)/2;
        zero_mult_angle=0;
    }
/* read motor states */
    bool updateCurrentState2();
    bool updateMultAngle();
    bool updateSingleAngle();
    void calculate_motionstate();

    
/* control motor */
    bool absposControl      (const uint16_t& speed_limit, const int32_t& pos);
    bool singleposControl   (const uint8_t& direct, const uint16_t& speed_limit, const uint16_t& pos);
    bool increposControl    (const uint16_t& speed_limit, const int32_t& pos);
    bool speedControl1      (const int32_t& speed);
    bool speedControl2      (const uint8_t& direction, const int32_t& speed);
    bool torqueControl1     (const int16_t& torque);
    bool torqueControl2     (const uint8_t& direction , const int16_t& torque);

    bool multAngleControl (const int32_t& target_mult_angle, const int32_t& speed, const int32_t& offset);
    bool motioncontrol    (const double& angle, const double& speed, const double& Kp, const double& Kd, const double& tor=0);
/* return motor states(inline) */
    inline const int8_t &getTemeperatureRAW()   {return current_state2.temperature;}
    inline const int16_t &getIqRAW()            {return current_state2.iq;}
    inline const int16_t &getSpeedRAW()         {return current_state2.speed;}
    inline const int16_t &getDegreeRAW()        {return current_state2.degree;}
    inline int32_t getMultAngleRAW()            {return mult_angle;}
    inline uint16_t getSingleAngleRAW()         {return single_angle;}
    inline const motor_state2 &getCurrentState2() 
    {
        state_update_flag = false;
        return current_state2;
    }
    inline const int32_t &getCWLockAngle()      {return cw_lock_mult_angle;}
    inline const int32_t &getCCWLockAngle()     {return ccw_lock_mult_angle;}
    inline const int32_t &getZeroMultAngle()    {return zero_mult_angle;}

    inline const bool& getUpdateFlag()          {return state_update_flag;}
/* diagnostic behavior */
    bool updateCurrentState1();


/* verbose */
    void showCurrentState1(bool update);
    void showCurrentState2(bool update);
    void showMotionState();
    void print_frame(char frame[],char len);
    friend void print_frame(char frame[], char len);
    void checking(int32_t b, int32_t c);
/* util */
    // static uint16_t encoderDiff(const uint16_t & prev, const uint16_t & curr);
};
#endif
