#include "rmd_motor_V3_GIGA.h"
#include "math.h"
#include "utils.h"
using namespace mbed;

RMDmotor::RMDmotor() : addr(0x141), can(CAN_RX, CAN_TX), mult_angle(0)
{    txmsg.id = addr;   }

RMDmotor::RMDmotor(uint8_t id) : addr(140 | id), can(CAN_RX, CAN_TX), mult_angle(0)
{    txmsg.id = addr;   }


void RMDmotor::begin(){
    while (!can.frequency(1000000))
        Serial.println("Bps setting....");
    while (!can.mode(CAN::Normal))
        Serial.println("Mode setting...");
}

bool RMDmotor::offMotor()
{
    byte res[8] = {0, };
    uint8_t can_res = sendCMD(frame_off, res);
    if (can_res == CAN_OK)
    {
        return memcmp(res, frame_off, 8) == 0;
    }
    return false;
}

bool RMDmotor::pauseMotor()
{
    byte res[8] = {0, };
    uint8_t can_res = sendCMD(frame_pause, res);
    if (can_res == CAN_OK)
    {
        return memcmp(res, frame_pause, 8) == 0;
    }
    return false;
}

bool RMDmotor::writePIDRAM()
{
    byte res[8] = {0, };
    uint8_t can_res = sendCMD(frame_write_RAMPID, res);
    if (can_res == CAN_OK)
    {
        return memcmp(res, frame_write_RAMPID, 8) == 0;
    }
    return false;
}

bool RMDmotor::writePIDRAM(byte set[])
{
    byte res[8] = {0, };
    memcpy(frame_write_RAMPID + 8 - sizeof(set), set, sizeof(set));
    uint8_t can_res = sendCMD(frame_write_RAMPID, res);
    if (can_res == CAN_OK)
    {
        return memcmp(res, frame_write_RAMPID, 8) == 0;
    }
    return false;
}

bool RMDmotor::restart()
{
    return CAN_OK == sendCMD(frame_system_reset);
}

uint8_t RMDmotor::sendCMD(byte req[])
{
    txmsg.id=addr;
    memcpy(txmsg.data, req, 8);
    uint8_t result = can.write(txmsg);
    return result;
}


uint8_t RMDmotor::sendCMD(byte req[], byte res[])
{
    txmsg.id=addr;
    memcpy(txmsg.data, req, 8);
    uint8_t result = can.write(txmsg);
    if(result==0) return CAN_FAILED; 
    uint32_t start_tmr=millis();
    while(!can.read(rxmsg)){
      if(millis()-start_tmr>200){
        break;
      }
    }
    memcpy(res, rxmsg.data, CAN_LEN);
    return CAN_OK;
}





bool RMDmotor::absposControl(const uint16_t &speed_limit, const int32_t &pos)
{
    state_update_flag = true;
    *((uint16_t *)(frame_abs_pos_control + 2)) = speed_limit;
    *((int32_t *)(frame_abs_pos_control + 4)) = pos;
    return CAN_OK == sendCMD(frame_abs_pos_control, (byte*)&current_state2);
}

bool RMDmotor::singleposControl(const uint8_t &direct, const uint16_t &speed_limit, const uint16_t &pos)
{
    state_update_flag = true;
    *((uint8_t *)(frame_single_pos_control + 1)) = direct;
    *((uint16_t *)(frame_single_pos_control + 2)) = speed_limit;
    *((uint16_t *)(frame_single_pos_control + 4)) = pos;
    return CAN_OK == sendCMD(frame_single_pos_control, (byte *)&current_state2);
}

bool RMDmotor::increposControl(const uint16_t &speed_limit, const int32_t &pos)
{
    state_update_flag = true;
    *((uint16_t *)(frame_incre_pos_control + 2)) = speed_limit;
    *((int32_t *)(frame_incre_pos_control + 4)) = pos;
    return CAN_OK == sendCMD(frame_incre_pos_control, (byte *)&current_state2);
}

/*speed resoultion: 0.01dps*/
bool RMDmotor::speedControl1(const int32_t &speed)
{
    state_update_flag = true;
    *((int32_t *)(frame_speed_control + 4)) = speed;
    return CAN_OK == sendCMD(frame_speed_control, (byte*)&current_state2);
}
bool RMDmotor::speedControl2(const uint8_t &direction, const int32_t &speed)
{
    state_update_flag = true;
    if (direction == CMD_MOTOR_DIR_CCW)
        return speedControl1(speed * (-1));
    else
        return speedControl1(speed);
}

bool RMDmotor::torqueControl1(const int16_t &torque)
{
    state_update_flag = true;
    *((int16_t *)(frame_torque_control + 4)) = torque;
    return CAN_OK == sendCMD(frame_torque_control, (byte *)&current_state2);
}
bool RMDmotor::torqueControl2(const uint8_t &direction, const int16_t &torque)
{
    state_update_flag = true;
    if (direction == CMD_MOTOR_DIR_CCW)
        return torqueControl1(torque * (-1));
    else
        return torqueControl1(torque);
}

bool RMDmotor::multAngleControl(const int32_t& target_mult_angle, const int32_t& speed, const int32_t& offset){
  updateMultAngle();
  if((target_mult_angle - mult_angle > -offset)&&(target_mult_angle - mult_angle < offset)) {
    pauseMotor();
    return true;
  }
  else if(target_mult_angle > mult_angle) speedControl1(speed);
  else if(target_mult_angle <= mult_angle) speedControl1(-speed);
  return false;
}


uint8_t RMDmotor::RAW_motionControl(const uint16_t &p_des, const uint16_t &v_des, const uint16_t &Kp, const uint16_t &Kd, const uint16_t &t_ff, byte res[])
{
    byte frame_motion_control[8] = {0, };
    *((uint16_t *)(frame_motion_control)) = (p_des << 8 | p_des >> 8);      // rad(-12.5~12.5)
    *((uint16_t *)(frame_motion_control + 2)) = (v_des << 12 | v_des >> 4); // rad/s (-45~45)
    *((uint16_t *)(frame_motion_control + 3)) |= (Kp << 8 | Kp >> 8);       // (0~500)
    *((uint16_t *)(frame_motion_control + 5)) = (Kd << 12 | Kd >> 4);       // (0~5)
    *((uint16_t *)(frame_motion_control + 6)) |= (t_ff << 8 | t_ff >> 8);   // (-24~24)
    txmsg.id = addr^0x540;
    memcpy(txmsg.data, frame_motion_control, CAN_LEN);
    uint8_t result = can.write(txmsg);
    if(result==0) return CAN_FAILED;
    uint32_t start_tmr=millis();
    while(!can.read(rxmsg)){
        if(millis()-start_tmr>200){
            break;
        }
    }
    //memcpy(res, rxmsg.data, sizeof(rxmsg.data));
    return result;
}

bool RMDmotor::motioncontrol(const double &angle, const double &speed, const double &Kp, const double &Kd, const double &tor)
{
    uint16_t pos=angle*(M_PI/180*65535./25)+65535./2;
    pos = (pos > 65535) ? 65535 : (pos < 0) ? 0 : pos;
    uint16_t vel = speed*(M_PI/180*4095./90)+4095./2;
    vel = (vel > 4095) ? 4095 : (vel < 0) ? 0 : vel;
    uint16_t kp = Kp;
    uint16_t kd = Kd;
    uint16_t t_ff = (tor + 24) / 48 * 4095;
    return CAN_OK == RAW_motionControl(pos, vel, kp, kd, t_ff, (byte *)&pre_state);
}

bool RMDmotor::zero_offset()
{
    uint8_t result = sendCMD(frame_zero_offset);
    return CAN_OK == result;
}

void RMDmotor::print_frame(char frame[], char len){
  for(char i = 0; i < len; i++){
    Serial.print(frame[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

bool RMDmotor::updateMultAngle()
{
    bool result = CAN_OK == sendCMD(frame_read_mult_angle, (byte *)frame_res_mult_angle);
    mult_angle = *((int32_t *)(frame_res_mult_angle + 4));
    return result;
}

bool RMDmotor::updateSingleAngle()
{
    bool result = CAN_OK == sendCMD(frame_read_single_angle, (byte *)frame_res_single_angle);
    single_angle = *((uint16_t *)(frame_res_single_angle + 6));
    return result;
}
bool RMDmotor::updateCurrentState2()
{
    return CAN_OK == sendCMD(frame_read_status2, (byte *)&current_state2);
}



void RMDmotor::calculate_motionstate()
{
    uint16_t pre_pos = (pre_state.angle << 8) | (pre_state.angle >> 8);
    uint16_t pre_vel = (pre_state.vel_tor << 4 & 0xff0) | (pre_state.vel_tor >> 12 & 0xf);
    uint16_t pre_tor = (pre_state.vel_tor & 0xf00) | (pre_state.vel_tor >> 16 & 0xff);

    motion_state.deg = ((pre_pos / 65535.) * 25. - 12.5) * 180 / M_PI;
    motion_state.vel = ((pre_vel / 4095.) * 90. - 45) * 180 / M_PI;
    motion_state.tor = (pre_tor / 4095. * 48.) - 24;
}


void RMDmotor::showCurrentState2(bool update){
  if(update) updateCurrentState2();
  Serial.print("current degree: ");
  Serial.println(current_state2.degree);
  Serial.print("current iq: ");
  Serial.println(current_state2.iq);
  Serial.print("current speed: ");
  Serial.println(current_state2.speed);
  Serial.print("current temperature: ");
  Serial.println(current_state2.temperature);
}
bool RMDmotor::updateCurrentState1(){
  return CAN_OK == sendCMD(frame_read_status1, (byte* )&current_state1);
}
void RMDmotor::showCurrentState1(bool update){
  if(update) updateCurrentState1();
  Serial.print("cmd: ");
  Serial.println(current_state1[0], HEX);
  Serial.print("current temeperature: ");
  Serial.println(*((int8_t*)(current_state1+1)));
  Serial.print("current voltage: ");
  Serial.println(*((uint16_t*)(current_state1+4)));
  Serial.print("currunt error state: ");
  Serial.println(*((uint16_t*)(current_state1+6)), HEX);
  Serial.print("brake release: ");
  Serial.println(current_state1[3],HEX);
}



void RMDmotor::showMotionState(){
  calculate_motionstate();
  Serial.print("deg : ");
  Serial.println(motion_state.deg);
  Serial.print("vel: ");
  Serial.println(motion_state.vel);
  Serial.print("tor : ");
  Serial.println(motion_state.tor);
}

void RMDmotor::checking(int32_t b, int32_t c){
    byte a[8]={0x00,};
    *((int32_t *)(a)) = b;  
    *((int32_t *)(a+4)) = c;
    memcpy(txmsg.data, a, CAN_LEN);
    txmsg.id=0x10;
    can.write(txmsg);

}