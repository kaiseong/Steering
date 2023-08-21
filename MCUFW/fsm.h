#ifndef FSM_H
#define FSM_H

// #include "Arduino.h"
#include "rmd_motor_V3_GIGA.h"
#include "states.h"
#include "utils.h"
#define NO_ENTER (0)
#define NO_UPDATE (0)
#define NO_EXIT (0)

extern bool ble_connected;

//define the finite state machine functionality
union msgToPC { /* for share memory */
  struct __attribute__((packed)) { /* for do not use byte padding */
    uint8_t mcu_status;
    uint8_t com;
    int8_t temp;
    int16_t torq;
    int16_t speed;
    int16_t ang;
};
  uint8_t bytes[9 * sizeof(uint8_t)];
};

class motorStateMachine {
	public:
    	motorStateMachine(RMDmotor&& rm) noexcept;
		~motorStateMachine();
		motorStateMachine& update(byte* cmd);
		motorStateMachine& transitionTo(STATES_IDX IDX, byte* cmd);
		motorStateMachine& immediateTransitionTo(STATES_IDX IDX);
		byte* checkMsgToPC(); 
		State& getCurrentState();
		boolean isInState(STATES_IDX &state );
		
		unsigned long timeInCurrentState();
		void updateCurrentState(byte* cmd);
    	int idx2cmd[11];
		msgToPC msg_to_pc;
		STATES_IDX cmd2idx[8]; 
	private:
    RMDmotor rm;
		bool needToTriggerEnter;
		STATES_IDX currentState;
    	STATES_IDX nextState;
		State** states;
		unsigned long stateChangeTime;
};

#endif
