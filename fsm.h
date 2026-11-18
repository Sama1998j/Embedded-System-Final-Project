#ifndef FSM_H
#define FSM_H

#include "parser.h"

void fsm_init(void);
void fsm_emergency_from_isr(void);   // emergency stop from INT1
void fsm_request_toggle(void);       // Halted/Moving toggle
void fsm_update(pc_reference ref[], int distance, double yaw);
void fsm_set_threshold(int cm);
void fsm_emergency_clear(void);

#endif
