#ifndef __chaoshengbo_H
#define __chaoshengbo_H

extern uint32_t startTime;
extern uint32_t endTime;
extern uint8_t captureState;
extern float distance_cm;
extern uint8_t distance_updated;
extern const uint16_t DISTANCE_THRESHOLD_CM;

void choashengbo(void);
void Trig_Pulse(void);



#endif
