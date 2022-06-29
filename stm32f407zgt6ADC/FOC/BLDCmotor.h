#ifndef BLDCMotor_H
#define BLDCMotor_H

#include "main.h"

typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;

extern long sensor_direction;
//extern float current_sp;
//extern float shaft_angle;
//extern float electrical_angle;
void Motor_init(void);
void Motor_initFOC(float zero_electric_offset, Direction _sensor_direction);
int alignSensor(void);
uint8_t setPhaseVoltage(float Uq, float Ud, float angle_el);
void move(float new_target);

float move_velocity(float new_target);
float move_position(float position_target);
//float loopFOCtest(uint8_t sec);
void loopFOCtest(void);
void loopFOC(void);
float move_position2(float OPP_shaft_angle);
float mymove(float new_target);

#endif


