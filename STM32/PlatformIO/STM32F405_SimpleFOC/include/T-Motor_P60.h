#ifndef __MOTOR_P60_H__
#define __MOTOR_P60_H__

#define KV170

#ifdef KV170

#define KV170_MAX_VOLTAGE   58
#define KV170_MIN_VOLTAGE   24
#define KV170_VOLTAGE_LIMIT 48
#define KV170_PEAK_CURRENT  38.0f // 60A Continuous 
#define KV170_LOW_SPEED_CURRENT  20.0f // 
#define KV170_INDUCTANCE    -12345.0f // Not set
#define KV170_RESISTANCE    0.08 // 80mΩ
#define KV170_POLE_PAIR     14





#elif KV340

#define KV340_MAX_VOLTAGE   33
#define KV340_MIN_VOLTAGE   22
#define KV340_VOLTAGE_LIMIT 24
#define KV340_RESISTANCE    35 // 35mΩ
#define KV340_POLE_PAIR     14


#endif


#endif
