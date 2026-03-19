#ifndef __ROBOT_MOTOR_H__
#define __ROBOT_MOTOR_H__

#define TMR_57_P       // Axis 5, Axis 6, Axis 7
// #define TMR_85_F
#define TMR_100_P   // Axis 3, Axis 4

// #define TBM_7615
#define TBM_12913_P   // Axis 1, Axis 2
// #define TBM_12941


#ifdef TMR_57_P

#define TMR_57_RATED_VOLTAGE    48
#define TMR_57_RATED_CURRENT    2.6
#define TMR_57_PEAK_CURRENT     5.3

#define TMR_57_MIN_VOLTAGE      24
#define TMR_57_VOLTAGE_LIMIT    48
#define TMR_57_INDUCTANCE       0.0003 // Stator inductance (H)
#define TMR_57_RESISTANCE       0.47 // 470mΩ  -> 520 mΩ measured in CCS
#define TMR_57_POLE_PAIR        10 // 20P -> 10PP (Pole Pair)

#define TMR_57_FREQ             3500 // Under 3500 RPM stable operation

#endif



#ifdef TMR_100_P

#define TMR_100_INDUCTANCE     0.000174
#define TMR_100_RESISTANCE     0.045 // 35mΩ
#define TMR_100_VOLTAGE_LIMIT  48
#define TMR_100_RATED_CURRENT  12.5
#define TMR_100_POLE_PAIR      10

#endif



#ifdef TBM_12913_P

#define TBM_12913_RATED_VOLTAGE   48
#define TBM_12913_RATED_CURRENT   12.5
#define TBM_12913_PEAK_CURRENT    57
#define TBM_12913_MAX_RPM         2505
#define TBM_12913_VOLTAGE_LIMIT   24
#define TBM_12913_INDUCTANCE      0.000082
#define TBM_12913_RESISTANCE      0.098 // 25'c
#define TBM_12913_POLE_PAIR       6





// #else if TBM_12913

#define TBM_12913_MAX_VOLTAGE   50


#endif


#endif
