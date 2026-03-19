#include <Arduino.h>
#include <SimpleFOC.h>
#include "T-Motor_P60.h"
#include "robot_Motor.h"


#define DEBUG

#ifdef DEBUG
#include <OneButton.h>
#define LED_BUILTIN PB2 // LED on board


#endif

// magnetic sensor instance - SPI
#define CS       PB2  // PA15?
#define SPI_SCK  PC10 // SPI3_SCK
#define SPI_MISO PC11 // SPI3_MISO
#define SPI_MOSI PC12 // SPI3_MOSI


// SHUNT SENSING
#define M0_IA _NC // Seulement 2 mesures de courant B&C, A = pas dispo lui.
#define M0_IB PC0
#define M0_IC PC1

// Odrive M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15

// M1 & M2 common enable pin
#define EN_GATE PB12

//Pole pair
#define PP 14  // 28P /2  = 14


//Temp
#define M0_TEMP PC5

// GPIO_3, GPIO_4
#define USART2_TX PA2 // TX GPIO_3
#define USART2_RX PA3 // RX GPIO_4


#define   _MON_TARGET 0b1000000  // monitor target value
#define   _MON_VOLT_Q 0b0100000  // monitor voltage q value
#define   _MON_VOLT_D 0b0010000  // monitor voltage d value
#define   _MON_CURR_Q 0b0001000  // monitor current q value - if measured
#define   _MON_CURR_D 0b0000100  // monitor current d value - if measured
#define   _MON_VEL    0b0000010  // monitor velocity value
#define   _MON_ANGLE  0b0000001  // monitor angle value

struct MotorParameters {
  const char* name;
  float resistance;
  float inductance;
  float pole_pairs;
  float voltage_limit;
  float current_limit;
  float default_velocity;
};

MotorParameters P60_KV170_Motor = {
  .name = "P60 KV170",
  .resistance    = KV170_RESISTANCE,
  .inductance    = KV170_INDUCTANCE,
  .pole_pairs    = KV170_POLE_PAIR,
  .voltage_limit = KV170_VOLTAGE_LIMIT,
  .current_limit = KV170_LOW_SPEED_CURRENT,
  .default_velocity = 1.0f
};

MotorParameters TMR_57_Motor = {
  .name = "TMR 57",
  .resistance    = TMR_57_RESISTANCE,
  .inductance    = TMR_57_INDUCTANCE,
  .pole_pairs    = TMR_57_POLE_PAIR,
  .voltage_limit = TMR_57_VOLTAGE_LIMIT,
  .current_limit = TMR_57_RATED_CURRENT,
  .default_velocity = 0.0f
};

MotorParameters TMR_100_Motor = {
  .name = "TMR 100",
  .resistance    = TMR_100_RESISTANCE,
  .inductance    = TMR_100_INDUCTANCE,
  .pole_pairs    = TMR_100_POLE_PAIR,
  .voltage_limit = TMR_100_VOLTAGE_LIMIT,
  .current_limit = TMR_100_RATED_CURRENT,
  .default_velocity = 0.0f
};


MotorParameters TBM_12913_Motor = {
  .name = "TBM 12913",
  .resistance    = TBM_12913_RESISTANCE,
  .inductance    = TBM_12913_INDUCTANCE,
  .pole_pairs    = TBM_12913_POLE_PAIR,
  .voltage_limit = TBM_12913_VOLTAGE_LIMIT,
  .current_limit = TBM_12913_RATED_CURRENT,
  .default_velocity = 0.0f
};




MotorParameters motors[] = {P60_KV170_Motor, TMR_57_Motor, TMR_100_Motor, TBM_12913_Motor };
int activeMotorType = 3; // Default motor selection
enum MotorType { P60_KV170, TMR_57, TMR_100, TBM_12913 };


uint32_t currentMillis = 0;
uint32_t previousMillis[4] = {0,};
int counter = 0;
bool ledState = false;  

BLDCMotor motor = BLDCMotor(PP);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC); //shunt resistor 500uOhm, gain 10x
MagneticSensorSPI sensor = MagneticSensorSPI(CS, 14, 0x3FFF);

HardwareSerial Serial2(USART2_RX, USART2_TX);
SPIClass SPI_3(SPI_SCK, SPI_MISO, SPI_MOSI);

PhaseCurrent_s current;
float current_magnitude;
float memory_max_current = motor.current_limit;
float target_position = 0;
float target_velocity = 0.0f; // rad/s   20rad/s -> ~190rpm | 3000rpm = 314rad/s
float last_target_velocity = 0.0f;





#ifdef DEBUG
OneButton button;

void buttonClick() {
  Serial.println("Button Clicked!");
}
#endif

void printActiveMotorStatus();
void applyProfile(int idx);

Commander command = Commander(Serial);   // Serial for command interface
// Commander command = Commander(Serial2);   // Serial for command interface
void doMotor(char* cmd) {
  int idx = atoi(cmd);
  if(idx < 0 || idx >= sizeof(motors)/sizeof(MotorParameters)) {
    Serial.println("Invalid motor index");
    return;
  }
  if(activeMotorType == idx) {
    Serial.println("Motor already selected");
    return;
  }

  applyProfile(idx);
}

// void doTarget(char* cmd) { command.scalar(&target_position, cmd); }
// void doVoltageLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
// void doCurentLimit(char* cmd) { command.scalar(&motor.current_limit, cmd); }
// void doVelocity(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }
void doVelocity(char* cmd) { command.scalar(&target_velocity, cmd); }



void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  Serial.println("SimpleFOC STM32F405 Example");
  

#ifdef DEBUG
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(PC13, INPUT_PULLDOWN); // Button
  button.setup(PC13, INPUT_PULLDOWN, false);
  button.attachClick(buttonClick);
#endif
  
  SimpleFOCDebug::enable(&Serial);
  driver.voltage_power_supply = 48.0f;
  driver.voltage_limit = 48.0f;
  // driver.pwm_frequency = 20000; //20kHz
  

  if(!driver.init())
  {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);
  motor.voltage_limit = motors[activeMotorType].voltage_limit;
  motor.current_limit = motors[activeMotorType].current_limit; // continuous 60A
  motor.velocity_limit = 500; // limit voltage change rate
  motor.pole_pairs = motors[activeMotorType].pole_pairs;
  // motor.phase_inductance = TMR_57_INDUCTANCE ; // -> FOC tuning
  motor.phase_resistance = motors[activeMotorType].resistance; // ohm
  motor.controller = MotionControlType::velocity_openloop; //  MAX 136 rad/s -> 1300rpm


  // motor.useMonitoring(Serial);
  // motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; 
  // motor.monitor_downsample = 100;

  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // command.add('T', doTarget, "target angle");
  // command.add('L', doLimit,  "voltage limit");
  // command.add('V', doLimit,  "movement velocity");

  command.add('V', doVelocity,  "movement velocity");

  Serial.printf("%d pole pairs motor initialized\r\n", motor.pole_pairs);
  Serial.printf("%s Motor Ready!\r\n", motors[activeMotorType].name);
  Serial.println("Set target position [rad]");
  
  _delay(1000);
  // sensor.init(&SPI_3);
  // motor.linkSensor(&sensor);

  // driver.pwm_frequency = 20000;        // 20kHz
  // driver.voltage_power_supply = 24.0f; // 24V
  // driver.voltage_limit = 56.0f;        // 56V
  
  // driver.init();
  // motor.linkDriver(&driver);

  // motor.torque_controller = TorqueControlType::foc_current; // Use FOC current control
  // motor.controller = MotionControlType::angle; // Use angle control
  // motor.foc_modulation = FOCModulationType::SinePWM; // Use Sine PWM modulation 
  // motor.modulation_centered = true; // Center modulation

  // motor.PID_velocity.P = 2.0f; // Velocity controller P gain
  // motor.PID_velocity.I = 30.0f; // Velocity controller I gain
  // motor.PID_velocity.D = 0.001f; // Velocity controller D gain
  // motor.PID_velocity.output_ramp = 1000.0f; // Velocity controller output ramp
  // motor.LPF_velocity.Tf = 0.01f; // Velocity controller low pass filter time

  
  // motor.init();

  // currentSense.linkDriver(&driver);
  // if(currentSense.init() == 0) {
  //   Serial.println("Current sense init failed!");
  // } else {
  //   Serial.println("Current sense init success!");
  // }

  
  // motor.linkCurrentSense(&currentSense);
  // currentSense.gain_a *=-1;
  // currentSense.gain_b *=-1;
  // currentSense.gain_c *=-1;
  // currentSense.skip_align = true; // Skip the alignment phase

  // motor.initFOC();

  command.decimal_places = 4; // Set number of decimal places for command output 출력형식
  command.add('M', doMotor, "Select motor Profile ==> M0, M1, M2, M3");
  motor.target = 0;
  // if (motor.motor_status != 4) { // 0 - fail initFOC
  //   Serial.println("ERROR:" + String(motor.motor_status));
  //   //return;
  // }

  // delay(1000);
}

void loop() {
  currentMillis = millis();

  

  if(currentMillis - previousMillis[0] >= 500) {
    previousMillis[0] = currentMillis;
    counter++;
    // Serial.printf("[%d] %d\r\n", counter, HSE_VALUE);
#ifdef DEBUG
    digitalWrite(LED_BUILTIN, ledState);
#endif
    ledState = !ledState;
  
  }
  else if(currentMillis - previousMillis[1] >= 10) {
    previousMillis[1] = currentMillis;
    button.tick(); // Check button state
  }
  else if(currentMillis - previousMillis[2] >= 1) {
    previousMillis[2] = currentMillis;
    if(Serial.available()){
      char c = Serial.read();
      Serial.printf("Received: %c\r\n", c);
      switch (c)
      {
      case 'a':
        target_velocity += 1.0f;
        Serial.printf("Target velocity: %.2f rad/s\r\n", target_velocity);
        break;
      case 'b':
        target_velocity -= 1.0f;
        Serial.printf("Target velocity: %.2f rad/s\r\n", target_velocity);
        break;
      case 'd':
        Serial.printf("Motor Disable\r\n");
        target_velocity = 0.0f;
        motor.disable();
        break;
      case 'e':
        Serial.printf("Motor Enable %f \r\n", motor.phase_resistance);
        motor.phase_resistance = TMR_100_RESISTANCE;
        Serial.printf("Change Resistance %f \r\n", motor.phase_resistance);
        motor.init();
        motor.enable();
        // target_velocity = 20.0f;
        break;
      case 'h':
        motor.current_limit = 5.3f;
        Serial.printf("Current limit set to %.2f A\r\n", motor.current_limit);
        break;
      case 'l':
        motor.current_limit = 2.6f;
        Serial.printf("Current limit set to %.2f A\r\n", motor.current_limit);
        break;
      case 'r':
        Serial.print("System Reset\r\n");
        NVIC_SystemReset();
        break;
      case 'p':
        applyProfile((activeMotorType + 1) % (sizeof(motors)/sizeof(MotorParameters)));
        printActiveMotorStatus();
        break;
      default:
        break;
      }
    }
  }
  else
  {
    motor.move(target_velocity);
    // command.run();
    if (fabs(target_velocity - last_target_velocity) > 0.0f) {
      // motor.move(target_velocity);
      Serial.printf("Move to velocity: %.2f rad/s \r\n", target_velocity);
      last_target_velocity = target_velocity;

      if(last_target_velocity == 0.0f)
      {
        motor.disable();
        Serial.printf("Motor Disabled\r\n");
      }
      else
      {
        motor.enable();
        Serial.printf("Motor Enabled\r\n");
      }
    }
  }

}

void printActiveMotorStatus()
{
  Serial.printf("Active Motor Type: %s\r\n", motors[activeMotorType].name);
  Serial.printf("Phase Resistance: %.2f Ohm\r\n", motor.phase_resistance);
  Serial.printf("Phase Inductance: %.6f H\r\n", motor.phase_inductance);
  Serial.printf("Pole Pairs: %d\r\n", motor.pole_pairs);
  Serial.printf("Voltage Limit: %.2f V\r\n", motor.voltage_limit);
  Serial.printf("Current Limit: %.2f A\r\n", motor.current_limit);
}

void applyProfile(int idx)
{
  motor.target = 0;                 // 안전
  delay(50);

  motor.disable();
  motor.pole_pairs       = motors[idx].pole_pairs;
  motor.phase_resistance = motors[idx].resistance;
  // motor.phase_inductance = motors[idx].inductance;
  motor.current_limit    = motors[idx].current_limit;
  motor.voltage_limit    = motors[idx].voltage_limit;
  motor.init();
  activeMotorType = idx;
  
  target_velocity = motors[idx].default_velocity;
  Serial.printf("Applied motor profile: %s %d\r\n", motors[idx].name, target_velocity);
}