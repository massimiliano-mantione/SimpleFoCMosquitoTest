/////////////////////////
// MOSQUITO BOARD
// SELECT STM32G031K8Ux

#include <Arduino.h>
#include <SimpleFOC.h>
#include <RTTStream.h>
#include <SPI.h>
#include <SimpleFOCDrivers.h>
#include <encoders/sc60228/MagneticSensorSC60228.h>

// TX/RX SERIAL PINS
#define serial1_tx_pin PA2
#define serial1_rx_pin PA3

// DRIVER PINS
#define PWM1 PA8
#define PWM2 PA11
#define PWM3 PB3
#define DRVENABLE PA15
#define DRVFAULT PB4
#define DRVRESET PB5
BLDCMotor motor = BLDCMotor(8, 6.9 /*, 480*/);  // 8 pole pairs, 6.9, 480kV estimated (todo: bump up to +70% or measure?)
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM1, PWM2, PWM3, DRVENABLE); //STM32G031K8Ux + MOSQUITO
float target = 0.0;

#define SENSOR1_CS PA4 // some digital pin that you're using as the nCS pin
SPIClass spi1(PA7, PA6, PA5);
MagneticSensorSC60228 sensor(SENSOR1_CS, 2048);

// #define SENSOR1_PWM PC6
// MagneticSensorPWM sensor = MagneticSensorPWM(SENSOR1_PWM, 20, 976);  // don't use this, it's very noisy on PWM
// void doPWM(){sensor.handlePWM();}

RTTStream rtt;

// HardwareSerial Serial1(serial1_rx_pin, serial1_tx_pin);

void serialLoop() {
  static String rx;
  while(rtt.available()){
    char in = (char) rtt.read();
    rx += in;
    if (in=='\n') {
      rtt.println("parsing rx");
      target = rx.toFloat();
      rtt.print("Target = "); rtt.println(target);
      rx = "";
    }
  }
}

void setup() {
  pinMode(DRVFAULT, INPUT); // Fault pin
  pinMode(DRVRESET, OUTPUT); // Reset pin
  digitalWrite(DRVRESET, HIGH); // Default low
  
  rtt.println("Starting setup");
  _delay(500);
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 20000;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 3; // [V]
  motor.velocity_limit = 20; // [rad/s] ~191rpm
  motor.torque_controller  = TorqueControlType::voltage;

  motor.PID_velocity.P = 0.02;  // 0.02
  motor.PID_velocity.I = 0.2;  // 0.1
  motor.PID_velocity.D = 0.0001;
  motor.PID_velocity.output_ramp = 300;
  motor.LPF_velocity.Tf = 0.05;  // 50ms low pass filter
  motor.controller = MotionControlType::velocity;
  motor.voltage_sensor_align = 1; // used by closed loop
  motor.current_limit = 1;

  // for PWM sensor:
  // sensor.init();
  // sensor.enableInterrupt(doPWM);

  // SPI sensor interface:
  sensor.init(&spi1);

  motor.useMonitoring(rtt); 
  motor.linkSensor(&sensor);

  rtt.println("Setup complete");

  motor.init();
  //motor.initFOC();  // to get the two parameters below
  motor.initFOC(5.89, Direction::CW); 

  rtt.println("FOC Init complete");

  //rtt.println(motor.zero_electric_angle);
  //rtt.println(motor.sensor_direction);
  //while(1){} // to let you read those parameters 
}

void loop() {
  //motor.PID_velocity.P = target;

  motor.loopFOC();
  motor.move(target);

  motor.monitor();
  serialLoop();
}
////////////////////