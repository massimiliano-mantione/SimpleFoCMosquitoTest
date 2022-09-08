/////////////////////////
// MOSQUITO BOARD
// SELECT STM32G031K8Ux
#include <Arduino.h>
#include <SimpleFOC.h>
#include <RTTStream.h>

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
BLDCMotor motor = BLDCMotor(16, 6.9, 480);
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM1, PWM2, PWM3, DRVENABLE); //STM32G031K8Ux + MOSQUITO
float target_velocity = 1.0;
//loop delay debug print command variables
uint32_t txDlyPrint = 100; // milli-seconds print delay counter
uint32_t lastprint = 0; // temp delay counter variable

#define SENSOR1_CS PA4 // some digital pin that you're using as the nCS pin

#define SENSOR1_PWM PC6
//MagneticSensorSC60228 sensor1(SENSOR1_CS);
MagneticSensorPWM sensor1 = MagneticSensorPWM(SENSOR1_PWM, 1, 976);

void doPWM(){sensor1.handlePWM();}

RTTStream rtt;

HardwareSerial Serial1(serial1_rx_pin, serial1_tx_pin);
void setup() {
  pinMode(DRVFAULT, INPUT); // Fault pin
  pinMode(DRVRESET, OUTPUT); // Reset pin
  digitalWrite(DRVRESET, HIGH); // Default low
  Serial1.begin(115200);
  Serial1.println("Motor ready.");
  _delay(500);
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 15000;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 4; // [V]
  motor.velocity_limit = 5; // [rad/s] cca 50rpm
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();

  sensor1.init();
  sensor1.enableInterrupt(doPWM);
}
void loop() {
  sensor1.update();
  if (millis() / txDlyPrint != lastprint) // send every txDlyPrint
  {
    lastprint = millis() / txDlyPrint;
    // display something to the terminal
    //Serial1.print("something");
    //Serial1.println("");
    rtt.print("enc=");
    rtt.print(sensor1.getAngle());
    rtt.print(" ,v=");
    rtt.println(target_velocity);
  }
  motor.move(target_velocity);

  if (rtt.available()) {
    target_velocity = rtt.parseFloat();
    //rtt.flush();
  }
}
////////////////////