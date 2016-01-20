#ifndef MBED_HASE_H
#define MBED_HASE_H

#define USE_IMU

#include "mbed.h"
#include <Motor.h>
#include <QEI.h>
#include <PID.h>
#if defined USE_IMU
#if defined TARGET_PAC_F401RB
#include <MPU9250.h>
#else
#include <MPU6050.h>
#endif
#endif

#if defined TARGET_PAC_F401RB
// Motors
// Left Motor
#define PWM_L   D2
#define FWD_L   D3
#define REV_L   D4
// Right Motor
#define PWM_R   D5
#define FWD_R   D7
#define REV_R   D6

// Left Encoder
#define QEIA_L  D1  // Yellow
#define QEIB_L  D0  // White
// Right Encoder
#define QEIA_R  D8  // White
#define QEIB_R  D16 // Yellow

// Serial debug interface
#define DBG_TX   SERIAL2_TX
#define DBG_RX   SERIAL2_RX

#else

// Motors
// Left Motor
#define PWM_L   p21
#define FWD_L   p22
#define REV_L   p23
// Right Motor
#define FWD_R   p24
#define REV_R   p25
#define PWM_R   p26

// Left Encoder
#define QEIA_L  p15 // Yellow
#define QEIB_L  p14 // White
// Right Encoder
#define QEIA_R  p12 // White
#define QEIB_R  p13 // Yellow

// IMU
#define IMU_SDA p9
#define IMU_SCL p10

// Serial debug interface
#define DBG_TX   USBTX
#define DBG_RX   USBRX
#endif // TARGET_PAC_F401RB

#define SYSLED LED1

// DEBUG
//#define DEBUG_ENABLED
//#define INFO_ENABLED
#define DEBUG_BAUDRATE    57600


class Hase
{
public:
  // SYS led blinking rate
#define SYSLED_RATE     2     // Hz
#define SYSLED_INTERVAL 1.0 / SYSLED_RATE

  // Rate at which encoders are sampled and PID loop is updated
#define PID_RATE        30     // Hz
#define PID_INTERVAL    1.0 / PID_RATE

  // PID Parameters
  float Kc1 = 1.6;
  float Ti1 = 0.2;
  float Td1 = 0.0;

  // Define the robot paramters
  int cprEncoder = 64; // Encoder ticks per revolution for motor
  int gearRatio = 30; // Gear ratio for motor gear
  int cpr = cprEncoder * gearRatio; // Encoder ticks per revolution for the Pololu 30:1 motor (1920)
  float wheelDiameter = 0.123825; // meters
  float wheelTrack = 0.23; // meters
  float ticksPerMeter = cpr / (3.141592 * wheelDiameter); // ~4935.635851
  float ticksPerRadian = cpr / (6.283184); // ~305.57755431

  // Stop the robot if it hasn't received a movement command in this number of milliseconds
#define AUTO_STOP_INTERVAL 0.5

  typedef enum Wheel
  {

    LEFT_WHEEL,
    RIGHT_WHEEL

  } Wheel;

  typedef enum Motors
  {

    LEFT_MOTOR,
    RIGHT_MOTOR

  } Motors;

  /** Create a hase control interface
   */
  Hase();

  /** Set the speed of each wheel of the robot
   *
   * @param lspeed The speed of the left wheel in ticks per second
   * @param rspeed The speed of the right wheel in ticks per second
   */
  void setSpeedsTicks(float lspeed, float rspeed);

  /** Set the lineal speed of each wheel of the robot
   *
   * @param lspeed The speed of the left wheel in meters per second
   * @param rspeed The speed of the right wheel in meters per second
   */
  void setLinearSpeeds(float lspeed, float rspeed);

  /** Set the angular speed of each wheel of the robot
   *
   * @param lspeed The speed of the left wheel in radians per second
   * @param rspeed The speed of the right wheel in radians per second
   */
  void setSpeeds(float lspeed, float rspeed);

  /** Get the duty cycle for the PWM applied to the motor
   *
   * @param wheel The wheel to obtain the pulses from
   * @return The specified wheel's encoder pulses
   */
  float getDutyCycle(Motors motor);

  /** Get the count of pulses for the specified wheel of the robot
   *
   * @param motor The motor to obtain the duty cycle from
   * @return The specified motor's applied duty cycle
   */
  int getPulses(Wheel wheel);
  int getPulses(Motors motor);

  /** Get the pulses per revolution of the specified wheel of the robot
   *
   * @param wheel The wheel to obtain the pulses per revolution from
   * @return The specified wheel's pulses per revolution
   */
  int getPulsesPerSecond(Wheel wheel);
  int getPulsesPerSecond(Motors motor);

  /** Get the pulses per revolution of from a wheel
   *
   * @param wheel The wheel to obtain the pulses per revolution from
   * @return The converted RPM using the CPR from the specified wheel
   */
  int getRPM(Wheel wheel);
  int getRPM(Motors motor);

  /** Get the wheel's linear speed in meters per second
   *
   * @param wheel The wheel to obtain the speed from
   * @return The specified wheel's speed (linear) in m/s
   */
  double getWheelLinearSpeed(Wheel wheel);

  /** Get the wheel's angular speed in radians per second
   *
   * @param wheel The wheel to obtain the speed from
   * @return The specified wheel's speed (angular) in rad/s
   */
  double getWheelSpeed(Wheel wheel);

  /** Get the wheel's travel in radians
   *
   * @param wheel The wheel to obtain the travel from
   * @return The specified wheel's travel (angular) in rad
   */
  double getWheelTravel(Wheel wheel);

  /** Get the count of revolutions for the specified wheel of the robot
   *
   * @param wheel The wheel to obtain the revolutions from
   * @return The specified wheel's encoder revolutions
   */
  int getRevolutions(Wheel wheel);

  /** Convert lineal speed to ticks
   *
   * @param float the peed in meters per second to convert
   * @return The converted ticks
   */
  int speedLinearToTicks(float);

  /** Convert angular speed to ticks
   *
   * @param float the peed in radians per second to convert
   * @return The converted ticks
   */
  int speedToTicks(float);

  /** Convert ticks per second to meter per seconds
   *
   * @param int ticks per second to convert
   * @return The converted speed in meters per second
   */
  float ticksToSpeed(int ticks);

#if defined USE_IMU
  /** Update measures from IMU */
  void updateImu();

  /** Get x accel from IMU
   *
   * @return float with the x accelereation in gravity units
   */
  float getXAccel();

  /** Get y accel from IMU
   *
   * @return float with the y accelereation in gravity units
   */
  float getYAccel();

  /** Get z accel from IMU
   *
   * @return float with the z accelereation in gravity units
   */
  float getZAccel();

  /** Get roll speed from IMU
   *
   * @return float with the roll speed in degree/sec
   */
  float getRollSpeed();

  /** Get pitch speed from IMU
   *
   * @return float with the pitch speed in degree/sec
   */
  float getPitchSpeed();

  /** Get yaw speed from IMU
   *
   * @return float with the yaw speed in degree/sec
   */
  float getYawSpeed();

  /** Get temperature from IMU
   *
   * @return float with the temperature in degrres Celsius
   */
  float getTemperature();

#if defined TARGET_PAC_F401RB
  /** Get x magnetometer from IMU
   *
   * @return float with the x magnetic field
   */
  float getXMagetometer();

  /** Get y magnetometer from IMU
   *
   * @return float with the y magnetic field
   */
  float getYMagetometer();

  /** Get z magnetometer from IMU
   *
   * @return float with the z magnetic field
   */
  float getZMagetometer();
#endif
#endif

  int debug(const char *fmt, ...);

  int info(const char *fmt, ...);

private:
#if defined DEBUG_ENABLED
  Serial _debug;
#endif
  Motor _lmotor;
  Motor _rmotor;
  QEI _lqei;
  QEI _rqei;
  PID _lpid;
  PID _rpid;

#if defined USE_IMU
#if defined TARGET_PAC_F401RB
  SPI spi;
  mpu9250_spi _imu;
#else
  MPU6050 _imu;
#endif // TARGET_PAC_F401RB
#endif // USE_IMU

  DigitalOut _sysLed;

  Ticker _readEncoderTicker;
  Ticker _sysLedTicker;
  Timer _lastMotorCommand;

  int _lpulses;
  int _rpulses;

  int _lpps;
  int _rpps;

  float _lduty_cycle;
  float _rduty_cycle;

  /** Callback executed via the Ticker to read encoder satus
   */
  void readEncoder();

  /** Reset the specified encoder pulse count
   *
   * @param wheel The wheel which encoder's will be reseted
   */
  void resetEncoder(Wheel wheel);

  /** Get the current state of the specified wheel of the robot
   *
   * @param wheel The wheel to obtain the current state from
   * @return The specified wheel's current state
   */
  int getCurrentState(Wheel wheel);

  /** Led blink callback
   */
  void sysBlink();

};

#endif
