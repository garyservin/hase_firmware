#include "Hase.h"
#include <stdarg.h>

Hase::Hase():
#if defined DEBUG_ENABLED
  _debug(DBG_TX, DBG_RX),
#endif
  _lmotor(PWM_L, FWD_L, REV_L),
  _rmotor(PWM_R, FWD_R, REV_R),
  _lqei(QEIA_L, QEIB_L, NC, cprEncoder, QEI::X4_ENCODING),
  _rqei(QEIA_R, QEIB_R, NC, cprEncoder, QEI::X4_ENCODING),
  _lpid(Kc1, Ti1, Td1, PID_INTERVAL),
  _rpid(Kc1, Ti1, Td1, PID_INTERVAL),
#if defined USE_IMU
#if defined TARGET_PAC_F401RB
  spi(SPI_MOSI, SPI_MISO, SPI_SCK),
  _imu(spi, SPI_CS),
#else
  _imu(IMU_SDA, IMU_SCL),
#endif
#endif
  _sysLed(SYSLED)
{

#if defined DEBUG_ENABLED
  _debug.baud(DEBUG_BAUDRATE);
#endif

  _lpid.setInputLimits(-10500.0, 10500.0);
  _lpid.setOutputLimits(-1.0, 1.0);
  _lpid.setBias(0.0);
  _lpid.setMode(AUTO_MODE);
  _lpid.setSetPoint(0.0);

  _rpid.setInputLimits(-10500.0, 10500.0);
  _rpid.setOutputLimits(-1.0, 1.0);
  _rpid.setBias(0.0);
  _rpid.setMode(AUTO_MODE);
  _rpid.setSetPoint(0.0);

  _lmotor.speed(0.0);
  _rmotor.speed(0.0);

#if defined USE_IMU && defined TARGET_PAC_F401RB
  _imu.set_gyro_scale(BITS_FS_2000DPS);
  wait(0.1);
  _imu.set_acc_scale(BITS_FS_16G);
  wait(0.1);
  _imu.AK8963_calib_Magnetometer();
  wait(0.1);
#endif

  wait(0.2);

  _readEncoderTicker.attach(this, &Hase::readEncoder, PID_INTERVAL);
  _sysLedTicker.attach(this, &Hase::sysBlink, SYSLED_INTERVAL);
  _lastMotorCommand.start();
}

// @params lspeed: speed in ticks per second
// @params rspeed: speed in ticks per second
void Hase::setSpeedsTicks(float lspeed, float rspeed)
{
  _lpid.setSetPoint(lspeed);
  _lpid.setProcessValue(_lpps);
  _lmotor.speed(_lpid.compute());

  _rpid.setSetPoint(rspeed);
  _rpid.setProcessValue(_rpps);
  _rmotor.speed(_rpid.compute());

  // Reset motor timeout timer
  _lastMotorCommand.reset();
}

// @params lspeed: speed in meters per second
// @params rspeed: speed in meters per second
void Hase::setLinearSpeeds(float lspeed, float rspeed)
{
  double vl = Hase::speedLinearToTicks(lspeed);
  double vr = Hase::speedLinearToTicks(rspeed);
  Hase::debug("setSpeeds(m->ticks): %.2f->%.2f, %.2f->%.2f", lspeed, vl, rspeed, vr);
  Hase::setSpeedsTicks(vl, vr);
}

// @params lspeed: speed in radians per second
// @params rspeed: speed in radians per second
void Hase::setSpeeds(float lspeed, float rspeed)
{
  double vl = Hase::speedToTicks(lspeed);
  double vr = Hase::speedToTicks(rspeed);
  Hase::debug("setSpeeds(rad->ticks): %.2f->%.2f, %.2f->%.2f", lspeed, vl, rspeed, vr);
  Hase::setSpeedsTicks(vl, vr);
}

float Hase::getDutyCycle(Motors motor)
{
  if (motor == LEFT_MOTOR)
  {
    return _lduty_cycle;
  }
  else
  {
    return _rduty_cycle;
  }
}

int Hase::getPulses(Wheel wheel)
{
  if (wheel == LEFT_WHEEL)
  {
    return _lqei.getPulses() / gearRatio;
  }
  else
  {
    return _rqei.getPulses() / gearRatio;
  }
}

int Hase::getPulses(Motors motor)
{
  if (motor == LEFT_MOTOR)
  {
    return _lqei.getPulses();
  }
  else
  {
    return _rqei.getPulses();
  }
}

int Hase::getRevolutions(Wheel wheel)
{
  if (wheel == LEFT_WHEEL)
  {
    return _lqei.getRevolutions();
  }
  else
  {
    return _rqei.getRevolutions();
  }
}

int Hase::getCurrentState(Wheel wheel)
{
  if (wheel == LEFT_WHEEL)
  {
    return _lqei.getCurrentState();
  }
  else
  {
    return _rqei.getCurrentState();
  }
}

double Hase::getWheelLinearSpeed(Wheel wheel)
{
  if (wheel == LEFT_WHEEL)
  {
    return _lpps / ticksPerMeter;
  }
  else
  {
    return _rpps / ticksPerMeter;
  }
}

double Hase::getWheelSpeed(Wheel wheel)
{
  if (wheel == LEFT_WHEEL)
  {
    return _lpps / ticksPerRadian;
  }
  else
  {
    return _rpps / ticksPerRadian;
  }
}

double Hase::getWheelTravel(Wheel wheel)
{
  if (wheel == LEFT_WHEEL)
  {
    return _lqei.getPulses() / ticksPerRadian;
  }
  else
  {
    return _rqei.getPulses() / ticksPerRadian;
  }
}

int Hase::getPulsesPerSecond(Wheel wheel)
{
  if (wheel == LEFT_WHEEL)
  {
    return _lpps / gearRatio;
  }
  else
  {
    return _rpps / gearRatio;
  }
}

int Hase::getPulsesPerSecond(Motors motor)
{
  if (motor == LEFT_MOTOR)
  {
    return _lpps;
  }
  else
  {
    return _rpps;
  }
}

void Hase::resetEncoder(Wheel wheel)
{
  if (wheel == LEFT_WHEEL)
  {
    _lqei.reset();
  }
  else
  {
    _rqei.reset();
  }
}

int Hase::getRPM(Wheel wheel)
{
  if (wheel == LEFT_WHEEL)
  {
    return ((long) _lpps * 60) / (cpr);
  }
  else
  {
    return ((long) _rpps * 60) / (cpr);
  }
}

int Hase::getRPM(Motors motor)
{
  if (motor == LEFT_MOTOR)
  {
    return ((long) _lpps * 60) / _lqei.getPulsesPerRevolution();
  }
  else
  {
    return ((long) _rpps * 60) / _rqei.getPulsesPerRevolution();
  }
}

// Harcoded access to te qei interface
void Hase::readEncoder()
{
  int currentPulses = 0;

  currentPulses = _lqei.getPulses();
  _lpps = (currentPulses - _lpulses) * PID_RATE;
  _lpulses = currentPulses;

  _lpid.setProcessValue(_lpps);
  _lduty_cycle = _lpid.compute();
  _lmotor.speed(_lduty_cycle);

  currentPulses = _rqei.getPulses();
  _rpps = (currentPulses - _rpulses) * PID_RATE;
  _rpulses = currentPulses;

  _rpid.setProcessValue(_rpps);
  _rduty_cycle = _rpid.compute();
  _rmotor.speed(_rduty_cycle);

  //Hase::debug("%i\t%.2f\t\t|\t%i\t%.2f\t", _lpulses, Hase::ticksToSpeed(_lpps), _rpulses, Hase::ticksToSpeed(_rpps));
  //Hase::debug("%i\t%i\t\t|\t%i\t%i\t", _lpulses, _lpps, _rpulses, _rpps);
  //Hase::debug("%i, %i", _lpulses, _rpps);

  // Motor timeout
  if (_lastMotorCommand >= AUTO_STOP_INTERVAL)
  {
    Hase::setSpeedsTicks(0, 0);
    Hase::debug("Timeout");
  }
}

/* Convert meters per second to ticks per second */
int Hase::speedLinearToTicks(float v)
{
  return int(v * ticksPerMeter);// * ODOM_INTERVAL);
}

/* Convert Radians per second to ticks per second */
int Hase::speedToTicks(float v)
{
  return int(v * ticksPerRadian);// * ODOM_INTERVAL);
}

/* Convert ticks per second to meters per second */
float Hase::ticksToSpeed(int ticks)
{
  return ticks / ticksPerMeter;
}

/* Blink system LED */
void Hase::sysBlink()
{
  _sysLed = !_sysLed;
}

#if defined USE_IMU
void Hase::updateImu()
{
#if defined TARGET_PAC_F401RB
  _imu.read_all();
#else
  _imu.readDataMPU6050();
#endif
}

float Hase::getXAccel()
{
#if defined TARGET_PAC_F401RB
  return _imu.accelerometer_data[0];
#else
  return _imu.ax;
#endif
}

float Hase::getYAccel()
{
#if defined TARGET_PAC_F401RB
  return _imu.accelerometer_data[1];
#else
  return _imu.ay;
#endif
}

float Hase::getZAccel()
{
#if defined TARGET_PAC_F401RB
  return _imu.accelerometer_data[2];
#else
  return _imu.az;
#endif
}

float Hase::getRollSpeed()
{
#if defined TARGET_PAC_F401RB
  return _imu.gyroscope_data[0];
#else
  return _imu.gx;
#endif
}

float Hase::getPitchSpeed()
{
#if defined TARGET_PAC_F401RB
  return _imu.gyroscope_data[1];
#else
  return _imu.gy;
#endif
}

float Hase::getYawSpeed()
{
#if defined TARGET_PAC_F401RB
  return _imu.gyroscope_data[2];
#else
  return _imu.gz;
#endif
}

float Hase::getTemperature()
{
#if defined TARGET_PAC_F401RB
  return _imu.Temperature;
#else
  return _imu.temperature;
#endif
}

#if defined TARGET_PAC_F401RB
float Hase::getXMagetometer()
{
  return _imu.Magnetometer[0];
}

float Hase::getYMagetometer()
{
  return _imu.Magnetometer[1];
}

float Hase::getZMagetometer()
{
  return _imu.Magnetometer[2];
}
#endif
#endif


// Debug method
int Hase::debug(const char *fmt, ...)
{
#if defined DEBUG_ENABLED
  char buffer[200] = {0};
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, 200, fmt, args);
  va_end(args);

  return _debug.printf("[DEBUG] %s\r\n", buffer);
#endif
  return 0;
}

// INFO method
int Hase::info(const char *fmt, ...)
{
#if defined INFO_ENABLED
  char buffer[200] = {0};
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, 200, fmt, args);
  va_end(args);

  return _debug.printf("[INFO] %s\r\n", buffer);
#endif
  return 0;
}
