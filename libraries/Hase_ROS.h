#ifndef MBED_HASE_ROS_H
#define MBED_HASE_ROS_H

#include <mbed.h>
#include <Hase.h>

#include <ros.h>
#include <hase_msgs/Feedback.h>
#include <hase_msgs/Drive.h>

#if defined USE_IMU
#include <sensor_msgs/Imu.h>
#endif

// Rate at which feedback is sent
#define FEEDBACK_RATE       10     // Hz
#define FEEDBACK_INTERVAL   1.0 / FEEDBACK_RATE

Hase robot;
Ticker feedback;

hase_msgs::Feedback feedback_msg;                       // Publisher for Feedback data.
ros::Publisher pub_feedback("feedback", &feedback_msg);
void sendFeedback();

void cmdDriveCallback(const hase_msgs::Drive& msg);
ros::Subscriber<hase_msgs::Drive> sub_cmd_drive("cmd_drive", &cmdDriveCallback);

#if defined TARGET_PAC_F401RB
DigitalOut blue_led(LED2);
DigitalOut green_led(LED3);
#else
DigitalOut led2(LED2);
DigitalOut led3(LED3);
#endif // TARGET_PAC_F401RB

#if defined TARGET_PAC_F401RB
// Create the ROS node handle
class PAC_F401RB : public MbedHardware
{
public:
  PAC_F401RB(): MbedHardware(SERIAL2_TX, SERIAL2_RX, 57600) {};
};
ros::NodeHandle_<PAC_F401RB> nh;
#else
ros::NodeHandle nh; // Create the ROS node handle
#endif

#if defined USE_IMU
// Rate at which imu data is sent
#define IMU_RATE       10     // Hz
#define IMU_INTERVAL   1.0 / IMU_RATE

Ticker imu;

sensor_msgs::Imu imu_msg;   // Publisher for yaw speed
ros::Publisher pub_imu("imu", &imu_msg);

double cov[9] = {0.0003, 0.0, 0.0, 0.0, 0.0003, 0.0, 0.0, 0.0, 0.0003};

void sendImu();
#endif

void sendFeedback()
{

  feedback_msg.header.stamp = nh.now();

  // Left Driver
  feedback_msg.drivers[hase_msgs::Drive::LEFT].duty_cycle = robot.getDutyCycle(robot.LEFT_MOTOR);
  feedback_msg.drivers[hase_msgs::Drive::LEFT].measured_velocity = robot.getWheelSpeed(robot.LEFT_WHEEL);
  feedback_msg.drivers[hase_msgs::Drive::LEFT].measured_travel = robot.getWheelTravel(robot.LEFT_WHEEL);

  // Right Driver
  feedback_msg.drivers[hase_msgs::Drive::RIGHT].duty_cycle = robot.getDutyCycle(robot.RIGHT_MOTOR);
  feedback_msg.drivers[hase_msgs::Drive::RIGHT].measured_velocity = robot.getWheelSpeed(robot.RIGHT_WHEEL);
  feedback_msg.drivers[hase_msgs::Drive::RIGHT].measured_travel = robot.getWheelTravel(robot.RIGHT_WHEEL);

  pub_feedback.publish(&feedback_msg);
}

void cmdDriveCallback(const hase_msgs::Drive& msg)
{
  float left_motor_velocity = msg.drivers[msg.LEFT]; // rad/s
  float right_motor_velocity = msg.drivers[msg.RIGHT]; // rad/s

  robot.info("Wheel Speeds to: L=%.2f, R=%.2f", left_motor_velocity, right_motor_velocity);

  robot.setSpeeds(left_motor_velocity, right_motor_velocity);
}

#if defined USE_IMU
void sendImu()
{
  robot.updateImu();

  // Fill header
  /*imu_msg.header.frame_id = "imu";

  // Fill quaternion orientation
  imu_msg.orientation.x = 0.0; //robot.imu.q[0];
  imu_msg.orientation.y = 0.0; //robot.imu.q[1];
  imu_msg.orientation.z = 0.0; //robot.imu.q[2];
  imu_msg.orientation.w = 0.0; //robot.imu.q[3];
  imu_msg.orientation_covariance[0] = 0.0003;
  imu_msg.orientation_covariance[4] = 0.0003;
  imu_msg.orientation_covariance[8] = 0.0003;

  // Fill angular velocities
  imu_msg.angular_velocity.x = robot.getRollSpeed();
  imu_msg.angular_velocity.y = robot.getPitchSpeed();
  imu_msg.angular_velocity.z = robot.getYawSpeed();
  imu_msg.angular_velocity_covariance[0] = 0.0003;
  imu_msg.angular_velocity_covariance[4] = 0.0003;
  imu_msg.angular_velocity_covariance[8] = 0.0003;

  // Fill linear accelerations
  imu_msg.linear_acceleration.x = robot.getXAccel();
  imu_msg.linear_acceleration.y = robot.getYAccel();
  imu_msg.linear_acceleration.z = robot.getZAccel();
  imu_msg.linear_acceleration_covariance[0] = 0.0003;
  imu_msg.linear_acceleration_covariance[4] = 0.0003;
  imu_msg.linear_acceleration_covariance[8] = 0.0003;

  pub_imu.publish(&imu_msg);*/
}
#endif // USE_IMU
#endif // MBED_HASE_ROS_H
