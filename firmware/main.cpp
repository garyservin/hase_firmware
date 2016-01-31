#include <mbed.h>
#include <Hase_ROS.h>

int main()
{
#if defined TARGET_PAC_F401RB
  blue_led = 1;
  green_led = 1;
#endif

  nh.initNode();

  nh.subscribe(sub_cmd_drive);

  nh.advertise(pub_feedback);
  feedback.attach(&sendFeedback, FEEDBACK_INTERVAL);

#if defined USE_IMU
  nh.advertise(pub_imu);
  imu.attach(&sendImu, IMU_INTERVAL);
#endif

  robot.setSpeeds(0.0, 0.0);
  while (1)
  {
    nh.spinOnce();
    wait(0.01);
  }
}
