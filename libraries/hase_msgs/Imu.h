#ifndef _ROS_hase_msgs_Imu_h
#define _ROS_hase_msgs_Imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace hase_msgs
{

  class Imu : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Vector3 angular_velocity;
      geometry_msgs::Vector3 linear_acceleration;
      geometry_msgs::Vector3 magnetic_field;

    Imu():
      header(),
      angular_velocity(),
      linear_acceleration(),
      magnetic_field()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      offset += this->magnetic_field.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      offset += this->magnetic_field.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "hase_msgs/Imu"; };
    const char * getMD5(){ return "eb7ba5b6b220b37f47b0c3abaec215f6"; };

  };

}
#endif