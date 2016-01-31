#ifndef _ROS_hase_msgs_DriveFeedback_h
#define _ROS_hase_msgs_DriveFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hase_msgs
{

  class DriveFeedback : public ros::Msg
  {
    public:
      float duty_cycle;
      float measured_velocity;
      float measured_travel;

    DriveFeedback():
      duty_cycle(0),
      measured_velocity(0),
      measured_travel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_duty_cycle;
      u_duty_cycle.real = this->duty_cycle;
      *(outbuffer + offset + 0) = (u_duty_cycle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duty_cycle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duty_cycle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duty_cycle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duty_cycle);
      union {
        float real;
        uint32_t base;
      } u_measured_velocity;
      u_measured_velocity.real = this->measured_velocity;
      *(outbuffer + offset + 0) = (u_measured_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_velocity);
      union {
        float real;
        uint32_t base;
      } u_measured_travel;
      u_measured_travel.real = this->measured_travel;
      *(outbuffer + offset + 0) = (u_measured_travel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_travel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_travel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_travel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_travel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_duty_cycle;
      u_duty_cycle.base = 0;
      u_duty_cycle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duty_cycle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duty_cycle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duty_cycle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->duty_cycle = u_duty_cycle.real;
      offset += sizeof(this->duty_cycle);
      union {
        float real;
        uint32_t base;
      } u_measured_velocity;
      u_measured_velocity.base = 0;
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_velocity = u_measured_velocity.real;
      offset += sizeof(this->measured_velocity);
      union {
        float real;
        uint32_t base;
      } u_measured_travel;
      u_measured_travel.base = 0;
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_travel = u_measured_travel.real;
      offset += sizeof(this->measured_travel);
     return offset;
    }

    const char * getType(){ return "hase_msgs/DriveFeedback"; };
    const char * getMD5(){ return "24cd33db359235d8671255ac1f4e6352"; };

  };

}
#endif