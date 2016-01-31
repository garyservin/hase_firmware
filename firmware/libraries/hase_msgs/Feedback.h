#ifndef _ROS_hase_msgs_Feedback_h
#define _ROS_hase_msgs_Feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "hase_msgs/DriveFeedback.h"

namespace hase_msgs
{

  class Feedback : public ros::Msg
  {
    public:
      std_msgs::Header header;
      hase_msgs::DriveFeedback drivers[2];

    Feedback():
      header(),
      drivers()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 2; i++){
      offset += this->drivers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 2; i++){
      offset += this->drivers[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "hase_msgs/Feedback"; };
    const char * getMD5(){ return "9f54e010374b889b9fa7b2d41f46068d"; };

  };

}
#endif