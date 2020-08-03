#ifndef _ROS_glider_Leak_h
#define _ROS_glider_Leak_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace glider
{

  class Leak : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _detected_type;
      _detected_type detected;

    Leak():
      header(),
      detected(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_detected;
      u_detected.real = this->detected;
      *(outbuffer + offset + 0) = (u_detected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->detected);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_detected;
      u_detected.base = 0;
      u_detected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->detected = u_detected.real;
      offset += sizeof(this->detected);
     return offset;
    }

    const char * getType(){ return "glider/Leak"; };
    const char * getMD5(){ return "5ba5405f3cb7ba61cee77b4b7f395eec"; };

  };

}
#endif
