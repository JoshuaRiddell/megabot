#ifndef _ROS_glider_Servos_h
#define _ROS_glider_Servos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace glider
{

  class Servos : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      double servos[8];
      enum { RUDDER = 2 };
      enum { ELEVATOR = 3 };
      enum { AILERON_LEFT = 0 };
      enum { AILERON_RIGHT = 1 };

    Servos():
      header(),
      servos()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      union {
        double real;
        uint64_t base;
      } u_servosi;
      u_servosi.real = this->servos[i];
      *(outbuffer + offset + 0) = (u_servosi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servosi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servosi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servosi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_servosi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_servosi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_servosi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_servosi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->servos[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      union {
        double real;
        uint64_t base;
      } u_servosi;
      u_servosi.base = 0;
      u_servosi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servosi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servosi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servosi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_servosi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_servosi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_servosi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_servosi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->servos[i] = u_servosi.real;
      offset += sizeof(this->servos[i]);
      }
     return offset;
    }

    const char * getType(){ return "glider/Servos"; };
    const char * getMD5(){ return "ae0632d4e09332a4ffcb43c8cea3a4ca"; };

  };

}
#endif
