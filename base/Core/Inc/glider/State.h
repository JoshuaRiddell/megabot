#ifndef _ROS_glider_State_h
#define _ROS_glider_State_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace glider
{

  class State : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _state_type;
      _state_type state;
      enum { MANUAL = 0 };
      enum { STABILISED = 1 };
      enum { DEPTH_HOLD = 2 };
      enum { ALTITUDE_HOLD = 3 };
      enum { EMERGENCY_RECOVERY = 4 };
      enum { ALTITUDE_HOLD_EXPERIMENTAL = 5 };
      enum { ALTITUDE_HOLD_CASCADE = 6 };

    State():
      header(),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "glider/State"; };
    const char * getMD5(){ return "a648a5141f7533a0aab9f6e087d49259"; };

  };

}
#endif
