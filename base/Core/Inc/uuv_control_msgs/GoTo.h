#ifndef _ROS_SERVICE_GoTo_h
#define _ROS_SERVICE_GoTo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "uuv_control_msgs/Waypoint.h"

namespace uuv_control_msgs
{

static const char GOTO[] = "uuv_control_msgs/GoTo";

  class GoToRequest : public ros::Msg
  {
    public:
      typedef uuv_control_msgs::Waypoint _waypoint_type;
      _waypoint_type waypoint;
      typedef double _max_forward_speed_type;
      _max_forward_speed_type max_forward_speed;
      typedef const char* _interpolator_type;
      _interpolator_type interpolator;
      enum { LIPB = 'lipb' };
      enum { CUBIC = 'cubic' };
      enum { DUBINS = 'dubins' };
      enum { LINEAR = 'linear' };

    GoToRequest():
      waypoint(),
      max_forward_speed(0),
      interpolator("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->waypoint.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_max_forward_speed;
      u_max_forward_speed.real = this->max_forward_speed;
      *(outbuffer + offset + 0) = (u_max_forward_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_forward_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_forward_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_forward_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_forward_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_forward_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_forward_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_forward_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_forward_speed);
      uint32_t length_interpolator = strlen(this->interpolator);
      varToArr(outbuffer + offset, length_interpolator);
      offset += 4;
      memcpy(outbuffer + offset, this->interpolator, length_interpolator);
      offset += length_interpolator;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->waypoint.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_max_forward_speed;
      u_max_forward_speed.base = 0;
      u_max_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_forward_speed = u_max_forward_speed.real;
      offset += sizeof(this->max_forward_speed);
      uint32_t length_interpolator;
      arrToVar(length_interpolator, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_interpolator; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_interpolator-1]=0;
      this->interpolator = (char *)(inbuffer + offset-1);
      offset += length_interpolator;
     return offset;
    }

    const char * getType(){ return GOTO; };
    const char * getMD5(){ return "ca7b4067b6783823c9f5c936e1c99b3a"; };

  };

  class GoToResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    GoToResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return GOTO; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class GoTo {
    public:
    typedef GoToRequest Request;
    typedef GoToResponse Response;
  };

}
#endif
