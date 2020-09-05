#ifndef _ROS_SERVICE_SetCurrentVelocity_h
#define _ROS_SERVICE_SetCurrentVelocity_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_world_ros_plugins_msgs
{

static const char SETCURRENTVELOCITY[] = "uuv_world_ros_plugins_msgs/SetCurrentVelocity";

  class SetCurrentVelocityRequest : public ros::Msg
  {
    public:
      typedef double _velocity_type;
      _velocity_type velocity;
      typedef double _horizontal_angle_type;
      _horizontal_angle_type horizontal_angle;
      typedef double _vertical_angle_type;
      _vertical_angle_type vertical_angle;

    SetCurrentVelocityRequest():
      velocity(0),
      horizontal_angle(0),
      vertical_angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_velocity;
      u_velocity.real = this->velocity;
      *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity);
      union {
        double real;
        uint64_t base;
      } u_horizontal_angle;
      u_horizontal_angle.real = this->horizontal_angle;
      *(outbuffer + offset + 0) = (u_horizontal_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_horizontal_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_horizontal_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_horizontal_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_horizontal_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_horizontal_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_horizontal_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_horizontal_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->horizontal_angle);
      union {
        double real;
        uint64_t base;
      } u_vertical_angle;
      u_vertical_angle.real = this->vertical_angle;
      *(outbuffer + offset + 0) = (u_vertical_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vertical_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vertical_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vertical_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vertical_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vertical_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vertical_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vertical_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vertical_angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_velocity;
      u_velocity.base = 0;
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velocity = u_velocity.real;
      offset += sizeof(this->velocity);
      union {
        double real;
        uint64_t base;
      } u_horizontal_angle;
      u_horizontal_angle.base = 0;
      u_horizontal_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_horizontal_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_horizontal_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_horizontal_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_horizontal_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_horizontal_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_horizontal_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_horizontal_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->horizontal_angle = u_horizontal_angle.real;
      offset += sizeof(this->horizontal_angle);
      union {
        double real;
        uint64_t base;
      } u_vertical_angle;
      u_vertical_angle.base = 0;
      u_vertical_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vertical_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vertical_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vertical_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vertical_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vertical_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vertical_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vertical_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vertical_angle = u_vertical_angle.real;
      offset += sizeof(this->vertical_angle);
     return offset;
    }

    const char * getType(){ return SETCURRENTVELOCITY; };
    const char * getMD5(){ return "44059aaf9c13a2ec083fad30e8a17ee3"; };

  };

  class SetCurrentVelocityResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetCurrentVelocityResponse():
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

    const char * getType(){ return SETCURRENTVELOCITY; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetCurrentVelocity {
    public:
    typedef SetCurrentVelocityRequest Request;
    typedef SetCurrentVelocityResponse Response;
  };

}
#endif
