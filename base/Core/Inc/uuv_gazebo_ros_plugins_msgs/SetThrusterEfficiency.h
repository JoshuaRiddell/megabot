#ifndef _ROS_SERVICE_SetThrusterEfficiency_h
#define _ROS_SERVICE_SetThrusterEfficiency_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_gazebo_ros_plugins_msgs
{

static const char SETTHRUSTEREFFICIENCY[] = "uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiency";

  class SetThrusterEfficiencyRequest : public ros::Msg
  {
    public:
      typedef double _efficiency_type;
      _efficiency_type efficiency;

    SetThrusterEfficiencyRequest():
      efficiency(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_efficiency;
      u_efficiency.real = this->efficiency;
      *(outbuffer + offset + 0) = (u_efficiency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_efficiency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_efficiency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_efficiency.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_efficiency.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_efficiency.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_efficiency.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_efficiency.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->efficiency);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_efficiency;
      u_efficiency.base = 0;
      u_efficiency.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_efficiency.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_efficiency.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_efficiency.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_efficiency.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_efficiency.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_efficiency.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_efficiency.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->efficiency = u_efficiency.real;
      offset += sizeof(this->efficiency);
     return offset;
    }

    const char * getType(){ return SETTHRUSTEREFFICIENCY; };
    const char * getMD5(){ return "b80ec71e671b93e4cc403df1ac4c8a86"; };

  };

  class SetThrusterEfficiencyResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetThrusterEfficiencyResponse():
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

    const char * getType(){ return SETTHRUSTEREFFICIENCY; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetThrusterEfficiency {
    public:
    typedef SetThrusterEfficiencyRequest Request;
    typedef SetThrusterEfficiencyResponse Response;
  };

}
#endif
