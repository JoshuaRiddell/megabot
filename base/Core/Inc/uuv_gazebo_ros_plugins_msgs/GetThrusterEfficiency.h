#ifndef _ROS_SERVICE_GetThrusterEfficiency_h
#define _ROS_SERVICE_GetThrusterEfficiency_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_gazebo_ros_plugins_msgs
{

static const char GETTHRUSTEREFFICIENCY[] = "uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiency";

  class GetThrusterEfficiencyRequest : public ros::Msg
  {
    public:

    GetThrusterEfficiencyRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETTHRUSTEREFFICIENCY; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetThrusterEfficiencyResponse : public ros::Msg
  {
    public:
      typedef double _efficiency_type;
      _efficiency_type efficiency;

    GetThrusterEfficiencyResponse():
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

    const char * getType(){ return GETTHRUSTEREFFICIENCY; };
    const char * getMD5(){ return "b80ec71e671b93e4cc403df1ac4c8a86"; };

  };

  class GetThrusterEfficiency {
    public:
    typedef GetThrusterEfficiencyRequest Request;
    typedef GetThrusterEfficiencyResponse Response;
  };

}
#endif
