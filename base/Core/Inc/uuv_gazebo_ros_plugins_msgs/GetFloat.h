#ifndef _ROS_SERVICE_GetFloat_h
#define _ROS_SERVICE_GetFloat_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_gazebo_ros_plugins_msgs
{

static const char GETFLOAT[] = "uuv_gazebo_ros_plugins_msgs/GetFloat";

  class GetFloatRequest : public ros::Msg
  {
    public:

    GetFloatRequest()
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

    const char * getType(){ return GETFLOAT; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetFloatResponse : public ros::Msg
  {
    public:
      typedef double _data_type;
      _data_type data;

    GetFloatResponse():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_data.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_data.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_data.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_data.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_data.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_data.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_data.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_data.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->data = u_data.real;
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return GETFLOAT; };
    const char * getMD5(){ return "fdb28210bfa9d7c91146260178d9a584"; };

  };

  class GetFloat {
    public:
    typedef GetFloatRequest Request;
    typedef GetFloatResponse Response;
  };

}
#endif
