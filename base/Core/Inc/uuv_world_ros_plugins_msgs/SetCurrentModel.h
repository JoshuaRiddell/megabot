#ifndef _ROS_SERVICE_SetCurrentModel_h
#define _ROS_SERVICE_SetCurrentModel_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_world_ros_plugins_msgs
{

static const char SETCURRENTMODEL[] = "uuv_world_ros_plugins_msgs/SetCurrentModel";

  class SetCurrentModelRequest : public ros::Msg
  {
    public:
      typedef double _mean_type;
      _mean_type mean;
      typedef double _min_type;
      _min_type min;
      typedef double _max_type;
      _max_type max;
      typedef double _noise_type;
      _noise_type noise;
      typedef double _mu_type;
      _mu_type mu;

    SetCurrentModelRequest():
      mean(0),
      min(0),
      max(0),
      noise(0),
      mu(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_mean;
      u_mean.real = this->mean;
      *(outbuffer + offset + 0) = (u_mean.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mean.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mean.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mean.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mean.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mean.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mean.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mean.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mean);
      union {
        double real;
        uint64_t base;
      } u_min;
      u_min.real = this->min;
      *(outbuffer + offset + 0) = (u_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_min.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_min.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_min.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_min.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->min);
      union {
        double real;
        uint64_t base;
      } u_max;
      u_max.real = this->max;
      *(outbuffer + offset + 0) = (u_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max);
      union {
        double real;
        uint64_t base;
      } u_noise;
      u_noise.real = this->noise;
      *(outbuffer + offset + 0) = (u_noise.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_noise.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_noise.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_noise.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_noise.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_noise.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_noise.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_noise.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->noise);
      union {
        double real;
        uint64_t base;
      } u_mu;
      u_mu.real = this->mu;
      *(outbuffer + offset + 0) = (u_mu.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_mean;
      u_mean.base = 0;
      u_mean.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mean.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mean.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mean.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mean.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mean.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mean.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mean.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mean = u_mean.real;
      offset += sizeof(this->mean);
      union {
        double real;
        uint64_t base;
      } u_min;
      u_min.base = 0;
      u_min.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_min.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_min.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_min.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_min.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->min = u_min.real;
      offset += sizeof(this->min);
      union {
        double real;
        uint64_t base;
      } u_max;
      u_max.base = 0;
      u_max.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max = u_max.real;
      offset += sizeof(this->max);
      union {
        double real;
        uint64_t base;
      } u_noise;
      u_noise.base = 0;
      u_noise.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_noise.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_noise.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_noise.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_noise.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_noise.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_noise.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_noise.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->noise = u_noise.real;
      offset += sizeof(this->noise);
      union {
        double real;
        uint64_t base;
      } u_mu;
      u_mu.base = 0;
      u_mu.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu = u_mu.real;
      offset += sizeof(this->mu);
     return offset;
    }

    const char * getType(){ return SETCURRENTMODEL; };
    const char * getMD5(){ return "b8222571af4e4180b9b706d1e17ad7e1"; };

  };

  class SetCurrentModelResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetCurrentModelResponse():
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

    const char * getType(){ return SETCURRENTMODEL; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetCurrentModel {
    public:
    typedef SetCurrentModelRequest Request;
    typedef SetCurrentModelResponse Response;
  };

}
#endif
