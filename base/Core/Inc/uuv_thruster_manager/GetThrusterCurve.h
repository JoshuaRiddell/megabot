#ifndef _ROS_SERVICE_GetThrusterCurve_h
#define _ROS_SERVICE_GetThrusterCurve_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_thruster_manager
{

static const char GETTHRUSTERCURVE[] = "uuv_thruster_manager/GetThrusterCurve";

  class GetThrusterCurveRequest : public ros::Msg
  {
    public:
      typedef double _min_type;
      _min_type min;
      typedef double _max_type;
      _max_type max;
      typedef int32_t _n_points_type;
      _n_points_type n_points;

    GetThrusterCurveRequest():
      min(0),
      max(0),
      n_points(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
        int32_t real;
        uint32_t base;
      } u_n_points;
      u_n_points.real = this->n_points;
      *(outbuffer + offset + 0) = (u_n_points.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_n_points.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_n_points.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_n_points.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->n_points);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
        int32_t real;
        uint32_t base;
      } u_n_points;
      u_n_points.base = 0;
      u_n_points.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_n_points.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_n_points.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_n_points.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->n_points = u_n_points.real;
      offset += sizeof(this->n_points);
     return offset;
    }

    const char * getType(){ return GETTHRUSTERCURVE; };
    const char * getMD5(){ return "3b8a1df953063c9de5f9a6419991fa0c"; };

  };

  class GetThrusterCurveResponse : public ros::Msg
  {
    public:
      uint32_t input_length;
      typedef double _input_type;
      _input_type st_input;
      _input_type * input;
      uint32_t thrust_length;
      typedef double _thrust_type;
      _thrust_type st_thrust;
      _thrust_type * thrust;

    GetThrusterCurveResponse():
      input_length(0), input(NULL),
      thrust_length(0), thrust(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->input_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->input_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->input_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->input_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_length);
      for( uint32_t i = 0; i < input_length; i++){
      union {
        double real;
        uint64_t base;
      } u_inputi;
      u_inputi.real = this->input[i];
      *(outbuffer + offset + 0) = (u_inputi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inputi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inputi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inputi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_inputi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_inputi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_inputi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_inputi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->input[i]);
      }
      *(outbuffer + offset + 0) = (this->thrust_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->thrust_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->thrust_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->thrust_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thrust_length);
      for( uint32_t i = 0; i < thrust_length; i++){
      union {
        double real;
        uint64_t base;
      } u_thrusti;
      u_thrusti.real = this->thrust[i];
      *(outbuffer + offset + 0) = (u_thrusti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrusti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_thrusti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_thrusti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_thrusti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_thrusti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_thrusti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_thrusti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->thrust[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t input_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      input_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      input_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      input_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->input_length);
      if(input_lengthT > input_length)
        this->input = (double*)realloc(this->input, input_lengthT * sizeof(double));
      input_length = input_lengthT;
      for( uint32_t i = 0; i < input_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_input;
      u_st_input.base = 0;
      u_st_input.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_input.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_input.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_input.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_input.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_input.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_input.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_input.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_input = u_st_input.real;
      offset += sizeof(this->st_input);
        memcpy( &(this->input[i]), &(this->st_input), sizeof(double));
      }
      uint32_t thrust_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      thrust_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      thrust_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      thrust_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->thrust_length);
      if(thrust_lengthT > thrust_length)
        this->thrust = (double*)realloc(this->thrust, thrust_lengthT * sizeof(double));
      thrust_length = thrust_lengthT;
      for( uint32_t i = 0; i < thrust_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_thrust;
      u_st_thrust.base = 0;
      u_st_thrust.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_thrust.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_thrust.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_thrust.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_thrust.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_thrust.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_thrust.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_thrust.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_thrust = u_st_thrust.real;
      offset += sizeof(this->st_thrust);
        memcpy( &(this->thrust[i]), &(this->st_thrust), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return GETTHRUSTERCURVE; };
    const char * getMD5(){ return "183802edaba8fb9ba8a2d917792277f6"; };

  };

  class GetThrusterCurve {
    public:
    typedef GetThrusterCurveRequest Request;
    typedef GetThrusterCurveResponse Response;
  };

}
#endif
