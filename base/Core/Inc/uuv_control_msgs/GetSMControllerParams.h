#ifndef _ROS_SERVICE_GetSMControllerParams_h
#define _ROS_SERVICE_GetSMControllerParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_control_msgs
{

static const char GETSMCONTROLLERPARAMS[] = "uuv_control_msgs/GetSMControllerParams";

  class GetSMControllerParamsRequest : public ros::Msg
  {
    public:

    GetSMControllerParamsRequest()
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

    const char * getType(){ return GETSMCONTROLLERPARAMS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetSMControllerParamsResponse : public ros::Msg
  {
    public:
      uint32_t K_length;
      typedef double _K_type;
      _K_type st_K;
      _K_type * K;
      uint32_t Kd_length;
      typedef double _Kd_type;
      _Kd_type st_Kd;
      _Kd_type * Kd;
      uint32_t Ki_length;
      typedef double _Ki_type;
      _Ki_type st_Ki;
      _Ki_type * Ki;
      uint32_t slope_length;
      typedef double _slope_type;
      _slope_type st_slope;
      _slope_type * slope;

    GetSMControllerParamsResponse():
      K_length(0), K(NULL),
      Kd_length(0), Kd(NULL),
      Ki_length(0), Ki(NULL),
      slope_length(0), slope(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->K_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->K_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->K_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->K_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->K_length);
      for( uint32_t i = 0; i < K_length; i++){
      union {
        double real;
        uint64_t base;
      } u_Ki;
      u_Ki.real = this->K[i];
      *(outbuffer + offset + 0) = (u_Ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Ki.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Ki.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Ki.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Ki.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Ki.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->K[i]);
      }
      *(outbuffer + offset + 0) = (this->Kd_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Kd_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Kd_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Kd_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Kd_length);
      for( uint32_t i = 0; i < Kd_length; i++){
      union {
        double real;
        uint64_t base;
      } u_Kdi;
      u_Kdi.real = this->Kd[i];
      *(outbuffer + offset + 0) = (u_Kdi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Kdi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Kdi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Kdi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Kdi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Kdi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Kdi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Kdi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Kd[i]);
      }
      *(outbuffer + offset + 0) = (this->Ki_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Ki_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Ki_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Ki_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Ki_length);
      for( uint32_t i = 0; i < Ki_length; i++){
      union {
        double real;
        uint64_t base;
      } u_Kii;
      u_Kii.real = this->Ki[i];
      *(outbuffer + offset + 0) = (u_Kii.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Kii.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Kii.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Kii.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Kii.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Kii.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Kii.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Kii.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Ki[i]);
      }
      *(outbuffer + offset + 0) = (this->slope_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->slope_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->slope_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->slope_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->slope_length);
      for( uint32_t i = 0; i < slope_length; i++){
      union {
        double real;
        uint64_t base;
      } u_slopei;
      u_slopei.real = this->slope[i];
      *(outbuffer + offset + 0) = (u_slopei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_slopei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_slopei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_slopei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_slopei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_slopei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_slopei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_slopei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->slope[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t K_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      K_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      K_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      K_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->K_length);
      if(K_lengthT > K_length)
        this->K = (double*)realloc(this->K, K_lengthT * sizeof(double));
      K_length = K_lengthT;
      for( uint32_t i = 0; i < K_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_K;
      u_st_K.base = 0;
      u_st_K.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_K.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_K.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_K.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_K.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_K.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_K.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_K.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_K = u_st_K.real;
      offset += sizeof(this->st_K);
        memcpy( &(this->K[i]), &(this->st_K), sizeof(double));
      }
      uint32_t Kd_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      Kd_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      Kd_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      Kd_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->Kd_length);
      if(Kd_lengthT > Kd_length)
        this->Kd = (double*)realloc(this->Kd, Kd_lengthT * sizeof(double));
      Kd_length = Kd_lengthT;
      for( uint32_t i = 0; i < Kd_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_Kd;
      u_st_Kd.base = 0;
      u_st_Kd.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_Kd.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_Kd.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_Kd.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_Kd.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_Kd.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_Kd.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_Kd.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_Kd = u_st_Kd.real;
      offset += sizeof(this->st_Kd);
        memcpy( &(this->Kd[i]), &(this->st_Kd), sizeof(double));
      }
      uint32_t Ki_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      Ki_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      Ki_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      Ki_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->Ki_length);
      if(Ki_lengthT > Ki_length)
        this->Ki = (double*)realloc(this->Ki, Ki_lengthT * sizeof(double));
      Ki_length = Ki_lengthT;
      for( uint32_t i = 0; i < Ki_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_Ki;
      u_st_Ki.base = 0;
      u_st_Ki.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_Ki.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_Ki.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_Ki.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_Ki.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_Ki.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_Ki.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_Ki.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_Ki = u_st_Ki.real;
      offset += sizeof(this->st_Ki);
        memcpy( &(this->Ki[i]), &(this->st_Ki), sizeof(double));
      }
      uint32_t slope_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      slope_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      slope_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      slope_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->slope_length);
      if(slope_lengthT > slope_length)
        this->slope = (double*)realloc(this->slope, slope_lengthT * sizeof(double));
      slope_length = slope_lengthT;
      for( uint32_t i = 0; i < slope_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_slope;
      u_st_slope.base = 0;
      u_st_slope.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_slope.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_slope.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_slope.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_slope.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_slope.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_slope.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_slope.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_slope = u_st_slope.real;
      offset += sizeof(this->st_slope);
        memcpy( &(this->slope[i]), &(this->st_slope), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return GETSMCONTROLLERPARAMS; };
    const char * getMD5(){ return "dc294c7929e39ce02ce0ce70a116b3b3"; };

  };

  class GetSMControllerParams {
    public:
    typedef GetSMControllerParamsRequest Request;
    typedef GetSMControllerParamsResponse Response;
  };

}
#endif
