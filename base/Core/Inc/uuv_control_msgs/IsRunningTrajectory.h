#ifndef _ROS_SERVICE_IsRunningTrajectory_h
#define _ROS_SERVICE_IsRunningTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_control_msgs
{

static const char ISRUNNINGTRAJECTORY[] = "uuv_control_msgs/IsRunningTrajectory";

  class IsRunningTrajectoryRequest : public ros::Msg
  {
    public:

    IsRunningTrajectoryRequest()
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

    const char * getType(){ return ISRUNNINGTRAJECTORY; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class IsRunningTrajectoryResponse : public ros::Msg
  {
    public:
      typedef double _progress_type;
      _progress_type progress;

    IsRunningTrajectoryResponse():
      progress(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_progress;
      u_progress.real = this->progress;
      *(outbuffer + offset + 0) = (u_progress.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_progress.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_progress.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_progress.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_progress.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_progress.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_progress.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_progress.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->progress);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_progress;
      u_progress.base = 0;
      u_progress.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_progress.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_progress.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_progress.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_progress.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_progress.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_progress.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_progress.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->progress = u_progress.real;
      offset += sizeof(this->progress);
     return offset;
    }

    const char * getType(){ return ISRUNNINGTRAJECTORY; };
    const char * getMD5(){ return "7afca0099e0cddc25243b1e3569895fe"; };

  };

  class IsRunningTrajectory {
    public:
    typedef IsRunningTrajectoryRequest Request;
    typedef IsRunningTrajectoryResponse Response;
  };

}
#endif
