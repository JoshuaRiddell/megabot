#ifndef _ROS_SERVICE_InitRectTrajectory_h
#define _ROS_SERVICE_InitRectTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Time.h"
#include "geometry_msgs/Point.h"

namespace uuv_control_msgs
{

static const char INITRECTTRAJECTORY[] = "uuv_control_msgs/InitRectTrajectory";

  class InitRectTrajectoryRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Time _start_time_type;
      _start_time_type start_time;
      typedef bool _start_now_type;
      _start_now_type start_now;
      typedef geometry_msgs::Point _origin_type;
      _origin_type origin;
      typedef double _height_type;
      _height_type height;
      typedef double _width_type;
      _width_type width;
      typedef double _angle_offset_type;
      _angle_offset_type angle_offset;
      typedef double _heading_offset_type;
      _heading_offset_type heading_offset;
      typedef double _max_forward_speed_type;
      _max_forward_speed_type max_forward_speed;

    InitRectTrajectoryRequest():
      start_time(),
      start_now(0),
      origin(),
      height(0),
      width(0),
      angle_offset(0),
      heading_offset(0),
      max_forward_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->start_time.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_start_now;
      u_start_now.real = this->start_now;
      *(outbuffer + offset + 0) = (u_start_now.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->start_now);
      offset += this->origin.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_height.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_height.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_height.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_height.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->height);
      union {
        double real;
        uint64_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_width.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_width.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_width.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_width.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->width);
      union {
        double real;
        uint64_t base;
      } u_angle_offset;
      u_angle_offset.real = this->angle_offset;
      *(outbuffer + offset + 0) = (u_angle_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_offset.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_angle_offset.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_angle_offset.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_angle_offset.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_angle_offset.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->angle_offset);
      union {
        double real;
        uint64_t base;
      } u_heading_offset;
      u_heading_offset.real = this->heading_offset;
      *(outbuffer + offset + 0) = (u_heading_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading_offset.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_heading_offset.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_heading_offset.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_heading_offset.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_heading_offset.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->heading_offset);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->start_time.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_start_now;
      u_start_now.base = 0;
      u_start_now.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->start_now = u_start_now.real;
      offset += sizeof(this->start_now);
      offset += this->origin.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        double real;
        uint64_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        double real;
        uint64_t base;
      } u_angle_offset;
      u_angle_offset.base = 0;
      u_angle_offset.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_offset.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_offset.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_offset.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_angle_offset.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_angle_offset.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_angle_offset.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_angle_offset.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->angle_offset = u_angle_offset.real;
      offset += sizeof(this->angle_offset);
      union {
        double real;
        uint64_t base;
      } u_heading_offset;
      u_heading_offset.base = 0;
      u_heading_offset.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading_offset.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading_offset.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading_offset.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_heading_offset.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_heading_offset.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_heading_offset.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_heading_offset.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->heading_offset = u_heading_offset.real;
      offset += sizeof(this->heading_offset);
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
     return offset;
    }

    const char * getType(){ return INITRECTTRAJECTORY; };
    const char * getMD5(){ return "82860775b3df6259cef2af6522eca70e"; };

  };

  class InitRectTrajectoryResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    InitRectTrajectoryResponse():
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

    const char * getType(){ return INITRECTTRAJECTORY; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class InitRectTrajectory {
    public:
    typedef InitRectTrajectoryRequest Request;
    typedef InitRectTrajectoryResponse Response;
  };

}
#endif
