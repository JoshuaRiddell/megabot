#ifndef _ROS_SERVICE_InitCircularTrajectory_h
#define _ROS_SERVICE_InitCircularTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Time.h"
#include "geometry_msgs/Point.h"

namespace uuv_control_msgs
{

static const char INITCIRCULARTRAJECTORY[] = "uuv_control_msgs/InitCircularTrajectory";

  class InitCircularTrajectoryRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Time _start_time_type;
      _start_time_type start_time;
      typedef bool _start_now_type;
      _start_now_type start_now;
      typedef double _radius_type;
      _radius_type radius;
      typedef geometry_msgs::Point _center_type;
      _center_type center;
      typedef bool _is_clockwise_type;
      _is_clockwise_type is_clockwise;
      typedef double _angle_offset_type;
      _angle_offset_type angle_offset;
      typedef int32_t _n_points_type;
      _n_points_type n_points;
      typedef double _heading_offset_type;
      _heading_offset_type heading_offset;
      typedef double _max_forward_speed_type;
      _max_forward_speed_type max_forward_speed;
      typedef double _duration_type;
      _duration_type duration;

    InitCircularTrajectoryRequest():
      start_time(),
      start_now(0),
      radius(0),
      center(),
      is_clockwise(0),
      angle_offset(0),
      n_points(0),
      heading_offset(0),
      max_forward_speed(0),
      duration(0)
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
      union {
        double real;
        uint64_t base;
      } u_radius;
      u_radius.real = this->radius;
      *(outbuffer + offset + 0) = (u_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radius.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_radius.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_radius.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_radius.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_radius.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->radius);
      offset += this->center.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_is_clockwise;
      u_is_clockwise.real = this->is_clockwise;
      *(outbuffer + offset + 0) = (u_is_clockwise.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_clockwise);
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
        int32_t real;
        uint32_t base;
      } u_n_points;
      u_n_points.real = this->n_points;
      *(outbuffer + offset + 0) = (u_n_points.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_n_points.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_n_points.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_n_points.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->n_points);
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
      union {
        double real;
        uint64_t base;
      } u_duration;
      u_duration.real = this->duration;
      *(outbuffer + offset + 0) = (u_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_duration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_duration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_duration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_duration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->duration);
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
      union {
        double real;
        uint64_t base;
      } u_radius;
      u_radius.base = 0;
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->radius = u_radius.real;
      offset += sizeof(this->radius);
      offset += this->center.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_is_clockwise;
      u_is_clockwise.base = 0;
      u_is_clockwise.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_clockwise = u_is_clockwise.real;
      offset += sizeof(this->is_clockwise);
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
      union {
        double real;
        uint64_t base;
      } u_duration;
      u_duration.base = 0;
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->duration = u_duration.real;
      offset += sizeof(this->duration);
     return offset;
    }

    const char * getType(){ return INITCIRCULARTRAJECTORY; };
    const char * getMD5(){ return "33f617e6e74b9a5a4089105d4a0a3b2f"; };

  };

  class InitCircularTrajectoryResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    InitCircularTrajectoryResponse():
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

    const char * getType(){ return INITCIRCULARTRAJECTORY; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class InitCircularTrajectory {
    public:
    typedef InitCircularTrajectoryRequest Request;
    typedef InitCircularTrajectoryResponse Response;
  };

}
#endif
