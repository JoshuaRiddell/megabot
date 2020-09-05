#ifndef _ROS_uuv_control_msgs_Waypoint_h
#define _ROS_uuv_control_msgs_Waypoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace uuv_control_msgs
{

  class Waypoint : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Point _point_type;
      _point_type point;
      typedef double _max_forward_speed_type;
      _max_forward_speed_type max_forward_speed;
      typedef double _heading_offset_type;
      _heading_offset_type heading_offset;
      typedef bool _use_fixed_heading_type;
      _use_fixed_heading_type use_fixed_heading;
      typedef double _radius_of_acceptance_type;
      _radius_of_acceptance_type radius_of_acceptance;

    Waypoint():
      header(),
      point(),
      max_forward_speed(0),
      heading_offset(0),
      use_fixed_heading(0),
      radius_of_acceptance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->point.serialize(outbuffer + offset);
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
        bool real;
        uint8_t base;
      } u_use_fixed_heading;
      u_use_fixed_heading.real = this->use_fixed_heading;
      *(outbuffer + offset + 0) = (u_use_fixed_heading.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_fixed_heading);
      union {
        double real;
        uint64_t base;
      } u_radius_of_acceptance;
      u_radius_of_acceptance.real = this->radius_of_acceptance;
      *(outbuffer + offset + 0) = (u_radius_of_acceptance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radius_of_acceptance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radius_of_acceptance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radius_of_acceptance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_radius_of_acceptance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_radius_of_acceptance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_radius_of_acceptance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_radius_of_acceptance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->radius_of_acceptance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->point.deserialize(inbuffer + offset);
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
        bool real;
        uint8_t base;
      } u_use_fixed_heading;
      u_use_fixed_heading.base = 0;
      u_use_fixed_heading.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_fixed_heading = u_use_fixed_heading.real;
      offset += sizeof(this->use_fixed_heading);
      union {
        double real;
        uint64_t base;
      } u_radius_of_acceptance;
      u_radius_of_acceptance.base = 0;
      u_radius_of_acceptance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_radius_of_acceptance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_radius_of_acceptance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_radius_of_acceptance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_radius_of_acceptance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_radius_of_acceptance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_radius_of_acceptance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_radius_of_acceptance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->radius_of_acceptance = u_radius_of_acceptance.real;
      offset += sizeof(this->radius_of_acceptance);
     return offset;
    }

    const char * getType(){ return "uuv_control_msgs/Waypoint"; };
    const char * getMD5(){ return "0efb7fda1b5980152de94b6064a5cf35"; };

  };

}
#endif
