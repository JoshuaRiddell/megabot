#ifndef _ROS_uuv_sensor_ros_plugins_msgs_ChemicalParticleConcentration_h
#define _ROS_uuv_sensor_ros_plugins_msgs_ChemicalParticleConcentration_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace uuv_sensor_ros_plugins_msgs
{

  class ChemicalParticleConcentration : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _position_type;
      _position_type position;
      typedef double _latitude_type;
      _latitude_type latitude;
      typedef double _longitude_type;
      _longitude_type longitude;
      typedef double _depth_type;
      _depth_type depth;
      typedef double _concentration_type;
      _concentration_type concentration;
      typedef bool _is_measuring_type;
      _is_measuring_type is_measuring;

    ChemicalParticleConcentration():
      header(),
      position(),
      latitude(0),
      longitude(0),
      depth(0),
      concentration(0),
      is_measuring(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->position.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_latitude;
      u_latitude.real = this->latitude;
      *(outbuffer + offset + 0) = (u_latitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_latitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_latitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_latitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_latitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_latitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_latitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_latitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->latitude);
      union {
        double real;
        uint64_t base;
      } u_longitude;
      u_longitude.real = this->longitude;
      *(outbuffer + offset + 0) = (u_longitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_longitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_longitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_longitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_longitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_longitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_longitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_longitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->longitude);
      union {
        double real;
        uint64_t base;
      } u_depth;
      u_depth.real = this->depth;
      *(outbuffer + offset + 0) = (u_depth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_depth.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_depth.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_depth.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_depth.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->depth);
      union {
        double real;
        uint64_t base;
      } u_concentration;
      u_concentration.real = this->concentration;
      *(outbuffer + offset + 0) = (u_concentration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_concentration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_concentration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_concentration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_concentration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_concentration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_concentration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_concentration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->concentration);
      union {
        bool real;
        uint8_t base;
      } u_is_measuring;
      u_is_measuring.real = this->is_measuring;
      *(outbuffer + offset + 0) = (u_is_measuring.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_measuring);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->position.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_latitude;
      u_latitude.base = 0;
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->latitude = u_latitude.real;
      offset += sizeof(this->latitude);
      union {
        double real;
        uint64_t base;
      } u_longitude;
      u_longitude.base = 0;
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->longitude = u_longitude.real;
      offset += sizeof(this->longitude);
      union {
        double real;
        uint64_t base;
      } u_depth;
      u_depth.base = 0;
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->depth = u_depth.real;
      offset += sizeof(this->depth);
      union {
        double real;
        uint64_t base;
      } u_concentration;
      u_concentration.base = 0;
      u_concentration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_concentration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_concentration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_concentration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_concentration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_concentration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_concentration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_concentration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->concentration = u_concentration.real;
      offset += sizeof(this->concentration);
      union {
        bool real;
        uint8_t base;
      } u_is_measuring;
      u_is_measuring.base = 0;
      u_is_measuring.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_measuring = u_is_measuring.real;
      offset += sizeof(this->is_measuring);
     return offset;
    }

    const char * getType(){ return "uuv_sensor_ros_plugins_msgs/ChemicalParticleConcentration"; };
    const char * getMD5(){ return "f17ac73d7dd3916a4fd6f721f0ccbe37"; };

  };

}
#endif
