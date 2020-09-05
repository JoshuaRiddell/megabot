#ifndef _ROS_uuv_sensor_ros_plugins_msgs_Salinity_h
#define _ROS_uuv_sensor_ros_plugins_msgs_Salinity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace uuv_sensor_ros_plugins_msgs
{

  class Salinity : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef double _salinity_type;
      _salinity_type salinity;
      typedef double _variance_type;
      _variance_type variance;
      typedef const char* _unit_type;
      _unit_type unit;
      enum { PSU = "psu" };
      enum { PPM = "ppm" };
      enum { PPT = "ppt" };

    Salinity():
      header(),
      salinity(0),
      variance(0),
      unit("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_salinity;
      u_salinity.real = this->salinity;
      *(outbuffer + offset + 0) = (u_salinity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_salinity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_salinity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_salinity.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_salinity.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_salinity.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_salinity.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_salinity.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->salinity);
      union {
        double real;
        uint64_t base;
      } u_variance;
      u_variance.real = this->variance;
      *(outbuffer + offset + 0) = (u_variance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_variance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_variance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_variance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_variance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_variance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_variance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_variance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->variance);
      uint32_t length_unit = strlen(this->unit);
      varToArr(outbuffer + offset, length_unit);
      offset += 4;
      memcpy(outbuffer + offset, this->unit, length_unit);
      offset += length_unit;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_salinity;
      u_salinity.base = 0;
      u_salinity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_salinity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_salinity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_salinity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_salinity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_salinity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_salinity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_salinity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->salinity = u_salinity.real;
      offset += sizeof(this->salinity);
      union {
        double real;
        uint64_t base;
      } u_variance;
      u_variance.base = 0;
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->variance = u_variance.real;
      offset += sizeof(this->variance);
      uint32_t length_unit;
      arrToVar(length_unit, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_unit; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_unit-1]=0;
      this->unit = (char *)(inbuffer + offset-1);
      offset += length_unit;
     return offset;
    }

    const char * getType(){ return "uuv_sensor_ros_plugins_msgs/Salinity"; };
    const char * getMD5(){ return "4d20de37b8b3a344b3f4c36f2192b257"; };

  };

}
#endif
