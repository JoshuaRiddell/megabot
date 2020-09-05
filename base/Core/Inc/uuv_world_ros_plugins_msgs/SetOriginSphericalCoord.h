#ifndef _ROS_SERVICE_SetOriginSphericalCoord_h
#define _ROS_SERVICE_SetOriginSphericalCoord_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_world_ros_plugins_msgs
{

static const char SETORIGINSPHERICALCOORD[] = "uuv_world_ros_plugins_msgs/SetOriginSphericalCoord";

  class SetOriginSphericalCoordRequest : public ros::Msg
  {
    public:
      typedef double _latitude_deg_type;
      _latitude_deg_type latitude_deg;
      typedef double _longitude_deg_type;
      _longitude_deg_type longitude_deg;
      typedef double _altitude_type;
      _altitude_type altitude;

    SetOriginSphericalCoordRequest():
      latitude_deg(0),
      longitude_deg(0),
      altitude(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_latitude_deg;
      u_latitude_deg.real = this->latitude_deg;
      *(outbuffer + offset + 0) = (u_latitude_deg.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_latitude_deg.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_latitude_deg.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_latitude_deg.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_latitude_deg.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_latitude_deg.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_latitude_deg.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_latitude_deg.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->latitude_deg);
      union {
        double real;
        uint64_t base;
      } u_longitude_deg;
      u_longitude_deg.real = this->longitude_deg;
      *(outbuffer + offset + 0) = (u_longitude_deg.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_longitude_deg.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_longitude_deg.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_longitude_deg.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_longitude_deg.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_longitude_deg.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_longitude_deg.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_longitude_deg.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->longitude_deg);
      union {
        double real;
        uint64_t base;
      } u_altitude;
      u_altitude.real = this->altitude;
      *(outbuffer + offset + 0) = (u_altitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_altitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_altitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_altitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_altitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_altitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_altitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_altitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->altitude);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_latitude_deg;
      u_latitude_deg.base = 0;
      u_latitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_latitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_latitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_latitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_latitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_latitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_latitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_latitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->latitude_deg = u_latitude_deg.real;
      offset += sizeof(this->latitude_deg);
      union {
        double real;
        uint64_t base;
      } u_longitude_deg;
      u_longitude_deg.base = 0;
      u_longitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_longitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_longitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_longitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_longitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_longitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_longitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_longitude_deg.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->longitude_deg = u_longitude_deg.real;
      offset += sizeof(this->longitude_deg);
      union {
        double real;
        uint64_t base;
      } u_altitude;
      u_altitude.base = 0;
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->altitude = u_altitude.real;
      offset += sizeof(this->altitude);
     return offset;
    }

    const char * getType(){ return SETORIGINSPHERICALCOORD; };
    const char * getMD5(){ return "60457d630fe21cc5f8f6bd5d0fc90156"; };

  };

  class SetOriginSphericalCoordResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetOriginSphericalCoordResponse():
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

    const char * getType(){ return SETORIGINSPHERICALCOORD; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetOriginSphericalCoord {
    public:
    typedef SetOriginSphericalCoordRequest Request;
    typedef SetOriginSphericalCoordResponse Response;
  };

}
#endif
