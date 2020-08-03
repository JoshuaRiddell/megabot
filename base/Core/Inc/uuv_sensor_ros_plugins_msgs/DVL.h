#ifndef _ROS_uuv_sensor_ros_plugins_msgs_DVL_h
#define _ROS_uuv_sensor_ros_plugins_msgs_DVL_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "uuv_sensor_ros_plugins_msgs/DVLBeam.h"

namespace uuv_sensor_ros_plugins_msgs
{

  class DVL : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _velocity_type;
      _velocity_type velocity;
      double velocity_covariance[9];
      typedef double _altitude_type;
      _altitude_type altitude;
      uint32_t beams_length;
      typedef uuv_sensor_ros_plugins_msgs::DVLBeam _beams_type;
      _beams_type st_beams;
      _beams_type * beams;

    DVL():
      header(),
      velocity(),
      velocity_covariance(),
      altitude(0),
      beams_length(0), beams(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_velocity_covariancei;
      u_velocity_covariancei.real = this->velocity_covariance[i];
      *(outbuffer + offset + 0) = (u_velocity_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_covariancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity_covariancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity_covariancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity_covariancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity_covariancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity_covariance[i]);
      }
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
      *(outbuffer + offset + 0) = (this->beams_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->beams_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->beams_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->beams_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->beams_length);
      for( uint32_t i = 0; i < beams_length; i++){
      offset += this->beams[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_velocity_covariancei;
      u_velocity_covariancei.base = 0;
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velocity_covariance[i] = u_velocity_covariancei.real;
      offset += sizeof(this->velocity_covariance[i]);
      }
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
      uint32_t beams_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      beams_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      beams_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      beams_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->beams_length);
      if(beams_lengthT > beams_length)
        this->beams = (uuv_sensor_ros_plugins_msgs::DVLBeam*)realloc(this->beams, beams_lengthT * sizeof(uuv_sensor_ros_plugins_msgs::DVLBeam));
      beams_length = beams_lengthT;
      for( uint32_t i = 0; i < beams_length; i++){
      offset += this->st_beams.deserialize(inbuffer + offset);
        memcpy( &(this->beams[i]), &(this->st_beams), sizeof(uuv_sensor_ros_plugins_msgs::DVLBeam));
      }
     return offset;
    }

    const char * getType(){ return "uuv_sensor_ros_plugins_msgs/DVL"; };
    const char * getMD5(){ return "02bba6182b6f271447d7f88473256572"; };

  };

}
#endif
