#ifndef _ROS_uuv_sensor_ros_plugins_msgs_DVLBeam_h
#define _ROS_uuv_sensor_ros_plugins_msgs_DVLBeam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace uuv_sensor_ros_plugins_msgs
{

  class DVLBeam : public ros::Msg
  {
    public:
      typedef double _range_type;
      _range_type range;
      typedef double _range_covariance_type;
      _range_covariance_type range_covariance;
      typedef double _velocity_type;
      _velocity_type velocity;
      typedef double _velocity_covariance_type;
      _velocity_covariance_type velocity_covariance;
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;

    DVLBeam():
      range(0),
      range_covariance(0),
      velocity(0),
      velocity_covariance(0),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_range;
      u_range.real = this->range;
      *(outbuffer + offset + 0) = (u_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_range.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_range.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_range.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_range.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->range);
      union {
        double real;
        uint64_t base;
      } u_range_covariance;
      u_range_covariance.real = this->range_covariance;
      *(outbuffer + offset + 0) = (u_range_covariance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range_covariance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range_covariance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range_covariance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_range_covariance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_range_covariance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_range_covariance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_range_covariance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->range_covariance);
      union {
        double real;
        uint64_t base;
      } u_velocity;
      u_velocity.real = this->velocity;
      *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity);
      union {
        double real;
        uint64_t base;
      } u_velocity_covariance;
      u_velocity_covariance.real = this->velocity_covariance;
      *(outbuffer + offset + 0) = (u_velocity_covariance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_covariance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_covariance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_covariance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity_covariance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity_covariance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity_covariance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity_covariance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity_covariance);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_range;
      u_range.base = 0;
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_range.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->range = u_range.real;
      offset += sizeof(this->range);
      union {
        double real;
        uint64_t base;
      } u_range_covariance;
      u_range_covariance.base = 0;
      u_range_covariance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range_covariance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range_covariance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range_covariance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_range_covariance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_range_covariance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_range_covariance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_range_covariance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->range_covariance = u_range_covariance.real;
      offset += sizeof(this->range_covariance);
      union {
        double real;
        uint64_t base;
      } u_velocity;
      u_velocity.base = 0;
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velocity = u_velocity.real;
      offset += sizeof(this->velocity);
      union {
        double real;
        uint64_t base;
      } u_velocity_covariance;
      u_velocity_covariance.base = 0;
      u_velocity_covariance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_covariance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_covariance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_covariance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velocity_covariance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velocity_covariance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velocity_covariance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velocity_covariance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velocity_covariance = u_velocity_covariance.real;
      offset += sizeof(this->velocity_covariance);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "uuv_sensor_ros_plugins_msgs/DVLBeam"; };
    const char * getMD5(){ return "9e6a08aaf7a740b2c17ba2ab21dac602"; };

  };

}
#endif
