#ifndef _ROS_glider_SonarConfiguration_h
#define _ROS_glider_SonarConfiguration_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace glider
{

  class SonarConfiguration : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _device_type_type;
      _device_type_type device_type;
      typedef uint8_t _device_model_type;
      _device_model_type device_model;
      typedef uint16_t _fw_version_major_type;
      _fw_version_major_type fw_version_major;
      typedef uint16_t _fw_version_minor_type;
      _fw_version_minor_type fw_version_minor;
      typedef uint32_t _speed_of_sound_type;
      _speed_of_sound_type speed_of_sound;
      typedef uint8_t _auto_mode_type;
      _auto_mode_type auto_mode;
      typedef uint16_t _ping_rate_ms_type;
      _ping_rate_ms_type ping_rate_ms;
      typedef uint8_t _gain_index_type;
      _gain_index_type gain_index;
      enum { GAIN_0dB6 =  0 };
      enum { GAIN_1dB8 =  1 };
      enum { GAIN_5dB5 =  2 };
      enum { GAIN_12dB9 =  3 };
      enum { GAIN_30dB2 =  4 };
      enum { GAIN_66dB1 =  5 };
      enum { GAIN_144dB =  6 };

    SonarConfiguration():
      header(),
      device_type(0),
      device_model(0),
      fw_version_major(0),
      fw_version_minor(0),
      speed_of_sound(0),
      auto_mode(0),
      ping_rate_ms(0),
      gain_index(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->device_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->device_type);
      *(outbuffer + offset + 0) = (this->device_model >> (8 * 0)) & 0xFF;
      offset += sizeof(this->device_model);
      *(outbuffer + offset + 0) = (this->fw_version_major >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fw_version_major >> (8 * 1)) & 0xFF;
      offset += sizeof(this->fw_version_major);
      *(outbuffer + offset + 0) = (this->fw_version_minor >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fw_version_minor >> (8 * 1)) & 0xFF;
      offset += sizeof(this->fw_version_minor);
      *(outbuffer + offset + 0) = (this->speed_of_sound >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speed_of_sound >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->speed_of_sound >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->speed_of_sound >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_of_sound);
      *(outbuffer + offset + 0) = (this->auto_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->auto_mode);
      *(outbuffer + offset + 0) = (this->ping_rate_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ping_rate_ms >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ping_rate_ms);
      *(outbuffer + offset + 0) = (this->gain_index >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gain_index);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->device_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->device_type);
      this->device_model =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->device_model);
      this->fw_version_major =  ((uint16_t) (*(inbuffer + offset)));
      this->fw_version_major |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->fw_version_major);
      this->fw_version_minor =  ((uint16_t) (*(inbuffer + offset)));
      this->fw_version_minor |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->fw_version_minor);
      this->speed_of_sound =  ((uint32_t) (*(inbuffer + offset)));
      this->speed_of_sound |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed_of_sound |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->speed_of_sound |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->speed_of_sound);
      this->auto_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->auto_mode);
      this->ping_rate_ms =  ((uint16_t) (*(inbuffer + offset)));
      this->ping_rate_ms |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ping_rate_ms);
      this->gain_index =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gain_index);
     return offset;
    }

    const char * getType(){ return "glider/SonarConfiguration"; };
    const char * getMD5(){ return "f1cb1aa4229974c81bf9b4207c3da5d3"; };

  };

}
#endif
