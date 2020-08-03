#ifndef _ROS_glider_SonarProfile_h
#define _ROS_glider_SonarProfile_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace glider
{

  class SonarProfile : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t profile_length;
      typedef uint8_t _profile_type;
      _profile_type st_profile;
      _profile_type * profile;
      typedef uint16_t _pulse_us_type;
      _pulse_us_type pulse_us;
      typedef uint32_t _ping_number_type;
      _ping_number_type ping_number;
      typedef uint32_t _scan_start_mm_type;
      _scan_start_mm_type scan_start_mm;
      typedef uint32_t _scan_length_mm_type;
      _scan_length_mm_type scan_length_mm;
      typedef uint8_t _gain_index_type;
      _gain_index_type gain_index;

    SonarProfile():
      header(),
      profile_length(0), profile(NULL),
      pulse_us(0),
      ping_number(0),
      scan_start_mm(0),
      scan_length_mm(0),
      gain_index(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->profile_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->profile_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->profile_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->profile_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->profile_length);
      for( uint32_t i = 0; i < profile_length; i++){
      *(outbuffer + offset + 0) = (this->profile[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->profile[i]);
      }
      *(outbuffer + offset + 0) = (this->pulse_us >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pulse_us >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pulse_us);
      *(outbuffer + offset + 0) = (this->ping_number >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ping_number >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ping_number >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ping_number >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ping_number);
      *(outbuffer + offset + 0) = (this->scan_start_mm >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scan_start_mm >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scan_start_mm >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scan_start_mm >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scan_start_mm);
      *(outbuffer + offset + 0) = (this->scan_length_mm >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scan_length_mm >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scan_length_mm >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scan_length_mm >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scan_length_mm);
      *(outbuffer + offset + 0) = (this->gain_index >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gain_index);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t profile_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      profile_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      profile_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      profile_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->profile_length);
      if(profile_lengthT > profile_length)
        this->profile = (uint8_t*)realloc(this->profile, profile_lengthT * sizeof(uint8_t));
      profile_length = profile_lengthT;
      for( uint32_t i = 0; i < profile_length; i++){
      this->st_profile =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_profile);
        memcpy( &(this->profile[i]), &(this->st_profile), sizeof(uint8_t));
      }
      this->pulse_us =  ((uint16_t) (*(inbuffer + offset)));
      this->pulse_us |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pulse_us);
      this->ping_number =  ((uint32_t) (*(inbuffer + offset)));
      this->ping_number |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ping_number |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->ping_number |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->ping_number);
      this->scan_start_mm =  ((uint32_t) (*(inbuffer + offset)));
      this->scan_start_mm |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->scan_start_mm |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->scan_start_mm |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->scan_start_mm);
      this->scan_length_mm =  ((uint32_t) (*(inbuffer + offset)));
      this->scan_length_mm |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->scan_length_mm |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->scan_length_mm |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->scan_length_mm);
      this->gain_index =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gain_index);
     return offset;
    }

    const char * getType(){ return "glider/SonarProfile"; };
    const char * getMD5(){ return "acbeeebd35f6cf0446714bc609ac9ba2"; };

  };

}
#endif
