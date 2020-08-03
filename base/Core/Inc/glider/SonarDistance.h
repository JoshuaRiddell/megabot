#ifndef _ROS_glider_SonarDistance_h
#define _ROS_glider_SonarDistance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace glider
{

  class SonarDistance : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _distance_type;
      _distance_type distance;
      typedef uint16_t _confidence_type;
      _confidence_type confidence;

    SonarDistance():
      header(),
      distance(0),
      confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->distance >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->distance >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->distance >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->distance >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      *(outbuffer + offset + 0) = (this->confidence >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->confidence >> (8 * 1)) & 0xFF;
      offset += sizeof(this->confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->distance =  ((uint32_t) (*(inbuffer + offset)));
      this->distance |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->distance |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->distance |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->distance);
      this->confidence =  ((uint16_t) (*(inbuffer + offset)));
      this->confidence |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->confidence);
     return offset;
    }

    const char * getType(){ return "glider/SonarDistance"; };
    const char * getMD5(){ return "19b0ab47929c1b8d1c3e7f26c10e2db1"; };

  };

}
#endif
