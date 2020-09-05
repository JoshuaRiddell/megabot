#ifndef _ROS_uuv_auv_control_allocator_AUVCommand_h
#define _ROS_uuv_auv_control_allocator_AUVCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Wrench.h"

namespace uuv_auv_control_allocator
{

  class AUVCommand : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef double _surge_speed_type;
      _surge_speed_type surge_speed;
      typedef geometry_msgs::Wrench _command_type;
      _command_type command;

    AUVCommand():
      header(),
      surge_speed(0),
      command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_surge_speed;
      u_surge_speed.real = this->surge_speed;
      *(outbuffer + offset + 0) = (u_surge_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_surge_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_surge_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_surge_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_surge_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_surge_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_surge_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_surge_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->surge_speed);
      offset += this->command.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_surge_speed;
      u_surge_speed.base = 0;
      u_surge_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_surge_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_surge_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_surge_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_surge_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_surge_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_surge_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_surge_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->surge_speed = u_surge_speed.real;
      offset += sizeof(this->surge_speed);
      offset += this->command.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "uuv_auv_control_allocator/AUVCommand"; };
    const char * getMD5(){ return "9d7c962f08b7f118399273df23351e7c"; };

  };

}
#endif
