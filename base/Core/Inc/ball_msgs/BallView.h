#ifndef _ROS_ball_msgs_BallView_h
#define _ROS_ball_msgs_BallView_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace ball_msgs
{

  class BallView : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t camera_view_length;
      typedef geometry_msgs::Point _camera_view_type;
      _camera_view_type st_camera_view;
      _camera_view_type * camera_view;
      uint32_t locations_length;
      typedef geometry_msgs::Point _locations_type;
      _locations_type st_locations;
      _locations_type * locations;

    BallView():
      header(),
      camera_view_length(0), camera_view(NULL),
      locations_length(0), locations(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->camera_view_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->camera_view_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->camera_view_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->camera_view_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->camera_view_length);
      for( uint32_t i = 0; i < camera_view_length; i++){
      offset += this->camera_view[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->locations_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->locations_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->locations_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->locations_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->locations_length);
      for( uint32_t i = 0; i < locations_length; i++){
      offset += this->locations[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t camera_view_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      camera_view_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      camera_view_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      camera_view_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->camera_view_length);
      if(camera_view_lengthT > camera_view_length)
        this->camera_view = (geometry_msgs::Point*)realloc(this->camera_view, camera_view_lengthT * sizeof(geometry_msgs::Point));
      camera_view_length = camera_view_lengthT;
      for( uint32_t i = 0; i < camera_view_length; i++){
      offset += this->st_camera_view.deserialize(inbuffer + offset);
        memcpy( &(this->camera_view[i]), &(this->st_camera_view), sizeof(geometry_msgs::Point));
      }
      uint32_t locations_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      locations_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      locations_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      locations_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->locations_length);
      if(locations_lengthT > locations_length)
        this->locations = (geometry_msgs::Point*)realloc(this->locations, locations_lengthT * sizeof(geometry_msgs::Point));
      locations_length = locations_lengthT;
      for( uint32_t i = 0; i < locations_length; i++){
      offset += this->st_locations.deserialize(inbuffer + offset);
        memcpy( &(this->locations[i]), &(this->st_locations), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    const char * getType(){ return "ball_msgs/BallView"; };
    const char * getMD5(){ return "44ebe3e1ce210babda97b8032578aaa1"; };

  };

}
#endif
