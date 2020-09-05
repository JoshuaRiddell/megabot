#ifndef _ROS_uuv_gazebo_ros_plugins_msgs_UnderwaterObjectModel_h
#define _ROS_uuv_gazebo_ros_plugins_msgs_UnderwaterObjectModel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Inertia.h"

namespace uuv_gazebo_ros_plugins_msgs
{

  class UnderwaterObjectModel : public ros::Msg
  {
    public:
      uint32_t added_mass_length;
      typedef double _added_mass_type;
      _added_mass_type st_added_mass;
      _added_mass_type * added_mass;
      uint32_t linear_damping_length;
      typedef double _linear_damping_type;
      _linear_damping_type st_linear_damping;
      _linear_damping_type * linear_damping;
      uint32_t linear_damping_forward_speed_length;
      typedef double _linear_damping_forward_speed_type;
      _linear_damping_forward_speed_type st_linear_damping_forward_speed;
      _linear_damping_forward_speed_type * linear_damping_forward_speed;
      uint32_t quadratic_damping_length;
      typedef double _quadratic_damping_type;
      _quadratic_damping_type st_quadratic_damping;
      _quadratic_damping_type * quadratic_damping;
      typedef double _volume_type;
      _volume_type volume;
      typedef double _bbox_height_type;
      _bbox_height_type bbox_height;
      typedef double _bbox_length_type;
      _bbox_length_type bbox_length;
      typedef double _bbox_width_type;
      _bbox_width_type bbox_width;
      typedef double _fluid_density_type;
      _fluid_density_type fluid_density;
      typedef geometry_msgs::Vector3 _cob_type;
      _cob_type cob;
      typedef bool _neutrally_buoyant_type;
      _neutrally_buoyant_type neutrally_buoyant;
      typedef geometry_msgs::Inertia _inertia_type;
      _inertia_type inertia;

    UnderwaterObjectModel():
      added_mass_length(0), added_mass(NULL),
      linear_damping_length(0), linear_damping(NULL),
      linear_damping_forward_speed_length(0), linear_damping_forward_speed(NULL),
      quadratic_damping_length(0), quadratic_damping(NULL),
      volume(0),
      bbox_height(0),
      bbox_length(0),
      bbox_width(0),
      fluid_density(0),
      cob(),
      neutrally_buoyant(0),
      inertia()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->added_mass_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->added_mass_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->added_mass_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->added_mass_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->added_mass_length);
      for( uint32_t i = 0; i < added_mass_length; i++){
      union {
        double real;
        uint64_t base;
      } u_added_massi;
      u_added_massi.real = this->added_mass[i];
      *(outbuffer + offset + 0) = (u_added_massi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_added_massi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_added_massi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_added_massi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_added_massi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_added_massi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_added_massi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_added_massi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->added_mass[i]);
      }
      *(outbuffer + offset + 0) = (this->linear_damping_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->linear_damping_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->linear_damping_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->linear_damping_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_damping_length);
      for( uint32_t i = 0; i < linear_damping_length; i++){
      union {
        double real;
        uint64_t base;
      } u_linear_dampingi;
      u_linear_dampingi.real = this->linear_damping[i];
      *(outbuffer + offset + 0) = (u_linear_dampingi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_dampingi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_dampingi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_dampingi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_linear_dampingi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_linear_dampingi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_linear_dampingi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_linear_dampingi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->linear_damping[i]);
      }
      *(outbuffer + offset + 0) = (this->linear_damping_forward_speed_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->linear_damping_forward_speed_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->linear_damping_forward_speed_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->linear_damping_forward_speed_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_damping_forward_speed_length);
      for( uint32_t i = 0; i < linear_damping_forward_speed_length; i++){
      union {
        double real;
        uint64_t base;
      } u_linear_damping_forward_speedi;
      u_linear_damping_forward_speedi.real = this->linear_damping_forward_speed[i];
      *(outbuffer + offset + 0) = (u_linear_damping_forward_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_damping_forward_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_damping_forward_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_damping_forward_speedi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_linear_damping_forward_speedi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_linear_damping_forward_speedi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_linear_damping_forward_speedi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_linear_damping_forward_speedi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->linear_damping_forward_speed[i]);
      }
      *(outbuffer + offset + 0) = (this->quadratic_damping_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->quadratic_damping_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->quadratic_damping_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->quadratic_damping_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->quadratic_damping_length);
      for( uint32_t i = 0; i < quadratic_damping_length; i++){
      union {
        double real;
        uint64_t base;
      } u_quadratic_dampingi;
      u_quadratic_dampingi.real = this->quadratic_damping[i];
      *(outbuffer + offset + 0) = (u_quadratic_dampingi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_quadratic_dampingi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_quadratic_dampingi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_quadratic_dampingi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_quadratic_dampingi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_quadratic_dampingi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_quadratic_dampingi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_quadratic_dampingi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->quadratic_damping[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_volume;
      u_volume.real = this->volume;
      *(outbuffer + offset + 0) = (u_volume.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_volume.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_volume.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_volume.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_volume.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_volume.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_volume.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_volume.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->volume);
      union {
        double real;
        uint64_t base;
      } u_bbox_height;
      u_bbox_height.real = this->bbox_height;
      *(outbuffer + offset + 0) = (u_bbox_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bbox_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bbox_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bbox_height.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bbox_height.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bbox_height.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bbox_height.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bbox_height.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bbox_height);
      union {
        double real;
        uint64_t base;
      } u_bbox_length;
      u_bbox_length.real = this->bbox_length;
      *(outbuffer + offset + 0) = (u_bbox_length.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bbox_length.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bbox_length.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bbox_length.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bbox_length.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bbox_length.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bbox_length.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bbox_length.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bbox_length);
      union {
        double real;
        uint64_t base;
      } u_bbox_width;
      u_bbox_width.real = this->bbox_width;
      *(outbuffer + offset + 0) = (u_bbox_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bbox_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bbox_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bbox_width.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bbox_width.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bbox_width.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bbox_width.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bbox_width.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bbox_width);
      union {
        double real;
        uint64_t base;
      } u_fluid_density;
      u_fluid_density.real = this->fluid_density;
      *(outbuffer + offset + 0) = (u_fluid_density.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fluid_density.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fluid_density.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fluid_density.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_fluid_density.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_fluid_density.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_fluid_density.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_fluid_density.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->fluid_density);
      offset += this->cob.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_neutrally_buoyant;
      u_neutrally_buoyant.real = this->neutrally_buoyant;
      *(outbuffer + offset + 0) = (u_neutrally_buoyant.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->neutrally_buoyant);
      offset += this->inertia.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t added_mass_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      added_mass_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      added_mass_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      added_mass_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->added_mass_length);
      if(added_mass_lengthT > added_mass_length)
        this->added_mass = (double*)realloc(this->added_mass, added_mass_lengthT * sizeof(double));
      added_mass_length = added_mass_lengthT;
      for( uint32_t i = 0; i < added_mass_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_added_mass;
      u_st_added_mass.base = 0;
      u_st_added_mass.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_added_mass.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_added_mass.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_added_mass.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_added_mass.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_added_mass.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_added_mass.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_added_mass.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_added_mass = u_st_added_mass.real;
      offset += sizeof(this->st_added_mass);
        memcpy( &(this->added_mass[i]), &(this->st_added_mass), sizeof(double));
      }
      uint32_t linear_damping_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      linear_damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      linear_damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      linear_damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->linear_damping_length);
      if(linear_damping_lengthT > linear_damping_length)
        this->linear_damping = (double*)realloc(this->linear_damping, linear_damping_lengthT * sizeof(double));
      linear_damping_length = linear_damping_lengthT;
      for( uint32_t i = 0; i < linear_damping_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_linear_damping;
      u_st_linear_damping.base = 0;
      u_st_linear_damping.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_linear_damping.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_linear_damping.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_linear_damping.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_linear_damping.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_linear_damping.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_linear_damping.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_linear_damping.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_linear_damping = u_st_linear_damping.real;
      offset += sizeof(this->st_linear_damping);
        memcpy( &(this->linear_damping[i]), &(this->st_linear_damping), sizeof(double));
      }
      uint32_t linear_damping_forward_speed_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      linear_damping_forward_speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      linear_damping_forward_speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      linear_damping_forward_speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->linear_damping_forward_speed_length);
      if(linear_damping_forward_speed_lengthT > linear_damping_forward_speed_length)
        this->linear_damping_forward_speed = (double*)realloc(this->linear_damping_forward_speed, linear_damping_forward_speed_lengthT * sizeof(double));
      linear_damping_forward_speed_length = linear_damping_forward_speed_lengthT;
      for( uint32_t i = 0; i < linear_damping_forward_speed_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_linear_damping_forward_speed;
      u_st_linear_damping_forward_speed.base = 0;
      u_st_linear_damping_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_linear_damping_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_linear_damping_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_linear_damping_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_linear_damping_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_linear_damping_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_linear_damping_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_linear_damping_forward_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_linear_damping_forward_speed = u_st_linear_damping_forward_speed.real;
      offset += sizeof(this->st_linear_damping_forward_speed);
        memcpy( &(this->linear_damping_forward_speed[i]), &(this->st_linear_damping_forward_speed), sizeof(double));
      }
      uint32_t quadratic_damping_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      quadratic_damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      quadratic_damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      quadratic_damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->quadratic_damping_length);
      if(quadratic_damping_lengthT > quadratic_damping_length)
        this->quadratic_damping = (double*)realloc(this->quadratic_damping, quadratic_damping_lengthT * sizeof(double));
      quadratic_damping_length = quadratic_damping_lengthT;
      for( uint32_t i = 0; i < quadratic_damping_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_quadratic_damping;
      u_st_quadratic_damping.base = 0;
      u_st_quadratic_damping.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_quadratic_damping.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_quadratic_damping.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_quadratic_damping.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_quadratic_damping.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_quadratic_damping.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_quadratic_damping.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_quadratic_damping.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_quadratic_damping = u_st_quadratic_damping.real;
      offset += sizeof(this->st_quadratic_damping);
        memcpy( &(this->quadratic_damping[i]), &(this->st_quadratic_damping), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_volume;
      u_volume.base = 0;
      u_volume.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_volume.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_volume.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_volume.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_volume.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_volume.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_volume.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_volume.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->volume = u_volume.real;
      offset += sizeof(this->volume);
      union {
        double real;
        uint64_t base;
      } u_bbox_height;
      u_bbox_height.base = 0;
      u_bbox_height.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bbox_height.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bbox_height.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bbox_height.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_bbox_height.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_bbox_height.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_bbox_height.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_bbox_height.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->bbox_height = u_bbox_height.real;
      offset += sizeof(this->bbox_height);
      union {
        double real;
        uint64_t base;
      } u_bbox_length;
      u_bbox_length.base = 0;
      u_bbox_length.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bbox_length.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bbox_length.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bbox_length.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_bbox_length.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_bbox_length.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_bbox_length.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_bbox_length.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->bbox_length = u_bbox_length.real;
      offset += sizeof(this->bbox_length);
      union {
        double real;
        uint64_t base;
      } u_bbox_width;
      u_bbox_width.base = 0;
      u_bbox_width.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bbox_width.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bbox_width.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bbox_width.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_bbox_width.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_bbox_width.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_bbox_width.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_bbox_width.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->bbox_width = u_bbox_width.real;
      offset += sizeof(this->bbox_width);
      union {
        double real;
        uint64_t base;
      } u_fluid_density;
      u_fluid_density.base = 0;
      u_fluid_density.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fluid_density.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fluid_density.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fluid_density.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_fluid_density.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_fluid_density.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_fluid_density.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_fluid_density.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->fluid_density = u_fluid_density.real;
      offset += sizeof(this->fluid_density);
      offset += this->cob.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_neutrally_buoyant;
      u_neutrally_buoyant.base = 0;
      u_neutrally_buoyant.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->neutrally_buoyant = u_neutrally_buoyant.real;
      offset += sizeof(this->neutrally_buoyant);
      offset += this->inertia.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel"; };
    const char * getMD5(){ return "35ada57addb7202af96020ea2ddc109c"; };

  };

}
#endif
