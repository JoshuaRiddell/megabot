#ifndef _ROS_uuv_gazebo_ros_plugins_msgs_ThrusterConversionFcn_h
#define _ROS_uuv_gazebo_ros_plugins_msgs_ThrusterConversionFcn_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uuv_gazebo_ros_plugins_msgs
{

  class ThrusterConversionFcn : public ros::Msg
  {
    public:
      typedef const char* _function_name_type;
      _function_name_type function_name;
      uint32_t tags_length;
      typedef char* _tags_type;
      _tags_type st_tags;
      _tags_type * tags;
      uint32_t data_length;
      typedef double _data_type;
      _data_type st_data;
      _data_type * data;
      uint32_t lookup_table_input_length;
      typedef double _lookup_table_input_type;
      _lookup_table_input_type st_lookup_table_input;
      _lookup_table_input_type * lookup_table_input;
      uint32_t lookup_table_output_length;
      typedef double _lookup_table_output_type;
      _lookup_table_output_type st_lookup_table_output;
      _lookup_table_output_type * lookup_table_output;

    ThrusterConversionFcn():
      function_name(""),
      tags_length(0), tags(NULL),
      data_length(0), data(NULL),
      lookup_table_input_length(0), lookup_table_input(NULL),
      lookup_table_output_length(0), lookup_table_output(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_function_name = strlen(this->function_name);
      varToArr(outbuffer + offset, length_function_name);
      offset += 4;
      memcpy(outbuffer + offset, this->function_name, length_function_name);
      offset += length_function_name;
      *(outbuffer + offset + 0) = (this->tags_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tags_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tags_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tags_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tags_length);
      for( uint32_t i = 0; i < tags_length; i++){
      uint32_t length_tagsi = strlen(this->tags[i]);
      varToArr(outbuffer + offset, length_tagsi);
      offset += 4;
      memcpy(outbuffer + offset, this->tags[i], length_tagsi);
      offset += length_tagsi;
      }
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      union {
        double real;
        uint64_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_datai.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_datai.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_datai.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_datai.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      *(outbuffer + offset + 0) = (this->lookup_table_input_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lookup_table_input_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lookup_table_input_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lookup_table_input_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lookup_table_input_length);
      for( uint32_t i = 0; i < lookup_table_input_length; i++){
      union {
        double real;
        uint64_t base;
      } u_lookup_table_inputi;
      u_lookup_table_inputi.real = this->lookup_table_input[i];
      *(outbuffer + offset + 0) = (u_lookup_table_inputi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lookup_table_inputi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lookup_table_inputi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lookup_table_inputi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_lookup_table_inputi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_lookup_table_inputi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_lookup_table_inputi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_lookup_table_inputi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->lookup_table_input[i]);
      }
      *(outbuffer + offset + 0) = (this->lookup_table_output_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lookup_table_output_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lookup_table_output_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lookup_table_output_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lookup_table_output_length);
      for( uint32_t i = 0; i < lookup_table_output_length; i++){
      union {
        double real;
        uint64_t base;
      } u_lookup_table_outputi;
      u_lookup_table_outputi.real = this->lookup_table_output[i];
      *(outbuffer + offset + 0) = (u_lookup_table_outputi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lookup_table_outputi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lookup_table_outputi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lookup_table_outputi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_lookup_table_outputi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_lookup_table_outputi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_lookup_table_outputi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_lookup_table_outputi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->lookup_table_output[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_function_name;
      arrToVar(length_function_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_function_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_function_name-1]=0;
      this->function_name = (char *)(inbuffer + offset-1);
      offset += length_function_name;
      uint32_t tags_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tags_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tags_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tags_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tags_length);
      if(tags_lengthT > tags_length)
        this->tags = (char**)realloc(this->tags, tags_lengthT * sizeof(char*));
      tags_length = tags_lengthT;
      for( uint32_t i = 0; i < tags_length; i++){
      uint32_t length_st_tags;
      arrToVar(length_st_tags, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_tags; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_tags-1]=0;
      this->st_tags = (char *)(inbuffer + offset-1);
      offset += length_st_tags;
        memcpy( &(this->tags[i]), &(this->st_tags), sizeof(char*));
      }
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (double*)realloc(this->data, data_lengthT * sizeof(double));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_data.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_data.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_data.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_data.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_data.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_data.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(double));
      }
      uint32_t lookup_table_input_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      lookup_table_input_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      lookup_table_input_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      lookup_table_input_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->lookup_table_input_length);
      if(lookup_table_input_lengthT > lookup_table_input_length)
        this->lookup_table_input = (double*)realloc(this->lookup_table_input, lookup_table_input_lengthT * sizeof(double));
      lookup_table_input_length = lookup_table_input_lengthT;
      for( uint32_t i = 0; i < lookup_table_input_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_lookup_table_input;
      u_st_lookup_table_input.base = 0;
      u_st_lookup_table_input.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_lookup_table_input.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_lookup_table_input.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_lookup_table_input.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_lookup_table_input.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_lookup_table_input.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_lookup_table_input.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_lookup_table_input.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_lookup_table_input = u_st_lookup_table_input.real;
      offset += sizeof(this->st_lookup_table_input);
        memcpy( &(this->lookup_table_input[i]), &(this->st_lookup_table_input), sizeof(double));
      }
      uint32_t lookup_table_output_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      lookup_table_output_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      lookup_table_output_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      lookup_table_output_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->lookup_table_output_length);
      if(lookup_table_output_lengthT > lookup_table_output_length)
        this->lookup_table_output = (double*)realloc(this->lookup_table_output, lookup_table_output_lengthT * sizeof(double));
      lookup_table_output_length = lookup_table_output_lengthT;
      for( uint32_t i = 0; i < lookup_table_output_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_lookup_table_output;
      u_st_lookup_table_output.base = 0;
      u_st_lookup_table_output.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_lookup_table_output.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_lookup_table_output.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_lookup_table_output.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_lookup_table_output.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_lookup_table_output.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_lookup_table_output.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_lookup_table_output.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_lookup_table_output = u_st_lookup_table_output.real;
      offset += sizeof(this->st_lookup_table_output);
        memcpy( &(this->lookup_table_output[i]), &(this->st_lookup_table_output), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "uuv_gazebo_ros_plugins_msgs/ThrusterConversionFcn"; };
    const char * getMD5(){ return "5cc7c4f30276fbc995f2325f64846776"; };

  };

}
#endif
