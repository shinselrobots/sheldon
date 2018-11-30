#ifndef _ROS_realsense_ros_object_Object_h
#define _ROS_realsense_ros_object_Object_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_object
{

  class Object : public ros::Msg
  {
    public:
      typedef int32_t _label_type;
      _label_type label;
      typedef const char* _object_name_type;
      _object_name_type object_name;
      typedef float _probability_type;
      _probability_type probability;

    Object():
      label(0),
      object_name(""),
      probability(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_label;
      u_label.real = this->label;
      *(outbuffer + offset + 0) = (u_label.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_label.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_label.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_label.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->label);
      uint32_t length_object_name = strlen(this->object_name);
      varToArr(outbuffer + offset, length_object_name);
      offset += 4;
      memcpy(outbuffer + offset, this->object_name, length_object_name);
      offset += length_object_name;
      union {
        float real;
        uint32_t base;
      } u_probability;
      u_probability.real = this->probability;
      *(outbuffer + offset + 0) = (u_probability.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_probability.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_probability.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_probability.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->probability);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_label;
      u_label.base = 0;
      u_label.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_label.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_label.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_label.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->label = u_label.real;
      offset += sizeof(this->label);
      uint32_t length_object_name;
      arrToVar(length_object_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_object_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_object_name-1]=0;
      this->object_name = (char *)(inbuffer + offset-1);
      offset += length_object_name;
      union {
        float real;
        uint32_t base;
      } u_probability;
      u_probability.base = 0;
      u_probability.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_probability.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_probability.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_probability.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->probability = u_probability.real;
      offset += sizeof(this->probability);
     return offset;
    }

    const char * getType(){ return "realsense_ros_object/Object"; };
    const char * getMD5(){ return "ec4d2c0f7ede0d9835355adac9d9a4e1"; };

  };

}
#endif