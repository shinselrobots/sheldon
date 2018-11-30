#ifndef _ROS_realsense_ros_object_ObjectArray_h
#define _ROS_realsense_ros_object_ObjectArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_ros_object/Object.h"

namespace realsense_ros_object
{

  class ObjectArray : public ros::Msg
  {
    public:
      uint32_t objects_vector_length;
      typedef realsense_ros_object::Object _objects_vector_type;
      _objects_vector_type st_objects_vector;
      _objects_vector_type * objects_vector;

    ObjectArray():
      objects_vector_length(0), objects_vector(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->objects_vector_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->objects_vector_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->objects_vector_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->objects_vector_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->objects_vector_length);
      for( uint32_t i = 0; i < objects_vector_length; i++){
      offset += this->objects_vector[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t objects_vector_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      objects_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      objects_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      objects_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->objects_vector_length);
      if(objects_vector_lengthT > objects_vector_length)
        this->objects_vector = (realsense_ros_object::Object*)realloc(this->objects_vector, objects_vector_lengthT * sizeof(realsense_ros_object::Object));
      objects_vector_length = objects_vector_lengthT;
      for( uint32_t i = 0; i < objects_vector_length; i++){
      offset += this->st_objects_vector.deserialize(inbuffer + offset);
        memcpy( &(this->objects_vector[i]), &(this->st_objects_vector), sizeof(realsense_ros_object::Object));
      }
     return offset;
    }

    const char * getType(){ return "realsense_ros_object/ObjectArray"; };
    const char * getMD5(){ return "d0eaf0f4593b2bff3780b9347ff0665f"; };

  };

}
#endif