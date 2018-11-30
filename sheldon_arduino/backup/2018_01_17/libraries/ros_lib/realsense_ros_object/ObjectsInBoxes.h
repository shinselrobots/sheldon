#ifndef _ROS_realsense_ros_object_ObjectsInBoxes_h
#define _ROS_realsense_ros_object_ObjectsInBoxes_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "realsense_ros_object/ObjectInBox.h"

namespace realsense_ros_object
{

  class ObjectsInBoxes : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t objects_vector_length;
      typedef realsense_ros_object::ObjectInBox _objects_vector_type;
      _objects_vector_type st_objects_vector;
      _objects_vector_type * objects_vector;

    ObjectsInBoxes():
      header(),
      objects_vector_length(0), objects_vector(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t objects_vector_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      objects_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      objects_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      objects_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->objects_vector_length);
      if(objects_vector_lengthT > objects_vector_length)
        this->objects_vector = (realsense_ros_object::ObjectInBox*)realloc(this->objects_vector, objects_vector_lengthT * sizeof(realsense_ros_object::ObjectInBox));
      objects_vector_length = objects_vector_lengthT;
      for( uint32_t i = 0; i < objects_vector_length; i++){
      offset += this->st_objects_vector.deserialize(inbuffer + offset);
        memcpy( &(this->objects_vector[i]), &(this->st_objects_vector), sizeof(realsense_ros_object::ObjectInBox));
      }
     return offset;
    }

    const char * getType(){ return "realsense_ros_object/ObjectsInBoxes"; };
    const char * getMD5(){ return "766265c76170a5de1a5179aa1e37cf42"; };

  };

}
#endif