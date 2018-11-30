#ifndef _ROS_realsense_ros_object_TrackedObjectsArray_h
#define _ROS_realsense_ros_object_TrackedObjectsArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "realsense_ros_object/TrackedObject.h"

namespace realsense_ros_object
{

  class TrackedObjectsArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t tracked_objects_vector_length;
      typedef realsense_ros_object::TrackedObject _tracked_objects_vector_type;
      _tracked_objects_vector_type st_tracked_objects_vector;
      _tracked_objects_vector_type * tracked_objects_vector;

    TrackedObjectsArray():
      header(),
      tracked_objects_vector_length(0), tracked_objects_vector(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->tracked_objects_vector_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tracked_objects_vector_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tracked_objects_vector_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tracked_objects_vector_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracked_objects_vector_length);
      for( uint32_t i = 0; i < tracked_objects_vector_length; i++){
      offset += this->tracked_objects_vector[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t tracked_objects_vector_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tracked_objects_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tracked_objects_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tracked_objects_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tracked_objects_vector_length);
      if(tracked_objects_vector_lengthT > tracked_objects_vector_length)
        this->tracked_objects_vector = (realsense_ros_object::TrackedObject*)realloc(this->tracked_objects_vector, tracked_objects_vector_lengthT * sizeof(realsense_ros_object::TrackedObject));
      tracked_objects_vector_length = tracked_objects_vector_lengthT;
      for( uint32_t i = 0; i < tracked_objects_vector_length; i++){
      offset += this->st_tracked_objects_vector.deserialize(inbuffer + offset);
        memcpy( &(this->tracked_objects_vector[i]), &(this->st_tracked_objects_vector), sizeof(realsense_ros_object::TrackedObject));
      }
     return offset;
    }

    const char * getType(){ return "realsense_ros_object/TrackedObjectsArray"; };
    const char * getMD5(){ return "eba302ff867767c2d7dce5a9b84a6ec3"; };

  };

}
#endif