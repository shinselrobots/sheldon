#ifndef _ROS_realsense_ros_person_Gesture_h
#define _ROS_realsense_ros_person_Gesture_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Vector3.h"

namespace realsense_ros_person
{

  class Gesture : public ros::Msg
  {
    public:
      typedef int32_t _type_type;
      _type_type type;
      typedef geometry_msgs::Point32 _vectorOrigin_type;
      _vectorOrigin_type vectorOrigin;
      typedef geometry_msgs::Vector3 _pointingVector_type;
      _pointingVector_type pointingVector;

    Gesture():
      type(0),
      vectorOrigin(),
      pointingVector()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      offset += this->vectorOrigin.serialize(outbuffer + offset);
      offset += this->pointingVector.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->type = u_type.real;
      offset += sizeof(this->type);
      offset += this->vectorOrigin.deserialize(inbuffer + offset);
      offset += this->pointingVector.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/Gesture"; };
    const char * getMD5(){ return "d46ba8b59c5b03a022b488e568706275"; };

  };

}
#endif