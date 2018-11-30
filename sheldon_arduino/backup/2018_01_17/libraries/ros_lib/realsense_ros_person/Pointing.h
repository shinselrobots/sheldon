#ifndef _ROS_realsense_ros_person_Pointing_h
#define _ROS_realsense_ros_person_Pointing_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Vector3.h"

namespace realsense_ros_person
{

  class Pointing : public ros::Msg
  {
    public:
      typedef int32_t _confidence_type;
      _confidence_type confidence;
      typedef geometry_msgs::Point32 _originColor_type;
      _originColor_type originColor;
      typedef geometry_msgs::Point32 _originWorld_type;
      _originWorld_type originWorld;
      typedef geometry_msgs::Point32 _orientationColor_type;
      _orientationColor_type orientationColor;
      typedef geometry_msgs::Vector3 _orientationWorld_type;
      _orientationWorld_type orientationWorld;

    Pointing():
      confidence(0),
      originColor(),
      originWorld(),
      orientationColor(),
      orientationWorld()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence);
      offset += this->originColor.serialize(outbuffer + offset);
      offset += this->originWorld.serialize(outbuffer + offset);
      offset += this->orientationColor.serialize(outbuffer + offset);
      offset += this->orientationWorld.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
      offset += this->originColor.deserialize(inbuffer + offset);
      offset += this->originWorld.deserialize(inbuffer + offset);
      offset += this->orientationColor.deserialize(inbuffer + offset);
      offset += this->orientationWorld.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/Pointing"; };
    const char * getMD5(){ return "e8e17bba059ae94c10bd49b0a03744e0"; };

  };

}
#endif