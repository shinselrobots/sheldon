#ifndef _ROS_realsense_ros_person_RectWithConfidence_h
#define _ROS_realsense_ros_person_RectWithConfidence_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point32.h"

namespace realsense_ros_person
{

  class RectWithConfidence : public ros::Msg
  {
    public:
      typedef int32_t _confidence_type;
      _confidence_type confidence;
      geometry_msgs::Point32 rectCorners[2];

    RectWithConfidence():
      confidence(0),
      rectCorners()
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
      for( uint32_t i = 0; i < 2; i++){
      offset += this->rectCorners[i].serialize(outbuffer + offset);
      }
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
      for( uint32_t i = 0; i < 2; i++){
      offset += this->rectCorners[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/RectWithConfidence"; };
    const char * getMD5(){ return "a4605f6e3ec78453799435502960c372"; };

  };

}
#endif