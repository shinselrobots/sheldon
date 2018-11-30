#ifndef _ROS_realsense_ros_person_EulerAnglesWithConfidence_h
#define _ROS_realsense_ros_person_EulerAnglesWithConfidence_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_ros_person/EulerAngles.h"

namespace realsense_ros_person
{

  class EulerAnglesWithConfidence : public ros::Msg
  {
    public:
      typedef realsense_ros_person::EulerAngles _angles_type;
      _angles_type angles;
      typedef int32_t _confidence_type;
      _confidence_type confidence;

    EulerAnglesWithConfidence():
      angles(),
      confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->angles.serialize(outbuffer + offset);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->angles.deserialize(inbuffer + offset);
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
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/EulerAnglesWithConfidence"; };
    const char * getMD5(){ return "5e76354a4d10460adcef4d7d5dd74f1f"; };

  };

}
#endif