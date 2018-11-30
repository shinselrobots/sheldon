#ifndef _ROS_realsense_ros_person_SkeletonJoint_h
#define _ROS_realsense_ros_person_SkeletonJoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"

namespace realsense_ros_person
{

  class SkeletonJoint : public ros::Msg
  {
    public:
      typedef int32_t _type_type;
      _type_type type;
      typedef float _confidence_type;
      _confidence_type confidence;
      typedef geometry_msgs::Point _location_type;
      _location_type location;
      typedef geometry_msgs::Point32 _realWorldCoordinates_type;
      _realWorldCoordinates_type realWorldCoordinates;
      enum { JOINT_ANKLE_LEFT = 0 };
      enum { JOINT_ANKLE_RIGHT = 1 };
      enum { JOINT_ELBOW_LEFT = 2 };
      enum { JOINT_ELBOW_RIGHT = 3 };
      enum { JOINT_FOOT_LEFT = 4 };
      enum { JOINT_FOOT_RIGHT = 5 };
      enum { JOINT_HAND_LEFT = 6 };
      enum { JOINT_HAND_RIGHT = 7 };
      enum { JOINT_HAND_TIP_LEFT = 8 };
      enum { JOINT_HAND_TIP_RIGHT = 9 };
      enum { JOINT_HEAD = 10 };
      enum { JOINT_HIP_LEFT = 11 };
      enum { JOINT_HIP_RIGHT = 12 };
      enum { JOINT_KNEE_LEFT = 13 };
      enum { JOINT_KNEE_RIGHT = 14 };
      enum { JOINT_NECK = 15 };
      enum { JOINT_SHOULDER_LEFT = 16 };
      enum { JOINT_SHOULDER_RIGHT = 17 };
      enum { JOINT_SPINE_BASE = 18 };
      enum { JOINT_SPINE_MID = 19 };
      enum { JOINT_SPINE_SHOULDER = 20 };
      enum { JOINT_THUMB_LEFT = 21 };
      enum { JOINT_THUMB_RIGHT = 22 };
      enum { JOINT_WRIST_LEFT = 23 };
      enum { JOINT_WRIST_RIGHT = 24 };
      enum { JOINT_UNKNOWN = 25 };

    SkeletonJoint():
      type(0),
      confidence(0),
      location(),
      realWorldCoordinates()
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
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence);
      offset += this->location.serialize(outbuffer + offset);
      offset += this->realWorldCoordinates.serialize(outbuffer + offset);
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
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
      offset += this->location.deserialize(inbuffer + offset);
      offset += this->realWorldCoordinates.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/SkeletonJoint"; };
    const char * getMD5(){ return "dfca6d8f4a6c4e847fc126d5f1b7b05b"; };

  };

}
#endif