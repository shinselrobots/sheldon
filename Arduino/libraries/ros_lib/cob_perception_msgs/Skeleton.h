#ifndef _ROS_cob_perception_msgs_Skeleton_h
#define _ROS_cob_perception_msgs_Skeleton_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace cob_perception_msgs
{

  class Skeleton : public ros::Msg
  {
    public:
      uint32_t joints_length;
      typedef geometry_msgs::Pose _joints_type;
      _joints_type st_joints;
      _joints_type * joints;
      enum { JOINT_HEAD =  0 };
      enum { JOINT_NECK =  1 };
      enum { JOINT_LEFT_SHOULDER =  2 };
      enum { JOINT_RIGHT_SHOULDER =  3 };
      enum { JOINT_LEFT_ELBOW =  4 };
      enum { JOINT_RIGHT_ELBOW =  5 };
      enum { JOINT_LEFT_HAND =  6 };
      enum { JOINT_RIGHT_HAND =  7 };
      enum { JOINT_TORSO =  8 };
      enum { JOINT_LEFT_HIP =  9 };
      enum { JOINT_RIGHT_HIP =  10 };
      enum { JOINT_LEFT_KNEE =  11 };
      enum { JOINT_RIGHT_KNEE =  12 };
      enum { JOINT_LEFT_FOOT =  13 };
      enum { JOINT_RIGHT_FOOT =  14 };
      enum { JOINT_NUMBER =  15 };

    Skeleton():
      joints_length(0), joints(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joints_length);
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->joints[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t joints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joints_length);
      if(joints_lengthT > joints_length)
        this->joints = (geometry_msgs::Pose*)realloc(this->joints, joints_lengthT * sizeof(geometry_msgs::Pose));
      joints_length = joints_lengthT;
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->st_joints.deserialize(inbuffer + offset);
        memcpy( &(this->joints[i]), &(this->st_joints), sizeof(geometry_msgs::Pose));
      }
     return offset;
    }

    const char * getType(){ return "cob_perception_msgs/Skeleton"; };
    const char * getMD5(){ return "2ffad426b6c23b6289166d68b5570546"; };

  };

}
#endif