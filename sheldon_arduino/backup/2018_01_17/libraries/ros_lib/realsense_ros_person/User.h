#ifndef _ROS_realsense_ros_person_User_h
#define _ROS_realsense_ros_person_User_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_ros_person/UserInfo.h"
#include "geometry_msgs/Point32.h"
#include "realsense_ros_person/RectWithConfidence.h"
#include "realsense_ros_person/LandmarksInfo.h"
#include "realsense_ros_person/EulerAnglesWithConfidence.h"
#include "realsense_ros_person/SkeletonJoint.h"
#include "realsense_ros_person/Gestures.h"

namespace realsense_ros_person
{

  class User : public ros::Msg
  {
    public:
      typedef realsense_ros_person::UserInfo _userInfo_type;
      _userInfo_type userInfo;
      typedef geometry_msgs::Point32 _centerOfMassImage_type;
      _centerOfMassImage_type centerOfMassImage;
      typedef geometry_msgs::Point32 _centerOfMassWorld_type;
      _centerOfMassWorld_type centerOfMassWorld;
      typedef realsense_ros_person::RectWithConfidence _userRect_type;
      _userRect_type userRect;
      typedef realsense_ros_person::RectWithConfidence _headBoundingBox_type;
      _headBoundingBox_type headBoundingBox;
      typedef realsense_ros_person::LandmarksInfo _landmarksInfo_type;
      _landmarksInfo_type landmarksInfo;
      typedef realsense_ros_person::EulerAnglesWithConfidence _headPose_type;
      _headPose_type headPose;
      uint32_t skeletonJoints_length;
      typedef realsense_ros_person::SkeletonJoint _skeletonJoints_type;
      _skeletonJoints_type st_skeletonJoints;
      _skeletonJoints_type * skeletonJoints;
      typedef realsense_ros_person::Gestures _gestures_type;
      _gestures_type gestures;

    User():
      userInfo(),
      centerOfMassImage(),
      centerOfMassWorld(),
      userRect(),
      headBoundingBox(),
      landmarksInfo(),
      headPose(),
      skeletonJoints_length(0), skeletonJoints(NULL),
      gestures()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->userInfo.serialize(outbuffer + offset);
      offset += this->centerOfMassImage.serialize(outbuffer + offset);
      offset += this->centerOfMassWorld.serialize(outbuffer + offset);
      offset += this->userRect.serialize(outbuffer + offset);
      offset += this->headBoundingBox.serialize(outbuffer + offset);
      offset += this->landmarksInfo.serialize(outbuffer + offset);
      offset += this->headPose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->skeletonJoints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->skeletonJoints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->skeletonJoints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->skeletonJoints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->skeletonJoints_length);
      for( uint32_t i = 0; i < skeletonJoints_length; i++){
      offset += this->skeletonJoints[i].serialize(outbuffer + offset);
      }
      offset += this->gestures.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->userInfo.deserialize(inbuffer + offset);
      offset += this->centerOfMassImage.deserialize(inbuffer + offset);
      offset += this->centerOfMassWorld.deserialize(inbuffer + offset);
      offset += this->userRect.deserialize(inbuffer + offset);
      offset += this->headBoundingBox.deserialize(inbuffer + offset);
      offset += this->landmarksInfo.deserialize(inbuffer + offset);
      offset += this->headPose.deserialize(inbuffer + offset);
      uint32_t skeletonJoints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      skeletonJoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      skeletonJoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      skeletonJoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->skeletonJoints_length);
      if(skeletonJoints_lengthT > skeletonJoints_length)
        this->skeletonJoints = (realsense_ros_person::SkeletonJoint*)realloc(this->skeletonJoints, skeletonJoints_lengthT * sizeof(realsense_ros_person::SkeletonJoint));
      skeletonJoints_length = skeletonJoints_lengthT;
      for( uint32_t i = 0; i < skeletonJoints_length; i++){
      offset += this->st_skeletonJoints.deserialize(inbuffer + offset);
        memcpy( &(this->skeletonJoints[i]), &(this->st_skeletonJoints), sizeof(realsense_ros_person::SkeletonJoint));
      }
      offset += this->gestures.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/User"; };
    const char * getMD5(){ return "6686a133e9edf3243aaa058301a3c9c6"; };

  };

}
#endif