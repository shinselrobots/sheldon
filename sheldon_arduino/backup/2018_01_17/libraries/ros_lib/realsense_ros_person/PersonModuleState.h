#ifndef _ROS_realsense_ros_person_PersonModuleState_h
#define _ROS_realsense_ros_person_PersonModuleState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_person
{

  class PersonModuleState : public ros::Msg
  {
    public:
      typedef bool _isRecognitionEnabled_type;
      _isRecognitionEnabled_type isRecognitionEnabled;
      typedef bool _isSkeletonEnabled_type;
      _isSkeletonEnabled_type isSkeletonEnabled;
      typedef bool _isGesturesEnabled_type;
      _isGesturesEnabled_type isGesturesEnabled;
      typedef bool _isLandmarksEnabled_type;
      _isLandmarksEnabled_type isLandmarksEnabled;
      typedef bool _isHeadBoundingBoxEnabled_type;
      _isHeadBoundingBoxEnabled_type isHeadBoundingBoxEnabled;
      typedef bool _isHeadPoseEnabled_type;
      _isHeadPoseEnabled_type isHeadPoseEnabled;
      typedef bool _isTrackingEnabled_type;
      _isTrackingEnabled_type isTrackingEnabled;
      typedef int32_t _trackingState_type;
      _trackingState_type trackingState;
      enum { TRACKING_STATE_TRACKING = 0 };
      enum { TRACKING_STATE_DETECTING = 1 };

    PersonModuleState():
      isRecognitionEnabled(0),
      isSkeletonEnabled(0),
      isGesturesEnabled(0),
      isLandmarksEnabled(0),
      isHeadBoundingBoxEnabled(0),
      isHeadPoseEnabled(0),
      isTrackingEnabled(0),
      trackingState(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_isRecognitionEnabled;
      u_isRecognitionEnabled.real = this->isRecognitionEnabled;
      *(outbuffer + offset + 0) = (u_isRecognitionEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isRecognitionEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isSkeletonEnabled;
      u_isSkeletonEnabled.real = this->isSkeletonEnabled;
      *(outbuffer + offset + 0) = (u_isSkeletonEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isSkeletonEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isGesturesEnabled;
      u_isGesturesEnabled.real = this->isGesturesEnabled;
      *(outbuffer + offset + 0) = (u_isGesturesEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isGesturesEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isLandmarksEnabled;
      u_isLandmarksEnabled.real = this->isLandmarksEnabled;
      *(outbuffer + offset + 0) = (u_isLandmarksEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isLandmarksEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isHeadBoundingBoxEnabled;
      u_isHeadBoundingBoxEnabled.real = this->isHeadBoundingBoxEnabled;
      *(outbuffer + offset + 0) = (u_isHeadBoundingBoxEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isHeadBoundingBoxEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isHeadPoseEnabled;
      u_isHeadPoseEnabled.real = this->isHeadPoseEnabled;
      *(outbuffer + offset + 0) = (u_isHeadPoseEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isHeadPoseEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isTrackingEnabled;
      u_isTrackingEnabled.real = this->isTrackingEnabled;
      *(outbuffer + offset + 0) = (u_isTrackingEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isTrackingEnabled);
      union {
        int32_t real;
        uint32_t base;
      } u_trackingState;
      u_trackingState.real = this->trackingState;
      *(outbuffer + offset + 0) = (u_trackingState.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trackingState.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trackingState.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trackingState.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trackingState);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_isRecognitionEnabled;
      u_isRecognitionEnabled.base = 0;
      u_isRecognitionEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isRecognitionEnabled = u_isRecognitionEnabled.real;
      offset += sizeof(this->isRecognitionEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isSkeletonEnabled;
      u_isSkeletonEnabled.base = 0;
      u_isSkeletonEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isSkeletonEnabled = u_isSkeletonEnabled.real;
      offset += sizeof(this->isSkeletonEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isGesturesEnabled;
      u_isGesturesEnabled.base = 0;
      u_isGesturesEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isGesturesEnabled = u_isGesturesEnabled.real;
      offset += sizeof(this->isGesturesEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isLandmarksEnabled;
      u_isLandmarksEnabled.base = 0;
      u_isLandmarksEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isLandmarksEnabled = u_isLandmarksEnabled.real;
      offset += sizeof(this->isLandmarksEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isHeadBoundingBoxEnabled;
      u_isHeadBoundingBoxEnabled.base = 0;
      u_isHeadBoundingBoxEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isHeadBoundingBoxEnabled = u_isHeadBoundingBoxEnabled.real;
      offset += sizeof(this->isHeadBoundingBoxEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isHeadPoseEnabled;
      u_isHeadPoseEnabled.base = 0;
      u_isHeadPoseEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isHeadPoseEnabled = u_isHeadPoseEnabled.real;
      offset += sizeof(this->isHeadPoseEnabled);
      union {
        bool real;
        uint8_t base;
      } u_isTrackingEnabled;
      u_isTrackingEnabled.base = 0;
      u_isTrackingEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isTrackingEnabled = u_isTrackingEnabled.real;
      offset += sizeof(this->isTrackingEnabled);
      union {
        int32_t real;
        uint32_t base;
      } u_trackingState;
      u_trackingState.base = 0;
      u_trackingState.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trackingState.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trackingState.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trackingState.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->trackingState = u_trackingState.real;
      offset += sizeof(this->trackingState);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/PersonModuleState"; };
    const char * getMD5(){ return "a004989fab301fe2ac94f9cab839eaed"; };

  };

}
#endif