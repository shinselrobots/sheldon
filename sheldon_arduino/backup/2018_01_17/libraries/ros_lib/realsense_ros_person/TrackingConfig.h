#ifndef _ROS_SERVICE_TrackingConfig_h
#define _ROS_SERVICE_TrackingConfig_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_person
{

static const char TRACKINGCONFIG[] = "realsense_ros_person/TrackingConfig";

  class TrackingConfigRequest : public ros::Msg
  {
    public:
      typedef bool _enableRecognition_type;
      _enableRecognition_type enableRecognition;
      typedef bool _enableSkeleton_type;
      _enableSkeleton_type enableSkeleton;
      typedef bool _enablePointingGesture_type;
      _enablePointingGesture_type enablePointingGesture;
      typedef bool _enableWaveGesture_type;
      _enableWaveGesture_type enableWaveGesture;
      typedef bool _enableLandmarks_type;
      _enableLandmarks_type enableLandmarks;
      typedef bool _enableHeadBoundingBox_type;
      _enableHeadBoundingBox_type enableHeadBoundingBox;
      typedef bool _enableHeadPose_type;
      _enableHeadPose_type enableHeadPose;

    TrackingConfigRequest():
      enableRecognition(0),
      enableSkeleton(0),
      enablePointingGesture(0),
      enableWaveGesture(0),
      enableLandmarks(0),
      enableHeadBoundingBox(0),
      enableHeadPose(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enableRecognition;
      u_enableRecognition.real = this->enableRecognition;
      *(outbuffer + offset + 0) = (u_enableRecognition.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableRecognition);
      union {
        bool real;
        uint8_t base;
      } u_enableSkeleton;
      u_enableSkeleton.real = this->enableSkeleton;
      *(outbuffer + offset + 0) = (u_enableSkeleton.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableSkeleton);
      union {
        bool real;
        uint8_t base;
      } u_enablePointingGesture;
      u_enablePointingGesture.real = this->enablePointingGesture;
      *(outbuffer + offset + 0) = (u_enablePointingGesture.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enablePointingGesture);
      union {
        bool real;
        uint8_t base;
      } u_enableWaveGesture;
      u_enableWaveGesture.real = this->enableWaveGesture;
      *(outbuffer + offset + 0) = (u_enableWaveGesture.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableWaveGesture);
      union {
        bool real;
        uint8_t base;
      } u_enableLandmarks;
      u_enableLandmarks.real = this->enableLandmarks;
      *(outbuffer + offset + 0) = (u_enableLandmarks.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableLandmarks);
      union {
        bool real;
        uint8_t base;
      } u_enableHeadBoundingBox;
      u_enableHeadBoundingBox.real = this->enableHeadBoundingBox;
      *(outbuffer + offset + 0) = (u_enableHeadBoundingBox.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableHeadBoundingBox);
      union {
        bool real;
        uint8_t base;
      } u_enableHeadPose;
      u_enableHeadPose.real = this->enableHeadPose;
      *(outbuffer + offset + 0) = (u_enableHeadPose.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableHeadPose);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enableRecognition;
      u_enableRecognition.base = 0;
      u_enableRecognition.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableRecognition = u_enableRecognition.real;
      offset += sizeof(this->enableRecognition);
      union {
        bool real;
        uint8_t base;
      } u_enableSkeleton;
      u_enableSkeleton.base = 0;
      u_enableSkeleton.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableSkeleton = u_enableSkeleton.real;
      offset += sizeof(this->enableSkeleton);
      union {
        bool real;
        uint8_t base;
      } u_enablePointingGesture;
      u_enablePointingGesture.base = 0;
      u_enablePointingGesture.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enablePointingGesture = u_enablePointingGesture.real;
      offset += sizeof(this->enablePointingGesture);
      union {
        bool real;
        uint8_t base;
      } u_enableWaveGesture;
      u_enableWaveGesture.base = 0;
      u_enableWaveGesture.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableWaveGesture = u_enableWaveGesture.real;
      offset += sizeof(this->enableWaveGesture);
      union {
        bool real;
        uint8_t base;
      } u_enableLandmarks;
      u_enableLandmarks.base = 0;
      u_enableLandmarks.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableLandmarks = u_enableLandmarks.real;
      offset += sizeof(this->enableLandmarks);
      union {
        bool real;
        uint8_t base;
      } u_enableHeadBoundingBox;
      u_enableHeadBoundingBox.base = 0;
      u_enableHeadBoundingBox.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableHeadBoundingBox = u_enableHeadBoundingBox.real;
      offset += sizeof(this->enableHeadBoundingBox);
      union {
        bool real;
        uint8_t base;
      } u_enableHeadPose;
      u_enableHeadPose.base = 0;
      u_enableHeadPose.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableHeadPose = u_enableHeadPose.real;
      offset += sizeof(this->enableHeadPose);
     return offset;
    }

    const char * getType(){ return TRACKINGCONFIG; };
    const char * getMD5(){ return "686ca0728cbcd739a9efb35cc7c81a46"; };

  };

  class TrackingConfigResponse : public ros::Msg
  {
    public:
      typedef bool _status_type;
      _status_type status;

    TrackingConfigResponse():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return TRACKINGCONFIG; };
    const char * getMD5(){ return "3a1255d4d998bd4d6585c64639b5ee9a"; };

  };

  class TrackingConfig {
    public:
    typedef TrackingConfigRequest Request;
    typedef TrackingConfigResponse Response;
  };

}
#endif
