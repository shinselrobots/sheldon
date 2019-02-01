#ifndef _ROS_SERVICE_StopObjectRecording_h
#define _ROS_SERVICE_StopObjectRecording_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_object_detection_msgs
{

static const char STOPOBJECTRECORDING[] = "cob_object_detection_msgs/StopObjectRecording";

  class StopObjectRecordingRequest : public ros::Msg
  {
    public:
      typedef bool _stop_although_model_is_incomplete_type;
      _stop_although_model_is_incomplete_type stop_although_model_is_incomplete;

    StopObjectRecordingRequest():
      stop_although_model_is_incomplete(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_stop_although_model_is_incomplete;
      u_stop_although_model_is_incomplete.real = this->stop_although_model_is_incomplete;
      *(outbuffer + offset + 0) = (u_stop_although_model_is_incomplete.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stop_although_model_is_incomplete);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_stop_although_model_is_incomplete;
      u_stop_although_model_is_incomplete.base = 0;
      u_stop_although_model_is_incomplete.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stop_although_model_is_incomplete = u_stop_although_model_is_incomplete.real;
      offset += sizeof(this->stop_although_model_is_incomplete);
     return offset;
    }

    const char * getType(){ return STOPOBJECTRECORDING; };
    const char * getMD5(){ return "60ec7755ba9c0b6779b757eba0e03e82"; };

  };

  class StopObjectRecordingResponse : public ros::Msg
  {
    public:
      typedef bool _recording_stopped_type;
      _recording_stopped_type recording_stopped;

    StopObjectRecordingResponse():
      recording_stopped(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_recording_stopped;
      u_recording_stopped.real = this->recording_stopped;
      *(outbuffer + offset + 0) = (u_recording_stopped.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->recording_stopped);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_recording_stopped;
      u_recording_stopped.base = 0;
      u_recording_stopped.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->recording_stopped = u_recording_stopped.real;
      offset += sizeof(this->recording_stopped);
     return offset;
    }

    const char * getType(){ return STOPOBJECTRECORDING; };
    const char * getMD5(){ return "6c4ef6fc8891dd138a543e6ac4a818da"; };

  };

  class StopObjectRecording {
    public:
    typedef StopObjectRecordingRequest Request;
    typedef StopObjectRecordingResponse Response;
  };

}
#endif
