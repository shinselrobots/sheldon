#ifndef _ROS_SERVICE_StopTracking_h
#define _ROS_SERVICE_StopTracking_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_person
{

static const char STOPTRACKING[] = "realsense_ros_person/StopTracking";

  class StopTrackingRequest : public ros::Msg
  {
    public:
      typedef int32_t _personId_type;
      _personId_type personId;

    StopTrackingRequest():
      personId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_personId;
      u_personId.real = this->personId;
      *(outbuffer + offset + 0) = (u_personId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_personId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_personId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_personId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->personId);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_personId;
      u_personId.base = 0;
      u_personId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_personId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_personId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_personId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->personId = u_personId.real;
      offset += sizeof(this->personId);
     return offset;
    }

    const char * getType(){ return STOPTRACKING; };
    const char * getMD5(){ return "7d4730fa7bd1fbdf419fdc75b1b7e004"; };

  };

  class StopTrackingResponse : public ros::Msg
  {
    public:
      typedef bool _status_type;
      _status_type status;

    StopTrackingResponse():
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

    const char * getType(){ return STOPTRACKING; };
    const char * getMD5(){ return "3a1255d4d998bd4d6585c64639b5ee9a"; };

  };

  class StopTracking {
    public:
    typedef StopTrackingRequest Request;
    typedef StopTrackingResponse Response;
  };

}
#endif
