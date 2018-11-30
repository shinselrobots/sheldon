#ifndef _ROS_SERVICE_RecognitionRegister_h
#define _ROS_SERVICE_RecognitionRegister_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_person
{

static const char RECOGNITIONREGISTER[] = "realsense_ros_person/RecognitionRegister";

  class RecognitionRegisterRequest : public ros::Msg
  {
    public:
      typedef int32_t _personId_type;
      _personId_type personId;

    RecognitionRegisterRequest():
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

    const char * getType(){ return RECOGNITIONREGISTER; };
    const char * getMD5(){ return "7d4730fa7bd1fbdf419fdc75b1b7e004"; };

  };

  class RecognitionRegisterResponse : public ros::Msg
  {
    public:
      typedef int32_t _status_type;
      _status_type status;
      typedef int32_t _recognitionId_type;
      _recognitionId_type recognitionId;
      enum { REGISTRATION_SUCCESSFULL = 0 };
      enum { REGISTRATION_FAILED = 1 };
      enum { REGISTRATION_FAILED_ALREADY_REGISTERED = 2 };
      enum { REGISTRATION_FAILED_FACE_NOT_DETECTED = 3 };
      enum { REGISTRATION_FAILED_FACE_NOT_CLEAR = 4 };
      enum { REGISTRATION_FAILED_PERSON_TO_FAR = 5 };
      enum { REGISTRATION_FAILED_PERSON_TO_CLOSE = 6 };

    RecognitionRegisterResponse():
      status(0),
      recognitionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_status.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status);
      union {
        int32_t real;
        uint32_t base;
      } u_recognitionId;
      u_recognitionId.real = this->recognitionId;
      *(outbuffer + offset + 0) = (u_recognitionId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_recognitionId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_recognitionId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_recognitionId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->recognitionId);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->status = u_status.real;
      offset += sizeof(this->status);
      union {
        int32_t real;
        uint32_t base;
      } u_recognitionId;
      u_recognitionId.base = 0;
      u_recognitionId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_recognitionId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_recognitionId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_recognitionId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->recognitionId = u_recognitionId.real;
      offset += sizeof(this->recognitionId);
     return offset;
    }

    const char * getType(){ return RECOGNITIONREGISTER; };
    const char * getMD5(){ return "0e27d8516c4e2e6a0dca385b76894dcd"; };

  };

  class RecognitionRegister {
    public:
    typedef RecognitionRegisterRequest Request;
    typedef RecognitionRegisterResponse Response;
  };

}
#endif
