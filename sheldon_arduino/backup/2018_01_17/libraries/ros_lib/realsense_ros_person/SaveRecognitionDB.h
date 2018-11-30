#ifndef _ROS_SERVICE_SaveRecognitionDB_h
#define _ROS_SERVICE_SaveRecognitionDB_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_person
{

static const char SAVERECOGNITIONDB[] = "realsense_ros_person/SaveRecognitionDB";

  class SaveRecognitionDBRequest : public ros::Msg
  {
    public:
      typedef const char* _saveToPath_type;
      _saveToPath_type saveToPath;

    SaveRecognitionDBRequest():
      saveToPath("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_saveToPath = strlen(this->saveToPath);
      varToArr(outbuffer + offset, length_saveToPath);
      offset += 4;
      memcpy(outbuffer + offset, this->saveToPath, length_saveToPath);
      offset += length_saveToPath;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_saveToPath;
      arrToVar(length_saveToPath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_saveToPath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_saveToPath-1]=0;
      this->saveToPath = (char *)(inbuffer + offset-1);
      offset += length_saveToPath;
     return offset;
    }

    const char * getType(){ return SAVERECOGNITIONDB; };
    const char * getMD5(){ return "b6f54f323bfdd90f5dc74498f19562e3"; };

  };

  class SaveRecognitionDBResponse : public ros::Msg
  {
    public:
      typedef bool _status_type;
      _status_type status;

    SaveRecognitionDBResponse():
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

    const char * getType(){ return SAVERECOGNITIONDB; };
    const char * getMD5(){ return "3a1255d4d998bd4d6585c64639b5ee9a"; };

  };

  class SaveRecognitionDB {
    public:
    typedef SaveRecognitionDBRequest Request;
    typedef SaveRecognitionDBResponse Response;
  };

}
#endif
