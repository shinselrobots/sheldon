#ifndef _ROS_SERVICE_LoadRecognitionDB_h
#define _ROS_SERVICE_LoadRecognitionDB_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_person
{

static const char LOADRECOGNITIONDB[] = "realsense_ros_person/LoadRecognitionDB";

  class LoadRecognitionDBRequest : public ros::Msg
  {
    public:
      typedef const char* _loadFromPath_type;
      _loadFromPath_type loadFromPath;

    LoadRecognitionDBRequest():
      loadFromPath("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_loadFromPath = strlen(this->loadFromPath);
      varToArr(outbuffer + offset, length_loadFromPath);
      offset += 4;
      memcpy(outbuffer + offset, this->loadFromPath, length_loadFromPath);
      offset += length_loadFromPath;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_loadFromPath;
      arrToVar(length_loadFromPath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_loadFromPath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_loadFromPath-1]=0;
      this->loadFromPath = (char *)(inbuffer + offset-1);
      offset += length_loadFromPath;
     return offset;
    }

    const char * getType(){ return LOADRECOGNITIONDB; };
    const char * getMD5(){ return "75bd894c03d881eed7a2d0484160e542"; };

  };

  class LoadRecognitionDBResponse : public ros::Msg
  {
    public:
      typedef bool _status_type;
      _status_type status;

    LoadRecognitionDBResponse():
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

    const char * getType(){ return LOADRECOGNITIONDB; };
    const char * getMD5(){ return "3a1255d4d998bd4d6585c64639b5ee9a"; };

  };

  class LoadRecognitionDB {
    public:
    typedef LoadRecognitionDBRequest Request;
    typedef LoadRecognitionDBResponse Response;
  };

}
#endif
