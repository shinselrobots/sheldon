#ifndef _ROS_SERVICE_KillProcess_h
#define _ROS_SERVICE_KillProcess_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rbx2_utils
{

static const char KILLPROCESS[] = "rbx2_utils/KillProcess";

  class KillProcessRequest : public ros::Msg
  {
    public:
      typedef const char* _process_id_type;
      _process_id_type process_id;

    KillProcessRequest():
      process_id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_process_id = strlen(this->process_id);
      varToArr(outbuffer + offset, length_process_id);
      offset += 4;
      memcpy(outbuffer + offset, this->process_id, length_process_id);
      offset += length_process_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_process_id;
      arrToVar(length_process_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_process_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_process_id-1]=0;
      this->process_id = (char *)(inbuffer + offset-1);
      offset += length_process_id;
     return offset;
    }

    const char * getType(){ return KILLPROCESS; };
    const char * getMD5(){ return "a19d2ed2d6b55934bac7cd67cfed8983"; };

  };

  class KillProcessResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    KillProcessResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return KILLPROCESS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class KillProcess {
    public:
    typedef KillProcessRequest Request;
    typedef KillProcessResponse Response;
  };

}
#endif
