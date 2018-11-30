#ifndef _ROS_SERVICE_LaunchProcess_h
#define _ROS_SERVICE_LaunchProcess_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rbx2_utils
{

static const char LAUNCHPROCESS[] = "rbx2_utils/LaunchProcess";

  class LaunchProcessRequest : public ros::Msg
  {
    public:
      typedef const char* _command_type;
      _command_type command;

    LaunchProcessRequest():
      command("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
     return offset;
    }

    const char * getType(){ return LAUNCHPROCESS; };
    const char * getMD5(){ return "cba5e21e920a3a2b7b375cb65b64cdea"; };

  };

  class LaunchProcessResponse : public ros::Msg
  {
    public:
      typedef const char* _process_id_type;
      _process_id_type process_id;

    LaunchProcessResponse():
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

    const char * getType(){ return LAUNCHPROCESS; };
    const char * getMD5(){ return "a19d2ed2d6b55934bac7cd67cfed8983"; };

  };

  class LaunchProcess {
    public:
    typedef LaunchProcessRequest Request;
    typedef LaunchProcessResponse Response;
  };

}
#endif
