#ifndef _ROS_SERVICE_SaveOutput_h
#define _ROS_SERVICE_SaveOutput_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_slam
{

static const char SAVEOUTPUT[] = "realsense_ros_slam/SaveOutput";

  class SaveOutputRequest : public ros::Msg
  {
    public:

    SaveOutputRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SAVEOUTPUT; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SaveOutputResponse : public ros::Msg
  {
    public:
      typedef bool _status_type;
      _status_type status;

    SaveOutputResponse():
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

    const char * getType(){ return SAVEOUTPUT; };
    const char * getMD5(){ return "3a1255d4d998bd4d6585c64639b5ee9a"; };

  };

  class SaveOutput {
    public:
    typedef SaveOutputRequest Request;
    typedef SaveOutputResponse Response;
  };

}
#endif
