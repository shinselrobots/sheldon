#ifndef _ROS_realsense_ros_object_UI_h
#define _ROS_realsense_ros_object_UI_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_object
{

  class UI : public ros::Msg
  {
    public:
      typedef int32_t _key_type;
      _key_type key;

    UI():
      key(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_key;
      u_key.real = this->key;
      *(outbuffer + offset + 0) = (u_key.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_key.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_key.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_key.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->key);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_key;
      u_key.base = 0;
      u_key.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_key.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_key.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_key.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->key = u_key.real;
      offset += sizeof(this->key);
     return offset;
    }

    const char * getType(){ return "realsense_ros_object/UI"; };
    const char * getMD5(){ return "d5f7d6b16cb4e9d7a81b607f04247968"; };

  };

}
#endif