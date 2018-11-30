#ifndef _ROS_realsense_ros_person_Wave_h
#define _ROS_realsense_ros_person_Wave_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_person
{

  class Wave : public ros::Msg
  {
    public:
      typedef int32_t _type_type;
      _type_type type;
      enum { WAVE_NOT_DETECTED = -1  };
      enum { WAVE_LEFT_LA = 1        };
      enum { WAVE_RIGHT_LA = 2       };
      enum { WAVE_LEFT_RA = 3        };
      enum { WAVE_RIGHT_RA = 4       };

    Wave():
      type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->type = u_type.real;
      offset += sizeof(this->type);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/Wave"; };
    const char * getMD5(){ return "bf1a205054ef9c51f0b3b2426adfe39a"; };

  };

}
#endif