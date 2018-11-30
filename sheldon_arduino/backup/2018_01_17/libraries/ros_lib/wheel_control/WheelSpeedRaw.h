#ifndef _ROS_wheel_control_WheelSpeedRaw_h
#define _ROS_wheel_control_WheelSpeedRaw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace wheel_control
{

  class WheelSpeedRaw : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _speed_ticks_left_type;
      _speed_ticks_left_type speed_ticks_left;
      typedef int32_t _speed_ticks_right_type;
      _speed_ticks_right_type speed_ticks_right;

    WheelSpeedRaw():
      header(),
      speed_ticks_left(0),
      speed_ticks_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_speed_ticks_left;
      u_speed_ticks_left.real = this->speed_ticks_left;
      *(outbuffer + offset + 0) = (u_speed_ticks_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_ticks_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_ticks_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_ticks_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_ticks_left);
      union {
        int32_t real;
        uint32_t base;
      } u_speed_ticks_right;
      u_speed_ticks_right.real = this->speed_ticks_right;
      *(outbuffer + offset + 0) = (u_speed_ticks_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_ticks_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_ticks_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_ticks_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_ticks_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_speed_ticks_left;
      u_speed_ticks_left.base = 0;
      u_speed_ticks_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_ticks_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_ticks_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_ticks_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_ticks_left = u_speed_ticks_left.real;
      offset += sizeof(this->speed_ticks_left);
      union {
        int32_t real;
        uint32_t base;
      } u_speed_ticks_right;
      u_speed_ticks_right.base = 0;
      u_speed_ticks_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_ticks_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_ticks_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_ticks_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_ticks_right = u_speed_ticks_right.real;
      offset += sizeof(this->speed_ticks_right);
     return offset;
    }

    const char * getType(){ return "wheel_control/WheelSpeedRaw"; };
    const char * getMD5(){ return "9052c6ae32b5bc3f391b44d2b9af21ed"; };

  };

}
#endif