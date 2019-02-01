#ifndef _ROS_cob_3d_mapping_msgs_FilterObject_h
#define _ROS_cob_3d_mapping_msgs_FilterObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_3d_mapping_msgs
{

  class FilterObject : public ros::Msg
  {
    public:
      typedef float _height_type;
      _height_type height;
      typedef float _width_type;
      _width_type width;
      typedef float _curv_h_type;
      _curv_h_type curv_h;
      typedef float _curv_w_type;
      _curv_w_type curv_w;

    FilterObject():
      height(0),
      width(0),
      curv_h(0),
      curv_w(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      union {
        float real;
        uint32_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        float real;
        uint32_t base;
      } u_curv_h;
      u_curv_h.real = this->curv_h;
      *(outbuffer + offset + 0) = (u_curv_h.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_curv_h.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_curv_h.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_curv_h.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->curv_h);
      union {
        float real;
        uint32_t base;
      } u_curv_w;
      u_curv_w.real = this->curv_w;
      *(outbuffer + offset + 0) = (u_curv_w.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_curv_w.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_curv_w.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_curv_w.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->curv_w);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        float real;
        uint32_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        float real;
        uint32_t base;
      } u_curv_h;
      u_curv_h.base = 0;
      u_curv_h.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_curv_h.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_curv_h.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_curv_h.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->curv_h = u_curv_h.real;
      offset += sizeof(this->curv_h);
      union {
        float real;
        uint32_t base;
      } u_curv_w;
      u_curv_w.base = 0;
      u_curv_w.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_curv_w.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_curv_w.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_curv_w.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->curv_w = u_curv_w.real;
      offset += sizeof(this->curv_w);
     return offset;
    }

    const char * getType(){ return "cob_3d_mapping_msgs/FilterObject"; };
    const char * getMD5(){ return "3ce8bd944e4374d48880ed66855bf0be"; };

  };

}
#endif