#ifndef _ROS_cob_3d_mapping_msgs_Point2D_h
#define _ROS_cob_3d_mapping_msgs_Point2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_3d_mapping_msgs
{

  class Point2D : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _tex_x_type;
      _tex_x_type tex_x;
      typedef float _tex_y_type;
      _tex_y_type tex_y;
      typedef float _var_type;
      _var_type var;

    Point2D():
      x(0),
      y(0),
      tex_x(0),
      tex_y(0),
      var(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_tex_x;
      u_tex_x.real = this->tex_x;
      *(outbuffer + offset + 0) = (u_tex_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tex_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tex_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tex_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tex_x);
      union {
        float real;
        uint32_t base;
      } u_tex_y;
      u_tex_y.real = this->tex_y;
      *(outbuffer + offset + 0) = (u_tex_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tex_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tex_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tex_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tex_y);
      union {
        float real;
        uint32_t base;
      } u_var;
      u_var.real = this->var;
      *(outbuffer + offset + 0) = (u_var.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_var.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_var.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_var.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->var);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_tex_x;
      u_tex_x.base = 0;
      u_tex_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tex_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tex_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tex_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tex_x = u_tex_x.real;
      offset += sizeof(this->tex_x);
      union {
        float real;
        uint32_t base;
      } u_tex_y;
      u_tex_y.base = 0;
      u_tex_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tex_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tex_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tex_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tex_y = u_tex_y.real;
      offset += sizeof(this->tex_y);
      union {
        float real;
        uint32_t base;
      } u_var;
      u_var.base = 0;
      u_var.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_var.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_var.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_var.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->var = u_var.real;
      offset += sizeof(this->var);
     return offset;
    }

    const char * getType(){ return "cob_3d_mapping_msgs/Point2D"; };
    const char * getMD5(){ return "fc725bda50adf2bf28e55ed21147f45e"; };

  };

}
#endif