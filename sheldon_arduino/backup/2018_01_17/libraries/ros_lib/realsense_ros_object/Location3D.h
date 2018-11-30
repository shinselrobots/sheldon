#ifndef _ROS_realsense_ros_object_Location3D_h
#define _ROS_realsense_ros_object_Location3D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point32.h"

namespace realsense_ros_object
{

  class Location3D : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point32 _coordinates_type;
      _coordinates_type coordinates;
      typedef float _horiz_margin_type;
      _horiz_margin_type horiz_margin;
      typedef float _vert_margin_type;
      _vert_margin_type vert_margin;

    Location3D():
      coordinates(),
      horiz_margin(0),
      vert_margin(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->coordinates.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_horiz_margin;
      u_horiz_margin.real = this->horiz_margin;
      *(outbuffer + offset + 0) = (u_horiz_margin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_horiz_margin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_horiz_margin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_horiz_margin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->horiz_margin);
      union {
        float real;
        uint32_t base;
      } u_vert_margin;
      u_vert_margin.real = this->vert_margin;
      *(outbuffer + offset + 0) = (u_vert_margin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vert_margin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vert_margin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vert_margin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vert_margin);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->coordinates.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_horiz_margin;
      u_horiz_margin.base = 0;
      u_horiz_margin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_horiz_margin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_horiz_margin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_horiz_margin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->horiz_margin = u_horiz_margin.real;
      offset += sizeof(this->horiz_margin);
      union {
        float real;
        uint32_t base;
      } u_vert_margin;
      u_vert_margin.base = 0;
      u_vert_margin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vert_margin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vert_margin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vert_margin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vert_margin = u_vert_margin.real;
      offset += sizeof(this->vert_margin);
     return offset;
    }

    const char * getType(){ return "realsense_ros_object/Location3D"; };
    const char * getMD5(){ return "9d0d637baabd2a2edf3e785062238586"; };

  };

}
#endif