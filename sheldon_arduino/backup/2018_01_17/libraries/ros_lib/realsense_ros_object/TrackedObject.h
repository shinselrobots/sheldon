#ifndef _ROS_realsense_ros_object_TrackedObject_h
#define _ROS_realsense_ros_object_TrackedObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_ros_object/Rect.h"
#include "realsense_ros_object/Location3D.h"

namespace realsense_ros_object
{

  class TrackedObject : public ros::Msg
  {
    public:
      typedef realsense_ros_object::Rect _bbox_type;
      _bbox_type bbox;
      typedef int32_t _id_type;
      _id_type id;
      typedef realsense_ros_object::Location3D _location_type;
      _location_type location;

    TrackedObject():
      bbox(),
      id(0),
      location()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->bbox.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      offset += this->location.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->bbox.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      offset += this->location.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_ros_object/TrackedObject"; };
    const char * getMD5(){ return "49a76037ad24b477fa0d232ca39d6579"; };

  };

}
#endif