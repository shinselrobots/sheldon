#ifndef _ROS_realsense_ros_object_ObjectInBox_h
#define _ROS_realsense_ros_object_ObjectInBox_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_ros_object/Object.h"
#include "realsense_ros_object/Rect.h"
#include "realsense_ros_object/Location3D.h"

namespace realsense_ros_object
{

  class ObjectInBox : public ros::Msg
  {
    public:
      typedef realsense_ros_object::Object _object_type;
      _object_type object;
      typedef realsense_ros_object::Rect _object_bbox_type;
      _object_bbox_type object_bbox;
      typedef realsense_ros_object::Location3D _location_type;
      _location_type location;

    ObjectInBox():
      object(),
      object_bbox(),
      location()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->object.serialize(outbuffer + offset);
      offset += this->object_bbox.serialize(outbuffer + offset);
      offset += this->location.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->object.deserialize(inbuffer + offset);
      offset += this->object_bbox.deserialize(inbuffer + offset);
      offset += this->location.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_ros_object/ObjectInBox"; };
    const char * getMD5(){ return "506c556ef4808f18c8fda7a0b6d365a7"; };

  };

}
#endif