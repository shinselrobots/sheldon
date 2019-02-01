#ifndef _ROS_SERVICE_GetObjectsOfClass_h
#define _ROS_SERVICE_GetObjectsOfClass_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_3d_mapping_msgs/ShapeArray.h"
#include "std_msgs/UInt32.h"

namespace cob_3d_mapping_msgs
{

static const char GETOBJECTSOFCLASS[] = "cob_3d_mapping_msgs/GetObjectsOfClass";

  class GetObjectsOfClassRequest : public ros::Msg
  {
    public:
      typedef std_msgs::UInt32 _class_id_type;
      _class_id_type class_id;

    GetObjectsOfClassRequest():
      class_id()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->class_id.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->class_id.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETOBJECTSOFCLASS; };
    const char * getMD5(){ return "8cd17435496133fe7b904bdd1a1f66ee"; };

  };

  class GetObjectsOfClassResponse : public ros::Msg
  {
    public:
      typedef cob_3d_mapping_msgs::ShapeArray _objects_type;
      _objects_type objects;

    GetObjectsOfClassResponse():
      objects()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->objects.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->objects.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETOBJECTSOFCLASS; };
    const char * getMD5(){ return "d3aa665b847f46ea666781345b385880"; };

  };

  class GetObjectsOfClass {
    public:
    typedef GetObjectsOfClassRequest Request;
    typedef GetObjectsOfClassResponse Response;
  };

}
#endif
