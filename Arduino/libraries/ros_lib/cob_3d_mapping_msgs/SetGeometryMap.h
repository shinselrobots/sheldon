#ifndef _ROS_SERVICE_SetGeometryMap_h
#define _ROS_SERVICE_SetGeometryMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_3d_mapping_msgs/ShapeArray.h"

namespace cob_3d_mapping_msgs
{

static const char SETGEOMETRYMAP[] = "cob_3d_mapping_msgs/SetGeometryMap";

  class SetGeometryMapRequest : public ros::Msg
  {
    public:
      typedef cob_3d_mapping_msgs::ShapeArray _map_type;
      _map_type map;

    SetGeometryMapRequest():
      map()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->map.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->map.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETGEOMETRYMAP; };
    const char * getMD5(){ return "f00740a6f429ef52cd72626c52eeda1c"; };

  };

  class SetGeometryMapResponse : public ros::Msg
  {
    public:

    SetGeometryMapResponse()
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

    const char * getType(){ return SETGEOMETRYMAP; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetGeometryMap {
    public:
    typedef SetGeometryMapRequest Request;
    typedef SetGeometryMapResponse Response;
  };

}
#endif
