#ifndef _ROS_SERVICE_SetPointMap_h
#define _ROS_SERVICE_SetPointMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/PointCloud2.h"

namespace cob_3d_mapping_msgs
{

static const char SETPOINTMAP[] = "cob_3d_mapping_msgs/SetPointMap";

  class SetPointMapRequest : public ros::Msg
  {
    public:
      typedef sensor_msgs::PointCloud2 _map_type;
      _map_type map;

    SetPointMapRequest():
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

    const char * getType(){ return SETPOINTMAP; };
    const char * getMD5(){ return "b84fbb39505086eb6a62d933c75cb7b4"; };

  };

  class SetPointMapResponse : public ros::Msg
  {
    public:

    SetPointMapResponse()
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

    const char * getType(){ return SETPOINTMAP; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetPointMap {
    public:
    typedef SetPointMapRequest Request;
    typedef SetPointMapResponse Response;
  };

}
#endif
