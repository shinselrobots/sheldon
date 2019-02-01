#ifndef _ROS_SERVICE_UpdateMap_h
#define _ROS_SERVICE_UpdateMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_3d_mapping_msgs/ShapeArray.h"

namespace cob_3d_mapping_msgs
{

static const char UPDATEMAP[] = "cob_3d_mapping_msgs/UpdateMap";

  class UpdateMapRequest : public ros::Msg
  {
    public:
      typedef cob_3d_mapping_msgs::ShapeArray _map_type;
      _map_type map;

    UpdateMapRequest():
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

    const char * getType(){ return UPDATEMAP; };
    const char * getMD5(){ return "f00740a6f429ef52cd72626c52eeda1c"; };

  };

  class UpdateMapResponse : public ros::Msg
  {
    public:
      typedef cob_3d_mapping_msgs::ShapeArray _map_type;
      _map_type map;

    UpdateMapResponse():
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

    const char * getType(){ return UPDATEMAP; };
    const char * getMD5(){ return "f00740a6f429ef52cd72626c52eeda1c"; };

  };

  class UpdateMap {
    public:
    typedef UpdateMapRequest Request;
    typedef UpdateMapResponse Response;
  };

}
#endif
