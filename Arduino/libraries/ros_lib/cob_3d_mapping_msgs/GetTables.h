#ifndef _ROS_SERVICE_GetTables_h
#define _ROS_SERVICE_GetTables_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_3d_mapping_msgs/ShapeArray.h"

namespace cob_3d_mapping_msgs
{

static const char GETTABLES[] = "cob_3d_mapping_msgs/GetTables";

  class GetTablesRequest : public ros::Msg
  {
    public:

    GetTablesRequest()
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

    const char * getType(){ return GETTABLES; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetTablesResponse : public ros::Msg
  {
    public:
      typedef cob_3d_mapping_msgs::ShapeArray _tables_type;
      _tables_type tables;

    GetTablesResponse():
      tables()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->tables.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->tables.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETTABLES; };
    const char * getMD5(){ return "66dab6432264ad910ef67f2a1c240146"; };

  };

  class GetTables {
    public:
    typedef GetTablesRequest Request;
    typedef GetTablesResponse Response;
  };

}
#endif
