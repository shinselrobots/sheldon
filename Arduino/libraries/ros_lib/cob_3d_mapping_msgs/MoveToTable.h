#ifndef _ROS_SERVICE_MoveToTable_h
#define _ROS_SERVICE_MoveToTable_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_3d_mapping_msgs/Shape.h"

namespace cob_3d_mapping_msgs
{

static const char MOVETOTABLE[] = "cob_3d_mapping_msgs/MoveToTable";

  class MoveToTableRequest : public ros::Msg
  {
    public:
      typedef cob_3d_mapping_msgs::Shape _targetTable_type;
      _targetTable_type targetTable;

    MoveToTableRequest():
      targetTable()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->targetTable.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->targetTable.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return MOVETOTABLE; };
    const char * getMD5(){ return "5d1ffd0679813cd64e0bfd504ebff126"; };

  };

  class MoveToTableResponse : public ros::Msg
  {
    public:

    MoveToTableResponse()
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

    const char * getType(){ return MOVETOTABLE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class MoveToTable {
    public:
    typedef MoveToTableRequest Request;
    typedef MoveToTableResponse Response;
  };

}
#endif
