#ifndef _ROS_SERVICE_ModifyMap_h
#define _ROS_SERVICE_ModifyMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_3d_mapping_msgs/ShapeArray.h"

namespace cob_3d_mapping_msgs
{

static const char MODIFYMAP[] = "cob_3d_mapping_msgs/ModifyMap";

  class ModifyMapRequest : public ros::Msg
  {
    public:
      typedef int32_t _action_type;
      _action_type action;
      typedef cob_3d_mapping_msgs::ShapeArray _shapes_type;
      _shapes_type shapes;
      enum { ADD = 0 };
      enum { MODIFY = 1 };
      enum { DELETE = 2 };

    ModifyMapRequest():
      action(0),
      shapes()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_action;
      u_action.real = this->action;
      *(outbuffer + offset + 0) = (u_action.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action);
      offset += this->shapes.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_action;
      u_action.base = 0;
      u_action.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action = u_action.real;
      offset += sizeof(this->action);
      offset += this->shapes.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return MODIFYMAP; };
    const char * getMD5(){ return "ea968d0c07a800a6b17527a191a32e88"; };

  };

  class ModifyMapResponse : public ros::Msg
  {
    public:

    ModifyMapResponse()
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

    const char * getType(){ return MODIFYMAP; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ModifyMap {
    public:
    typedef ModifyMapRequest Request;
    typedef ModifyMapResponse Response;
  };

}
#endif
