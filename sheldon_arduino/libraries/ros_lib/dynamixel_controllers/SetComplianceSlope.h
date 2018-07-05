#ifndef _ROS_SERVICE_SetComplianceSlope_h
#define _ROS_SERVICE_SetComplianceSlope_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETCOMPLIANCESLOPE[] = "dynamixel_controllers/SetComplianceSlope";

  class SetComplianceSlopeRequest : public ros::Msg
  {
    public:
      typedef uint8_t _slope_type;
      _slope_type slope;

    SetComplianceSlopeRequest():
      slope(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->slope >> (8 * 0)) & 0xFF;
      offset += sizeof(this->slope);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->slope =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->slope);
     return offset;
    }

    const char * getType(){ return SETCOMPLIANCESLOPE; };
    const char * getMD5(){ return "0b655cbe1b74daf357824dcc427c1004"; };

  };

  class SetComplianceSlopeResponse : public ros::Msg
  {
    public:

    SetComplianceSlopeResponse()
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

    const char * getType(){ return SETCOMPLIANCESLOPE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetComplianceSlope {
    public:
    typedef SetComplianceSlopeRequest Request;
    typedef SetComplianceSlopeResponse Response;
  };

}
#endif
