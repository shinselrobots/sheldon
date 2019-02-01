#ifndef _ROS_SERVICE_BagTrainObject_h
#define _ROS_SERVICE_BagTrainObject_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace cob_object_detection_msgs
{

static const char BAGTRAINOBJECT[] = "cob_object_detection_msgs/BagTrainObject";

  class BagTrainObjectRequest : public ros::Msg
  {
    public:
      typedef std_msgs::String _object_name_type;
      _object_name_type object_name;

    BagTrainObjectRequest():
      object_name()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->object_name.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->object_name.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return BAGTRAINOBJECT; };
    const char * getMD5(){ return "adb33ef101026c3e15d799e3decf403f"; };

  };

  class BagTrainObjectResponse : public ros::Msg
  {
    public:
      typedef std_msgs::String _trained_type;
      _trained_type trained;

    BagTrainObjectResponse():
      trained()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->trained.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->trained.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return BAGTRAINOBJECT; };
    const char * getMD5(){ return "a94192f68364f039ed20f2ad560e48fd"; };

  };

  class BagTrainObject {
    public:
    typedef BagTrainObjectRequest Request;
    typedef BagTrainObjectResponse Response;
  };

}
#endif
