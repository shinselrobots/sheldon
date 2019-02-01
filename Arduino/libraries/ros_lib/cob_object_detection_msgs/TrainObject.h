#ifndef _ROS_SERVICE_TrainObject_h
#define _ROS_SERVICE_TrainObject_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_object_detection_msgs
{

static const char TRAINOBJECT[] = "cob_object_detection_msgs/TrainObject";

  class TrainObjectRequest : public ros::Msg
  {
    public:
      typedef const char* _object_name_type;
      _object_name_type object_name;

    TrainObjectRequest():
      object_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_object_name = strlen(this->object_name);
      varToArr(outbuffer + offset, length_object_name);
      offset += 4;
      memcpy(outbuffer + offset, this->object_name, length_object_name);
      offset += length_object_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_object_name;
      arrToVar(length_object_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_object_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_object_name-1]=0;
      this->object_name = (char *)(inbuffer + offset-1);
      offset += length_object_name;
     return offset;
    }

    const char * getType(){ return TRAINOBJECT; };
    const char * getMD5(){ return "2f12226348d323c2e5b2031b3da53f1b"; };

  };

  class TrainObjectResponse : public ros::Msg
  {
    public:

    TrainObjectResponse()
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

    const char * getType(){ return TRAINOBJECT; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class TrainObject {
    public:
    typedef TrainObjectRequest Request;
    typedef TrainObjectResponse Response;
  };

}
#endif
