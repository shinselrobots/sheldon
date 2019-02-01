#ifndef _ROS_cob_object_detection_msgs_PoseRT_h
#define _ROS_cob_object_detection_msgs_PoseRT_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_object_detection_msgs
{

  class PoseRT : public ros::Msg
  {
    public:
      float rvec[3];
      float tvec[3];

    PoseRT():
      rvec(),
      tvec()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->rvec[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tvec[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rvec[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tvec[i]));
      }
     return offset;
    }

    const char * getType(){ return "cob_object_detection_msgs/PoseRT"; };
    const char * getMD5(){ return "d6802a5a6d10cce0b5eea91c5defc39f"; };

  };

}
#endif