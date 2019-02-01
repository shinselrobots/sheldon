#ifndef _ROS_cob_perception_msgs_Mask_h
#define _ROS_cob_perception_msgs_Mask_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_perception_msgs/Rect.h"
#include "sensor_msgs/Image.h"

namespace cob_perception_msgs
{

  class Mask : public ros::Msg
  {
    public:
      typedef cob_perception_msgs::Rect _roi_type;
      _roi_type roi;
      typedef sensor_msgs::Image _mask_type;
      _mask_type mask;

    Mask():
      roi(),
      mask()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->roi.serialize(outbuffer + offset);
      offset += this->mask.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->roi.deserialize(inbuffer + offset);
      offset += this->mask.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "cob_perception_msgs/Mask"; };
    const char * getMD5(){ return "e1240778c6320b394ae629a7eb8c26ba"; };

  };

}
#endif