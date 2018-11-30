#ifndef _ROS_realsense_ros_person_FrameTest_h
#define _ROS_realsense_ros_person_FrameTest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Image.h"
#include "realsense_ros_person/Frame.h"

namespace realsense_ros_person
{

  class FrameTest : public ros::Msg
  {
    public:
      typedef sensor_msgs::Image _colorImage_type;
      _colorImage_type colorImage;
      typedef realsense_ros_person::Frame _frameData_type;
      _frameData_type frameData;

    FrameTest():
      colorImage(),
      frameData()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->colorImage.serialize(outbuffer + offset);
      offset += this->frameData.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->colorImage.deserialize(inbuffer + offset);
      offset += this->frameData.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/FrameTest"; };
    const char * getMD5(){ return "758e664f737fbe3bbd452e73af9b8744"; };

  };

}
#endif