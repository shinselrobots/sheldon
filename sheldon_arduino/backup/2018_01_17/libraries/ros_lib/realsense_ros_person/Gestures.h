#ifndef _ROS_realsense_ros_person_Gestures_h
#define _ROS_realsense_ros_person_Gestures_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_ros_person/Pointing.h"
#include "realsense_ros_person/Wave.h"

namespace realsense_ros_person
{

  class Gestures : public ros::Msg
  {
    public:
      typedef realsense_ros_person::Pointing _pointing_type;
      _pointing_type pointing;
      typedef realsense_ros_person::Wave _wave_type;
      _wave_type wave;

    Gestures():
      pointing(),
      wave()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pointing.serialize(outbuffer + offset);
      offset += this->wave.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pointing.deserialize(inbuffer + offset);
      offset += this->wave.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/Gestures"; };
    const char * getMD5(){ return "d3676ee03e8d847bc8d129f9b683163e"; };

  };

}
#endif