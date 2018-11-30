#ifndef _ROS_realsense_ros_person_Landmark_h
#define _ROS_realsense_ros_person_Landmark_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"

namespace realsense_ros_person
{

  class Landmark : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _location_type;
      _location_type location;
      typedef geometry_msgs::Point32 _realWorldCoordinates_type;
      _realWorldCoordinates_type realWorldCoordinates;

    Landmark():
      location(),
      realWorldCoordinates()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->location.serialize(outbuffer + offset);
      offset += this->realWorldCoordinates.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->location.deserialize(inbuffer + offset);
      offset += this->realWorldCoordinates.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/Landmark"; };
    const char * getMD5(){ return "40b8559cad6d27176fd0a970412b03f0"; };

  };

}
#endif