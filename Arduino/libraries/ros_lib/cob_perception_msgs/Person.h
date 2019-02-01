#ifndef _ROS_cob_perception_msgs_Person_h
#define _ROS_cob_perception_msgs_Person_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "cob_perception_msgs/Skeleton.h"

namespace cob_perception_msgs
{

  class Person : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _detector_type;
      _detector_type detector;
      typedef geometry_msgs::Pose _position_type;
      _position_type position;
      typedef geometry_msgs::Point _velocity_type;
      _velocity_type velocity;
      typedef cob_perception_msgs::Skeleton _skeleton_type;
      _skeleton_type skeleton;

    Person():
      name(""),
      detector(""),
      position(),
      velocity(),
      skeleton()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_detector = strlen(this->detector);
      varToArr(outbuffer + offset, length_detector);
      offset += 4;
      memcpy(outbuffer + offset, this->detector, length_detector);
      offset += length_detector;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->skeleton.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_detector;
      arrToVar(length_detector, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_detector; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_detector-1]=0;
      this->detector = (char *)(inbuffer + offset-1);
      offset += length_detector;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->skeleton.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "cob_perception_msgs/Person"; };
    const char * getMD5(){ return "0726793e65e7420b1c7d52ed8e449b17"; };

  };

}
#endif