#ifndef _ROS_realsense_ros_slam_TrackingAccuracy_h
#define _ROS_realsense_ros_slam_TrackingAccuracy_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace realsense_ros_slam
{

  class TrackingAccuracy : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _tracking_accuracy_type;
      _tracking_accuracy_type tracking_accuracy;
      enum { TRACKING_ACCURACY_FAILED = 0 };
      enum { TRACKING_ACCURACY_LOW = 1 };
      enum { TRACKING_ACCURACY_MEDIUM = 2 };
      enum { TRACKING_ACCURACY_HIGH = 3 };

    TrackingAccuracy():
      header(),
      tracking_accuracy(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->tracking_accuracy >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tracking_accuracy >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tracking_accuracy >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tracking_accuracy >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracking_accuracy);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->tracking_accuracy =  ((uint32_t) (*(inbuffer + offset)));
      this->tracking_accuracy |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tracking_accuracy |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tracking_accuracy |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tracking_accuracy);
     return offset;
    }

    const char * getType(){ return "realsense_ros_slam/TrackingAccuracy"; };
    const char * getMD5(){ return "f02ead1dd88aaa736f807c98890ab00c"; };

  };

}
#endif