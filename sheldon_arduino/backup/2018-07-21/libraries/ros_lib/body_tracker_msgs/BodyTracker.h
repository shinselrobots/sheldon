#ifndef _ROS_body_tracker_msgs_BodyTracker_h
#define _ROS_body_tracker_msgs_BodyTracker_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point32.h"

namespace body_tracker_msgs
{

  class BodyTracker : public ros::Msg
  {
    public:
      typedef int32_t _body_id_type;
      _body_id_type body_id;
      typedef int32_t _tracking_status_type;
      _tracking_status_type tracking_status;
      typedef int32_t _gesture_type;
      _gesture_type gesture;
      typedef geometry_msgs::Point32 _position2d_type;
      _position2d_type position2d;
      typedef geometry_msgs::Point32 _position3d_type;
      _position3d_type position3d;

    BodyTracker():
      body_id(0),
      tracking_status(0),
      gesture(0),
      position2d(),
      position3d()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_body_id;
      u_body_id.real = this->body_id;
      *(outbuffer + offset + 0) = (u_body_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_body_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_body_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_body_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->body_id);
      union {
        int32_t real;
        uint32_t base;
      } u_tracking_status;
      u_tracking_status.real = this->tracking_status;
      *(outbuffer + offset + 0) = (u_tracking_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tracking_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tracking_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tracking_status.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracking_status);
      union {
        int32_t real;
        uint32_t base;
      } u_gesture;
      u_gesture.real = this->gesture;
      *(outbuffer + offset + 0) = (u_gesture.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gesture.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gesture.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gesture.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gesture);
      offset += this->position2d.serialize(outbuffer + offset);
      offset += this->position3d.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_body_id;
      u_body_id.base = 0;
      u_body_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_body_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_body_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_body_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->body_id = u_body_id.real;
      offset += sizeof(this->body_id);
      union {
        int32_t real;
        uint32_t base;
      } u_tracking_status;
      u_tracking_status.base = 0;
      u_tracking_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tracking_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tracking_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tracking_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tracking_status = u_tracking_status.real;
      offset += sizeof(this->tracking_status);
      union {
        int32_t real;
        uint32_t base;
      } u_gesture;
      u_gesture.base = 0;
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gesture = u_gesture.real;
      offset += sizeof(this->gesture);
      offset += this->position2d.deserialize(inbuffer + offset);
      offset += this->position3d.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "body_tracker_msgs/BodyTracker"; };
    const char * getMD5(){ return "59d0edfa4591c9ca5d11e99e57c730b1"; };

  };

}
#endif