#ifndef _ROS_SERVICE_AcquireObjectImage_h
#define _ROS_SERVICE_AcquireObjectImage_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace cob_object_detection_msgs
{

static const char ACQUIREOBJECTIMAGE[] = "cob_object_detection_msgs/AcquireObjectImage";

  class AcquireObjectImageRequest : public ros::Msg
  {
    public:
      typedef const char* _object_name_type;
      _object_name_type object_name;
      typedef bool _reset_image_counter_type;
      _reset_image_counter_type reset_image_counter;
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;
      uint32_t sdh_joints_length;
      typedef geometry_msgs::PoseStamped _sdh_joints_type;
      _sdh_joints_type st_sdh_joints;
      _sdh_joints_type * sdh_joints;

    AcquireObjectImageRequest():
      object_name(""),
      reset_image_counter(0),
      pose(),
      sdh_joints_length(0), sdh_joints(NULL)
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
      union {
        bool real;
        uint8_t base;
      } u_reset_image_counter;
      u_reset_image_counter.real = this->reset_image_counter;
      *(outbuffer + offset + 0) = (u_reset_image_counter.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reset_image_counter);
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->sdh_joints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sdh_joints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sdh_joints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sdh_joints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sdh_joints_length);
      for( uint32_t i = 0; i < sdh_joints_length; i++){
      offset += this->sdh_joints[i].serialize(outbuffer + offset);
      }
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
      union {
        bool real;
        uint8_t base;
      } u_reset_image_counter;
      u_reset_image_counter.base = 0;
      u_reset_image_counter.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reset_image_counter = u_reset_image_counter.real;
      offset += sizeof(this->reset_image_counter);
      offset += this->pose.deserialize(inbuffer + offset);
      uint32_t sdh_joints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sdh_joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sdh_joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sdh_joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sdh_joints_length);
      if(sdh_joints_lengthT > sdh_joints_length)
        this->sdh_joints = (geometry_msgs::PoseStamped*)realloc(this->sdh_joints, sdh_joints_lengthT * sizeof(geometry_msgs::PoseStamped));
      sdh_joints_length = sdh_joints_lengthT;
      for( uint32_t i = 0; i < sdh_joints_length; i++){
      offset += this->st_sdh_joints.deserialize(inbuffer + offset);
        memcpy( &(this->sdh_joints[i]), &(this->st_sdh_joints), sizeof(geometry_msgs::PoseStamped));
      }
     return offset;
    }

    const char * getType(){ return ACQUIREOBJECTIMAGE; };
    const char * getMD5(){ return "a834da64b488488418ecf10d2737befd"; };

  };

  class AcquireObjectImageResponse : public ros::Msg
  {
    public:

    AcquireObjectImageResponse()
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

    const char * getType(){ return ACQUIREOBJECTIMAGE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class AcquireObjectImage {
    public:
    typedef AcquireObjectImageRequest Request;
    typedef AcquireObjectImageResponse Response;
  };

}
#endif
