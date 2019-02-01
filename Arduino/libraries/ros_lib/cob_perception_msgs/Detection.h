#ifndef _ROS_cob_perception_msgs_Detection_h
#define _ROS_cob_perception_msgs_Detection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "cob_perception_msgs/Mask.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

namespace cob_perception_msgs
{

  class Detection : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _label_type;
      _label_type label;
      typedef int32_t _id_type;
      _id_type id;
      typedef const char* _detector_type;
      _detector_type detector;
      typedef float _score_type;
      _score_type score;
      typedef cob_perception_msgs::Mask _mask_type;
      _mask_type mask;
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Point _bounding_box_lwh_type;
      _bounding_box_lwh_type bounding_box_lwh;

    Detection():
      header(),
      label(""),
      id(0),
      detector(""),
      score(0),
      mask(),
      pose(),
      bounding_box_lwh()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_label = strlen(this->label);
      varToArr(outbuffer + offset, length_label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, length_label);
      offset += length_label;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_detector = strlen(this->detector);
      varToArr(outbuffer + offset, length_detector);
      offset += 4;
      memcpy(outbuffer + offset, this->detector, length_detector);
      offset += length_detector;
      union {
        float real;
        uint32_t base;
      } u_score;
      u_score.real = this->score;
      *(outbuffer + offset + 0) = (u_score.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_score.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_score.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_score.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->score);
      offset += this->mask.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->bounding_box_lwh.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_label;
      arrToVar(length_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      uint32_t length_detector;
      arrToVar(length_detector, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_detector; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_detector-1]=0;
      this->detector = (char *)(inbuffer + offset-1);
      offset += length_detector;
      union {
        float real;
        uint32_t base;
      } u_score;
      u_score.base = 0;
      u_score.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_score.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_score.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_score.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->score = u_score.real;
      offset += sizeof(this->score);
      offset += this->mask.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->bounding_box_lwh.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "cob_perception_msgs/Detection"; };
    const char * getMD5(){ return "5914d80850a216f4299709f0376ee04e"; };

  };

}
#endif