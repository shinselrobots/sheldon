#ifndef _ROS_cob_perception_msgs_ColorDepthImageArray_h
#define _ROS_cob_perception_msgs_ColorDepthImageArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "cob_perception_msgs/ColorDepthImage.h"

namespace cob_perception_msgs
{

  class ColorDepthImageArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t head_detections_length;
      typedef cob_perception_msgs::ColorDepthImage _head_detections_type;
      _head_detections_type st_head_detections;
      _head_detections_type * head_detections;

    ColorDepthImageArray():
      header(),
      head_detections_length(0), head_detections(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->head_detections_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->head_detections_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->head_detections_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->head_detections_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->head_detections_length);
      for( uint32_t i = 0; i < head_detections_length; i++){
      offset += this->head_detections[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t head_detections_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      head_detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      head_detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      head_detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->head_detections_length);
      if(head_detections_lengthT > head_detections_length)
        this->head_detections = (cob_perception_msgs::ColorDepthImage*)realloc(this->head_detections, head_detections_lengthT * sizeof(cob_perception_msgs::ColorDepthImage));
      head_detections_length = head_detections_lengthT;
      for( uint32_t i = 0; i < head_detections_length; i++){
      offset += this->st_head_detections.deserialize(inbuffer + offset);
        memcpy( &(this->head_detections[i]), &(this->st_head_detections), sizeof(cob_perception_msgs::ColorDepthImage));
      }
     return offset;
    }

    const char * getType(){ return "cob_perception_msgs/ColorDepthImageArray"; };
    const char * getMD5(){ return "11e05088e9df0279d7f4df2d773584c2"; };

  };

}
#endif