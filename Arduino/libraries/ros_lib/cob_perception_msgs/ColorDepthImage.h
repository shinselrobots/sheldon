#ifndef _ROS_cob_perception_msgs_ColorDepthImage_h
#define _ROS_cob_perception_msgs_ColorDepthImage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_perception_msgs/Rect.h"
#include "sensor_msgs/Image.h"

namespace cob_perception_msgs
{

  class ColorDepthImage : public ros::Msg
  {
    public:
      typedef cob_perception_msgs::Rect _head_detection_type;
      _head_detection_type head_detection;
      typedef sensor_msgs::Image _color_image_type;
      _color_image_type color_image;
      typedef sensor_msgs::Image _depth_image_type;
      _depth_image_type depth_image;
      uint32_t face_detections_length;
      typedef cob_perception_msgs::Rect _face_detections_type;
      _face_detections_type st_face_detections;
      _face_detections_type * face_detections;

    ColorDepthImage():
      head_detection(),
      color_image(),
      depth_image(),
      face_detections_length(0), face_detections(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->head_detection.serialize(outbuffer + offset);
      offset += this->color_image.serialize(outbuffer + offset);
      offset += this->depth_image.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->face_detections_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->face_detections_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->face_detections_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->face_detections_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->face_detections_length);
      for( uint32_t i = 0; i < face_detections_length; i++){
      offset += this->face_detections[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->head_detection.deserialize(inbuffer + offset);
      offset += this->color_image.deserialize(inbuffer + offset);
      offset += this->depth_image.deserialize(inbuffer + offset);
      uint32_t face_detections_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      face_detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      face_detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      face_detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->face_detections_length);
      if(face_detections_lengthT > face_detections_length)
        this->face_detections = (cob_perception_msgs::Rect*)realloc(this->face_detections, face_detections_lengthT * sizeof(cob_perception_msgs::Rect));
      face_detections_length = face_detections_lengthT;
      for( uint32_t i = 0; i < face_detections_length; i++){
      offset += this->st_face_detections.deserialize(inbuffer + offset);
        memcpy( &(this->face_detections[i]), &(this->st_face_detections), sizeof(cob_perception_msgs::Rect));
      }
     return offset;
    }

    const char * getType(){ return "cob_perception_msgs/ColorDepthImage"; };
    const char * getMD5(){ return "03f1995e7c816d8ab016aa6ceb57208a"; };

  };

}
#endif