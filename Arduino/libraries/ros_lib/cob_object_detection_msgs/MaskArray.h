#ifndef _ROS_cob_object_detection_msgs_MaskArray_h
#define _ROS_cob_object_detection_msgs_MaskArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "cob_object_detection_msgs/Mask.h"

namespace cob_object_detection_msgs
{

  class MaskArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t masks_length;
      typedef cob_object_detection_msgs::Mask _masks_type;
      _masks_type st_masks;
      _masks_type * masks;

    MaskArray():
      header(),
      masks_length(0), masks(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->masks_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->masks_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->masks_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->masks_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->masks_length);
      for( uint32_t i = 0; i < masks_length; i++){
      offset += this->masks[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t masks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      masks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      masks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      masks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->masks_length);
      if(masks_lengthT > masks_length)
        this->masks = (cob_object_detection_msgs::Mask*)realloc(this->masks, masks_lengthT * sizeof(cob_object_detection_msgs::Mask));
      masks_length = masks_lengthT;
      for( uint32_t i = 0; i < masks_length; i++){
      offset += this->st_masks.deserialize(inbuffer + offset);
        memcpy( &(this->masks[i]), &(this->st_masks), sizeof(cob_object_detection_msgs::Mask));
      }
     return offset;
    }

    const char * getType(){ return "cob_object_detection_msgs/MaskArray"; };
    const char * getMD5(){ return "1d4620a4218f096b919a95d7fb64146f"; };

  };

}
#endif