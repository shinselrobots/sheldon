#ifndef _ROS_cob_3d_mapping_msgs_ShapeArray_h
#define _ROS_cob_3d_mapping_msgs_ShapeArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "cob_3d_mapping_msgs/Shape.h"

namespace cob_3d_mapping_msgs
{

  class ShapeArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t shapes_length;
      typedef cob_3d_mapping_msgs::Shape _shapes_type;
      _shapes_type st_shapes;
      _shapes_type * shapes;

    ShapeArray():
      header(),
      shapes_length(0), shapes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->shapes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->shapes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->shapes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->shapes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->shapes_length);
      for( uint32_t i = 0; i < shapes_length; i++){
      offset += this->shapes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t shapes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      shapes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      shapes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      shapes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->shapes_length);
      if(shapes_lengthT > shapes_length)
        this->shapes = (cob_3d_mapping_msgs::Shape*)realloc(this->shapes, shapes_lengthT * sizeof(cob_3d_mapping_msgs::Shape));
      shapes_length = shapes_lengthT;
      for( uint32_t i = 0; i < shapes_length; i++){
      offset += this->st_shapes.deserialize(inbuffer + offset);
        memcpy( &(this->shapes[i]), &(this->st_shapes), sizeof(cob_3d_mapping_msgs::Shape));
      }
     return offset;
    }

    const char * getType(){ return "cob_3d_mapping_msgs/ShapeArray"; };
    const char * getMD5(){ return "bc723f28be387c6f3061c2c124641aac"; };

  };

}
#endif