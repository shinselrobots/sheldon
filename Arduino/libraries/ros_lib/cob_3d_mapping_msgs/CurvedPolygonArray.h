#ifndef _ROS_cob_3d_mapping_msgs_CurvedPolygonArray_h
#define _ROS_cob_3d_mapping_msgs_CurvedPolygonArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "cob_3d_mapping_msgs/CurvedPolygon.h"

namespace cob_3d_mapping_msgs
{

  class CurvedPolygonArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t polygons_length;
      typedef cob_3d_mapping_msgs::CurvedPolygon _polygons_type;
      _polygons_type st_polygons;
      _polygons_type * polygons;

    CurvedPolygonArray():
      header(),
      polygons_length(0), polygons(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->polygons_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->polygons_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->polygons_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->polygons_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->polygons_length);
      for( uint32_t i = 0; i < polygons_length; i++){
      offset += this->polygons[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t polygons_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->polygons_length);
      if(polygons_lengthT > polygons_length)
        this->polygons = (cob_3d_mapping_msgs::CurvedPolygon*)realloc(this->polygons, polygons_lengthT * sizeof(cob_3d_mapping_msgs::CurvedPolygon));
      polygons_length = polygons_lengthT;
      for( uint32_t i = 0; i < polygons_length; i++){
      offset += this->st_polygons.deserialize(inbuffer + offset);
        memcpy( &(this->polygons[i]), &(this->st_polygons), sizeof(cob_3d_mapping_msgs::CurvedPolygon));
      }
     return offset;
    }

    const char * getType(){ return "cob_3d_mapping_msgs/CurvedPolygonArray"; };
    const char * getMD5(){ return "dca7a231f0d66f77a98e4e50cf3157dc"; };

  };

}
#endif