#ifndef _ROS_cob_3d_mapping_msgs_PlaneScene_h
#define _ROS_cob_3d_mapping_msgs_PlaneScene_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "cob_3d_mapping_msgs/Plane.h"

namespace cob_3d_mapping_msgs
{

  class PlaneScene : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t planes_length;
      typedef cob_3d_mapping_msgs::Plane _planes_type;
      _planes_type st_planes;
      _planes_type * planes;

    PlaneScene():
      header(),
      planes_length(0), planes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->planes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->planes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->planes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->planes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->planes_length);
      for( uint32_t i = 0; i < planes_length; i++){
      offset += this->planes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t planes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      planes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      planes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      planes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->planes_length);
      if(planes_lengthT > planes_length)
        this->planes = (cob_3d_mapping_msgs::Plane*)realloc(this->planes, planes_lengthT * sizeof(cob_3d_mapping_msgs::Plane));
      planes_length = planes_lengthT;
      for( uint32_t i = 0; i < planes_length; i++){
      offset += this->st_planes.deserialize(inbuffer + offset);
        memcpy( &(this->planes[i]), &(this->st_planes), sizeof(cob_3d_mapping_msgs::Plane));
      }
     return offset;
    }

    const char * getType(){ return "cob_3d_mapping_msgs/PlaneScene"; };
    const char * getMD5(){ return "5aa30c5b0316a946cb91f45ed3f1d963"; };

  };

}
#endif