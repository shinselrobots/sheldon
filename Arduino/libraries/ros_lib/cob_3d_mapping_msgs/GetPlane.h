#ifndef _ROS_SERVICE_GetPlane_h
#define _ROS_SERVICE_GetPlane_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud2.h"

namespace cob_3d_mapping_msgs
{

static const char GETPLANE[] = "cob_3d_mapping_msgs/GetPlane";

  class GetPlaneRequest : public ros::Msg
  {
    public:

    GetPlaneRequest()
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

    const char * getType(){ return GETPLANE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetPlaneResponse : public ros::Msg
  {
    public:
      typedef sensor_msgs::PointCloud2 _pc_type;
      _pc_type pc;
      typedef sensor_msgs::PointCloud2 _hull_type;
      _hull_type hull;
      uint32_t plane_coeffs_length;
      typedef std_msgs::Float32 _plane_coeffs_type;
      _plane_coeffs_type st_plane_coeffs;
      _plane_coeffs_type * plane_coeffs;

    GetPlaneResponse():
      pc(),
      hull(),
      plane_coeffs_length(0), plane_coeffs(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pc.serialize(outbuffer + offset);
      offset += this->hull.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->plane_coeffs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->plane_coeffs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->plane_coeffs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->plane_coeffs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->plane_coeffs_length);
      for( uint32_t i = 0; i < plane_coeffs_length; i++){
      offset += this->plane_coeffs[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pc.deserialize(inbuffer + offset);
      offset += this->hull.deserialize(inbuffer + offset);
      uint32_t plane_coeffs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      plane_coeffs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      plane_coeffs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      plane_coeffs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->plane_coeffs_length);
      if(plane_coeffs_lengthT > plane_coeffs_length)
        this->plane_coeffs = (std_msgs::Float32*)realloc(this->plane_coeffs, plane_coeffs_lengthT * sizeof(std_msgs::Float32));
      plane_coeffs_length = plane_coeffs_lengthT;
      for( uint32_t i = 0; i < plane_coeffs_length; i++){
      offset += this->st_plane_coeffs.deserialize(inbuffer + offset);
        memcpy( &(this->plane_coeffs[i]), &(this->st_plane_coeffs), sizeof(std_msgs::Float32));
      }
     return offset;
    }

    const char * getType(){ return GETPLANE; };
    const char * getMD5(){ return "e11c006bc7fabf06881bc7d0de7ba820"; };

  };

  class GetPlane {
    public:
    typedef GetPlaneRequest Request;
    typedef GetPlaneResponse Response;
  };

}
#endif
