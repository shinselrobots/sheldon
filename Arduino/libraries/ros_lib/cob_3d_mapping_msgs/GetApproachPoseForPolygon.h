#ifndef _ROS_SERVICE_GetApproachPoseForPolygon_h
#define _ROS_SERVICE_GetApproachPoseForPolygon_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_3d_mapping_msgs/Shape.h"
#include "geometry_msgs/PoseArray.h"

namespace cob_3d_mapping_msgs
{

static const char GETAPPROACHPOSEFORPOLYGON[] = "cob_3d_mapping_msgs/GetApproachPoseForPolygon";

  class GetApproachPoseForPolygonRequest : public ros::Msg
  {
    public:
      typedef cob_3d_mapping_msgs::Shape _polygon_type;
      _polygon_type polygon;

    GetApproachPoseForPolygonRequest():
      polygon()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->polygon.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->polygon.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETAPPROACHPOSEFORPOLYGON; };
    const char * getMD5(){ return "a515bd312108ad51f996146221a5438a"; };

  };

  class GetApproachPoseForPolygonResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseArray _approach_poses_type;
      _approach_poses_type approach_poses;

    GetApproachPoseForPolygonResponse():
      approach_poses()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->approach_poses.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->approach_poses.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETAPPROACHPOSEFORPOLYGON; };
    const char * getMD5(){ return "19bb4815c71649ba6ce299aa4a70b51e"; };

  };

  class GetApproachPoseForPolygon {
    public:
    typedef GetApproachPoseForPolygonRequest Request;
    typedef GetApproachPoseForPolygonResponse Response;
  };

}
#endif
