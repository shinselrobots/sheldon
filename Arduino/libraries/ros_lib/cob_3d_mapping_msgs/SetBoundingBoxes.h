#ifndef _ROS_SERVICE_SetBoundingBoxes_h
#define _ROS_SERVICE_SetBoundingBoxes_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cob_object_detection_msgs/DetectionArray.h"

namespace cob_3d_mapping_msgs
{

static const char SETBOUNDINGBOXES[] = "cob_3d_mapping_msgs/SetBoundingBoxes";

  class SetBoundingBoxesRequest : public ros::Msg
  {
    public:
      typedef cob_object_detection_msgs::DetectionArray _bounding_boxes_type;
      _bounding_boxes_type bounding_boxes;

    SetBoundingBoxesRequest():
      bounding_boxes()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->bounding_boxes.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->bounding_boxes.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETBOUNDINGBOXES; };
    const char * getMD5(){ return "de59b28161e64013a1dd6ca094ca3cc7"; };

  };

  class SetBoundingBoxesResponse : public ros::Msg
  {
    public:

    SetBoundingBoxesResponse()
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

    const char * getType(){ return SETBOUNDINGBOXES; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetBoundingBoxes {
    public:
    typedef SetBoundingBoxesRequest Request;
    typedef SetBoundingBoxesResponse Response;
  };

}
#endif
