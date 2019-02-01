#ifndef _ROS_cob_3d_mapping_msgs_SimilarityScore_h
#define _ROS_cob_3d_mapping_msgs_SimilarityScore_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_3d_mapping_msgs
{

  class SimilarityScore : public ros::Msg
  {
    public:
      typedef uint32_t _ID_type;
      _ID_type ID;
      typedef float _prob_type;
      _prob_type prob;

    SimilarityScore():
      ID(0),
      prob(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ID >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ID >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ID >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ID >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ID);
      union {
        float real;
        uint32_t base;
      } u_prob;
      u_prob.real = this->prob;
      *(outbuffer + offset + 0) = (u_prob.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prob.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prob.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prob.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prob);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->ID =  ((uint32_t) (*(inbuffer + offset)));
      this->ID |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ID |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->ID |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->ID);
      union {
        float real;
        uint32_t base;
      } u_prob;
      u_prob.base = 0;
      u_prob.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prob.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prob.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prob.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->prob = u_prob.real;
      offset += sizeof(this->prob);
     return offset;
    }

    const char * getType(){ return "cob_3d_mapping_msgs/SimilarityScore"; };
    const char * getMD5(){ return "bae66c7dce1ddf1ccfcfb042f78f22aa"; };

  };

}
#endif