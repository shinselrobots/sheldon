#ifndef _ROS_cob_perception_msgs_ActionRecognitionmsg_h
#define _ROS_cob_perception_msgs_ActionRecognitionmsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace cob_perception_msgs
{

  class ActionRecognitionmsg : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t labels_length;
      typedef char* _labels_type;
      _labels_type st_labels;
      _labels_type * labels;
      uint32_t probabilities_length;
      typedef float _probabilities_type;
      _probabilities_type st_probabilities;
      _probabilities_type * probabilities;

    ActionRecognitionmsg():
      header(),
      labels_length(0), labels(NULL),
      probabilities_length(0), probabilities(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->labels_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->labels_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->labels_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->labels_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->labels_length);
      for( uint32_t i = 0; i < labels_length; i++){
      uint32_t length_labelsi = strlen(this->labels[i]);
      varToArr(outbuffer + offset, length_labelsi);
      offset += 4;
      memcpy(outbuffer + offset, this->labels[i], length_labelsi);
      offset += length_labelsi;
      }
      *(outbuffer + offset + 0) = (this->probabilities_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->probabilities_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->probabilities_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->probabilities_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->probabilities_length);
      for( uint32_t i = 0; i < probabilities_length; i++){
      union {
        float real;
        uint32_t base;
      } u_probabilitiesi;
      u_probabilitiesi.real = this->probabilities[i];
      *(outbuffer + offset + 0) = (u_probabilitiesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_probabilitiesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_probabilitiesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_probabilitiesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->probabilities[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t labels_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->labels_length);
      if(labels_lengthT > labels_length)
        this->labels = (char**)realloc(this->labels, labels_lengthT * sizeof(char*));
      labels_length = labels_lengthT;
      for( uint32_t i = 0; i < labels_length; i++){
      uint32_t length_st_labels;
      arrToVar(length_st_labels, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_labels; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_labels-1]=0;
      this->st_labels = (char *)(inbuffer + offset-1);
      offset += length_st_labels;
        memcpy( &(this->labels[i]), &(this->st_labels), sizeof(char*));
      }
      uint32_t probabilities_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      probabilities_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      probabilities_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      probabilities_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->probabilities_length);
      if(probabilities_lengthT > probabilities_length)
        this->probabilities = (float*)realloc(this->probabilities, probabilities_lengthT * sizeof(float));
      probabilities_length = probabilities_lengthT;
      for( uint32_t i = 0; i < probabilities_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_probabilities;
      u_st_probabilities.base = 0;
      u_st_probabilities.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_probabilities.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_probabilities.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_probabilities.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_probabilities = u_st_probabilities.real;
      offset += sizeof(this->st_probabilities);
        memcpy( &(this->probabilities[i]), &(this->st_probabilities), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "cob_perception_msgs/ActionRecognitionmsg"; };
    const char * getMD5(){ return "35ce3f516ef25de507f2f0b3b42469d4"; };

  };

}
#endif