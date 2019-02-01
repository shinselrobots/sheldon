#ifndef _ROS_cob_perception_msgs_PositionMeasurement_h
#define _ROS_cob_perception_msgs_PositionMeasurement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

namespace cob_perception_msgs
{

  class PositionMeasurement : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _object_id_type;
      _object_id_type object_id;
      typedef geometry_msgs::Point _pos_type;
      _pos_type pos;
      typedef geometry_msgs::Vector3 _vel_type;
      _vel_type vel;
      typedef float _reliability_type;
      _reliability_type reliability;
      float covariance[9];
      typedef int8_t _initialization_type;
      _initialization_type initialization;

    PositionMeasurement():
      header(),
      name(""),
      object_id(""),
      pos(),
      vel(),
      reliability(0),
      covariance(),
      initialization(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_object_id = strlen(this->object_id);
      varToArr(outbuffer + offset, length_object_id);
      offset += 4;
      memcpy(outbuffer + offset, this->object_id, length_object_id);
      offset += length_object_id;
      offset += this->pos.serialize(outbuffer + offset);
      offset += this->vel.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->reliability);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->covariance[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_initialization;
      u_initialization.real = this->initialization;
      *(outbuffer + offset + 0) = (u_initialization.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->initialization);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_object_id;
      arrToVar(length_object_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_object_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_object_id-1]=0;
      this->object_id = (char *)(inbuffer + offset-1);
      offset += length_object_id;
      offset += this->pos.deserialize(inbuffer + offset);
      offset += this->vel.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->reliability));
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->covariance[i]));
      }
      union {
        int8_t real;
        uint8_t base;
      } u_initialization;
      u_initialization.base = 0;
      u_initialization.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->initialization = u_initialization.real;
      offset += sizeof(this->initialization);
     return offset;
    }

    const char * getType(){ return "cob_perception_msgs/PositionMeasurement"; };
    const char * getMD5(){ return "a1ad9f58d180ee0dd8bd6c516af86714"; };

  };

}
#endif