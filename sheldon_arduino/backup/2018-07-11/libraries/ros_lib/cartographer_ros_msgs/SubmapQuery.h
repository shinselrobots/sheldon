#ifndef _ROS_SERVICE_SubmapQuery_h
#define _ROS_SERVICE_SubmapQuery_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace cartographer_ros_msgs
{

static const char SUBMAPQUERY[] = "cartographer_ros_msgs/SubmapQuery";

  class SubmapQueryRequest : public ros::Msg
  {
    public:
      typedef int32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;
      typedef int32_t _submap_index_type;
      _submap_index_type submap_index;

    SubmapQueryRequest():
      trajectory_id(0),
      submap_index(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_trajectory_id;
      u_trajectory_id.real = this->trajectory_id;
      *(outbuffer + offset + 0) = (u_trajectory_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trajectory_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trajectory_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trajectory_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_id);
      union {
        int32_t real;
        uint32_t base;
      } u_submap_index;
      u_submap_index.real = this->submap_index;
      *(outbuffer + offset + 0) = (u_submap_index.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_submap_index.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_submap_index.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_submap_index.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->submap_index);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_trajectory_id;
      u_trajectory_id.base = 0;
      u_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->trajectory_id = u_trajectory_id.real;
      offset += sizeof(this->trajectory_id);
      union {
        int32_t real;
        uint32_t base;
      } u_submap_index;
      u_submap_index.base = 0;
      u_submap_index.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_submap_index.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_submap_index.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_submap_index.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->submap_index = u_submap_index.real;
      offset += sizeof(this->submap_index);
     return offset;
    }

    const char * getType(){ return SUBMAPQUERY; };
    const char * getMD5(){ return "5fc429a478a6d73822616720a31a2158"; };

  };

  class SubmapQueryResponse : public ros::Msg
  {
    public:
      typedef int32_t _submap_version_type;
      _submap_version_type submap_version;
      uint32_t cells_length;
      typedef uint8_t _cells_type;
      _cells_type st_cells;
      _cells_type * cells;
      typedef int32_t _width_type;
      _width_type width;
      typedef int32_t _height_type;
      _height_type height;
      typedef float _resolution_type;
      _resolution_type resolution;
      typedef geometry_msgs::Pose _slice_pose_type;
      _slice_pose_type slice_pose;
      typedef const char* _error_message_type;
      _error_message_type error_message;

    SubmapQueryResponse():
      submap_version(0),
      cells_length(0), cells(NULL),
      width(0),
      height(0),
      resolution(0),
      slice_pose(),
      error_message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_submap_version;
      u_submap_version.real = this->submap_version;
      *(outbuffer + offset + 0) = (u_submap_version.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_submap_version.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_submap_version.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_submap_version.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->submap_version);
      *(outbuffer + offset + 0) = (this->cells_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cells_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cells_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cells_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cells_length);
      for( uint32_t i = 0; i < cells_length; i++){
      *(outbuffer + offset + 0) = (this->cells[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cells[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      offset += serializeAvrFloat64(outbuffer + offset, this->resolution);
      offset += this->slice_pose.serialize(outbuffer + offset);
      uint32_t length_error_message = strlen(this->error_message);
      varToArr(outbuffer + offset, length_error_message);
      offset += 4;
      memcpy(outbuffer + offset, this->error_message, length_error_message);
      offset += length_error_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_submap_version;
      u_submap_version.base = 0;
      u_submap_version.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_submap_version.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_submap_version.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_submap_version.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->submap_version = u_submap_version.real;
      offset += sizeof(this->submap_version);
      uint32_t cells_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cells_length);
      if(cells_lengthT > cells_length)
        this->cells = (uint8_t*)realloc(this->cells, cells_lengthT * sizeof(uint8_t));
      cells_length = cells_lengthT;
      for( uint32_t i = 0; i < cells_length; i++){
      this->st_cells =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_cells);
        memcpy( &(this->cells[i]), &(this->st_cells), sizeof(uint8_t));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->resolution));
      offset += this->slice_pose.deserialize(inbuffer + offset);
      uint32_t length_error_message;
      arrToVar(length_error_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_error_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_error_message-1]=0;
      this->error_message = (char *)(inbuffer + offset-1);
      offset += length_error_message;
     return offset;
    }

    const char * getType(){ return SUBMAPQUERY; };
    const char * getMD5(){ return "d714bb0d07bc98995c3ddaa9d96d2db4"; };

  };

  class SubmapQuery {
    public:
    typedef SubmapQueryRequest Request;
    typedef SubmapQueryResponse Response;
  };

}
#endif
