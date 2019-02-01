#ifndef _ROS_cob_3d_mapping_msgs_Shape_h
#define _ROS_cob_3d_mapping_msgs_Shape_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/ColorRGBA.h"

namespace cob_3d_mapping_msgs
{

  class Shape : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _type_type;
      _type_type type;
      typedef int32_t _id_type;
      _id_type id;
      typedef float _weight_type;
      _weight_type weight;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      uint32_t params_length;
      typedef float _params_type;
      _params_type st_params;
      _params_type * params;
      uint32_t points_length;
      typedef sensor_msgs::PointCloud2 _points_type;
      _points_type st_points;
      _points_type * points;
      uint32_t vertices_length;
      typedef int32_t _vertices_type;
      _vertices_type st_vertices;
      _vertices_type * vertices;
      typedef std_msgs::ColorRGBA _color_type;
      _color_type color;
      uint32_t holes_length;
      typedef bool _holes_type;
      _holes_type st_holes;
      _holes_type * holes;
      enum { POLYGON = 0 };
      enum { LINE = 1 };
      enum { CURVED = 2 };
      enum { MESH = 3 };
      enum { OTHER = 4 };
      enum { CYLINDER = 5 };

    Shape():
      header(),
      type(0),
      id(0),
      weight(0),
      pose(),
      params_length(0), params(NULL),
      points_length(0), points(NULL),
      vertices_length(0), vertices(NULL),
      color(),
      holes_length(0), holes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_weight;
      u_weight.real = this->weight;
      *(outbuffer + offset + 0) = (u_weight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_weight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_weight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_weight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->weight);
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->params_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->params_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->params_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->params_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->params_length);
      for( uint32_t i = 0; i < params_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->params[i]);
      }
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->vertices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertices_length);
      for( uint32_t i = 0; i < vertices_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_verticesi;
      u_verticesi.real = this->vertices[i];
      *(outbuffer + offset + 0) = (u_verticesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_verticesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_verticesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_verticesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertices[i]);
      }
      offset += this->color.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->holes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->holes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->holes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->holes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->holes_length);
      for( uint32_t i = 0; i < holes_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_holesi;
      u_holesi.real = this->holes[i];
      *(outbuffer + offset + 0) = (u_holesi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->holes[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->type = u_type.real;
      offset += sizeof(this->type);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_weight;
      u_weight.base = 0;
      u_weight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_weight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_weight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_weight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->weight = u_weight.real;
      offset += sizeof(this->weight);
      offset += this->pose.deserialize(inbuffer + offset);
      uint32_t params_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->params_length);
      if(params_lengthT > params_length)
        this->params = (float*)realloc(this->params, params_lengthT * sizeof(float));
      params_length = params_lengthT;
      for( uint32_t i = 0; i < params_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_params));
        memcpy( &(this->params[i]), &(this->st_params), sizeof(float));
      }
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (sensor_msgs::PointCloud2*)realloc(this->points, points_lengthT * sizeof(sensor_msgs::PointCloud2));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(sensor_msgs::PointCloud2));
      }
      uint32_t vertices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vertices_length);
      if(vertices_lengthT > vertices_length)
        this->vertices = (int32_t*)realloc(this->vertices, vertices_lengthT * sizeof(int32_t));
      vertices_length = vertices_lengthT;
      for( uint32_t i = 0; i < vertices_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_vertices;
      u_st_vertices.base = 0;
      u_st_vertices.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_vertices.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_vertices.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_vertices.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_vertices = u_st_vertices.real;
      offset += sizeof(this->st_vertices);
        memcpy( &(this->vertices[i]), &(this->st_vertices), sizeof(int32_t));
      }
      offset += this->color.deserialize(inbuffer + offset);
      uint32_t holes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->holes_length);
      if(holes_lengthT > holes_length)
        this->holes = (bool*)realloc(this->holes, holes_lengthT * sizeof(bool));
      holes_length = holes_lengthT;
      for( uint32_t i = 0; i < holes_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_holes;
      u_st_holes.base = 0;
      u_st_holes.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_holes = u_st_holes.real;
      offset += sizeof(this->st_holes);
        memcpy( &(this->holes[i]), &(this->st_holes), sizeof(bool));
      }
     return offset;
    }

    const char * getType(){ return "cob_3d_mapping_msgs/Shape"; };
    const char * getMD5(){ return "d5fc6a3556290a571009cfc613a557d0"; };

  };

}
#endif