#ifndef _ROS_SERVICE_FinishTrajectory_h
#define _ROS_SERVICE_FinishTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cartographer_ros_msgs
{

static const char FINISHTRAJECTORY[] = "cartographer_ros_msgs/FinishTrajectory";

  class FinishTrajectoryRequest : public ros::Msg
  {
    public:
      typedef int32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;

    FinishTrajectoryRequest():
      trajectory_id(0)
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
     return offset;
    }

    const char * getType(){ return FINISHTRAJECTORY; };
    const char * getMD5(){ return "6e190c4be941828bcd09ea05053f4bb5"; };

  };

  class FinishTrajectoryResponse : public ros::Msg
  {
    public:

    FinishTrajectoryResponse()
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

    const char * getType(){ return FINISHTRAJECTORY; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class FinishTrajectory {
    public:
    typedef FinishTrajectoryRequest Request;
    typedef FinishTrajectoryResponse Response;
  };

}
#endif
