#ifndef _ROS_realsense_ros_person_UserInfo_h
#define _ROS_realsense_ros_person_UserInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_person
{

  class UserInfo : public ros::Msg
  {
    public:
      typedef int32_t _Id_type;
      _Id_type Id;

    UserInfo():
      Id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Id;
      u_Id.real = this->Id;
      *(outbuffer + offset + 0) = (u_Id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Id;
      u_Id.base = 0;
      u_Id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Id = u_Id.real;
      offset += sizeof(this->Id);
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/UserInfo"; };
    const char * getMD5(){ return "4c256f183ddb18ca1c8cac601691eb32"; };

  };

}
#endif