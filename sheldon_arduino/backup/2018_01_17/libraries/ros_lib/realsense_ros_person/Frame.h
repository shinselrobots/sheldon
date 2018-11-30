#ifndef _ROS_realsense_ros_person_Frame_h
#define _ROS_realsense_ros_person_Frame_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_ros_person/User.h"

namespace realsense_ros_person
{

  class Frame : public ros::Msg
  {
    public:
      typedef int32_t _numberOfUsers_type;
      _numberOfUsers_type numberOfUsers;
      uint32_t usersData_length;
      typedef realsense_ros_person::User _usersData_type;
      _usersData_type st_usersData;
      _usersData_type * usersData;

    Frame():
      numberOfUsers(0),
      usersData_length(0), usersData(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_numberOfUsers;
      u_numberOfUsers.real = this->numberOfUsers;
      *(outbuffer + offset + 0) = (u_numberOfUsers.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_numberOfUsers.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_numberOfUsers.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_numberOfUsers.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->numberOfUsers);
      *(outbuffer + offset + 0) = (this->usersData_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->usersData_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->usersData_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->usersData_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->usersData_length);
      for( uint32_t i = 0; i < usersData_length; i++){
      offset += this->usersData[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_numberOfUsers;
      u_numberOfUsers.base = 0;
      u_numberOfUsers.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_numberOfUsers.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_numberOfUsers.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_numberOfUsers.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->numberOfUsers = u_numberOfUsers.real;
      offset += sizeof(this->numberOfUsers);
      uint32_t usersData_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      usersData_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      usersData_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      usersData_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->usersData_length);
      if(usersData_lengthT > usersData_length)
        this->usersData = (realsense_ros_person::User*)realloc(this->usersData, usersData_lengthT * sizeof(realsense_ros_person::User));
      usersData_length = usersData_lengthT;
      for( uint32_t i = 0; i < usersData_length; i++){
      offset += this->st_usersData.deserialize(inbuffer + offset);
        memcpy( &(this->usersData[i]), &(this->st_usersData), sizeof(realsense_ros_person::User));
      }
     return offset;
    }

    const char * getType(){ return "realsense_ros_person/Frame"; };
    const char * getMD5(){ return "0fc1584508d060a88fda97d4fdfd469b"; };

  };

}
#endif