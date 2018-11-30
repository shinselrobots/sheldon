#ifndef _ROS_realsense_ros_object_cpu_gpu_h
#define _ROS_realsense_ros_object_cpu_gpu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_ros_object
{

  class cpu_gpu : public ros::Msg
  {
    public:
      typedef const char* _CPU_GPU_type;
      _CPU_GPU_type CPU_GPU;

    cpu_gpu():
      CPU_GPU("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_CPU_GPU = strlen(this->CPU_GPU);
      varToArr(outbuffer + offset, length_CPU_GPU);
      offset += 4;
      memcpy(outbuffer + offset, this->CPU_GPU, length_CPU_GPU);
      offset += length_CPU_GPU;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_CPU_GPU;
      arrToVar(length_CPU_GPU, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_CPU_GPU; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_CPU_GPU-1]=0;
      this->CPU_GPU = (char *)(inbuffer + offset-1);
      offset += length_CPU_GPU;
     return offset;
    }

    const char * getType(){ return "realsense_ros_object/cpu_gpu"; };
    const char * getMD5(){ return "61995a7e6325b9685a2e0414cf8b8822"; };

  };

}
#endif