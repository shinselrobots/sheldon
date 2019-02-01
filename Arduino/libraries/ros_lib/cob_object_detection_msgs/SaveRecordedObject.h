#ifndef _ROS_SERVICE_SaveRecordedObject_h
#define _ROS_SERVICE_SaveRecordedObject_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_object_detection_msgs
{

static const char SAVERECORDEDOBJECT[] = "cob_object_detection_msgs/SaveRecordedObject";

  class SaveRecordedObjectRequest : public ros::Msg
  {
    public:
      typedef const char* _storage_location_type;
      _storage_location_type storage_location;

    SaveRecordedObjectRequest():
      storage_location("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_storage_location = strlen(this->storage_location);
      varToArr(outbuffer + offset, length_storage_location);
      offset += 4;
      memcpy(outbuffer + offset, this->storage_location, length_storage_location);
      offset += length_storage_location;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_storage_location;
      arrToVar(length_storage_location, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_storage_location; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_storage_location-1]=0;
      this->storage_location = (char *)(inbuffer + offset-1);
      offset += length_storage_location;
     return offset;
    }

    const char * getType(){ return SAVERECORDEDOBJECT; };
    const char * getMD5(){ return "980da3132a7c1c17973e490fc0fbba86"; };

  };

  class SaveRecordedObjectResponse : public ros::Msg
  {
    public:

    SaveRecordedObjectResponse()
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

    const char * getType(){ return SAVERECORDEDOBJECT; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SaveRecordedObject {
    public:
    typedef SaveRecordedObjectRequest Request;
    typedef SaveRecordedObjectResponse Response;
  };

}
#endif
