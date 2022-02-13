#include <ros/ros.h>

#include "AL/al.h"
#include "AL/alc.h"
#include "AL/alext.h"

struct AudioManager {
    AudioManager()
    {
        if(InitAL() != 0)
            throw std::runtime_error{"Failed to initialize OpenAL"};
    }
    
    ~AudioManager() { CloseAL(); }
    
    int InitAL()
    {
        const ALCchar *name;
        ALCdevice *device;
        ALCcontext *ctx;
        
        /* Open and initialize the default device */
        device = alcOpenDevice(NULL);
        if(!device)
        {
            ROS_ERROR("Could not open the default device!\n");
            return 1;
        }
        ctx = alcCreateContext(device, NULL);
        if(ctx == NULL || alcMakeContextCurrent(ctx) == ALC_FALSE)
        {
            if(ctx != NULL)
                alcDestroyContext(ctx);
            alcCloseDevice(device);
            ROS_ERROR("Could not set a context!\n");
            return 1;
        }
        name = NULL;
        if(alcIsExtensionPresent(device, "ALC_ENUMERATE_ALL_EXT"))
            name = alcGetString(device, ALC_ALL_DEVICES_SPECIFIER);
        if(!name || alcGetError(device) != AL_NO_ERROR)
            name = alcGetString(device, ALC_DEVICE_SPECIFIER);

        ROS_INFO("Opened \"%s\"\n", name);

        // Configure the DistanceModel
        //alDistanceModel(AL_LINEAR_DISTANCE_CLAMPED);
        alDistanceModel(AL_INVERSE_DISTANCE_CLAMPED);

        ALfloat listener_orientation[6];
        alGetListenerfv(AL_ORIENTATION, listener_orientation);
        // ROS_INFO("Listener orientation: ");
        // for (int i = 0; i < 6; i++) {
        //     ROS_INFO("%f ", listener_orientation[i]);
        // }
        // ROS_INFO("\n");

        // Controls master gain
        // ALfloat listener_gain = 0.5;
        // alListenerf(AL_GAIN, listener_gain);
        
        return 0;
    }
    
    void CloseAL(void)
    {
        ALCdevice *device;
        ALCcontext *ctx;
        ctx = alcGetCurrentContext();
        if(ctx == NULL)
            return;
        device = alcGetContextsDevice(ctx);
        alcMakeContextCurrent(NULL);
        alcDestroyContext(ctx);
        alcCloseDevice(device);
    }

    bool update_listener_position(float pos_x, float pos_y)
    {
        ALenum state;
        ALfloat offset;

        if(alGetError() == AL_NO_ERROR) {
            ALCcontext *context = alcGetCurrentContext();
            
            alcSuspendContext(context);

            ALfloat listener_pos[3];
            alGetListenerfv(AL_POSITION, listener_pos);
            listener_pos[0] = pos_x;
            listener_pos[1] = pos_y;
            alListenerfv(AL_POSITION, listener_pos);
            // ROS_INFO("Listener position: %f, %f, %f", 
            //           listener_pos[0], listener_pos[1], listener_pos[2]);
           
            alcProcessContext(context);
        } else if (alGetError() == AL_NO_ERROR && state == AL_STOPPED) {
            return false;
        }
        return true;
   }
}; 