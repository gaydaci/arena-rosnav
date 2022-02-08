#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <cstdlib>

#include <ros/ros.h>

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "sndfile.h"

#include "AL/al.h"
#include "AL/alc.h"
#include "AL/alext.h"


struct StreamPlayer {

    ALuint mSource{0};

    StreamPlayer()
    {   
        // alGenBuffers(1, &mBuffer);
        // if(ALenum err{alGetError()})
        //     throw std::runtime_error{"alGenBuffers failed"};
        // ROS_INFO("StreamPlayer: Generated buffer with id %ld", mBuffer);
        
        alGenSources(1, &mSource);
        if(ALenum err{alGetError()})
        {
            // alDeleteBuffers(1, &mBuffer);
            ROS_ERROR("alGenSources failed");
        }
        ROS_INFO("StreamPlayer: Generated source with id %d", mSource);

    }
    
    ~StreamPlayer()
    {
        alDeleteSources(1, &mSource);
        // alDeleteBuffers(1, &mBuffer);
        // if(mSndfile)
        //     sf_close(mSndfile);
    }

    void close()
    {
        
        ROS_INFO("!!!Closing StreamPlayer %d!\n", mSource);
        alSourceRewind(mSource);
        alSourcei(mSource, AL_BUFFER, 0);
    }

    bool prepare(float pos_x_source, float pos_y_source)
    {

        // ROS_INFO("prepare: %d id", mSource);
        // Source position is not relative to the listener
        alSourcei(mSource, AL_SOURCE_RELATIVE, AL_FALSE);
        
        ALfloat source_pos[3];
        alGetSourcefv(mSource, AL_POSITION, source_pos);
        source_pos[0] = pos_x_source;
        source_pos[1] = pos_y_source;
        alSourcefv(mSource, AL_POSITION, source_pos);

        ALfloat source_gain = 1.0f;
        alSourcef(mSource, AL_GAIN, source_gain);

        // ALfloat max_distance = 1.0f;
        // alSourcef(mSource, AL_MAX_DISTANCE, max_distance);

        //ALfloat reference_distance = 2.5f;
        // turns looping on
        alSourcei(mSource, AL_LOOPING, AL_TRUE);
        
        if(ALenum err{alGetError()})
        {
            ROS_ERROR("Failed to setup sound source%d %s (0x%04x)\n", mSource, alGetString(err), err);
            return false;
        }
        return true;
    }

    bool source_stopped() 
    {
        ALenum state;
        alGetSourcei(mSource, AL_SOURCE_STATE, &state);
        // ROS_INFO("source's state is: %d", state);
        if (state == AL_STOPPED || state == AL_INITIAL) {
            close();
            return true;
        }
        return false;
    }

    bool update_source_position(float pos_x, float pos_y)
    {
        ALenum state;
        ALfloat offset;
        
        alGetSourcei(mSource, AL_SOURCE_STATE, &state);
        if(ALenum err{alGetError()})
        {
            ROS_ERROR("Failed to get sound source %s (0x%04x)\n", alGetString(err), err);
            return false;
        }

        if(alGetError() == AL_NO_ERROR && state == AL_PLAYING) {
            ALCcontext *context = alcGetCurrentContext();
            
            alcSuspendContext(context);
            
            /* Get the source offset. */
            alGetSourcef(mSource, AL_SEC_OFFSET, &offset);
            // ROS_INFO_STREAM("Offset: " << offset);
               
            ALfloat source_pos[3];
            alGetSourcefv(mSource, AL_POSITION, source_pos);
            source_pos[0] = pos_x;
            source_pos[1] = pos_y;
            alSourcefv(mSource, AL_POSITION, source_pos);
            // ROS_INFO("Source position: %f, %f, %f", 
            //         source_pos[0], source_pos[1], source_pos[2]);
           
            alcProcessContext(context);
        } else if (alGetError() == AL_NO_ERROR && state == AL_STOPPED) {
            close();
            return false;
        }
        return true;
    }

    bool play(int buffer_id)
    {
        close();
        if( buffer_id != 0) {
            alSourcei(mSource, AL_BUFFER, buffer_id);
            alSourcePlay(mSource);
        } 

        if(ALenum err{alGetError()})
        {
            ROS_ERROR("Failed to play sound source %s (0x%04x)\n", alGetString(err), err);
            return false;
        }
        return true;
    }

    int get_source_offset() {
        
        ALenum state;
        alGetSourcei(mSource, AL_SOURCE_STATE, &state);
        // ROS_INFO("source's state is: %d", state);
        if (state != AL_PLAYING) {
            return 0;
        }
        int sample_offset;
        alGetSourcei(mSource, AL_SAMPLE_OFFSET, &sample_offset);
        // ROS_INFO("!!!! Current byte offset of source %d is %d", mSource, byte_offset);

        if(ALenum err{alGetError()})
        {
            ROS_ERROR("Failed to get source's %d current offset %s (0x%04x)\n", mSource, alGetString(err), err);
            return -1;
        }

        return sample_offset*2;
    }

    // void al_nssleep(unsigned long nsec)
    // {
    //     struct timespec ts, rem;
    //     ts.tv_sec = static_cast<time_t>(nsec / 1000000000ul);
    //     ts.tv_nsec = static_cast<long>(nsec % 1000000000ul);
    //     while(nanosleep(&ts, &rem) == -1 && errno == EINTR)
    //         ts = rem;
    // }   
};