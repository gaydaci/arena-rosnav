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
    
    std::unique_ptr<ALbyte[]> mBufferData;
    size_t mBufferDataSize{0};

    
    ALuint mBuffer{0}, mSource{0};
    size_t mStartOffset{0};

    /* Handle for the audio file to decode. */
    SNDFILE *mSndfile{nullptr};
    SF_INFO mSfInfo{};
    sf_count_t num_frames{0};
    ALsizei num_bytes{0};

    /* The format of the samples. */
    ALenum mFormat;

    StreamPlayer()
    {   
        alGenBuffers(1, &mBuffer);
        if(ALenum err{alGetError()})
            throw std::runtime_error{"alGenBuffers failed"};
        
        alGenSources(1, &mSource);
        if(ALenum err{alGetError()})
        {
            alDeleteBuffers(1, &mBuffer);
            throw std::runtime_error{"alGenSources failed"};
        }
    }
    
    ~StreamPlayer()
    {
        alDeleteSources(1, &mSource);
        alDeleteBuffers(1, &mBuffer);
        if(mSndfile)
            sf_close(mSndfile);
    }

    void close()
    {
        if(mSndfile)
        {
            ROS_INFO("Closing StreamPlayer!\n");
            alSourceRewind(mSource);
            alSourcei(mSource, AL_BUFFER, 0);
            sf_close(mSndfile);
            mSndfile = nullptr;
        }
    }

    bool open(const char *filename)
    {
        close();

        /* Open the file and figure out the OpenAL format. */
        mSndfile = sf_open(filename, SFM_READ, &mSfInfo);
        if(!mSndfile)
        {
            ROS_ERROR("Could not open audio in %s: %s\n", filename, sf_strerror(mSndfile));
            return false;
        }

        if(mSfInfo.frames < 1 || mSfInfo.frames > static_cast<sf_count_t>(INT_MAX/sizeof(short))/mSfInfo.channels)
        {
            ROS_ERROR("Bad sample count in %s (%ld)\n", filename, mSfInfo.frames);
            sf_close(mSndfile);
            return false;
        }

        mFormat = AL_NONE;
        if(mSfInfo.channels == 1)
            mFormat = AL_FORMAT_MONO16;
        else if(mSfInfo.channels == 2)
            mFormat = AL_FORMAT_STEREO16;
        else if(mSfInfo.channels == 3)
        {
            if(sf_command(mSndfile, SFC_WAVEX_GET_AMBISONIC, NULL, 0) == SF_AMBISONIC_B_FORMAT)
                mFormat = AL_FORMAT_BFORMAT2D_16;
        }
        else if(mSfInfo.channels == 4)
        {
            if(sf_command(mSndfile, SFC_WAVEX_GET_AMBISONIC, NULL, 0) == SF_AMBISONIC_B_FORMAT)
                mFormat = AL_FORMAT_BFORMAT3D_16;
        }
        if(!mFormat)
        {
            ROS_ERROR("Unsupported channel count: %d\n", mSfInfo.channels);
            sf_close(mSndfile);
            mSndfile = nullptr;

            return false;
        }

        mBufferDataSize = static_cast<ALuint>(mSfInfo.frames*mSfInfo.channels) * static_cast<ALuint>(sizeof(short));
        mBufferData = std::unique_ptr<ALbyte[]>(new ALbyte[mBufferDataSize]);
        
        num_frames = sf_readf_short(mSndfile, 
                            reinterpret_cast<short*>(&mBufferData[0]),
                            static_cast<sf_count_t>(mSfInfo.frames));
        if(num_frames < 1)
        {
            sf_close(mSndfile);
            ROS_ERROR("Failed to read samples in %s (%ld)\n", filename, num_frames);
            return 0;
        }
       
        num_bytes = static_cast<ALsizei>(num_frames * mSfInfo.channels) * static_cast<ALsizei>(sizeof(short));
        
        auto mBufferDataPtr = mBufferData.release();
        alBufferData(mBuffer, mFormat, mBufferDataPtr, num_bytes, mSfInfo.samplerate);

        sf_close(mSndfile);

        if(ALenum err{alGetError()}) {
            throw std::runtime_error{"OpenAL failed"};
            if(mBuffer && alIsBuffer(mBuffer)) {
                alDeleteBuffers(1, &mBuffer);
                return false;
            }
        }
            
        return true;
    }

    bool prepare(float pos_x_source, float pos_y_source)
    {
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
        alSourcei(mSource, AL_BUFFER, static_cast<ALint>(mBuffer));
        if(ALenum err{alGetError()})
        {
            ROS_ERROR("Failed to setup sound source %s (0x%04x)\n", alGetString(err), err);
            return false;
        }
        return true;
    }

    bool source_stopped() 
    {
        ALenum state;
        alGetSourcei(mSource, AL_SOURCE_STATE, &state);
        ROS_INFO("source's state is: %d", state);
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

    void play()
    {
        alSourcePlay(mSource);
        if(ALenum err{alGetError()})
        {
            ROS_ERROR("Failed to play sound source %s (0x%04x)\n", alGetString(err), err);
            return;
        }
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