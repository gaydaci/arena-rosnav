#ifndef SOUND_MANAGER_H
#define SOUND_MANAGER_H

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <cstdlib>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <map>

#include "sndfile.h"

#include "AL/al.h"
#include "AL/alc.h"
#include "AL/alext.h"

#include "almgr.h"
#include "stream_player.h"
#include <arena_sound_srvs/AudioBufferData.h>
#include <arena_sound_srvs/AudioCurrentOffset.h>
#include <arena_sound_srvs/CreatePedSources.h>
#include <arena_sound_srvs/PrepareSource.h>
#include <arena_sound_srvs/PlaySource.h>
#include <arena_sound_srvs/UpdateSourcePos.h>
#include <arena_sound_srvs/UpdateListenerPos.h>
#include <arena_sound_srvs/GetSourceVolume.h>

class SoundManager{

private:
    ros::NodeHandle nh;

    std::unique_ptr<AudioManager> almgr;

    std::vector<StreamPlayer*> playerVector;
    // std::vector<std::shared_ptr<StreamPlayer>> playerVector;

    std::string sound_files_path;
    YAML::Node config;

    std::vector<ALuint> buffers;
    int num_buffers;

    std::map<std::string, std::string> socialStateToSoundFile;
    std::map<std::string, int> soundFileToBufferId;

    std::vector<std::vector<short>> bufferedData;
    std::vector<double> bufferMaxSignal;

    ros::ServiceServer create_ped_sources_service_;
    ros::ServiceServer prepare_source_service_;
    ros::ServiceServer play_source_service_;
    ros::ServiceServer update_source_pos_service_;
    ros::ServiceServer update_listener_pos_service_;
    ros::ServiceServer get_source_volume_service_;

    std::vector<ros::Publisher> buffer_data_pubs;
    std::vector<arena_sound_srvs::AudioBufferData> buffer_data_msgs;

    std::vector<ros::Publisher> audio_offset_pubs;
public:
    SoundManager() {
    }

    ~SoundManager() { 
        alDeleteBuffers(num_buffers, buffers.data());
    }

    void init(ros::NodeHandle & nh);

    bool CreatePedSources(arena_sound_srvs::CreatePedSources::Request &request,
                          arena_sound_srvs::CreatePedSources::Response &response);

    bool PrepareSource(arena_sound_srvs::PrepareSource::Request &request,
                       arena_sound_srvs::PrepareSource::Response &response);

    bool PlaySource(arena_sound_srvs::PlaySource::Request &request,
                    arena_sound_srvs::PlaySource::Response &response);

    bool UpdateSourcePos(arena_sound_srvs::UpdateSourcePos::Request &request,
                         arena_sound_srvs::UpdateSourcePos::Response &response);

    bool UpdateListenerPos(arena_sound_srvs::UpdateListenerPos::Request &request,
                         arena_sound_srvs::UpdateListenerPos::Response &response);

    bool GetSourceVolume(arena_sound_srvs::GetSourceVolume::Request &request,
                         arena_sound_srvs::GetSourceVolume::Response &response);

    void CreateSocialStateToFileMap();
    
    bool CreateBuffers();

    bool LoadBuffer(ALuint buffer_id, const char *filename);

};

#endif //SOUND_MANAGER_H