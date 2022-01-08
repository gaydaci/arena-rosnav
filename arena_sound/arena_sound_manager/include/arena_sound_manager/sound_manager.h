#ifndef SOUND_MANAGER_H
#define SOUND_MANAGER_H

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

#include "almgr.h"
#include "stream_player.h"
#include <arena_sound_srvs/CreateSource.h>
#include <arena_sound_srvs/UpdateSourcePos.h>
#include <arena_sound_srvs/UpdateListenerPos.h>
#include <arena_sound_srvs/SourceStopped.h>

class SoundManager{

private:
    std::unique_ptr<AudioManager> almgr;

    std::vector<StreamPlayer*> playerVector;
    // std::vector<std::shared_ptr<StreamPlayer>> playerVector;

    ros::ServiceServer create_source_service_;
    ros::ServiceServer update_source_pos_service_;
    ros::ServiceServer update_listener_pos_service_;
    ros::ServiceServer source_stopped_service_;

    // callbacks

public:
    SoundManager() {
    }

    ~SoundManager() { 
    }

    void init(ros::NodeHandle & nh);

    bool CreateSource(arena_sound_srvs::CreateSource::Request &request,
                      arena_sound_srvs::CreateSource::Response &response);

    bool UpdateSourcePos(arena_sound_srvs::UpdateSourcePos::Request &request,
                         arena_sound_srvs::UpdateSourcePos::Response &response);

    bool UpdateListenerPos(arena_sound_srvs::UpdateListenerPos::Request &request,
                         arena_sound_srvs::UpdateListenerPos::Response &response);

    bool SourceStopped(arena_sound_srvs::SourceStopped::Request &request,
                       arena_sound_srvs::SourceStopped::Response &response);

};

#endif //SOUND_MANAGER_H