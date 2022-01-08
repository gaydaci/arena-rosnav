#include <arena_sound_manager/sound_manager.h>

using namespace std;

void SoundManager::init(ros::NodeHandle &nh)
{
    ros::NodeHandle n;

    almgr = std::unique_ptr<AudioManager>(new AudioManager());

    create_source_service_ = nh.advertiseService("create_source", &SoundManager::CreateSource, this);
    update_source_pos_service_ = nh.advertiseService("update_source_pos", &SoundManager::UpdateSourcePos, this);
    update_listener_pos_service_ = nh.advertiseService("update_listener_pos", &SoundManager::UpdateListenerPos, this);
    source_stopped_service_ = nh.advertiseService("source_stopped", &SoundManager::SourceStopped, this);
    ros::spin();
}


bool SoundManager::CreateSource(arena_sound_srvs::CreateSource::Request &request,
                                arena_sound_srvs::CreateSource::Response &response)
{
    StreamPlayer* player = new StreamPlayer();
    // auto player = std::make_shared<StreamPlayer>();
    
    int source_id = playerVector.size();
    playerVector.push_back(player);

    if(!player->open(request.sound_file_path.c_str())) {
        response.success = false;
        return false;
    }
    ROS_INFO("Created Source playing: %s (%dhz)\n", request.sound_file_path.c_str(), player->mSfInfo.samplerate);


    if(!player->prepare(request.pos_x, request.pos_y))
    {
       player->close();
       response.success = false;
       return false;
    }
    
    player->play();

    response.source_id = source_id;
    response.success = true;

    return true;
}

bool SoundManager::UpdateSourcePos(arena_sound_srvs::UpdateSourcePos::Request &request,
                                   arena_sound_srvs::UpdateSourcePos::Response &response)
{
    if(!playerVector.at(request.source_id)->update_source_position(request.pos_x, request.pos_y)) {
        response.success = false;
        return false;
    }
    response.success = true;
    return true;
}

bool SoundManager::UpdateListenerPos(arena_sound_srvs::UpdateListenerPos::Request &request,
                       arena_sound_srvs::UpdateListenerPos::Response &response)
{
    if(!almgr->update_listener_position(request.pos_x, request.pos_y)) {
        response.success = false;
        return false;
    }

    response.success = true;
    return true;
}

bool SoundManager::SourceStopped(arena_sound_srvs::SourceStopped::Request &request,
                   arena_sound_srvs::SourceStopped::Response &response)
{
    if (playerVector.at(request.source_id)->source_stopped()) {
        response.stopped = true;
        return true;
    }
    response.stopped = false;
    return false;
}