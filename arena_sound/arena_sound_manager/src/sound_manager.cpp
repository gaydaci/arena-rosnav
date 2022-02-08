#include <arena_sound_manager/sound_manager.h>

using namespace std;

void SoundManager::init(ros::NodeHandle &nh)
{
    ros::NodeHandle n;

    almgr = std::unique_ptr<AudioManager>(new AudioManager());

    sound_files_path = ros::package::getPath("arena_sound_manager") + "/sound_files/";
    CreateSocialStateToFileMap();
    CreateBuffers();

    create_ped_sources_service_ = nh.advertiseService("create_ped_sources", &SoundManager::CreatePedSources, this);
    prepare_source_service_ = nh.advertiseService("prepare_source", &SoundManager::PrepareSource, this);
    play_source_service_ = nh.advertiseService("play_source", &SoundManager::PlaySource, this);
    update_source_pos_service_ = nh.advertiseService("update_source_pos", &SoundManager::UpdateSourcePos, this);
    update_listener_pos_service_ = nh.advertiseService("update_listener_pos", &SoundManager::UpdateListenerPos, this);
    get_source_volume_service_ = nh.advertiseService("get_source_volume", &SoundManager::GetSourceVolume, this);
    ros::spin();
}

bool SoundManager::CreatePedSources(arena_sound_srvs::CreatePedSources::Request &request,
                                    arena_sound_srvs::CreatePedSources::Response &response) 
{
    for (int i = 0; i < request.source_ids.size(); i++) {
        ROS_INFO("CreatePedSources: %d", request.source_ids[i]);
        StreamPlayer* player = new StreamPlayer();
        playerVector.push_back(player);
    }

    response.success = true;
    return true;
}

bool SoundManager::PrepareSource(arena_sound_srvs::PrepareSource::Request &request,
                   arena_sound_srvs::PrepareSource::Response &response)
{    
    if(!playerVector[request.source_id-1]->prepare(request.pos_x, request.pos_y)) {
        response.success = false;
        return false;
    }
    response.success = true;
    return true;
}

bool SoundManager::PlaySource(arena_sound_srvs::PlaySource::Request &request,
                              arena_sound_srvs::PlaySource::Response &response) {
    std::string sound_file = socialStateToSoundFile[request.social_state];
    int buffer_id = soundFileToBufferId[sound_file];

    ROS_INFO("!!!!!! PlaySource: buffer_id: %d",buffer_id);
    if(!playerVector[request.source_id-1]->play(buffer_id)) {
        response.success = false;
        return false;
    }
    response.success = true;
    return true;
}

bool SoundManager::UpdateSourcePos(arena_sound_srvs::UpdateSourcePos::Request &request,
                                   arena_sound_srvs::UpdateSourcePos::Response &response)
{
    if(!playerVector[request.source_id-1]->update_source_position(request.pos_x, request.pos_y)) {
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

bool SoundManager::GetSourceVolume(arena_sound_srvs::GetSourceVolume::Request &request,
                                  arena_sound_srvs::GetSourceVolume::Response &response) 
{
    std::string sound_file = socialStateToSoundFile[request.social_state];
    int buffer_id = soundFileToBufferId[sound_file];
    // ROS_INFO("!!!!!! GetSourceVolume: buffer_id: %d",buffer_id);

    if (buffer_id == 0) {
        response.success = true;
        response.volume = 0.0f;
        return true;
    }

    int source_offset = playerVector[request.source_id-1]->get_source_offset();

    if (source_offset < 0 ) {
        response.success = false;
        return false;
    } else if (source_offset == 0) {
        response.success = true;
        response.volume = 0.0f;
        return true;
    }


    int amplitude_at_offset = abs(bufferedData[buffer_id-1][source_offset]); 
    double volume = amplitude_at_offset/bufferMaxSignal[buffer_id-1];
    // ROS_INFO("!!!! Volume at buffer %d is %f\n amplitude_at_offset %d, max_signal %f", 
    //             buffer_id, volume,amplitude_at_offset, bufferMaxSignal[buffer_id-1]);
    
    response.volume = volume;
    response.success = true;
    return true;
}

void SoundManager::CreateSocialStateToFileMap() {
    socialStateToSoundFile.insert(std::make_pair("Waiting", ""));
    socialStateToSoundFile.insert(std::make_pair("Walking", (sound_files_path + ("footsteps-3s.wav")).c_str()));
    socialStateToSoundFile.insert(std::make_pair("Running", (sound_files_path + ("running-on-street-trimmed-3s.wav")).c_str()));
    socialStateToSoundFile.insert(std::make_pair("GroupWalking", (sound_files_path + ("footsteps-3s.wav")).c_str()));
    socialStateToSoundFile.insert(std::make_pair("Talking", (sound_files_path + ("agent_talking.wav")).c_str()));
    socialStateToSoundFile.insert(std::make_pair("TalkingAndWalking", (sound_files_path + ("agent_talking_and_walking.wav")).c_str()));
    socialStateToSoundFile.insert(std::make_pair("ListeningAndWalking", (sound_files_path + ("footsteps-3s.wav")).c_str()));
    socialStateToSoundFile.insert(std::make_pair("Listening", ""));


    soundFileToBufferId.insert(std::make_pair("", 0));
}

bool SoundManager::CreateBuffers()
{
    alGenBuffers(NUM_BUFFERS, buffers);

    if(ALenum err{alGetError()})
    {
        ROS_WARN("alGenBuffers failed");
        return false;
    }
    
    for (int i = 0; i<NUM_BUFFERS; i++) {
        if(alIsBuffer(buffers[i]) != AL_TRUE)
            ROS_ERROR("SoundManager: Error creating buffer with id: %d!", buffers[i]);
    }

    if(!LoadBuffer(buffers[0], (sound_files_path + "footsteps-3s.wav").c_str()))
        ROS_ERROR("Loading buffer with id: %d failed!", buffers[0]);
    if(!LoadBuffer(buffers[1], (sound_files_path + "running-on-street-trimmed-3s.wav").c_str()))
        ROS_ERROR("Loading buffer with id: %d failed!", buffers[1]);
    if(!LoadBuffer(buffers[2], (sound_files_path + "agent_talking.wav").c_str()))
        ROS_ERROR("Loading buffer with id: %d failed!", buffers[2]);
    if(!LoadBuffer(buffers[3], (sound_files_path + "agent_talking_and_walking.wav").c_str()))
        ROS_ERROR("Loading buffer with id: %d failed!", buffers[3]);

    ALint bufferSize[NUM_BUFFERS];
    for (int i = 0; i<NUM_BUFFERS; i++) {
        alGetBufferi(buffers[i], AL_SIZE, &bufferSize[i]);
        if(alIsBuffer(buffers[i]) == AL_TRUE)
            ROS_INFO("Loaded buffer with ID %d and size: %d", buffers[i], bufferSize[i]);
    }

    return true;
}

bool SoundManager::LoadBuffer(ALuint buffer_id, const char *filename)
{
    // std::unique_ptr<ALbyte[]> mBufferData;
    short *mBufferData;
    size_t mBufferDataSize{0};

     /* Handle for the audio file to decode. */
    SNDFILE *mSndfile{nullptr};
    SF_INFO mSfInfo{};
    sf_count_t num_frames{0};
    ALsizei num_bytes{0};

    /* The format of the samples. */
    ALenum mFormat;

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
    // mBufferData = std::unique_ptr<ALbyte[]>(new ALbyte[mBufferDataSize]);
    mBufferData = new short[mBufferDataSize];

    num_frames = sf_readf_short(mSndfile, 
                        // reinterpret_cast<short*>(&mBufferData[0]),
                        mBufferData,
                        static_cast<sf_count_t>(mSfInfo.frames));    
    if(num_frames < 1)
    {
        sf_close(mSndfile);
        ROS_ERROR("Failed to read samples in %s (%ld)\n", filename, num_frames);
        return false;
    }
    
    num_bytes = static_cast<ALsizei>(num_frames * mSfInfo.channels) * static_cast<ALsizei>(sizeof(short));
    
    // auto mBufferDataPtr = mBufferData.release();
    
    // alBufferData(buffer_id, mFormat, mBufferDataPtr, num_bytes, mSfInfo.samplerate);
    alBufferData(buffer_id, mFormat, mBufferData, num_bytes, mSfInfo.samplerate);

    double signal_max;
    sf_command (mSndfile, SFC_CALC_SIGNAL_MAX, &signal_max, sizeof (signal_max)) ;
    ROS_INFO("LoadBuffers: filename %s\n, buffer_id: %d, buffer_size: %d, signal_max: %f, num_frames: %d",
                 filename, buffer_id, num_bytes, signal_max, num_frames);
    
    bufferMaxSignal.push_back(signal_max);
    bufferedData.push_back(mBufferData);

    sf_close(mSndfile);

    if(ALenum err{alGetError()}) {
        ROS_ERROR("OpenAL failed, err: %s (0x%04x) ,buffer_id: %d", alGetString(err), err, buffer_id);
        if(buffer_id && alIsBuffer(buffer_id)) 
            alDeleteBuffers(1, &buffer_id);
        return false;
    }
        
    soundFileToBufferId.insert(std::make_pair(filename, buffer_id));
    return true;
}