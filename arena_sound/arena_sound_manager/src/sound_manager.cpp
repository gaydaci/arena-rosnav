#include <arena_sound_manager/sound_manager.h>

using namespace std;

void SoundManager::init(ros::NodeHandle &nh)
{
    nh = nh; //TODO fix

    almgr = std::unique_ptr<AudioManager>(new AudioManager());

    sound_files_path = ros::package::getPath("arena_sound_manager") + "/sound_files/";
    config = YAML::LoadFile(sound_files_path + "sound_files_configs.yaml");
    if (config.Type() != YAML::NodeType::Map)
        ROS_ERROR("SoundManager: YAML-Node loaded from 'sound_files_configs.yaml' is not of type YAML::NodeType::Map");
    
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

        string buffer_data_topic = "audio_buffer_data_source_" + to_string(request.source_ids[i]);
        // ROS_INFO("CreatePedSources %s", buffer_data_topic.c_str());
        ros::Publisher buffer_data_pub = nh.advertise<arena_sound_srvs::AudioBufferData>(buffer_data_topic, 1);
        buffer_data_pubs.push_back(buffer_data_pub);
    }

    response.success = true;
    return true;
}

bool SoundManager::PrepareSource(arena_sound_srvs::PrepareSource::Request &request,
                   arena_sound_srvs::PrepareSource::Response &response)
{    
    if(!playerVector[request.source_id-1]->prepare(request.pos_x, request.pos_y, request.gain)) {
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

    if (buffer_id > 0) {
        // Publich the buffer data of the sound file currently playing
        buffer_data_pubs[request.source_id-1].publish(buffer_data_msgs[buffer_id-1]);
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

    int source_sample_offset = playerVector[request.source_id-1]->get_source_offset();

    if (source_sample_offset < 0 ) {
        response.success = false;
        return false;
    } else if (source_sample_offset == 0) {
        response.success = true;
        response.volume = 0.0f;
        return true;
    }

    int source_byte_offset = source_sample_offset*2;
    int amplitude_at_offset = abs(bufferedData[buffer_id-1][source_byte_offset]); 
    double volume = amplitude_at_offset/bufferMaxSignal[buffer_id-1];
    // ROS_INFO("!!!! Volume at buffer %d is %f\n amplitude_at_offset %d, max_signal %f", 
    //             buffer_id, volume,amplitude_at_offset, bufferMaxSignal[buffer_id-1]);
    
    response.volume = volume;
    response.success = true;
    return true;
}

void SoundManager::CreateSocialStateToFileMap() {

    socialStateToSoundFile.insert(std::make_pair("Waiting", config["Waiting"].as<string>()));
    socialStateToSoundFile.insert(std::make_pair("Walking", (config["Walking"].as<string>()).c_str()));
    socialStateToSoundFile.insert(std::make_pair("Running", (config["Running"].as<string>()).c_str()));
    socialStateToSoundFile.insert(std::make_pair("GroupWalking", (config["GroupWalking"].as<string>()).c_str()));
    socialStateToSoundFile.insert(std::make_pair("Talking", (config["Talking"].as<string>()).c_str()));
    socialStateToSoundFile.insert(std::make_pair("TalkingAndWalking", (config["TalkingAndWalking"].as<string>()).c_str()));
    socialStateToSoundFile.insert(std::make_pair("ListeningAndWalking", (config["ListeningAndWalking"].as<string>()).c_str()));
    socialStateToSoundFile.insert(std::make_pair("Listening", config["Listening"].as<string>()));
    socialStateToSoundFile.insert(std::make_pair("TellStory", config["TellStory"].as<string>()));
    socialStateToSoundFile.insert(std::make_pair("Driving", (config["Driving"].as<string>()).c_str()));
    socialStateToSoundFile.insert(std::make_pair("GroupTalking", (config["GroupTalking"].as<string>()).c_str()));

    soundFileToBufferId.insert(std::make_pair("", 0));
}

bool SoundManager::CreateBuffers()
{
    vector<string> filenames;
    for (auto it = config.begin(); it != config.end(); ++it) {
      auto f = it->second.as<string>();

      if (find(filenames.begin(), filenames.end(), f) == filenames.end() and !f.empty())
        filenames.push_back(f);
    }

    num_buffers = filenames.size();
    buffers.reserve(num_buffers);
    vector<int> bufferSize(num_buffers);

    alGenBuffers(num_buffers, buffers.data());

    if(ALenum err{alGetError()})
    {
        ROS_ERROR("alGenBuffers failed");
        return false;
    }
    
    for (int i = 0; i<num_buffers; i++) {
        if(alIsBuffer(buffers[i]) != AL_TRUE)
            ROS_ERROR("SoundManager: Error creating buffer with id: %d!", buffers[i]);

        if(!LoadBuffer(buffers[i], filenames[i].c_str()))
            ROS_ERROR("Loading buffer with id: %d failed!", buffers[i]);

        alGetBufferi(buffers[i], AL_SIZE, &bufferSize[i]);
        // ROS_INFO("Loaded buffer with ID %d and size: %d", buffers[i], bufferSize[i]);
    }

    return true;
}

bool SoundManager::LoadBuffer(ALuint buffer_id, const char *filename)
{
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
    mBufferData = new short[mBufferDataSize];

    // Fill all the frames of the sound file into the array  
    num_frames = sf_readf_short(mSndfile, 
                        mBufferData,
                        static_cast<sf_count_t>(mSfInfo.frames));    
    if(num_frames < 1)
    {
        sf_close(mSndfile);
        ROS_ERROR("Failed to read samples in %s (%ld)\n", filename, num_frames);
        return false;
    }
    
    num_bytes = static_cast<ALsizei>(num_frames * mSfInfo.channels) * static_cast<ALsizei>(sizeof(short));
        
    // Fill buffer with the PCM data
    alBufferData(buffer_id, mFormat, mBufferData, num_bytes, mSfInfo.samplerate);

    double signal_max;
    sf_command (mSndfile, SFC_CALC_SIGNAL_MAX, &signal_max, sizeof (signal_max)) ;
    bufferMaxSignal.push_back(signal_max);

    ROS_INFO("LoadBuffers: filename %s\n, buffer_id: %d,mBufferDataSize: %ld\n num_bytes: %d, signal_max: %f, num_frames: %ld",
             filename, buffer_id, mBufferDataSize,
             num_bytes, signal_max, num_frames);

    vector<short> bufferDataVector;
    for (int i=0; i < num_bytes; i++) {
        bufferDataVector.push_back(mBufferData[i]);
    }
    bufferedData.push_back(bufferDataVector);

    // Define the AudioBufferData messages to be published when a buffer is played
    arena_sound_srvs::AudioBufferData msg;
    msg.audio_buffer_data = bufferDataVector;
    msg.buffer_size = num_bytes;
    msg.buffer_id = buffer_id;
    msg.sample_rate = mSfInfo.samplerate;
    msg.channels = mSfInfo.channels;
    buffer_data_msgs.push_back(msg);

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