#include <ros/ros.h>
#include <signal.h>

#include <arena_sound_manager/sound_manager.h>

void SigintHandler(int sig) {
    ROS_WARN_NAMED("Node", "*** Shutting down... ***");

    ROS_INFO_STREAM_NAMED("Node", "Beginning ros shutdown");
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sound_manager");
    std::string ns = ros::this_node::getNamespace();
    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    ros::NodeHandle node_handle("");

    SoundManager sound_manager;
    
    // Register sigint shutdown handler
    signal(SIGINT, SigintHandler);

    sound_manager.init(node_handle);
    ROS_INFO_STREAM(":\tSound manager successfully loaded for namespace\t"<<ns);
    ros::spin();

    return 0;
}