#include <ros/ros.h>

#include <map_origin_broadcaster/map_origin_broadcaster.h>

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "map_origin_broadcaster");

    MapOriginBroadcaster mob;
	mob.start_broadcasting();

	ros::spin();

    return 0;
}
