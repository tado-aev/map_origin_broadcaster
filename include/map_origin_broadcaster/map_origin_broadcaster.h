#ifndef MAP_ORIGIN_BROADCASTER_H_
#define MAP_ORIGIN_BROADCASTER_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <geo_pos_conv.hh>

#include <string>

class MapOriginBroadcaster {
public:
	MapOriginBroadcaster();

	void
	start_broadcasting();

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_p;
	ros::Subscriber fix_sub;
	ros::Subscriber hdg_sub;
	geometry_msgs::TransformStamped transform;
	tf2_ros::StaticTransformBroadcaster tf_sb;

	bool fix_ready;
	bool hdg_ready;

    std::string gps_origin_frame;
	std::string map_frame;
	bool save_on_exit;
	int plane;

	/**
	 * GPS coordinates of the map origin
	 */

	/* Private Methods */
	void
	get_params();

	void
	fix_callback(const sensor_msgs::NavSatFix& msg);

	void
	hdg_callback(const geometry_msgs::QuaternionStamped& msg);
};

#endif /* end of include guard */
