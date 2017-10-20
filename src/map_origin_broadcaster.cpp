#include <map_origin_broadcaster/map_origin_broadcaster.h>

MapOriginBroadcaster::MapOriginBroadcaster()
    : nh{}
    , nh_p{"~"}
    , transform{}
    , tf_sb{}
    , fix_ready{false}
    , hdg_ready{false}
{
	get_params();
	fix_sub = nh.subscribe("fix",
                           1,
                           &MapOriginBroadcaster::fix_callback,
                           this);
	hdg_sub = nh.subscribe("heading",
                           1,
                           &MapOriginBroadcaster::hdg_callback,
                           this);
}

void
MapOriginBroadcaster::start_broadcasting() {
	ros::Rate rate{10};
	while (ros::ok() && (!fix_ready || !hdg_ready)) {
		rate.sleep();
		ros::spinOnce();
	}
	tf_sb.sendTransform(transform);
    ROS_INFO_STREAM("Map origin is:");
    ROS_INFO_STREAM("  x:" << transform.transform.translation.x);
    ROS_INFO_STREAM("  y:" << transform.transform.translation.y);
    ROS_INFO_STREAM("  z:" << transform.transform.translation.z);
    ROS_INFO_STREAM("  x:" << transform.transform.rotation.x);
    ROS_INFO_STREAM("  y:" << transform.transform.rotation.y);
    ROS_INFO_STREAM("  z:" << transform.transform.rotation.z);
    ROS_INFO_STREAM("  w:" << transform.transform.rotation.w);

	// We already got all the information we need
	fix_sub.shutdown();
	hdg_sub.shutdown();
}

void
MapOriginBroadcaster::get_params() {
	if (!nh_p.getParam("gps_origin_frame", gps_origin_frame)) {
		gps_origin_frame = "gps_origin";
	}
	if (!nh_p.getParam("map_frame", map_frame)) {
		map_frame = "map";
	}
	if (!nh_p.getParam("save_on_exit", save_on_exit)) {
		save_on_exit = true;
	}
    if (!nh_p.getParam("plane", plane)) {
        plane = 10;
        ROS_ERROR_STREAM("Parameter 'plane' not found.");
    }
    ROS_INFO_STREAM("Using plane: " << plane);
}

void
MapOriginBroadcaster::fix_callback(const sensor_msgs::NavSatFix& msg) {
	auto converter = geo_pos_conv{};
	converter.set_plane(plane);
	converter.llh_to_xyz(msg.latitude, msg.longitude, msg.altitude);

	transform.header.stamp = msg.header.stamp;
	transform.header.seq = 0;
	transform.header.frame_id = gps_origin_frame;
	transform.child_frame_id = map_frame;
	auto& offset = transform.transform.translation;
	offset.x = converter.y();
	offset.y = converter.x();
	offset.z = converter.z();

	fix_ready = true;
}

void
MapOriginBroadcaster::hdg_callback(const geometry_msgs::QuaternionStamped& msg) {
	transform.header.stamp = msg.header.stamp;
	transform.header.seq = 0;
	transform.header.frame_id = gps_origin_frame;
	transform.child_frame_id = map_frame;
	transform.transform.rotation = msg.quaternion;

	hdg_ready = true;
}
