#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <fstream>

geometry_msgs::TransformStamped
get_transform(tf2_ros::Buffer& buffer,
              const std::string& gps_origin_frame,
              const std::string& gps_antenna_frame,
              const std::string& map_frame,
              const std::string& odom_frame,
              const std::string& base_frame) {

    auto odom = buffer.lookupTransform(odom_frame,
                                       base_frame,
                                       ros::Time{0},
                                       ros::Duration{5});
    auto transform = buffer.lookupTransform(gps_origin_frame,
                                            gps_antenna_frame,
                                            ros::Time{0},
                                            ros::Duration{5});
    // "Subtract" base_frame from gps_antenna_frame
    tf2::Transform gps_tf, odom_tf;
    tf2::convert(transform.transform, gps_tf);
    tf2::convert(odom.transform, odom_tf);
    transform.transform = tf2::toMsg(gps_tf * odom_tf.inverse());
    transform.child_frame_id = map_frame;
    return transform;
}

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "map_origin_broadcaster");

    ros::NodeHandle nh_p{"~"};

    std::string gps_origin_frame;
    std::string gps_antenna_frame;
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;
    bool save_on_exit;

    if (!nh_p.getParam("gps_origin_frame", gps_origin_frame)) {
        gps_origin_frame = "gps_origin";
    }
    if (!nh_p.getParam("gps_antenna_frame", gps_antenna_frame)) {
        gps_antenna_frame = "gps_antenna";
    }
    if (!nh_p.getParam("map_frame", map_frame)) {
        map_frame = "map";
    }
    if (!nh_p.getParam("odom_frame", odom_frame)) {
        odom_frame = "odom";
    }
    if (!nh_p.getParam("base_frame", base_frame)) {
        base_frame = "base_link";
    }
    if (!nh_p.getParam("save_on_exit", save_on_exit)) {
        save_on_exit = true;
    }

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_l{buffer};
    tf2_ros::StaticTransformBroadcaster tf_sb;

    // Make sure that the map frame actually exists
    while (ros::ok()) {
        bool odom_ready = buffer.canTransform(odom_frame,
                                              base_frame,
                                              ros::Time{0},
                                              ros::Duration{5});
        if (odom_ready) {
            break;
        }

        ROS_WARN_STREAM("Waiting for odometry");
    }

    while (ros::ok()) {
        // Transformation between GPS origin and GPS antenna
        bool gnss_ready = buffer.canTransform(gps_origin_frame,
                                              gps_antenna_frame,
                                              ros::Time{0},
                                              ros::Duration{5});
        if (gnss_ready) {
            ROS_INFO_STREAM("Okay, got GNSS location of " << map_frame);
            auto transform = get_transform(buffer,
                                           gps_origin_frame,
                                           gps_antenna_frame,
                                           map_frame,
                                           odom_frame,
                                           base_frame);
            tf_sb.sendTransform(transform);
            if (save_on_exit) {
                std::ofstream ofs;
                ofs.open("map_origin.txt");
                ofs << boost::posix_time::to_iso_extended_string(
                       transform.header.stamp.toBoost()) << ","
                    << transform.header.frame_id << ","
                    << transform.child_frame_id << ","
                    << transform.transform.translation.x << ","
                    << transform.transform.translation.y << ","
                    << transform.transform.translation.z << ","
                    << transform.transform.rotation.x << ","
                    << transform.transform.rotation.y << ","
                    << transform.transform.rotation.z << ","
                    << transform.transform.rotation.w << std::endl;
                ofs.close();
            }
            break;
        }

        ROS_WARN_STREAM("Waiting for GNSS data");
    }

	ros::spin();

    return 0;
}
