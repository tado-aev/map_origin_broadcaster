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
              const std::string& gps_base_frame,
              const std::string& map_frame,
              const std::string& odom_frame,
              const std::string& base_frame) {

    auto odom = buffer.lookupTransform(odom_frame,
                                       base_frame,
                                       ros::Time{0},
                                       ros::Duration{5});
    auto transform = buffer.lookupTransform(gps_origin_frame,
                                            gps_base_frame,
                                            ros::Time{0},
                                            ros::Duration{5});
    tf2::Transform gps_tf, odom_tf;
    tf2::convert(transform.transform, gps_tf);
    tf2::convert(odom.transform, odom_tf);
    // "Subtract" odom_frame (i.e. however much travelled in odom)
    transform.transform = tf2::toMsg(gps_tf * odom_tf.inverse());
    transform.child_frame_id = map_frame;
    return transform;
}

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "map_origin_broadcaster");

    ros::NodeHandle nh_p{"~"};

    std::string gps_origin_frame;
    std::string gps_base_frame;
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;
    bool save_on_exit;

    nh_p.param("gps_origin_frame", gps_origin_frame, "gps_origin");
    nh_p.param("gps_base_frame", gps_base_frame, "gps_base_link");
    nh_p.param("map_frame", map_frame, "map");
    nh_p.param("odom_frame", odom_frame, "odom");
    nh_p.param("base_frame", base_frame, "base_link");
    nh_p.param("save_on_exit", save_on_exit, true);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_l{buffer};
    tf2_ros::StaticTransformBroadcaster tf_sb;

    // Make sure that the odom frame actually exists
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
                                              gps_base_frame,
                                              ros::Time{0},
                                              ros::Duration{5});
        if (gnss_ready) {
            ROS_INFO_STREAM("Okay, got GNSS location of " << map_frame);
            auto transform = get_transform(buffer,
                                           gps_origin_frame,
                                           gps_base_frame,
                                           map_frame,
                                           odom_frame,
                                           base_frame);
            tf_sb.sendTransform(transform);
            if (save_on_exit) {
                // In the form: 2017-10-27T10:31:05
                std::string datetime =
                    boost::posix_time::to_iso_extended_string(transform.header.stamp.toBoost());
                std::ofstream ofs;
                ofs.open(datetime + ".csv");
                // Write header
                ofs << "stamp" << ","
                    << "gps_origin_frame" << ","
                    << "map_frame" << ","
                    << "translation.x" << ","
                    << "translation.y" << ","
                    << "translation.z" << ","
                    << "rotation.x" << ","
                    << "rotation.y" << ","
                    << "rotation.z" << ","
                    << "rotation.w" << std::endl;
                // Write data
                ofs << datetime << ","
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
