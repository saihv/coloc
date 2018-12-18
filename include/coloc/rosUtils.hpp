#pragma once

#include <ros/ros.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "colocData.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace coloc {
    class ROSUtils
    {
    public:
        ROSUtils(unsigned int& nDrones) {
            numDrones = &nDrones;


            std::cout << "Number of drones: " << nDrones << std::endl;
            for (unsigned int i = 0; i < nDrones; ++i) {

                std::string topicName = "coloc/drone" + std::to_string(i) + "/pose";
                ros::Publisher pub = nh.advertise <geometry_msgs::PoseStamped> (topicName, 1);

                posePub.push_back(pub);

                geometry_msgs::PoseStamped pose_msg;
                poseMsg.push_back(pose_msg);
            }

            mapPub = nh.advertise <PointCloud> ("coloc/map", 1);
        }

        void loadMapIntoPointCloudMsg(colocData &data)
        {
            mapMsg.reset(new PointCloud);
            const Landmarks & landmarks = data.scene.GetLandmarks();
            for (const auto & iterLandmarks : landmarks) {
                pcData.push_back(cv::Point3f(iterLandmarks.second.X(0), iterLandmarks.second.X(1), iterLandmarks.second.X(2)));
            }

            std::cout << "Read point cloud with " << pcData.size() << " points" << std::endl;
            mapMsg->header.frame_id = "world";
            mapMsg->height = 1;
            mapMsg->width = pcData.size();

            for (int i = 0 ; i < pcData.size() ; ++i)
                mapMsg->points.push_back (pcl::PointXYZ(pcData[i].x, pcData[i].y, pcData[i].z));
        }

        void publishMsgs()
        {
            pcl_conversions::toPCL(ros::Time::now(), mapMsg->header.stamp);
            mapPub.publish(mapMsg);

            for (int i = 0; i < *numDrones; ++i)
                posePub[i].publish(poseMsg[i]);
        }

        void loadPoseIntoMsg(int droneIdx, Pose3 &pose) {
            Vec3 translation = pose.center();
            Vec3 eulerAngles = pose.rotation().eulerAngles(2, 1, 0);

            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(eulerAngles(0), eulerAngles(1), eulerAngles(2));
            // geometry_msgs::PoseStamped pose_stmp;

            poseMsg[droneIdx].header.frame_id = "world";
            poseMsg[droneIdx].pose.position.x = translation(0);
            poseMsg[droneIdx].pose.position.y = translation(1);
            poseMsg[droneIdx].pose.position.z = translation(2);
            poseMsg[droneIdx].pose.orientation = q;
        }


    private:
        ros::NodeHandle nh;
        unsigned int *numDrones;
        ros::Publisher mapPub;
        std::vector<ros::Publisher> posePub;

        std::vector<cv::Point3f> pcData;
        PointCloud::Ptr mapMsg;
        std::vector <geometry_msgs::PoseStamped> poseMsg;
    };
}
