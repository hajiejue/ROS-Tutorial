#ifndef TURTLECLASS_H
#define TURTLECLASS_H

/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <turtle_vis/DesiredPose.h>
#include <turtle_vis/send_desired_pose.h>

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>

using namespace Eigen;

namespace turtleSpace
{
    class TurtleClass
    {
    public:

        pthread_mutex_t count_mutex;

        TurtleClass();
	~TurtleClass();

                bool getDPose(turtle_vis::send_desired_pose::Request &req, turtle_vis::send_desired_pose::Response &res);

                void getPose(const turtle_vis::DesiredPose::ConstPtr &msg);
		void tiagocall(const nav_msgs::Odometry::ConstPtr &msg);
                Vector3d getLocalPose();
                Vector3d getLocalDesiredPose();

                Vector3d getLocalPose_tiago();



                Vector3d turtlePose_g; 
                Vector3d turtlePose_desired_g;



                Vector3d turtlePose_tiago; 
                Vector3d turtlePose_desired_tiago;
    };






}

#endif // TURTLECLASS_H
