/*********************************************************************
* Compiler:         gcc 4.6.3
*
* Company:          Institute for Cognitive Systems
*                   Technical University of Munich
*
* Author:           Emmanuel Dean (dean@tum.de)
*                   Karinne Ramirez (karinne.ramirez@tum.de)
*
* Compatibility:    Ubuntu 12.04 64bit (ros hydro)
*
* Software Version: V0.1
*
* Created:          01.06.2015
*
* Comment:          turtle connection and visualization (Sensor and Signals)
*
********************************************************************/


/*********************************************************************
* STD INCLUDES
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>


/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>

/*********************************************************************
 * CUSTOM CLASS
 * ******************************************************************/
#include <turtle_vis/myClass/TurtleClass.h>



int main( int argc, char** argv )
{

    ros::init(argc, argv, "turtle_control",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Publishing turtle control..");

    ros::NodeHandle n;
    ros::Rate r(60);



    //ADVERTISE THE SERVICE
    turtleSpace::TurtleClass turtleF;
    ros::ServiceServer service=n.advertiseService("TurtlePose",
                                                  &turtleSpace::TurtleClass::getDPose,
                                                  &turtleF);
    //CALL SERVICE FROM TERMINAL//
    //rosservice call /TurtlePose '{p: [0.5, 0.0, 3.0]}'
    //rosservice call /TurtlePose "{p: {x: 1.5, y: 1.0, theta: 0.0}}"
    //DON'T FORGET TO SOURCE THE WORKSPACE BEFORE CALLING THE SERVICE

    //ADVERTIZE THE TOPIC
    ros::Publisher  pub=n.advertise<turtle_vis::DesiredPose>("turtle_control",100);
    ros::Subscriber sub=n.subscribe("mobile_base_controller/odom",100, &turtleSpace::TurtleClass::tiagocall,
				   &turtleF);
    ros::Publisher  control_position=n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
    ros::Time ti, tf;
    ti=ros::Time::now();

    //Proportional Gain
   // Matrix3d Kp;

    //#>>>>TODO: SET GAINS

    //double p_g=0.0;

    //#>>>>TODO: LOAD p_gain FROM THE ROS PARAMETER SERVER 
    
   // ros::param::get("/turtle_gains/p_gain",p_g);
    //ROS_INFO_STREAM("p_g= "<<p_g);


    //Proportional Gain

   // Kp<<p_g,0  ,0,
     //       0  ,p_g,0,
    //        0  ,0  ,p_g;

   // ROS_INFO_STREAM("Kp= \n"<<Kp);

    Vector3d turtlePose,turtlePose_old,turtleVel;
    Vector3d error;

    
    double dt;
    double d = 0.3,k = 0.5;
    double x_prime,y_prime;
    double T_v,T_w,theat;
    
    //INITIALIZE THE TURTLE POSE
    turtlePose<<0,0,0;
    turtlePose_old=turtlePose;
    turtleVel<<0,0,0;

    //DESIRED POSE
    Vector3d turtlePose_desired_local;
    
    ////#>>>>TODO: INITIALIZE THE DESIRED POSE VARIABLE OF THE CLASS TURTLE
    turtleF.turtlePose_desired_g=turtlePose;
    turtlePose_desired_local=turtlePose;

    


    //CREATE A DESIREDOSE MSG VARIABLE
    turtle_vis::DesiredPose msg; //#>>>>TODO:DEFINE THE MSG TYPE
    geometry_msgs::Twist T;

    while(ros::ok())
    {
        tf=ros::Time::now();

        dt=tf.toSec()-ti.toSec();

        ////#>>>>TODO: Get Desired Pose from the class variable
        turtlePose_desired_local=turtleF.getLocalDesiredPose();
        turtlePose = turtleF.getLocalPose_tiago();
        //CONTROL
        ////#>>>>TODO:COMPUTE THE ERROR BETWEEN CURRENT POSE AND DESIRED
        error=turtlePose_desired_local-turtlePose;
        x_prime = k*error[0];
        y_prime = k*error[1];
        theat = turtlePose[2];

        T_v = 1/(d*d)*(d*cos(theat)*x_prime+d*sin(theat)*y_prime);
        T_w = 1/(d*d)*(-sin(theat)*x_prime+cos(theat)*y_prime);
        T.linear.x = T_v; 
        T.linear.y = T_v;
        T.angular.z = T_w;
        control_position.publish(T);

 
        // COMPUTE THE INCREMENTS
        //turtleVel=error*dt;

        ////#>>>>TODO:COMPUTE THE NEW TURTLE POSE
        //turtlePose += -Kp*turtleVel;

        //Publish Data
        ////#>>>>TODO:SET THE MSG VARIABLE WITH THE NEW TURTLE POSE
        msg.x = turtlePose[0];
        msg.y = turtlePose[1];
        msg.theta = turtlePose[2];
        pub.publish(msg);

        ti=tf;

        //ROS::SPIN IS IMPORTANT TO UPDATE ALL THE SERVICES AND TOPICS
        ros::spinOnce();

        r.sleep();
    }

    return 0;
}


