#include<turtle_vis/myClass/TurtleClass.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
namespace turtleSpace {

TurtleClass::TurtleClass()
{
    //#>>>>TODO: INITIALIZE MEMBER VARIABLE

    count_mutex = PTHREAD_MUTEX_INITIALIZER;

}
TurtleClass::~TurtleClass()
{

}

void TurtleClass::getPose(const turtle_vis::DesiredPose::ConstPtr &msg)
{
    pthread_mutex_lock( &this->count_mutex );

    this->turtlePose_g = {msg->x, msg->y, msg->theta};
    pthread_mutex_unlock( &this->count_mutex );

    //#>>>>TODO:PLOT THE OBTAINED DATA
    ROS_INFO_STREAM( "The Obtained Data:" << "x:" << msg->x << "     y:" << msg->y << "    theta:" << msg->theta  );
                
                
}
void TurtleClass::tiagocall(const nav_msgs::Odometry::ConstPtr &msg)
{
     pthread_mutex_lock( &this->count_mutex );
     Quaterniond q;
     q.x() = msg->pose.pose.orientation.x;
     q.y() = msg->pose.pose.orientation.y;
     q.z() = msg->pose.pose.orientation.z;
     q.w() = msg->pose.pose.orientation.w;
     Vector3d euler = q.toRotationMatrix().eulerAngles(0,1,2);
     euler[0] = msg->pose.pose.position.x;
     euler[1] = msg->pose.pose.position.y;
     this->turtlePose_tiago = {euler[0], euler[1], euler[2]};
     pthread_mutex_unlock( &this->count_mutex );


}

bool TurtleClass::getDPose(turtle_vis::send_desired_pose::Request &req, turtle_vis::send_desired_pose::Response &res)
{
    pthread_mutex_lock( &this->count_mutex );
    //#>>>>TODO:COPY THE REQUEST MSG TO A LOCAL VARIABLE
    this->turtlePose_desired_g = {
              			  req.desired_pose.x,
               			  req.desired_pose.y,  
               			  req.desired_pose.theta};
   				  pthread_mutex_unlock( &this->count_mutex );

    //#>>>>TODO:PLOT THE OBTAINED DATA

    ROS_INFO_STREAM( "Get the service:" << "x:   " <<req.desired_pose.x << " y:   " <<req.desired_pose.y << "theta:   " << req.desired_pose.theta  );
                

    res.reply=1;

    return true;
}

Vector3d TurtleClass::getLocalPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}

Vector3d TurtleClass::getLocalPose_tiago()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_tiago;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}

Vector3d TurtleClass::getLocalDesiredPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_desired_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}





}
