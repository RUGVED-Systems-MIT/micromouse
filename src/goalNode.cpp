 #include "ros/ros.h"
 #include "micromouse/position.h"
 #include<cstdlib>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"goalnode_client");
    if(argc!=4)
    {
        ROS_INFO("usage: get the position x y");
        return 1;
    }
ros::NodeHandle n;
ros::ServiceClient client=n.serviceClient<micromouse::position>("position");
micromouse::position srv;
srv.request.x=atoll(argv[1]);
srv.request.y=atoll(argv[2]);
if (client.call(srv))
{
        ROS_INFO("Position status: [%d]",srv.response.goal_received);
}
else
{
        ROS_ERROR("Failed to call service");
        return 1;
}
return 0;
}