 #include "ros/ros.h"
 #include "micromouse/position.h"
 #include "sensor_msgs/LaserScan.h"
 #include<cstdlib>
 #include<vector>

class client
{
    public:
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
        client(ros::NodeHandle& nh);
        void check_wall(float& maxl,float& maxf, float& maxr);
        float goal_x,goal_y;
        float current_x=-1.35,current_y=1.227;
        
        // std::vector<std::vector<int,int>>{{14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14},
        // {13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
        // {12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
        // {11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
        // {10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
        // {9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
        // {8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
        // {7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7},
        // {7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7},
        // {8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
        // {9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
        // {10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
        // {11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
        // {12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
        // {13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
        // {14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14}};
    private:
        ros::NodeHandle nh;
        sensor_msgs::LaserScan laser_msg;
        ros::Subscriber laser_subscriber;

};

client::client(ros::NodeHandle& nh)
{
    this->laser_subscriber = nh.subscribe<sensor_msgs::LaserScan>("/my_mm_robot/laser/scan", 100, &client::laser_callback,this);
}

void client::check_wall(float& maxl,float& maxf, float& maxr)
{
    if(maxl>0.2)
    {
        this->goal_x=this->current_x+0.16;
    }
    else if(maxf>0.2)
    {
        this->goal_y=this->current_y-0.16;
    }
    else if(maxr>0.2)
    {
        this->goal_x=this->current_x-0.16;
    }
}

void client::laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{ 
    std::vector<float> left;
    std::vector<float> right;
    std::vector<float> front;
    float maxf=0,maxl=0,maxr=0;
    for (int i = 0; i < scan_msg->ranges.size(); i++) 
    {
        if(i<120)
        {
            right.push_back(scan_msg->ranges[i]);
            if (maxr<scan_msg->ranges[i])
                maxr=scan_msg->ranges[i];
        }
        else if(i>=240  &&  i<=480)
        {
            front.push_back(scan_msg->ranges[i]);
            if (maxf<scan_msg->ranges[i])
                maxf=scan_msg->ranges[i];
        }
        else if(i>=600)
        {
            left.push_back(scan_msg->ranges[i]);
            if (maxl<scan_msg->ranges[i])
            maxl=scan_msg->ranges[i];
        }
    }
    ROS_INFO("maxf %f",maxf);
    ROS_INFO("maxr %f",maxr);
    ROS_INFO("maxl %f",maxl);
    check_wall(maxl,maxf,maxr);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"goalnode_client");
    // if(argc!=4)
    // {
    //     ROS_INFO("usage: get the position x y");
    //     return 1;
    // }
    ros::NodeHandle n;
    ros::ServiceClient cli=n.serviceClient<micromouse::position>("position");
    micromouse::position srv;
    client ob(n);
    srv.request.x=ob.goal_x;
    srv.request.y=ob.goal_y;
    if (cli.call(srv))
    {
            ROS_INFO("Position status: [%f]",srv.response.goal_received[0]);
            ob.current_x=ob.goal_x;
    }
    else
    {
            ROS_ERROR("Failed to call service");
            return 1;
    }
    ros::spin();
return 0;
}