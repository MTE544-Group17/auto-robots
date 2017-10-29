#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

double ips_x;
double ips_y;
double ips_yaw;
int map_resolution = 20;

short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{

    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);

}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the Laser Scan data topic
void laser_callback(const sensor_msgs::LaserScan& msg)
{

}

int main(int argc, char **argv)
{
	  //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, laser_callback);

    //Setup topics that this node will Publish to
    ros::Publisher map_publisher = n.adverstise<nav_msgs::OccupancyGrid>("/map",1);

    //Initialize our empty map grid
    nav_msgs::OccupancyGrid map;
    nav_msgs::MapMetaData meta_data;
    int8 map_data[(map_resolution*map_resolution)];
    //Set all cell probabilities to -1 (unknown)
    for (int i=0; i<(map_resolution*map_resolution): i++)
    {
      map_data[i] = -1;
    }
    map.data = map_data;

    meta_data.time = ros::Time::now();
    meta_data.resolution = 5/map_resolution;
    meta_data.width = map_resolution;
    meta_data.height = map_resolution;

    geometry_msgs::Pose origin_pose;
    geometry_msgs::Point origin_point;
    geometry_msgs::Quaternion origin_quat;

    origin_point.x = 0;
    origin_point.y = 0;
    origin_point.z = 0;
    origin_quat.x = 0;
    origin_quat.y = 0;
    origin_quat.z = 0;
    origin_quat.w = 0;

    meta_data.origin = origin_pose;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
      loop_rate.sleep(); //Maintain the loop rate
      ros::spinOnce();   //Check for new messages

      //Main loop code goes here:
    }

    return 0;
}
