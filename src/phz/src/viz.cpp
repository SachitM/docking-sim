#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <vector>
#include <tuple>

#define PI 3.14159

// Global Variables
// limits of phz around pod location
double hw = 2.0;
double fl = 4.0;
double bl = 1.0;

// to store phz waypoints
std::vector<std::tuple<double, double, geometry_msgs::Quaternion>> pred_coords;
std::tuple<double, double, geometry_msgs::Quaternion> gt_coords;
std::tuple<double, double, geometry_msgs::Quaternion> phz_coords;

// callback flags
int flag_pred = 0;
int flag_gt = 0;
int flag_phz = 0;

// Functions
// Subscriber Callbacks
void pod_pred_CB(const geometry_msgs::PoseArray msg)
{
  std::vector<std::tuple<double, double, geometry_msgs::Quaternion>> coords;
  for (geometry_msgs::Pose pose : msg.poses)
  {
    std::tuple<double, double, geometry_msgs::Quaternion> tup(pose.position.x, pose.position.y, pose.orientation);
    coords.push_back(tup);
  }
  pred_coords = coords;
  flag_pred = 1;
}

void pod_gt_CB(const geometry_msgs::PoseStamped msg)
{
  std::tuple<double, double, geometry_msgs::Quaternion> tup(msg.pose.position.x, msg.pose.position.y,
                                                            msg.pose.orientation);
  gt_coords = tup;
  flag_gt = 1;
}

void phz_gt_CB(const geometry_msgs::PoseStamped msg)
{
  std::tuple<double, double, geometry_msgs::Quaternion> tup(msg.pose.position.x, msg.pose.position.y,
                                                            msg.pose.orientation);
  phz_coords = tup;
  flag_phz = 1;
}

std::vector<geometry_msgs::Point> generate_box_corners(double x, double y, geometry_msgs::Quaternion q)
{
  std::vector<geometry_msgs::Point> pts;

  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, theta;
  mat.getRPY(roll, pitch, theta);

  double beta = (PI / 2) - theta;

  double sx = x - fl * cos(theta);
  double sy = y - fl * sin(theta);
  double ex = x + bl * cos(theta);
  double ey = y + bl * sin(theta);

  geometry_msgs::Point ll;
  ll.x = sx - hw * cos(beta);
  ll.y = sy + hw * sin(beta);
  ll.z = 0.0;

  geometry_msgs::Point lr;
  lr.x = sx + hw * cos(beta);
  lr.y = sy - hw * sin(beta);
  lr.z = 0.0;

  geometry_msgs::Point ur;
  ur.x = ex + hw * cos(beta);
  ur.y = ey - hw * sin(beta);
  ur.z = 0.0;

  geometry_msgs::Point ul;
  ul.x = ex - hw * cos(beta);
  ul.y = ey + hw * sin(beta);
  ul.z = 0.0;

  pts.push_back(ll);
  pts.push_back(lr);
  pts.push_back(ur);
  pts.push_back(ul);
  pts.push_back(ll);

  return pts;
}

visualization_msgs::Marker getBBMarker(std::string name, int id, float scale, float r)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.ns = name;
  mk.id = id;
  mk.type = mk.LINE_STRIP;
  mk.action = mk.ADD;
  mk.scale.x = scale;
  mk.color.a = 1.0;
  mk.color.r = r;
  mk.color.g = 1.0;
  mk.color.b = 0.0;

  return mk;
}

visualization_msgs::Marker getArrowMarker(std::string name, int id, float r, float g, float b, double x, double y,
                                          geometry_msgs::Quaternion q)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.ns = name;
  mk.id = id;
  mk.type = mk.ARROW;
  mk.action = mk.ADD;
  mk.scale.x = 0.4;
  mk.scale.y = 0.09;
  mk.scale.z = 0.1;
  mk.color.a = 1.0;
  mk.color.r = r;
  mk.color.g = g;
  mk.color.b = b;
  mk.pose.orientation = q;
  mk.pose.position.x = x;
  mk.pose.position.y = y;
  mk.pose.position.z = 0;

  return mk;
}

std::vector<visualization_msgs::Marker> getMarkers()
{
  std::vector<visualization_msgs::Marker> v;

  int id = 0;
  // for ground truth waypoint markers
  if (flag_gt)
  {
    // int n = gt_coords.size() //check data present

    visualization_msgs::Marker mk = getBBMarker("PHZ_groundtruth", id, 0.2, 1.0);
    mk.points = generate_box_corners(std::get<0>(gt_coords), std::get<1>(gt_coords), std::get<2>(gt_coords));
    id++;
    v.push_back(mk);
    v.push_back(getArrowMarker("pod_groundtruth_location", id, 1.0, 1.0, 0.0, std::get<0>(gt_coords),
                               std::get<1>(gt_coords), std::get<2>(gt_coords)));
    id++;
    flag_gt = 0;
  }

  // for predicted waypoint markers
  if (flag_pred)
  {
    int n = pred_coords.size();
    if (n > 0)
    {
      visualization_msgs::Marker mk = getBBMarker("PHZ_predicted", id, 0.2, 0.0);
      mk.points = generate_box_corners(std::get<0>(pred_coords[n - 1]), std::get<1>(pred_coords[n - 1]),
                                       std::get<2>(pred_coords[n - 1]));
      id++;
      v.push_back(mk);

      for (int i = 0; i < n; i++)
      {
        v.push_back(getArrowMarker("predicted_wp", id, 0.0, 1.0, 0.0, std::get<0>(pred_coords[i]),
                                   std::get<1>(pred_coords[i]), std::get<2>(pred_coords[i])));
        id++;
      }
    }

    flag_pred = 0;
  }

  // for phz start location
  if (flag_phz)
  {
    v.push_back(getArrowMarker("PHZ_start_location", id, 0.0, 0.0, 1.0, std::get<0>(phz_coords),
                               std::get<1>(phz_coords), std::get<2>(phz_coords)));
    flag_phz = 0;
  }

  return v;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_visualizer_node");
  ros::NodeHandle n;

  ros::Subscriber pod_pred_sub = n.subscribe("waypoints_goal", 1000, pod_pred_CB);
  ros::Subscriber pod_gt_sub = n.subscribe("pod_groundtruth", 1000, pod_gt_CB);
  ros::Subscriber phz_start_sub = n.subscribe("phz_start_groundtruth", 1000, phz_gt_CB);

  ros::Publisher phz_pub = n.advertise<visualization_msgs::MarkerArray>("phz", 10);

  visualization_msgs::MarkerArray phz_array;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (flag_pred || flag_gt || flag_phz)
    {
      phz_array.markers = getMarkers();
      phz_pub.publish(phz_array);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}