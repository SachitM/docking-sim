#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <vector>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::CameraSubscriber intrinisics_sub_;
  image_geometry::PinholeCameraModel cam_model_;
  tf::TransformListener listener;
public:
  
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribeCamera("/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
    // intrinisics_sub_ = it_.subscribeCamera("/usb_cam/camera_info", 1,
      // &ImageConverter::getIntrinisics, this);
      x_arr[0] = 0;
      y_arr[0] = 0;
      z_arr[0] = 0;
      x_arr[1] = 0;
      y_arr[1] = 0;
      z_arr[1] = 0;

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


   void getIntrinisics(const sensor_msgs::CameraInfoConstPtr& msg)
  {
      cam_model_.fromCameraInfo(msg);
  }

  void setTagLocations(float x_det, float y_det, float z_det, int tag_id)
  {
	  //TODO: Update tag locations
    // cv::Point3d wrd(x_det,y_det,z_det);
    // cv::Point2d pts;
    // std::vector<cv::Point2d> imagePts{pts};
    // std::vector<cv::Point3d> worldPts{wrd};  
    // cv::projectPoints(worldPts,cv::Mat::zeros(3,1,CV_64F),cv::Mat::zeros(3,1,CV_64F),cv::Mat(3,3,CV_64F, HARDCODE_K),cv::Mat(5,1,CV_64F, HARDCODE_D), imagePts );
    
    if(!int_set)
      return;

    if(tag_id == -1)
    {
      
      if(int(x_arr[0]) == 0  &&  int(x_arr[1]) == 0)
      {
        x_arr[2]=0;
        y_arr[2]=0;
        z_arr[2]=0;
       ROS_DEBUG("None");
      }
      else if(int(x_arr[0]) == 0)
      {
        x_arr[2]=x_arr[1];
        y_arr[2]=y_arr[1];
        z_arr[2]=z_arr[1];
        ROS_DEBUG("2 vis%f,%f", x_arr[0],x_arr[1]);
      }
      else if(int(x_arr[1]) == 0)
      {
        x_arr[2]=x_arr[0];
        y_arr[2]=y_arr[0];
        z_arr[2]=z_arr[0];
        ROS_DEBUG("1 vis");
      }
      else
      {
        x_arr[2]=(x_arr[1]+x_arr[0])/2;
        y_arr[2]=(y_arr[1]+y_arr[0])/2;
        z_arr[2]=(z_arr[1]+z_arr[0])/2;
        ROS_DEBUG("Both vis");
        
      }


      x_arr[0] = 0;
      y_arr[0] = 0;
      z_arr[0] = 0;
      x_arr[1] = 0;
      y_arr[1] = 0;
      z_arr[1] = 0;

      return;
    }

    cv::Point3d pt_cv(x_det, y_det, z_det);
    cv::Point2d imagePts;
    imagePts = cam_model_.project3dToPixel(pt_cv);

    x_arr[tag_id] = imagePts.x;

    y_arr[tag_id] = imagePts.y;
    // ROS_INFO_STREAM(x_arr.back());
    // ROS_INFO_STREAM(y_arr.back());
    z_arr[tag_id]=z_det;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& int_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cam_model_.fromCameraInfo(int_msg);
    int_set = true;

    tf::StampedTransform transform;
       try{

        ros::Time now = ros::Time::now();
        // listener.waitForTransform("camera", "april_tf",
        //                       now, ros::Duration(3.0));

         listener.lookupTransform("camera_link", "april_tf",ros::Time(0), transform);
        //  ROS_INFO("3D : %f,%f,%f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        //  setTagLocations(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
       }
       catch (tf::TransformException ex){
        //  ROS_ERROR("%s",ex.what());
        //  ros::Duration(1.0).sleep();
       }


	//TODO: Draw circles at tag locations on image. 
  
  if(int(x_arr[2]))
  {
    // ROS_INFO("Circles%f",x_arr[2]);
    cv::circle(cv_ptr->image, cv::Point(x_arr[2]+7.2, y_arr[2]), 20/z_arr[2], CV_RGB(0,255,0), -1);
  }
  else
  {
    // ROS_INFO("NoCircles%f",x_arr[2]);
  }
    // for(int i=1;i<x_arr.size();i++)
    // {
      
    //   cv::line(cv_ptr->image, cv::Point(x_arr[i-1], y_arr[i-1]), cv::Point(x_arr[i], y_arr[i]), CV_RGB(0,255,0), 8/z_arr[i]);
    // }
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // TODO:Output modified video stream
    // Convert the modified frames into sensor_msgs::Image message and publish it using image_pub

    image_pub_.publish(cv_ptr->toImageMsg());
  }

private:
  float x_loc ,y_loc, z_loc;
  float x_arr[3];
  float y_arr[3];
  float z_arr[3];
  bool int_set = false;
   std::vector<std::string> frame_ids_;            
};
