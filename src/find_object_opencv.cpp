#include "ros/ros.h"
#include "my_id_robot/FindObjectOpenCV.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// params: image  - Mat array? Holds real data to image
//         x      - reference to width of image?
//         y      - reference to height of image?
void findContours(Mat image, int &x, int &y)
{

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  int i;
  
  findContours(image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
 
  if (contours.size() == 0)
    {
      ROS_INFO("No objects found");
      return;
    }
  else
    {
      for (i = 0; i < contours.size(); i++)
	{
	  //TODO: add a way to find the width of the object.
	  //      because a fat pen needs the hand to close
	  //      less than a thin cord.
	  //      Alternative: add pressure sensing to hand.
	  
	  Point2f center;
	  float radius;

	  // circle the object in the image
	  minEnclosingCircle(contours[i], center, radius);
	  circle(image, center, radius, (0, 255, 0), 2, 8);

	  // Show the countoured image and wait for a keypress to move on
	  //	  imshow("Image", image);
	  //waitKey(0);
	  //destroyWindow("Image");

	  // mark the center of the object
	  x = int(center.x);
	  y = int(center.y);
	  ROS_INFO("Object found %d, %d", x, y);
	  break;
	}
    }
  return;
}
bool find_object(my_id_robot::FindObjectOpenCV::Request  &req,
         my_id_robot::FindObjectOpenCV::Response &res)
{
  ROS_INFO("Got a call to find an Object");
  VideoCapture capture(0);

  // test to make sure video is working
  if (!capture.isOpened())
    ROS_INFO("Camera not opened");
  else
    {
      Mat image;
      Mat grey_image;
      Mat img_noise;
      Mat image_threshold;
      int x = 0;
      int y = 0;
      capture >> image;
      namedWindow("Display Image", WINDOW_AUTOSIZE);
      imshow("Display Image", image);
      waitKey(0);
      destroyWindow("Display Image");
      cvtColor(image, grey_image, COLOR_BGR2GRAY);
      medianBlur(grey_image,img_noise, 3);
      threshold(img_noise, image_threshold, 0, 255, THRESH_OTSU);
      findContours(image_threshold, x, y);
      // res.x and res.y represent object x and y on the image
      res.x = x;
      res.y = y;
      ROS_INFO("request: s=%s", req.id_object.c_str() );
      ROS_INFO("sending back response: x=%d, y=%d", res.x, res.y);
    }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_object");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("my_id_robot", find_object);
  ROS_INFO("Ready to find Object");
  ros::spin();

  return 0;
}
