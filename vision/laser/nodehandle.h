#include <cstdio>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <math.h>
#include <signal.h>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include "vision/center.h"
#include "vision/white.h"
#include "vision/black.h"
#include "vision/bin.h"
#include "vision/color.h"
#include "vision/visionlook.h"
#include "vision/avoid.h"
#define PI 3.14159265
#define FRAME_COLS 659 //width  x659
#define FRAME_ROWS 493 //height y493
#define VISION_TOPIC "/camera/image_raw"
#define YAML_PATH ros::package::getPath("vision") + "/config/FIRA.yaml"
#define IMAGE "/src/vision/1.bmp"

typedef unsigned char BYTE;
using namespace cv;
using namespace std;

class NodeHandle
{
  protected:
    NodeHandle();
    void Readyaml();
    void Parameter_getting();
    //==========================================
    void AngleLUT();
    vector<double> Angle_sin;
    vector<double> Angle_cos;
    int Frame_Area(int coordinate, int range);
    int Angle_Adjustment(int angle);
    void Pub_blackframe(Mat frame);
    void Pub_blackdis(std_msgs::Int32MultiArray distance);
    void Pub_redframe(Mat frame);
    void Pub_reddis(std_msgs::Int32MultiArray distance);

    ros::Publisher blackframe_pub;
    ros::Publisher blackdis_pub;
    ros::Publisher redframe_pub;
    ros::Publisher red_pub;
    vector <double> black_item_distance;
    vector <double> red_line_distance;
    //================center====================
    int CenterXMsg;
    int CenterYMsg;
    int InnerMsg;
    int OuterMsg;
    int FrontMsg;
    int HorizonMsg;
    double Camera_HighMsg;
    //==============distance==================
    double camera_f(double Omni_pixel);
    double Omni_distance(double pixel_dis);
    //========================================
    vector<int> HSV_green;
    vector<int> HSV_red;
    //========================================
    //============laser scan=================
    vector<double> ranges;
    vector<double> ranges2;
    vector<double> ranges3;
    vector<double> ranges4;

    int robot_angle_1;
    int robot_distance_1;
    int scan_angle_1;
    int robot_angle_2;
    int robot_distance_2;
    int scan_angle_2;
    int robot_angle_3;
    int robot_distance_3;
    int scan_angle_3;
    vector <int> scan_enable;
    vector <int> scan_parameter;

    int device_number;

    ros::Publisher mpicture;
    vision::visionlook to_strategy;

  private:
    ros::NodeHandle nh;
    ros::Subscriber save_sub;

    ros::Subscriber scan_sub;
    ros::Subscriber scan_sub2;
    ros::Subscriber scan_sub3;
    ros::Subscriber scan_sub4;
    ros::Subscriber scan_enable_sub;
    ros::Subscriber scan_parameter_sub;

    void SaveButton_setting(const vision::bin msg);
    void scancall(const sensor_msgs::LaserScan msg);
    void scancall2(const sensor_msgs::LaserScan msg);
    void scancall3(const sensor_msgs::LaserScan msg);
    void scancall4(const sensor_msgs::LaserScan msg);
    void scan_enable_call(const std_msgs::Int32MultiArray msg);
    void scan_parameter_call(const std_msgs::Int32MultiArray msg);
};
