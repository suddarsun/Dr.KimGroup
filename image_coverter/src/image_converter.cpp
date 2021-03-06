/*
MULTI THREADING WAS ADDED ON APRIL 19th
Description:
  Program to compensate the movements of a Drosophila melanogaster walking on
  the Active Omni-directional Treadmill (AOT). This program uses OpenCV Library
  to perform image-processing on the images captured by a Near-Infrared Camera
Written by:
  Suddarsun Shivakumar and Dr. Dal Hyung Kim
  April 2018

*/


// HEADER FILES FOR ROS
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <image_coverter/position.h>

// HEADER FILES FOR OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;


// HEADER FILES FOR OTHERS
#include <fstream> // FILE OPERATIONS
#include <chrono>  // CLOCK
#include <thread>  // MULTITHREADING
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
typedef std::chrono::high_resolution_clock Clock;
auto prev_clock = Clock::now();



// FOR DYNAMIXEL MOTORS TO BE USED IN CONJUNCTION WITH DynamixelSDK

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include "DynamixelSDK.h"
int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

// CONTROL TABLE ADDRESS FOR DIFFERENT DYNAMIXEL FUNCTIONS/OPERATIONS

#define ADDR_PRO_TORQUE_ENABLE          64    // ENABLING TORQUE
#define ADDR_PRO_GOAL_VELOCITY          104   // GOAL VELOCITY
#define ADDR_PRO_PRESENT_POSITION       132   // GETTING PRESENT POSITION
#define ADDR_FOR_OPERATING_MODE         11    // USED FOR OPERATING MODE
#define VELOCITY_MODE                   1     // VELOCITY MODE
#define POSITION_MODE                   3     // POSITION MODE

// Data Byte Length
#define LEN_PRO_GOAL_VELOCITY           4     // LENGTH OF GOAL VELOCITY IN BYTES
#define LEN_PRO_PRESENT_POSITION        4     // LENGTH OF PRESENT POSITION IN BYTES

// Protocol version
#define PROTOCOL_VERSION                2.0

// IDS OF THE THREE DYNAMIXEL MOTORS -  Can be changed to calibrate the system

#define DXL1_ID                         102
#define DXL2_ID                         101
#define DXL3_ID                         100
#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0

#define BAUDRATE                        1000000        // If motors are changed, check if this baud rate is the same for the new motors
#define DEVICENAME                      "/dev/ttyUSB0" // Can be checked if this device is present by using the "ls /dev/ttyUSB0" command in the terminal



#define datasave_variable_count         17       /* These are the variable count that's being written to the binary file:
                                                  sequence number (frame number), elapsed time, sampling time, centroid x, centroid y,
                                                  centroid theta, dxm, dym, dthm, omega_1, omega_2, omega_3, maxArea, time in seconds,
                                                  dynamixel 1 present position, dynamixel 2 present position, dynamixel 3 present position */


#define ESC_ASCII_VALUE                 0x1b


// DECLARATION OF GLOBAL VARIABLES TO COMPUTE OMEGA 1, 2, AND 3
double theta = 0.0;
double J1M[9] = {0.0,0.000408326,-0.00856566,-0.000353621,-0.000204163,-0.00856566,0.000353621,-0.000204163,-0.00856566}; // J inverse * M
double omega_1 = 0.0; // ANGULAR VELOCITY FOR MOTOR 1 IN AOT
double omega_2 = 0.0; // ANGULAR VELOCITY FOR MOTOR 2 IN AOT
double omega_3 = 0.0; // ANGULAR VELOCITY FOR MOTOR 3 IN AOT
double K = 100.0; // GAIN USED FOR THE MOTORS
double SF = 60.0;
double lin_vel =0.0;
double ang_vel = 0.0;
double target_location[3] = {320.0, 188.0, 0.0};
double thm = 20*3.141592/180; // IN RADIANS - COMPENSATION FOR CAMERA AXIS
double csth_thm[2] = {cos(thm), sin(thm)}; // COMPENSATION FOR CAMERA AXIS

Mat src; Mat src_gray;  // MATRIX TO STORE IMAGES CAPTURED
RNG rng(12345);
int repeat_time = 20;
int width = 640;  // IMAGE WIDTH
int height = 376; // IMAGE HEIGHT
static const std::string OPENCV_WINDOW = "Image window"; // CREATING A WINDOW TO DISPLAY THE IMAGE

// FOR DYNAMIXEL MOTORS TO BE USED IN CONJUNCTION WITH DYNAMIXEL DynamixelSDK

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME); // PORT HANDLER TO PERFORM OPERATIONS ON MOTORS THRU USB PORT
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION); // TO READ AND WRITE DATA PACKETS TO AND FROM MOTORS
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_VELOCITY, LEN_PRO_GOAL_VELOCITY); // USED TO WRITE THE VELOCITY OF TO THE MOTORS
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION); // USED TO READ THE POSITION OF THE MOTORS
 int dxl_comm_result = COMM_TX_FAIL;               // Communication result when the system communicates with the motors, set to false by default. If false, no other operation is done and program is terminated
 bool dxl_addparam_result = false;                 // addParam result
 bool dxl_getdata_result = false;                  // GetParam result
 uint8_t dxl_error = 0;                            // Dynamixel error
 int32_t dxl1_present_position = 0, dxl2_present_position = 0, dxl3_present_position = 0; // To obtain the positions of the three motors


// TO SAVE THE DATA
 ofstream OutFile, OutFile_Data;
 int64_t OutFile_Data_size_byte = datasave_variable_count*sizeof(double); // 13 numbers will be save per image (you may change)
 enum{s_seq, s_elapt, s_sampt, s_cx, s_cy, s_th, s_dxm, s_dym, s_dthm, s_omg1, s_omg2, s_omg3, s_maxa, s_sec, s_dxl1, s_dxl2, s_dxl3}; // ENUMERATING THE VARIABLES THAT NEED TO PUSHED INTO A VECTOR
 struct AOTResult
 {
   uint8_t * ptr;
   double data[datasave_variable_count];
 };



// Function to initialize the binary files in which the data will be recorded
void save_init(){

  OutFile.open("Fly_Image_May17_2018_150PM.bin", ios::out | ios::binary);
  OutFile_Data.open("Fly_Data_May17_2018_150PM.bin", ios::out | ios::binary);

}

// Function to flush and close the binary files in which the data will be recorded

void save_deinit(){
OutFile.close();
OutFile_Data.close();
}


// INITIALIZING THE MOTORS - In this funtion, the USB port is opened, the BaudRate is set and a connection to the motors is established. The torque of the motors are also enabled and the operating mode is set


void motor_init(){

 if (portHandler->openPort())
{
 printf("Succeeded to open the port!\n");
}
else
{
 ROS_ERROR("Failed to open the port!\n");
 ROS_ERROR("Press any key to terminate...\n");
 getch();
}

// Set port baudrate
if (portHandler->setBaudRate(BAUDRATE))
{
 printf("Succeeded to change the baudrate!\n");
}
else
{
 ROS_ERROR("Failed to change the baudrate!\n");
 ROS_ERROR("Press any key to terminate...\n");
 getch();
}

// The Torque of the motors need to be disabled first to set the operating mode

dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
if (dxl_comm_result != COMM_SUCCESS)
{
 ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

}
else if (dxl_error != 0)
{
 ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
}
else
{
 printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
}

// Enable Dynamixel#2 Torque
dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
if (dxl_comm_result != COMM_SUCCESS)
{
 ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
}
else if (dxl_error != 0)
{
 ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
}
else
{
 printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
}

dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
if (dxl_comm_result != COMM_SUCCESS)
{
 ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
}
else if (dxl_error != 0)
{
 ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
}
else
{
 printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);
}


// Next three lines are the commands to set the motor operating mode to velocity mode
// Address for Operating Mode = 11 , Velocity Mode value = 1

dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_FOR_OPERATING_MODE, VELOCITY_MODE, &dxl_error);
dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_FOR_OPERATING_MODE, VELOCITY_MODE, &dxl_error);
dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_FOR_OPERATING_MODE, VELOCITY_MODE, &dxl_error);


// Enabling torque post operating mode assignment

dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, 1, &dxl_error);
if (dxl_comm_result != COMM_SUCCESS)
{
  ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
}
else if (dxl_error != 0)
{
  ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
}
else
{
  printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
}

// Enable Dynamixel#2 Torque
dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, 1, &dxl_error);
if (dxl_comm_result != COMM_SUCCESS)
{
  ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
}
else if (dxl_error != 0)
{
  ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
}
else
{
  printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
}

dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, 1, &dxl_error);
if (dxl_comm_result != COMM_SUCCESS)
{
  ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
}
else if (dxl_error != 0)
{
  ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
}
else
{
  printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);
}

dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
 if (dxl_addparam_result != true)
 {
   ROS_ERROR("[ID:%d] groupSyncRead addparam failed", DXL1_ID);
 }

 // Add parameter storage for Dynamixel#2 present position value
 dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
 if (dxl_addparam_result != true)
 {
   ROS_ERROR("[ID:%d] groupSyncRead addparam failed", DXL2_ID);
 }


 dxl_addparam_result = groupSyncRead.addParam(DXL3_ID);
 if (dxl_addparam_result != true)
 {
   ROS_ERROR("[ID:%d] groupSyncRead addparam failed", DXL3_ID);
 }
}


// WRITING THE VELOCITIES TO THE AOT MOTORS

void Motor_Assign(double v1, double v2, double v3)
{
  // Length of the goal velocity variable is 4 bytes.
  // DXL_HIWORD GETS THE HIGHER TWO BYTES OF THE 4 BYTE DATA AND SO ON

  uint8_t param_vel_position[4];
  param_vel_position[0] = DXL_LOBYTE(DXL_LOWORD((int)v1));
  param_vel_position[1] = DXL_HIBYTE(DXL_LOWORD((int)v1));
  param_vel_position[2] = DXL_LOBYTE(DXL_HIWORD((int)v1));
  param_vel_position[3] = DXL_HIBYTE(DXL_HIWORD((int)v1));

dxl_comm_result = groupSyncWrite.addParam(DXL1_ID, (uint8_t*)param_vel_position);
 if (dxl_comm_result != true)
 ROS_ERROR("FAILED!!");


 param_vel_position[0] = DXL_LOBYTE(DXL_LOWORD((int)v2));
 param_vel_position[1] = DXL_HIBYTE(DXL_LOWORD((int)v2));
 param_vel_position[2] = DXL_LOBYTE(DXL_HIWORD((int)v2));
 param_vel_position[3] = DXL_HIBYTE(DXL_HIWORD((int)v2));

 dxl_comm_result = groupSyncWrite.addParam(DXL2_ID, (uint8_t*)param_vel_position);
 if (dxl_comm_result != true)
   ROS_ERROR("FAILED!!");

   param_vel_position[0] = DXL_LOBYTE(DXL_LOWORD((int)v3));
   param_vel_position[1] = DXL_HIBYTE(DXL_LOWORD((int)v3));
   param_vel_position[2] = DXL_LOBYTE(DXL_HIWORD((int)v3));
   param_vel_position[3] = DXL_HIBYTE(DXL_HIWORD((int)v3));

 dxl_comm_result = groupSyncWrite.addParam(DXL3_ID, (uint8_t*)param_vel_position);
 if (dxl_comm_result != true)
    ROS_ERROR("FAILED!!");

  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    ROS_ERROR("HEREABSC");

  }

groupSyncWrite.clearParam();

}


// DEINITIALIZING THE MOTORS

void motor_deinit()
{
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
   ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
   ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
   printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
  }

  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
   ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
   ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
   printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
   ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
   ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
   printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);
  }


}



// MULTITHREADING
bool g_bProcessRunning; // may set in the class, so you can set this true when constructor, and set this false in destructor
AOTResult lastResult; // AOTResult Class (structure) you define (imgptr, other data included)
std::vector<AOTResult> result_save;
std::thread m_pThreadProc;
std::thread m_pThreadSave;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher Fly_Pos;
  ros::Subscriber seq_sub;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    ros::NodeHandle nh;
    Fly_Pos = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); // PUBLISHING INPUT FOR THE MOBILE ROBOT

    image_sub_ = it_.subscribe("/image_raw", 1,
      &ImageConverter::imageCb, this);             // SUBSCRIBING TO THE RAW IMAGES FROM CAMERA


    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // the last one (or in activate functions)
  }

    ~ImageConverter()
      {
     }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    int sequence_now=0;
    double seq_init;
    geometry_msgs::Twist MSG;     //INSTANCE TO PUBLISH MESSAGE
    cv_bridge::CvImagePtr cv_ptr;

    std_msgs::Header h = msg->header; //TO GET FRAME NUMBER
    auto begin = Clock::now();

    dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Check if groupsyncread data of Dynamixel#1 is available
    dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
      ROS_ERROR( "[ID:%d] groupSyncRead getdata failed", DXL1_ID);
    }

    // Check if groupsyncread data of Dynamixel#2 is available
    dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
      ROS_ERROR( "[ID:%d] groupSyncRead getdata failed", DXL2_ID);
    }

    dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
      ROS_ERROR( "[ID:%d] groupSyncRead getdata failed", DXL3_ID);
    }

    // Get Dynamixel#1 present position value
    dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

    // Get Dynamixel#2 present position value
    dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);


    dxl3_present_position = groupSyncRead.getData(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    // time start

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      seq_init=h.seq;
      sequence_now=h.seq%(repeat_time*3);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }



    auto samplingtime_secs =  std::chrono::duration_cast<std::chrono::microseconds>(begin - prev_clock).count() ; //LOOP TIME
    prev_clock = begin;

    Mat im = cv_ptr->image;
    Mat im_bw;

    // IMAGE PROCESSING

    threshold(im, im_bw, 50.0, 255.0, 0);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int savedContour = -1; // contour index (largest contour)
    double maxArea = 0.0; // size of the largest contour
    findContours(im_bw, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    for (int i = 0; i< contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            savedContour = i;
        }

    }

    // COMPTUTING THE CENTROID USING THE MOMENTS

    double _cx = target_location[0]; double _cy = target_location[1]; double _theta = target_location[2];
    if (maxArea > 100.0) {
      Moments mu = moments(contours[savedContour], false );
      // compute centroid & orientation*0.01
      _cx = mu.m10/mu.m00;
      _cy = mu.m01/mu.m00;
      double mu20p = mu.m20/mu.m00-_cx*_cx;
      double mu02p= mu.m02/mu.m00-_cy*_cy;
      double mu11p= mu.m11/mu.m00-_cx*_cy;
      _theta = atan2(2*mu11p,(mu20p-mu02p))/2; // in radian
    }

    // COMPUTING THE DEL. X, DEL. Y AND DEL. THETA NEEDED TO CALCULATE THE
    // MOTOR VELOCITIES

    double dx = (_cx-target_location[0]);
    double dy = (_cy-target_location[1]);
    double dth = -(_theta - target_location[2]);

    // ACCOUNTNIG FOR THE DIFFERNECE IN ALLIGNMENT OF CAMERA AND MOTOR AXES

    double dxm = csth_thm[0]*dx - csth_thm[1]*dy;
    double dym = csth_thm[1]*dx + csth_thm[0]*dy;
    double dthm = dth ;//+ thm;

    // SETTING TOLERANCE

    double limit_r = 5;
    double limit_th = 1*3.141592/180;

    // COMPUTING AOT MOTOR VELOCITIES

    if ((sqrt(dx*dx + dy*dy) > limit_r) || (fabs(dth) > limit_th)) {
      K = sqrt(dx*dx + dy*dy) * 5 + 50;
      if(K>300)
      K=300;
      omega_1 = (J1M[0]*dxm + J1M[1]*dym + J1M[2]*dthm*SF)*K;
    	omega_2 = (J1M[3]*dxm + J1M[4]*dym + J1M[5]*dthm*SF)*K;
    	omega_3 = (J1M[6]*dxm + J1M[7]*dym + J1M[8]*dthm*SF)*K;
    }
    else {
      omega_1 = 0; omega_2 =0; omega_3 = 0;
    }

    // CALCULATING THE LINEAR AND ANGULAR VELOCITIES FOR THE ROBOT

    if ((sqrt(dx*dx + dy*dy) > limit_r))  {
      lin_vel = (sqrt(dx*dx + dy*dy)*38.40983291722681*0.01)/(samplingtime_secs*0.001);
    }

    else lin_vel =0;

    if((fabs(dth) > limit_th))
      ang_vel = (dthm/(samplingtime_secs*0.001))*57;

    else ang_vel = 0;

  // PUBLISHING THE LINEAR AND ANGULAR VELOCITIES FOR THE MOBILE ROBOT

   MSG.linear.x= lin_vel;
   MSG.angular.z= ang_vel;

  // DRAWING THE CONTOUR CENTROID ON THE IMAGE
/*
    Mat drawing = Mat::zeros( im.size(), CV_8UC3 );
    cv::cvtColor(im, drawing, cv::COLOR_GRAY2BGR);
    //im.copyTo(drawing);
    if (maxArea > 0.0) {
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      drawContours(drawing, contours, savedContour, color, 2, 8, hierarchy, 0, Point() );
    }
    circle( drawing, Point(_cx,_cy), 3, Scalar(255,0,255), 1, 8, 0);
    circle( drawing, Point(target_location[0],target_location[1]), limit_r, Scalar(0,0,255), 1, 8, 0);

    imshow("a",im);
    waitKey(1);
*/

  // WRITING VELOCITIES TO MOTORS

    Motor_Assign(omega_1,omega_2,omega_3);


  // READING CURRENT POSITION OF THE MOTOR AFTER COMPENSATION
/*



*/



    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    Fly_Pos.publish(MSG);
    //Writing Image AND data to Binary File
    //OutFile.write((char *)im.data, width*height*sizeof(uchar));
    //double secs = ros::Time::now().toSec();
    //double output_data[] = {seq_init, elapsed_secs, (double)samplingtime_secs, _cx, _cy, _theta, dxm, dym, dthm, omega_1, omega_2, omega_3, maxArea, (double)dxl1_present_position, (double)dxl2_present_position, (double)dxl3_present_position, secs}; // 13 variables
    //OutFile_Data.write((char *)output_data, OutFile_Data_size_byte);
    auto end = Clock::now();

    double elapsed_secs = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() ;

    double secs = ros::Time::now().toSec();

    AOTResult current_result;

    current_result.ptr = (uint8_t *)im.data;
    current_result.data[s_seq] = seq_init;
    current_result.data[s_elapt] = elapsed_secs;
    current_result.data[s_sampt] = (double)samplingtime_secs;
    current_result.data[s_cx] = _cx;
    current_result.data[s_cy] = _cy;
    current_result.data[s_th] = _theta;
    current_result.data[s_dxm] = dxm;
    current_result.data[s_dym] = dym;
    current_result.data[s_dthm] = dthm;
    current_result.data[s_omg1] = omega_1;
    current_result.data[s_omg2] = omega_2;
    current_result.data[s_omg3] = omega_3;
    current_result.data[s_maxa] = maxArea;
    current_result.data[s_sec] = secs;
    current_result.data[s_dxl1] = dxl1_present_position;
    current_result.data[s_dxl2] = dxl2_present_position;
    current_result.data[s_dxl3] = dxl3_present_position;

    result_save.push_back(current_result);

    lastResult.ptr = (uint8_t *)im.data;
    ROS_INFO ("%0.3f,%0.3f, %lu, %d", elapsed_secs/1000,(double)samplingtime_secs/1000, std::this_thread::get_id(), result_save.size());

  }
};

void thread_proc()
{
  motor_init(); // INITIALIZING MOTORS
  ImageConverter ic;
  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();
  //ros::spin();
  motor_deinit(); // DEINITIALIZING MOTORS
  //save_deinit();  // CLOSING FILES
  ROS_ERROR( "proc Thread[ID:%lu]", std::this_thread::get_id());
}

void thread_save()
{
  ROS_ERROR( "save thread init [ID:%lu]", std::this_thread::get_id());
  save_init();  // OPENING FILES
 while(g_bProcessRunning) {
   if (result_save.size() > 1) {  auto begin_save = Clock::now();

     // saving data in result_save[0]
     // delete the saved data
     AOTResult obj = result_save[0];
     //Writing Image AND data to Binary File
     OutFile.write((char *)obj.ptr, width*height);
     OutFile_Data.write((char *)obj.data, OutFile_Data_size_byte);
     result_save.erase(result_save.begin());
     auto end_save = Clock::now();
     double save_time = std::chrono::duration_cast<std::chrono::microseconds>(end_save - begin_save).count() ;
    // ROS_INFO("Savetime = %0.3f", save_time);

   }
   sleep(0.005);
 }
 save_deinit();

 ROS_ERROR( "save thread is shutting down[ID:%lu]", std::this_thread::get_id());
}


int main(int argc, char** argv)
{
  ROS_ERROR( "Main Thread[ID:%lu]", std::this_thread::get_id());
  lastResult.ptr = NULL;
  // INITIALIZING ROS NODE
  ros::init(argc, argv, "image_converter");

  g_bProcessRunning = true;
  m_pThreadProc = std::thread(thread_proc); // call function in other thread
  m_pThreadSave = std::thread(thread_save); // call function in other thread

  while(g_bProcessRunning) {
    if (lastResult.ptr) {
        Mat img = cv::Mat(height, width, CV_8UC1, lastResult.ptr);
        Mat drawing = Mat::zeros( img.size(), CV_8UC3 );
        cv::cvtColor(img, drawing, cv::COLOR_GRAY2BGR);
        std::ostringstream str;
        str << "ANY TEXT %d" << ::getpid();
        cv::putText(img, str.str(), cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,0), 1, CV_AA);
        //if (maxArea > 0.0) {
        //  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //  drawContours(drawing, contours, savedContour, color, 2, 8, hierarchy, 0, Point() );
        //}
        //circle( drawing, Point(_cx,_cy), 3, Scalar(255,0,255), 1, 8, 0);
        //circle( drawing, Point(target_location[0],target_location[1]), limit_r, Scalar(0,0,255), 1, 8, 0);
        imshow("a",drawing);
        char aaa = waitKey(50);
        if (aaa == 'q') {
          destroyWindow("a");
          g_bProcessRunning = false;
        }
    }
    else {
      sleep(0.01);
    }
  }

  ROS_ERROR( "Shutting Down");
  // Wait until all multithreading functions are terminated
  m_pThreadSave.join();
  //m_pThreadProc.join();
  return 0;
}
