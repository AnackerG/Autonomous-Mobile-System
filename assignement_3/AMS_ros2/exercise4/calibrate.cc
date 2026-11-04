#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <ostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#if CV_VERSION_MAJOR == 4
# include <opencv2/highgui/highgui_c.h> // https://github.com/MIT-SPARK/Kimera-VIO/issues/1
# undef CV_CAP_PROP_FRAME_WIDTH
# undef CV_CAP_PROP_FRAME_HEIGHT
# define CV_CAP_PROP_FRAME_WIDTH CAP_PROP_FRAME_WIDTH
# define CV_CAP_PROP_FRAME_HEIGHT CAP_PROP_FRAME_HEIGHT
#else
# warning "The student assumes OpenCV-Version 4.x.x"
#endif

#include <fstream>
#include <iostream>

rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub;

using namespace std;
using namespace cv;

enum { INTERACTIVE_MODE, PRESPECIFIED_MODE };

#define QUIET

#define KEY_ESCAPE 1048603
#define KEY_SPACE 1048608
#define KEY_CLOSE_WINDOW -1

#define COUNT_SQUARES_X 4
#define COUNT_SQUARES_Y 6


/**
 * Method for detecting pattern in current frame. 
 * Returns world and image coordinates of detected corners via out parameters.
 */
bool detectPattern(Mat frame,
			    vector< vector<Point3f> >& object_points,
			    vector< vector<Point2f> >& image_points,
          int maximum_sets = 50)
{
  // number of squares in the pattern, a.k.a, interior number of corners
  Size pattern_size(COUNT_SQUARES_X, COUNT_SQUARES_Y);
  // storage for the detected corners in findChessboardConrners
  vector<Point2f> corners;

  bool pattern_found = findChessboardCorners(
            frame, pattern_size, corners,
            CALIB_CB_ADAPTIVE_THRESH +
            CALIB_CB_NORMALIZE_IMAGE +
            CALIB_CB_FAST_CHECK);

  if (!pattern_found)
    return false;

  // if corners are detected, they are further refined by
  // calculating subpixel corners from the grayscale image
  // this iterative process terminates after the given number
  // of iterations and error epsilon
  cornerSubPix(frame, corners, Size(11, 11), Size(-1, -1),
               TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100,
               0.15));
  // draw the detected corners as sanity check
  drawChessboardCorners(frame, pattern_size, Mat(corners),
                        pattern_found);

  // show detected corners in a different window
  imshow("Detected pattern", frame);

  // build a grid of 3D points (z component is 0 because the pattern is
  // in one plane) to fit the square pattern area
  // (COUNT_SQUARES_X * COUNT_SQUARES_Y)


  /* 
    ,,Enhance the program, such that multiple camera images are used´´
    => remove the following two lines
  */

  vector<Point3f> pattern_points;
  for (int j = 0; j < COUNT_SQUARES_X * COUNT_SQUARES_Y; ++j)
    pattern_points.push_back(Point3f(j / COUNT_SQUARES_X,
				     j % COUNT_SQUARES_X,
				     0.0f));

  // I do not want to torture my (your) pc, so the number of sets is limited
  if (object_points.size() > maximum_sets ||
      image_points.size() > maximum_sets)
  {
    cout << "Calibration buffer is full. Skipping set...\n";
    return true;
  }

  // populate image points with corners and object points with grid points
  object_points.push_back(pattern_points);
  image_points.push_back(corners);

  return true;
}

/**
 * Main method for interactive behavior. 
 * Requires user to present calibration pattern in front of camera. 
 * By pressing any key an image is grabbed from the camera stream,
 * pressing ESC finishes calibration.
 */
int runInteractive(int iCap = 1)
{
  cout << "Camera calibration using interactive behavior. "
	  << " Press any key to grab frame, ESC to perform calibration.\n";
  // show camera image in a separate window
  namedWindow("Camera Image", CV_WINDOW_KEEPRATIO);

  // current camera frame and first captured frame 
  Mat frame, undistorted_frame;
  // storage for object points (world coords)
  // and image points (image coords) for use in calibrateCamera 
  vector< vector< Point3f > > object_points;
  vector< vector< Point2f > > image_points;
  // calibration parameters
  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs, tvecs;

  // open the default camera
  VideoCapture capture(iCap, cv::CAP_V4L);

  // check if opening camera stream succeeded
  if (!capture.isOpened()) 
  {
    cerr << "Camera could not be opened. Try another index as argument? Exiting.\n";
    return -1;
  }

  // set frame width and height by hand, defaults to 160x120
  capture.set (CV_CAP_PROP_FRAME_WIDTH, 640);
  capture.set (CV_CAP_PROP_FRAME_HEIGHT, 480);

  // flag for determining whether pattern was detected in at
  // least one of the camera grabs
  bool calibration_ready = false;
  int count_frames = 0;
  
  for (; ; count_frames++) {
#ifndef QUIET
    cout << "Frame: " << count_frames << endl;
#endif

    capture >> frame; // get a new frame from camera
    
    Mat gray_frame;
    // convert current frame to grayscale
    cvtColor(frame, gray_frame, CV_BGR2GRAY); 
    
    int key_pressed = waitKey(25); // get user key press
    if (key_pressed == 27) // ESC
	    break;

    if (detectPattern(gray_frame, object_points, image_points)) {

	    calibrateCamera(object_points, image_points, frame.size(), intrinsic, distCoeffs, rvecs, tvecs);

      cout << "Intrinsic Matrix:\n" << intrinsic << endl;
      cout << "Distortion Coefficients:\n" << distCoeffs << endl;
      cout << "Extrensic values: \n" << endl << endl;
      for (int i = 0; i < rvecs.size(); i++) {
          cout << "  Rotation:    " << rvecs[i] << endl;
          cout << "  Translation: " << tvecs[i] << endl;
      }

      cv::Mat rotation_vector;
      cv::Mat translation_vector;

      bool success = cv::solvePnP(
      object_points[0],      
      image_points[0],      
      intrinsic,     
      distCoeffs,       
      rotation_vector,    
      translation_vector, 
      false,              
      cv::SOLVEPNP_ITERATIVE 
  );


      if (success) {

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = translation_vector.at<double>(0);
        pose_msg.position.y = translation_vector.at<double>(1);
        pose_msg.position.z = translation_vector.at<double>(2);

        cv::Mat rotation_matrix;
        cv::Rodrigues(rotation_vector, rotation_matrix);

        double tr = rotation_matrix.at<double>(0,0) + rotation_matrix.at<double>(1,1) + rotation_matrix.at<double>(2,2);
        double qw, qx, qy, qz;
        if (tr > 0) { 
            double s = sqrt(tr + 1.0) * 2;
            qw = 0.25 * s;
            qx = (rotation_matrix.at<double>(2,1) - rotation_matrix.at<double>(1,2)) / s;
            qy = (rotation_matrix.at<double>(0,2) - rotation_matrix.at<double>(2,0)) / s; 
            qz = (rotation_matrix.at<double>(1,0) - rotation_matrix.at<double>(0,1)) / s; 
        }
        pose_msg.orientation.x = qx;
        pose_msg.orientation.y = qy;
        pose_msg.orientation.z = qz;
        pose_msg.orientation.w = qw;

        // Publication on ROS2 
        pose_pub->publish(pose_msg);

        std::string text1 = "Pos: X=" + std::to_string(translation_vector.at<double>(0)).substr(0, 4) + 
                    " Y=" + std::to_string(translation_vector.at<double>(1)).substr(0, 4) + 
                    " Z=" + std::to_string(translation_vector.at<double>(2)).substr(0, 4);
                        
        std::string text2 = "Rotation: Ox=" + std::to_string(rotation_vector.at<double>(0)).substr(0, 4) + 
                      " 0y=" + std::to_string(rotation_vector.at<double>(1)).substr(0, 4) + 
                      " Oz=" + std::to_string(rotation_vector.at<double>(2)).substr(0, 4);

        cv::Point text_origin1(10, 30); 
        cv::Point text_origin2(10, 60); 
        
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.7;
        cv::Scalar color(0, 255, 0);   
        int thickness = 2;

        cv::putText(frame, text1, text_origin1, font_face, font_scale, color, thickness);
        cv::putText(frame, text2, text_origin2, font_face, font_scale, color, thickness);
      }
      
     
  
    } 
    imshow("Camera Image", frame);
    if (waitKey(25) == 27) break ;
  }
  
  // if at least one video capture contains the pattern, perform calibration
//   if (calibration_ready)
//   {
//     // FIX ME
//     calibrateCamera(object_points, image_points, frame.size(), intrinsic, distCoeffs, rvecs, tvecs);


//     cout << "Intrinsic Matrix:\n" << intrinsic << endl;
//     cout << "Distortion Coefficients:\n" << distCoeffs << endl;
//     cout << "Extrensic values: \n" << endl << endl;
//     for (int i = 0; i < rvecs.size(); i++) {
//         cout << "  Rotation:    " << rvecs[i] << endl;
//         cout << "  Translation: " << tvecs[i] << endl;
//     }

//     cv::Mat rotation_vector;
//     cv::Mat translation_vector;

//     bool success = cv::solvePnP(
//     object_points[0],      // Pass the first set of points (vector<Point3f>)
//     image_points[0],       // Pass the first set of points (vector<Point2f>)
//     intrinsic,     
//     distCoeffs,       
//     rotation_vector,    
//     translation_vector, 
//     false,              
//     cv::SOLVEPNP_ITERATIVE 
// );
//     if (success) {
//       std::string text1 = "Pos: X=" + std::to_string(translation_vector.at<double>(0)).substr(0, 4) + 
//                    " Y=" + std::to_string(translation_vector.at<double>(1)).substr(0, 4) + 
//                    " Z=" + std::to_string(translation_vector.at<double>(2)).substr(0, 4);
                      
//       std::string text2 = "Rotation: Ox=" + std::to_string(rotation_vector.at<double>(0)).substr(0, 4) + 
//                     " 0y=" + std::to_string(rotation_vector.at<double>(1)).substr(0, 4) + 
//                     " Oz=" + std::to_string(rotation_vector.at<double>(2)).substr(0, 4);

//       cv::Point text_origin1(10, 30); 
//       cv::Point text_origin2(10, 60); 

//       int font_face = cv::FONT_HERSHEY_SIMPLEX;
//       double font_scale = 0.7;
//       cv::Scalar color(0, 255, 0);   
//       int thickness = 2;

//       cv::putText(frame, text1, text_origin1, font_face, font_scale, color, thickness);
//       cv::putText(frame, text2, text_origin2, font_face, font_scale, color, thickness);
//     }
    
//     imshow("Camera Image", frame);

//   }
//   else 
//   {
//     cerr << "No pattern found in any video capture. Exiting." << endl;
//     return -1;
//   }

  for (; ; count_frames++) {
#ifndef QUIET
    cout << "Frame: " << count_frames << " (calibrated) " << endl;
#endif
    capture >> frame; // get a new frame from camera
    undistort(frame, undistorted_frame, intrinsic, distCoeffs);
    imshow("Camera Image", undistorted_frame); // update camera image

    int key_pressed = waitKey(25); // get user key press
    if (key_pressed == 27) // ESC
	    break;
  }
  
  // release camera
  capture.release();

  return 0;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("pose_estimator_node");
    pose_pub = node->create_publisher<geometry_msgs::msg::Pose>("camera_pose", 10);

    // 2. Determine camera index
    int camera_index = (argc == 1) ? 1 : atoi(argv[1]);

    int result = runInteractive(camera_index);

    rclcpp::shutdown();
    
    return result;
}
