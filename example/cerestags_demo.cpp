/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both visualized in the live image and shown
 * in the text console. Optionally allows selecting of a specific
 * camera in case multiple ones are present and specifying image
 * resolution as long as supported by the camera. Also includes the
 * option to send tag detections via a serial port, for example when
 * running on a Raspberry Pi that is connected to an Arduino.
 */

using namespace std;

#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <algorithm>
const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [IMG1 [IMG2...]]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -a              Arduino (send tag ids over serial port)\n"
  "  -d              Disable graphics\n"
  "  -t              Timing of tag extraction\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -D <id>         Video device ID (if multiple cameras present)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 480, availability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";


#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/RunningStats.h"


// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

// For Arduino: locally defined serial port access class
#include "Serial.h"


const char* windowName = "apriltags_demo";


// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}


#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}


class Demo {

  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;
  // AprilTags::TagCodes m_ann_tagCodes;

  bool m_draw; // draw image and April tag detections?
  bool m_timing; // print timing information for each tag extraction call

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)

  list<string> m_imgNames;

  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;

  RunningStats m_runningStatsX;

public:

  // default constructor
  Demo() :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),
    // m_ann_tagCodes(AprilTags::tagCodes64),

    m_draw(true),
    m_timing(false),

    m_width(640),
    m_height(480),
    m_tagSize(6),
    m_fx(555.56),
    m_fy(517.94),
    m_px(292.79),
    m_py(262.73),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    m_deviceId(0)
  {}

  // changing the tag family
  void setTagCodes(string s) {
    if (s=="16h5") {
      m_tagCodes = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
      m_tagCodes = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
      m_tagCodes = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
      m_tagCodes = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
      m_tagCodes = AprilTags::tagCodes36h11;
    } else {
      cout << "Invalid tag family specified" << endl;
      exit(1);
    }
  }

  // parse command line options to change default behavior
  void parseOptions(int argc, char* argv[]) {
    int c;
    while ((c = getopt(argc, argv, ":h?adtC:F:H:S:W:E:G:B:D:")) != -1) {
      // Each option character has to be in the string in getopt();
      // the first colon changes the error character from '?' to ':';
      // a colon after an option means that there is an extra
      // parameter to this option; 'W' is a reserved character
      switch (c) {
      case 'h':
      case '?':
        cout << usage;
        exit(0);
        break;
      case 'd':
        m_draw = false;
        break;
      case 't':
        m_timing = true;
        break;
      case 'C':
        setTagCodes(optarg);
        break;
      case 'F':
        m_fx = atof(optarg);
        m_fy = m_fx;
        break;
      case 'H':
        m_height = atoi(optarg);
        m_py = m_height/2;
         break;
      case 'S':
        m_tagSize = atof(optarg);
        break;
      case 'W':
        m_width = atoi(optarg);
        m_px = m_width/2;
        break;
      case 'E':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Exposure option (-E) not available" << endl;
        exit(1);
#endif
        m_exposure = atoi(optarg);
        break;
      case 'G':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Gain option (-G) not available" << endl;
        exit(1);
#endif
        m_gain = atoi(optarg);
        break;
      case 'B':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Brightness option (-B) not available" << endl;
        exit(1);
#endif
        m_brightness = atoi(optarg);
        break;
      case 'D':
        m_deviceId = atoi(optarg);
        break;
      case ':': // unknown option, from getopt
        cout << usage;
        exit(1);
        break;
      }
    }

    if (argc > optind) {
      for (int i=0; i<argc-optind; i++) {
        m_imgNames.push_back(argv[optind+i]);
      }
    }
  }

  void setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
    // m_tagDetector = new AprilTags::TagDetector(m_tagCodes,m_ann_tagCodes);

    // prepare window for drawing the camera images
    if (m_draw) {
      cv::namedWindow(windowName, 1);
    }

  }

  void setupVideo() {

#ifdef EXPOSURE_CONTROL
    // manually setting camera exposure settings; OpenCV/v4l1 doesn't
    // support exposure control; so here we manually use v4l2 before
    // opening the device via OpenCV; confirmed to work with Logitech
    // C270; try exposure=20, gain=100, brightness=150

    string video_str = "/dev/video0";
    video_str[10] = '0' + m_deviceId;
    int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

    if (m_exposure >= 0) {
      // not sure why, but v4l2_set_control() does not work for
      // V4L2_CID_EXPOSURE_AUTO...
      struct v4l2_control c;
      c.id = V4L2_CID_EXPOSURE_AUTO;
      c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
      if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
        cout << "Failed to set... " << strerror(errno) << endl;
      }
      cout << "exposure: " << m_exposure << endl;
      v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
    }
    if (m_gain >= 0) {
      cout << "gain: " << m_gain << endl;
      v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
    }
    if (m_brightness >= 0) {
      cout << "brightness: " << m_brightness << endl;
      v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
    }
    v4l2_close(device);
#endif 

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
      exit(1);
    }
    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

  }

  void print_detection(AprilTags::TagDetection& detection) const {

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

    // Eigen::Matrix3d F;
    // F <<
    //   1, 0,  0,
    //   0,  -1,  0,
    //   0,  0,  1;
    // Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(rotation, yaw, pitch, roll);

    printf("x = %.3f y = %.3f z = %.3f y = %.3f p = %.3f r = %.3f\n",
      translation(0),translation(1),translation(2),yaw,pitch,roll);

  }

  void processImage(cv::Mat& image, cv::Mat& image_gray) {
    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    //      m_cap.grab();
    //      m_cap.retrieve(image);

    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    double t0;
    if (m_timing) {
      t0 = tic();
    }
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
    if (m_timing) {
      double dt = tic()-t0;
      //cout << "Extracting tags took " << dt << " seconds." << endl;
    }

    if(detections.size()){
      //stack all ps and Ps and solvePnP
      std::vector<cv::Point3f> objPts;
      std::vector<cv::Point2f> imgPts;
      double s = 0.105/2.;
      double centre_dist = 0.3;
      int grid_c = 5;
      double x_c,y_c;  // bottom left AT, x y
      for (int i=0; i<detections.size(); i++) {
        // NOTE: uncomment this in case you want to reject too close ATs
        // if(detections[i].id % 2)
        //   continue;
        int num = detections[i].id-1;    //starts with 1 at top left
        int row = num / grid_c;
        int col = num % grid_c;
        x_c = double(col) * centre_dist; 
        y_c = - double(row) * centre_dist; 

        objPts.push_back(cv::Point3f(x_c - s, y_c - s, 0));
        objPts.push_back(cv::Point3f(x_c + s, y_c - s, 0));
        objPts.push_back(cv::Point3f(x_c + s, y_c + s, 0));
        objPts.push_back(cv::Point3f(x_c - s, y_c + s, 0));

        std::pair<float, float> p1 = detections[i].p[0];
        std::pair<float, float> p2 = detections[i].p[1];
        std::pair<float, float> p3 = detections[i].p[2];
        std::pair<float, float> p4 = detections[i].p[3];
        imgPts.push_back(cv::Point2f(p1.first, p1.second));
        imgPts.push_back(cv::Point2f(p2.first, p2.second));
        imgPts.push_back(cv::Point2f(p3.first, p3.second));
        imgPts.push_back(cv::Point2f(p4.first, p4.second)); 

      }

      if(!objPts.size())
        return;

      cv::Mat rvec, tvec;
      cv::Matx33f cameraMatrix(
                               m_fx, 0, m_px,
                               0, m_fy, m_py,
                               0,  0,  1);
      cv::Vec4f distParam(0,0,0,0); // all 0?
      cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
      // NOTE: can chose between RANSAC and normal solvepPnP. Atm normal works better
      // cv::solvePnPRansac(objPts, imgPts, cameraMatrix, distParam, rvec, tvec,
      //   false,100,8.0,(0.95 * float(detections.size()) * 4.0));

      cv::Mat r;
      cv::Rodrigues(rvec, r);

      r = r.t();  // rotation of inverse
      cv::Mat new_tvec(-r * tvec); // translation of inverse

      Eigen::Matrix3d wRo;
      wRo <<  r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), 
              r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2),
              r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2);
      double yaw, pitch, roll;
      wRo_to_euler(wRo, yaw, pitch, roll);

      //add X observation to running stats
      m_runningStatsX.Push(new_tvec.at<double>(0));

      printf("%4d %3.4f %3.4f %3.4f %3.4f %3.4f %3.4f %3.4f %3.4f\n", detections.size(),
        new_tvec.at<double>(0), new_tvec.at<double>(1), new_tvec.at<double>(2),
          yaw,pitch,roll,m_runningStatsX.Mean(),m_runningStatsX.StandardDeviation());
    }

    // show the current image including any detections
    if (m_draw) {
      for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(image);
      }
      imshow(windowName, image); // OpenCV call
    }
  }

  // Load and process a single image
  void loadImages() {
    cv::Mat image;
    cv::Mat image_gray;

    for (list<string>::iterator it=m_imgNames.begin(); it!=m_imgNames.end(); it++) {
      image = cv::imread(*it); // load image with opencv
      processImage(image, image_gray);
      while (cv::waitKey(100) == -1) {}
    }
  }

  // Video or image processing?
  bool isVideo() {
    return m_imgNames.empty();
  }

  // The processing loop where images are retrieved, tags detected,
  // and information about detections generated
  void loop() {

    cv::Mat image;
    cv::Mat image_gray;

    int frame = 0;
    double last_t = tic();
    while (true) {

      // capture frame
      m_cap >> image;

      processImage(image, image_gray);

      // print out the frame rate at which image frames are being processed
      frame++;
      if (frame % 10 == 0) {
        double t = tic();
        // cout << "  " << 10./(t-last_t) << " fps" << endl;
        last_t = t;
      }

      // exit if any key is pressed
      if (cv::waitKey(1) >= 0) break;
    }
  }

}; // Demo

int main(int argc, char* argv[]) {
  Demo demo;
  // process command line options
  demo.parseOptions(argc, argv);
  demo.setup();
  if (demo.isVideo()) {
    // setup image source, window for drawing, serial port...
    demo.setupVideo();
    // the actual processing loop where tags are detected and visualized
    demo.loop();
  } else {
    // process single image
    demo.loadImages();
  }

  return 0;
}
