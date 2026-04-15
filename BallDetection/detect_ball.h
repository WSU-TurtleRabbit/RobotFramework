#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#if defined(__unix__) || defined(__APPLE__)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif


// One ball observation from the onboard camera.
// Coordinates are in image pixels; bearing is the horizontal angle from
// the camera's optical axis in radians (positive = right of center).
// `confidence` is a [0..1] heuristic based on contour area / roundness.
struct BallObservation {
    bool  found;
    float px;          // centroid x in image pixels
    float py;          // centroid y in image pixels
    float radius;      // enclosing-circle radius in pixels
    float bearing;     // horizontal bearing from camera axis (radians)
    float confidence;  // 0..1, rough quality score
};


class BallDetection{

    private:
        cv::VideoCapture capture;
        cv::Scalar lower_orange;
        cv::Scalar upper_orange;
        std::vector<std::vector<cv::Point>> contours;

        int frame_w;
        int frame_h;
        float focal_px;      // horizontal focal length in pixels

    public:
        BallDetection();
        ~BallDetection()=default;

        // Legacy boolean wrapper — preserved so existing callers still link.
        bool find_ball();

        // Full observation: contour centroid, radius, bearing, confidence.
        BallObservation observe();

        int open_cam();

        int image_width()  const { return frame_w; }
        int image_height() const { return frame_h; }
};
