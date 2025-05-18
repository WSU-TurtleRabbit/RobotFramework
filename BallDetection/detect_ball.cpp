#include "detect_ball.h"

BallDetection::BallDetection(){

    capture.open(0);
    if (!capture.isOpened()) {
        std::cerr << "Error: Could not open camera\n";
    }
    // capture.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    // capture.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    capture.set(cv::CAP_PROP_FPS, 30);

    lower_orange = cv::Scalar(5, 100, 100);
    upper_orange = cv::Scalar(15, 255, 255);
}

bool BallDetection::find_ball() {
    cv::Mat frame, hsv, mask;
    capture >> frame;
    if (frame.empty()) {
        std::cerr << "Error: Empty frame\n";
    }

    // Convert to HSV and threshold for orange
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lower_orange, upper_orange, mask);
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    ball_detected = false;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 300) { // Adjust this threshold as needed
            ball_detected = true;
            break;
        }
    }
    return ball_detected;
}