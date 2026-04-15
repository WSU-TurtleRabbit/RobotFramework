#include "detect_ball.h"
#include <cmath>

// Rough default horizontal field of view for a 320x240 USB camera.
// Override at calibration time by editing `focal_px` after open_cam().
static constexpr float DEFAULT_HFOV_RAD = 1.0472f; // 60 deg

BallDetection::BallDetection(){

    lower_orange = cv::Scalar(5, 100, 100);
    upper_orange = cv::Scalar(15, 255, 255);
    frame_w = 320;
    frame_h = 240;
    focal_px = (frame_w * 0.5f) / std::tan(DEFAULT_HFOV_RAD * 0.5f);
}

int BallDetection::open_cam(){
    capture.open(0);
    if (!capture.isOpened()) {
        std::cerr << "Error: Could not open camera\n";
        return -1;
    }
    capture.set(cv::CAP_PROP_FRAME_WIDTH, frame_w);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_h);
    capture.set(cv::CAP_PROP_FPS, 30);
    return 1;
}

BallObservation BallDetection::observe() {
    BallObservation obs{false, 0.f, 0.f, 0.f, 0.f, 0.f};

    cv::Mat frame, hsv, mask;
    capture >> frame;
    if (frame.empty()) {
        std::cerr << "Error: Empty frame\n";
        return obs;
    }

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lower_orange, upper_orange, mask);
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // Pick the largest contour above the area threshold.
    double best_area = 0.0;
    const std::vector<cv::Point>* best = nullptr;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 300 && area > best_area) {
            best_area = area;
            best = &contour;
        }
    }
    if (best == nullptr) return obs;

    cv::Point2f center;
    float radius = 0.f;
    cv::minEnclosingCircle(*best, center, radius);

    // Roundness = (observed area) / (enclosing-circle area).  1.0 = perfect circle.
    float circle_area = static_cast<float>(M_PI) * radius * radius;
    float roundness = (circle_area > 1.f)
        ? static_cast<float>(best_area) / circle_area
        : 0.f;

    obs.found      = true;
    obs.px         = center.x;
    obs.py         = center.y;
    obs.radius     = radius;
    obs.bearing    = std::atan2(center.x - frame_w * 0.5f, focal_px);
    obs.confidence = std::min(1.0f, roundness);
    return obs;
}

bool BallDetection::find_ball() {
    return observe().found;
}
