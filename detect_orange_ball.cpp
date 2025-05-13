#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Open the default camera (usually /dev/video0)
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera\n";
        return -1;
    }

    // Define HSV range for detecting orange
    // Tune as needed for your lighting conditions
    cv::Scalar lower_orange(5, 100, 100);
    cv::Scalar upper_orange(15, 255, 255);

    while (true) {
        cv::Mat frame, hsv, mask;
        cap >> frame;

        if (frame.empty()) {
            std::cerr << "Error: Empty frame\n";
            break;
        }

        // Convert to HSV color space
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Threshold the HSV image to get only orange colors
        cv::inRange(hsv, lower_orange, upper_orange, mask);

        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            
            size_t largest_index = 0;
            double max_area = 0;
            for (size_t i = 0; i < contours.size(); ++i) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    largest_index = i;
                }
            }

            
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contours[largest_index], center, radius);
            if (radius > 10) {  
                cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, center, 5, cv::Scalar(255, 0, 0), -1);
            }
        }

        
        cv::imshow("Camera Feed", frame);
        cv::imshow("Orange Mask", mask);

        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
