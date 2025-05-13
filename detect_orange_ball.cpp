#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 50514
#define ROBOT_IP "192.168.220.56"
#define BUFFER_SIZE 1024

int main() {
    // Open camera
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera\n";
        return -1;
    }

    // Define HSV range for detecting orange
    cv::Scalar lower_orange(5, 100, 100);
    cv::Scalar upper_orange(15, 255, 255);

    
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating UDP socket\n";
        return -1;
    }

    struct sockaddr_in robot_addr{};
    robot_addr.sin_family = AF_INET;
    robot_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, ROBOT_IP, &robot_addr.sin_addr);

    while (true) {
        cv::Mat frame, hsv, mask;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Empty frame\n";
            break;
        }

        // Convert to HSV and threshold for orange
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower_orange, upper_orange, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // bool ball_detected = false;
        bool ball_detected = !contours.empty();

        const char* message = ball_detected ? "true" : "false";
        sendto(sockfd, message, strlen(message), 0,
               (struct sockaddr*)&robot_addr, sizeof(robot_addr));
        std::cout << "Ball Detected: " << message << std::endl;

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
         
        std::cout << "Ball Detected: " << (ball_detected ? "true" : "false") << std::endl;
        
        cv::imshow("Camera Feed", frame);
        // cv::imshow("Orange Mask", mask);

        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    cap.release();
    close(sockfd);
    cv::destroyAllWindows();
    return 0;
}



          // True if *any* contour found

   