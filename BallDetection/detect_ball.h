#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>


class BallDetection{

    private:
        cv::VideoCapture capture;
        cv::Scalar lower_orange;
        cv::Scalar upper_orange;
        bool ball_detected;
        std::vector<std::vector<cv::Point>> contours;
    public:
        BallDetection();
        bool find_ball();
    
};
    