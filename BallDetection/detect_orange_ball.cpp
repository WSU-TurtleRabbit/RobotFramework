#include "detect_ball.h"

int main() {
    BallDetection b;

    while(true) {
        auto start_time = std::chrono::high_resolution_clock::now();
        bool detected = b.find_ball();
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;
        sleep(1);
        std::cout << detected << std::endl;
    };
    
    return 0;
}


   