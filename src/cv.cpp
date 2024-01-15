#include <iostream>
#include <opencv2/core/cuda.hpp>

int main() {
    // Check if CUDA is available
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        std::cout << "CUDA is available. Found CUDA-enabled device(s)." << std::endl;
    } else {
        std::cout << "No CUDA-enabled devices found." << std::endl;
    }

    return 0;
}

