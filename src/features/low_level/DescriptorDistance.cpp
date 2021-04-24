//
// Created by cv-bhlab on 3/10/21.
//

#include "DescriptorDistance.h"

namespace HYSLAM{

float ORBDistance::distance(const cv::Mat &D1, const cv::Mat &D2) {
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    const int *pa = D1.ptr<int32_t>();
    const int *pb = D2.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return static_cast<float>(dist);
}

float SURFDistance::distance(const cv::Mat &D1, const cv::Mat &D2) {
    return cv::norm(D1, D2, cv::NORM_L1);
}



}