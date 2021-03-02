//
// Created by cv-bhlab on 3/2/21.
//

#include "SURFFinder.h"

namespace HYSLAM {

SURFFinder::SURFFinder(){
    detector = cv::xfeatures2d::SURF::create(threshold, nOctaves, nOctaveLayers);
}

SURFFinder::SURFFinder(double threshold_) : threshold(threshold_)
{
    detector = cv::xfeatures2d::SURF::create(threshold, nOctaves, nOctaveLayers);
}

}
