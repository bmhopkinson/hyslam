//
// Created by cv-bhlab on 3/10/21.
//

#ifndef HYSLAM_FEATUREDESCRIPTOR_H
#define HYSLAM_FEATUREDESCRIPTOR_H

/*
 * represents a Feature Descriptor, distance function can be supplied making the class generic to type of feature (as long as it's a KeyPoint type)
 * data:
 *  descriptor - as cv::Mat so can be integer or float of variable length
 *  distance function
 *
 * Functionality:
 *  distance() - distance/dissimilarity measure between itself and another FeatureDescriptor of the same type - uses supplied distance function
 */

#include <DescriptorDistance.h>

#include <opencv2/core/core.hpp>

#include <memory>

namespace HYSLAM {

class FeatureDescriptor {
public:
    FeatureDescriptor(){};
    FeatureDescriptor(cv::Mat desc, std::shared_ptr<DescriptorDistance> distfunc_ );
    float distance(const FeatureDescriptor &d2) const;
    cv::Mat rawDescriptor() const {return descriptor.clone(); };
    bool isEmpty() {return is_empty;}
private:
    cv::Mat descriptor;
    std::shared_ptr<DescriptorDistance> distfunc;
    bool is_empty = true;

};
}


#endif //HYSLAM_FEATUREDESCRIPTOR_H
