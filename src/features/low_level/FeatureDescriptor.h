//
// Created by cv-bhlab on 3/10/21.
//

#ifndef HYSLAM_FEATUREDESCRIPTOR_H
#define HYSLAM_FEATUREDESCRIPTOR_H

#include <DescriptorDistance.h>

#include <opencv2/core/core.hpp>

#include <memory>

namespace HYSLAM {

class FeatureDescriptor {
public:
    FeatureDescriptor(){};
    FeatureDescriptor(cv::Mat desc, std::shared_ptr<DescriptorDistance> distfunc_ );
    int distance(const FeatureDescriptor &d2) const;
    cv::Mat rawDescriptor() const {return descriptor.clone(); };
    bool isEmpty() {return is_empty;}
private:
    cv::Mat descriptor;
    std::shared_ptr<DescriptorDistance> distfunc;
    bool is_empty = true;

};
}


#endif //HYSLAM_FEATUREDESCRIPTOR_H
