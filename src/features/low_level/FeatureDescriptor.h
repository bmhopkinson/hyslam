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
    FeatureDescriptor(cv::Mat desc, std::shared_ptr<DescriptorDistance> distfunc_ );
    int distance(const FeatureDescriptor &d2) const;
    cv::Mat rawDescriptor() const {return descriptor.clone(); };
private:
    cv::Mat descriptor;
    std::shared_ptr<DescriptorDistance> distfunc;

};
}


#endif //HYSLAM_FEATUREDESCRIPTOR_H
