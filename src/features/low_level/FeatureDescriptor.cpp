//
// Created by cv-bhlab on 3/10/21.
//

#include "FeatureDescriptor.h"

namespace HYSLAM {
FeatureDescriptor::FeatureDescriptor(cv::Mat desc, std::shared_ptr<DescriptorDistance> distfunc_ ):
        distfunc(distfunc_), is_empty(false)
{
    descriptor = desc.clone();
}

float FeatureDescriptor::distance(const FeatureDescriptor &d2) const
{
    cv::Mat d2_raw = d2.rawDescriptor();
    return distfunc->distance(descriptor, d2_raw);
}

} //end namespace