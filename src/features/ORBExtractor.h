/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

/*
 * implements FeatureExtractor for ORB Features using ORBSLAM2's implementation
 * ORBExtractor(std::unique_ptr<FeatureFinder> feature_finder_, std::shared_ptr<DescriptorDistance> dist_func_, FeatureExtractorSettings settings)
 *      constructor is used to obtain the lower-level feature_finder object (must be ORBFinder currently), appropriate feature descriptor distance function and settings
 * the extraction function ()overload finds number of keypoints requested in settings distributed over an image pyramid and spatially dispersed using a quadtree
 * the input image is broken into N_CELLS subsections to aid dispersion of features across image. N_CELLS is suprisingly high (~30) but this seems to work well.
 *
 *
 */

#include <FeatureExtractor.h>
#include <FeatureFinder.h>
#include <FeatureDescriptor.h>
#include <DescriptorDistance.h>

#include <opencv/cv.h>
#include <vector>
#include <list>
#include <memory>


namespace HYSLAM
{



class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBExtractor : public FeatureExtractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBExtractor(std::unique_ptr<FeatureFinder> feature_finder_, std::shared_ptr<DescriptorDistance> dist_func_, FeatureExtractorSettings settings);

    ~ORBExtractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      std::vector<FeatureDescriptor> &descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);


    std::unique_ptr<FeatureFinder> feature_finder;
    std::shared_ptr<DescriptorDistance> dist_func;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;
    int N_CELLS = 30;

    std::vector<int> mnFeaturesPerLevel;


    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

