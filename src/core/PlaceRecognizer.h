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

#ifndef PLACERECOGNIZER_H
#define PLACERECOGNIZER_H

/*
 * this class provides visually similar KeyFrames (from a set of stored frames) to query KeyFrames or Frames
 * similarity between query (Key)Frames and database frames is based on feature similarity through a hierarchical Vocabulary
 * data:
 *      logged_frames - KeyFrame Database
 *      feature vocubulary - discretizes feature descriptors to words, can compute Bag of Words (BoW) vector from set of features, and can compare two BoW vectors
 *
 * key functionality
 *   std::vector<KeyFrame *> detectLoopCandidates(KeyFrame* pKF, float minScore, std::set<KeyFrame*> KFs_excluded); //soon substitute (pKF, minScore, excludedKFs) excludedKFs = current frame and those connected in covis_graph
 *          provides potential loop candidates based on visual similarity to query frame pKF, similarity must exceed minScore, and KFs_exculded are not conisdered in database search
 *   std::vector<KeyFrame*> detectRelocalizationCandidates(Frame* F);
 *      provides potential relocalization candidates based on visual similarity to query frame F, includes checks involving co-visible keyframes to ensure the similarity is not spurious
 *      stores some intermediate data (mnRelocWords, mRelocScore) in KeyFrames - remove this - this data is only required locally within place recognizer
 *
 */

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include <FeatureVocabulary.h>

#include <mutex>


namespace HYSLAM
{

class KeyFrame;
class Frame;


class PlaceRecognizer
{
public:

    PlaceRecognizer();

    void setVocab(FeatureVocabulary* mpVoc_);

    FeatureVocabulary* getVocab(){return mpVoc;};

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFrame *> detectLoopCandidates(KeyFrame* pKF, float minScore, std::set<KeyFrame*> KFs_excluded); //soon substitute (pKF, minScore, excludedKFs) excludedKFs = current frame and those connected in covis_graph

   // Relocalization
   std::vector<KeyFrame*> detectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  FeatureVocabulary* mpVoc;

  std::vector<KeyFrame*> logged_frames;

  // Inverted file
  std::vector<std::list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
