#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "core/ports/visual_place_recognition.h"

namespace DBoW2 {
class BowVector;
class FeatureVector;
}

namespace ORB_SLAM3 {
class ORBVocabulary;
}

namespace SmartDrone::Adapters::Slam {

bool LoadOrbVocabularyWithCache(const std::string &textPath, ORB_SLAM3::ORBVocabulary &vocabulary);

bool TransformOrbDescriptors(const ORB_SLAM3::ORBVocabulary &vocabulary, const cv::Mat &descriptors,
                             DBoW2::BowVector &bow, DBoW2::FeatureVector &features,
                             int levelsUp = 4);

bool TransformOrbDescriptors(const ORB_SLAM3::ORBVocabulary &vocabulary, const cv::Mat &descriptors,
                             Core::Ports::VisualVocabularyTransform &out, int levelsUp = 4);

} // namespace SmartDrone::Adapters::Slam
