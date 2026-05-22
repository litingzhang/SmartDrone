#pragma once

#include <memory>
#include <string>

#include "core/ports/visual_place_recognition.h"

namespace SmartDrone::adapters::slam {

using VisualBowVector = core::ports::VisualBowVector;
using VisualFeatureIndex = core::ports::VisualFeatureIndex;
using VisualVocabularyTransform = core::ports::VisualVocabularyTransform;
using VisualWordWeight = core::ports::VisualWordWeight;

class OrbVisualVocabulary final : public core::ports::IVisualVocabulary {
  public:
    OrbVisualVocabulary();
    ~OrbVisualVocabulary() override;

    bool Load(const std::string &path) override;
    bool Loaded() const override;
    bool Transform(const cv::Mat &descriptors, VisualVocabularyTransform &out,
                   int levelsUp = 4) const override;
    double Score(const VisualBowVector &left, const VisualBowVector &right) const override;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::adapters::slam
