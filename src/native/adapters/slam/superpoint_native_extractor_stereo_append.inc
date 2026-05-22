void AppendStereoFeaturePairs(SuperPointFeatureSet &leftOut,
                              SuperPointFeatureSet &rightOut,
                              const SuperPointFeatureSet &leftSupplement,
                              const SuperPointFeatureSet &rightSupplement,
                              int maxPoints)
{
    if (leftSupplement.descriptors.empty() ||
        rightSupplement.descriptors.empty() ||
        leftSupplement.descriptors.type() != CV_32F ||
        rightSupplement.descriptors.type() != CV_32F ||
        leftSupplement.descriptors.cols != rightSupplement.descriptors.cols ||
        leftSupplement.descriptors.rows !=
            static_cast<int>(leftSupplement.keypoints.size()) ||
        rightSupplement.descriptors.rows !=
            static_cast<int>(rightSupplement.keypoints.size())) {
        return;
    }
    if ((!leftOut.descriptors.empty() &&
         leftOut.descriptors.cols != leftSupplement.descriptors.cols) ||
        (!rightOut.descriptors.empty() &&
         rightOut.descriptors.cols != rightSupplement.descriptors.cols)) {
        return;
    }

    const int limit = std::max(1, maxPoints);
    const size_t sourceCount = std::min(leftSupplement.keypoints.size(),
                                        rightSupplement.keypoints.size());
    for (size_t i = 0;
         i < sourceCount && static_cast<int>(leftOut.keypoints.size()) < limit;
         ++i) {
        const cv::Point2f &leftPoint = leftSupplement.keypoints[i];
        const cv::Point2f &rightPoint = rightSupplement.keypoints[i];
        if (IsStereoFeaturePairNearExisting(leftPoint, rightPoint, leftOut,
                                            rightOut)) {
            continue;
        }
        leftOut.keypoints.push_back(leftPoint);
        rightOut.keypoints.push_back(rightPoint);
        leftOut.descriptors.push_back(
            leftSupplement.descriptors.row(static_cast<int>(i)));
        rightOut.descriptors.push_back(
            rightSupplement.descriptors.row(static_cast<int>(i)));
    }
}
