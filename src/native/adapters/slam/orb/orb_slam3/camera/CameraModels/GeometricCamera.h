/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Geometry>

#include "Converter.h"
#include "GeometricTools.h"

namespace ORB_SLAM3 {
    class GeometricCamera;

    struct TwoViewReconstructionRequest {
        const std::vector<cv::KeyPoint>* keys1 = nullptr;
        const std::vector<cv::KeyPoint>* keys2 = nullptr;
        const std::vector<int>* matches12 = nullptr;
    };

    struct TwoViewReconstructionResult {
        Sophus::SE3f* T21 = nullptr;
        std::vector<cv::Point3f>* points3D = nullptr;
        std::vector<bool>* triangulated = nullptr;
    };

    struct EpipolarConstraintRequest {
        GeometricCamera* otherCamera = nullptr;
        const cv::KeyPoint* keypoint1 = nullptr;
        const cv::KeyPoint* keypoint2 = nullptr;
        const Eigen::Matrix3f* R12 = nullptr;
        const Eigen::Vector3f* t12 = nullptr;
        float sigmaLevel = 1.0f;
        float uncertainty = 1.0f;
    };

    struct CameraMatchTriangulationRequest {
        const cv::KeyPoint* keypoint1 = nullptr;
        const cv::KeyPoint* keypoint2 = nullptr;
        GeometricCamera* otherCamera = nullptr;
        Sophus::SE3f* Tcw1 = nullptr;
        Sophus::SE3f* Tcw2 = nullptr;
        float sigmaLevel1 = 1.0f;
        float sigmaLevel2 = 1.0f;
    };

    struct CameraTriangulationRequest {
        GeometricCamera* otherCamera = nullptr;
        const cv::KeyPoint* keypoint1 = nullptr;
        const cv::KeyPoint* keypoint2 = nullptr;
        const Eigen::Matrix3f* R12 = nullptr;
        const Eigen::Vector3f* t12 = nullptr;
        float sigmaLevel = 1.0f;
        float uncertainty = 1.0f;
    };

    struct CameraTriangulationResult {
        Eigen::Vector3f* point3D = nullptr;
    };

    class GeometricCamera {

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & mnId;
            ar & mnType;
            ar & mvParameters;
        }


    public:
        GeometricCamera() {}
        GeometricCamera(const std::vector<float> &_vParameters) : mvParameters(_vParameters) {}
        ~GeometricCamera() {}

        virtual cv::Point2f project(const cv::Point3f &p3D) = 0;
        virtual Eigen::Vector2d project(const Eigen::Vector3d & v3D) = 0;
        virtual Eigen::Vector2f project(const Eigen::Vector3f & v3D) = 0;
        virtual Eigen::Vector2f projectMat(const cv::Point3f& p3D) = 0;

        virtual float uncertainty2(const Eigen::Matrix<double,2,1> &p2D) = 0;

        virtual Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) = 0;
        virtual cv::Point3f unproject(const cv::Point2f &p2D) = 0;

        virtual Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D) = 0;

        virtual bool ReconstructWithTwoViews(const TwoViewReconstructionRequest& request,
                                             const TwoViewReconstructionResult& result) = 0;

        virtual cv::Mat toK() = 0;
        virtual Eigen::Matrix3f toK_() = 0;

        virtual bool epipolarConstrain(const EpipolarConstraintRequest& request) = 0;

        float getParameter(const int i){return mvParameters[i];}
        void setParameter(const float p, const size_t i){mvParameters[i] = p;}

        size_t size(){return mvParameters.size();}

        virtual bool matchAndtriangulate(const CameraMatchTriangulationRequest& request,
                                 const CameraTriangulationResult& result) = 0;

        unsigned int GetId() { return mnId; }

        unsigned int GetType() { return mnType; }

        const static unsigned int CAM_PINHOLE = 0;
        const static unsigned int CAM_FISHEYE = 1;

        static long unsigned int nNextId;

    protected:
        std::vector<float> mvParameters;

        unsigned int mnId;

        unsigned int mnType;
    };
}


#endif //CAMERAMODELS_GEOMETRICCAMERA_H
