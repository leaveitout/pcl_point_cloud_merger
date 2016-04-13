//
// Created by sean on 21/01/16.
//

#ifndef PCL_CLOUD_REGISTRATION_CAMERAEXTRINSICSLOADER_HPP
#define PCL_CLOUD_REGISTRATION_CAMERAEXTRINSICSLOADER_HPP

#include <cstddef>
#include <Eigen/Dense>

namespace CameraExtrinsicsIO {

bool loadExtrinsicsRT(std::string filepath, Eigen::Matrix4f &RT);

bool loadExtrinsics(std::string filepath, Eigen::Matrix4f &RT);

bool saveExtrinsics(std::string filepath, const Eigen::Matrix4f &RT);

};

#endif //PCL_CLOUD_REGISTRATION_CAMERAEXTRINSICSLOADER_HPP
