//
// Created by sean on 21/01/16.
//

#include "CameraExtrinsicsIO.hpp"
#include <opencv2/core/persistence.hpp>
#include "Logger.hpp"

bool CameraExtrinsicsIO::loadExtrinsicsRT(std::string filepath,
                                           Eigen::Matrix4f &RT) {
  cv::FileStorage fs_params;
  if (!fs_params.open(filepath.c_str(), cv::FileStorage::READ))
    return false;

  cv::Mat R = cv::Mat_<double>::zeros(3, 3);
  cv::Mat T = cv::Mat_<double>::zeros(3, 1);

  fs_params["R"] >> R;
  fs_params["T"] >> T;

  auto range = std::vector<int>{0, 1, 2};

  for (const auto & row : range)
    for (const auto & col : range)
      RT(row, col) = static_cast<float>(R.at<double>(row, col));

  for (const auto & row : range)
    RT(row, 3) = static_cast<float>(T.at<double>(row, 0));

  RT(3, 0) = 0.0;
  RT(3, 1) = 0.0;
  RT(3, 2) = 0.0;
  RT(3, 3) = 1.0;

  auto ss1 = std::stringstream();
  ss1 << "Loaded RT from" << filepath << ":\n" << RT << std::endl;

  Logger::log(Logger::INFO, ss1.str());

  return true;
}


bool CameraExtrinsicsIO::loadExtrinsics(std::string filepath, Eigen::Matrix4f &RT) {
  cv::FileStorage fs_params;
  if (!fs_params.open(filepath.c_str(), cv::FileStorage::READ))
    return false;

  cv::Mat RT_mat = cv::Mat_<float>::zeros(4, 4);
  fs_params["RT"] >> RT_mat;

  for(auto i = 0; i < RT_mat.rows; ++i)
    for(auto j = 0; j <RT_mat.cols; ++j)
       RT(i, j) = RT_mat.at<float>(i, j);


  auto ss1 = std::stringstream();
  ss1 << "Loaded RT from" << filepath << ":\n" << RT << std::endl;
  Logger::log(Logger::INFO, ss1.str());

  return true;
}


bool CameraExtrinsicsIO::saveExtrinsics(std::string filepath, const Eigen::Matrix4f &RT) {
  cv::FileStorage fs_params;
  if (!fs_params.open(filepath.c_str(), cv::FileStorage::WRITE))
    return false;

  cv::Mat RT_mat = cv::Mat_<float>::zeros(4, 4);

  for(auto i = 0; i < RT.rows(); ++i)
    for(auto j = 0; j <RT.cols(); ++j)
      RT_mat.at<float>(i, j) = RT(i, j);

  fs_params << "RT" << RT_mat;
  fs_params.release();

  auto ss = std::stringstream();
  ss << "Saved RT to " << filepath << ":\n" << RT << std::endl;
  Logger::log(Logger::INFO, ss.str());

  return true;
}


