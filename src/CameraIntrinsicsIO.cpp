//
// Created by sean on 15/01/16.
//

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include "util/CameraIntrinsicsLoader.hpp"

bool CameraIntrinsicsIO::applyIntrinsics(size_t camera_num, std::shared_ptr<pcl::io::OpenNI2Grabber> grabber) {
    std::string path = "/home/sean/Documents/cameraparams/";
    std::stringstream ss1, ss2;
    ss1 << path << camera_num << "rgb.yml";
    ss2 << path << camera_num << "ir.yml";
    std::string rgb_filename = ss1.str();
    std::string ir_filename = ss2.str();

    cv::FileStorage fs_rgb;
    if(!fs_rgb.open(rgb_filename.c_str(), cv::FileStorage::READ) )
        return false;
    cv::FileStorage fs_ir;
    if(!fs_ir.open(ir_filename.c_str(), cv::FileStorage::READ) )
        return false;

    cv::Mat C_rgb = cv::Mat_<double>::zeros(3, 3);
    cv::Mat C_ir = cv::Mat_<double>::zeros(3, 3);

    fs_rgb["camera_matrix"] >> C_rgb;
    fs_ir["camera_matrix"] >> C_ir;

    double fx_rgb = C_rgb.at<double>(0, 0);
    double fy_rgb = C_rgb.at<double>(1, 1);
    double cx_rgb = C_rgb.at<double>(0, 2);
    double cy_rgb = C_rgb.at<double>(1, 2);

    double fx_ir = C_ir.at<double>(0, 0);
    double fy_ir = C_ir.at<double>(1, 1);
    double cx_ir = C_ir.at<double>(0, 2);
    double cy_ir = C_ir.at<double>(1, 2);

    grabber->setRGBCameraIntrinsics(fx_rgb, fy_rgb, cx_rgb, cy_rgb);
    grabber->setDepthCameraIntrinsics(fx_ir, fy_ir, cx_ir, cy_ir);

    return true;
}
