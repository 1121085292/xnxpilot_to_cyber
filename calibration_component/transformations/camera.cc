#include "camera.h"

Eigen::MatrixXd getViewFrameFromRoadFrame(double roll, double pitch, double yaw, double height){
    Eigen::Vector3d diag;
    diag << 1, -1, -1;
    Eigen::Matrix3d digMatrix = diag.asDiagonal();
    Eigen::Matrix3d rotation_matrix = euler2rot(Eigen::Vector3d(roll, pitch, yaw));
    Eigen::Matrix3d device_from_road = rotation_matrix * digMatrix;

    // 构造设备坐标系到视角坐标系的变换矩阵
    Eigen::Matrix3d device_frame_from_view_frame;
    device_frame_from_view_frame << 0., 0., 1.,
                                    1., 0., 0.,
                                    0., 1., 0.;
    // 求解视角坐标系到设备坐标系的变换矩阵
    auto view_frame_from_device_frame = device_frame_from_view_frame.transpose();

    Eigen::Matrix3d view_from_road = view_frame_from_device_frame * device_from_road;
    Eigen::MatrixXd result(3, 4);
    result << view_from_road, Eigen::Vector3d(0, height, 0);
    return result;
}