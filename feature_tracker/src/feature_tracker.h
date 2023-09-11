#pragma once

#include <execinfo.h>

#include <csignal>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

///@brief 边界判断
bool inBorder(const cv::Point2f& pt);

///@brief 根据status筛选v
void reduceVector(vector<cv::Point2f>& v, vector<uchar> status);

///@brief 根据status筛选v
void reduceVector(vector<int>& v, vector<uchar> status);

class FeatureTracker {
 public:
  FeatureTracker();

  void readImage(const cv::Mat& _img, double _cur_time);

  /// 设置筛除关键点的mask区域
  void setMask();

  void addPoints();

  ///@brief 更新关键点id
  bool updateID(unsigned int i);

  ///@brief 获取内参
  void readIntrinsicParameter(const string& calib_file);

  ///@brief 可视化畸变纠正后的
  void showUndistortion(const string& name);

  ///@brief 通过F阵(基础矩阵)剔外点
  void rejectWithF();

  ///@brief 对点进行畸变纠正
  void undistortedPoints();

  cv::Mat mask;
  cv::Mat fisheye_mask;
  cv::Mat prev_img;
  cv::Mat cur_img;                   /// 名义上的当前帧(实际是上一帧--LK光流定义为cur&next)
  cv::Mat forw_img;                  /// 名义上的下一帧(实际是当前帧)
  vector<cv::Point2f> n_pts;         /// 保存关键点
  vector<cv::Point2f> prev_pts;      /// 名义上的上一关键点(实际为上上帧)
  vector<cv::Point2f> cur_pts;       /// 名义上的当前关键点(实际为上帧)
  vector<cv::Point2f> forw_pts;      /// 名义上的下一关键点(实际为当前帧)
  vector<cv::Point2f> prev_un_pts;   /// 存储上一帧畸变纠正后的关键点
  vector<cv::Point2f> cur_un_pts;    /// 存储当前帧畸变纠正后的关键点
  vector<cv::Point2f> pts_velocity;  /// 存储关键点的速度
  vector<int> ids;                   /// 当添加关键点,添加-1,后面使用n_id进行计数
  vector<int> track_cnt;
  map<int, cv::Point2f> cur_un_pts_map;
  map<int, cv::Point2f> prev_un_pts_map;
  camodocal::CameraPtr m_camera;  /// 相机模型
  double cur_time;                /// 当前时间
  double prev_time;               /// 上一帧时间点

  static int n_id;  /// 静态变量,关键点id计数
};
