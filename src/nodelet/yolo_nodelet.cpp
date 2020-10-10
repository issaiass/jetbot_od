// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * This is a demo of yolo image processing,
 */

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <jetbot_od/nodelet.h>
#include <jetbot_od/YOLOConfig.h>

#include <dynamic_reconfigure/server.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn/dnn.hpp>

using namespace cv;
using namespace dnn;

namespace jetbot_od
{
class YOLONodelet : public jetbot_od::Nodelet
{
  ////////////////////////////////////////////////////////
  // Dynamic Reconfigure
  ////////////////////////////////////////////////////////
  typedef jetbot_od::YOLOConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;

  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  boost::mutex mutex_;

  std::string yolo_cfg_;
  std::string yolo_weights_;
  std::string yolo_names_;
  double_t conf_thr_;
  double_t nms_thr_;
  int inputW_;
  int inputH_;

  std::vector<std::string> classes;
  Net net;
  std::string classesFile;
  String modelConfiguration;
  String modelWeights;

  // Remove the bounding boxes with low confidence using non-maxima suppression
  void postprocess(Mat& frame, const std::vector<Mat>& outs)
  {
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > conf_thr_)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    NMSBoxes(boxes, confidences, conf_thr_, nms_thr_, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
  }

  // Draw the predicted bounding box
  void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
  {
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

    //Get the label for the class name and its confidence
    std::string label = format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0),1);
  }

  // Get the names of the output layers
  std::vector<String> getOutputsNames(const Net& net)
  {
    static std::vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        std::vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &YOLONodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &YOLONodelet::imageCallback, this);
  }

  void unsubscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  void reconfigureCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    conf_thr_ = config.conf_thr;
    nms_thr_ = config.nms_thr;
  }

  void doWork(const sensor_msgs::Image::ConstPtr& image_msg, const std::string& input_frame_from_msg)
  {
    try
    {
      Mat src_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      Mat blob;
      blobFromImage(src_image, blob, 1/255.0, Size(inputW_, inputH_), Scalar(0,0,0), true, false);
      //Sets the input to the network
      net.setInput(blob);
      // Runs the forward pass to get output of the output layers
      std::vector<Mat> outs;
      net.forward(outs, getOutputsNames(net));

      // Remove the bounding boxes with low confidence
      postprocess(src_image, outs);
      // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
      std::vector<double> layersTimes;
      double freq = getTickFrequency() / 1000;
      double t = net.getPerfProfile(layersTimes) / freq;
      std::string label = format("Inference time for a frame : %.2f ms", t);
      putText(src_image, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

      if (debug_view_)
      {
        namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        resize(src_image, src_image, Size(), 0.8, 0.8, INTER_LINEAR);
        imshow(window_name_, src_image);
        int c = waitKey(1);
      }
      img_pub_.publish(
          cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::MONO8, src_image).toImageMsg());
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  }

public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("debug_view", debug_view_, false);

    pnh_->param<std::string>("yolo_cfg", yolo_cfg_, "/home/robot/ros_ws/src/jetbot_od/object_detection/yolo/yolov3.cfg");
    pnh_->param<std::string>("yolo_weights", yolo_weights_, "/home/robot/ros_ws/src/jetbot_od/object_detection/yolo/yolov3.weights");
    pnh_->param<std::string>("yolo_names", yolo_names_, "/home/robot/ros_ws/src/jetbot_od/object_detection/yolo/coco.names");
    pnh_->param("inputW", inputW_, 416);
    pnh_->param("inputH", inputH_, 416);

    if (debug_view_)
    {
      always_subscribe_ = true;
    }

    // Load names of classes
    classesFile = yolo_names_;
    std::ifstream ifs(classesFile.c_str());
    std::string line;
    while (getline(ifs, line)) {       
      classes.push_back(line);
    }

    // Give the configuration and weight files for the model
    modelConfiguration = yolo_cfg_;
    modelWeights = yolo_weights_;
    // Load the network
    net = readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);

    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&YOLONodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    onInitPostProcess();
  }
};
}  // namespace jetbot_od

namespace yolo
{
class YOLONodelet : public jetbot_od::YOLONodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet yolo/yolo is deprecated, "
             "and renamed to jetbot_od/yolo.");
    jetbot_od::YOLONodelet::onInit();
  }
};
}  // namespace yolo

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jetbot_od::YOLONodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(yolo::YOLONodelet, nodelet::Nodelet);