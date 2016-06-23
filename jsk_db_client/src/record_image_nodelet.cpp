/*
 * record_image_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_topic_tools/log_utils.h>
#include <boost/assign.hpp>
#include <jsk_db_client/record_image.h>

namespace jsk_db_client
{

  void RecordImage::onInit()
  {
    DiagnosticNodelet::onInit();

    srv_ = boost::make_shared<ConfServer> (*pnh_);
    ConfServer::CallbackType f = boost::bind(&RecordImage::configCallback, this, _1, _2);
    srv_->setCallback(f);

    onInitPostProcess();
  }

  void RecordImage::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // TODO: config
  }

  void RecordImage::subscribe()
  {
    it_.reset(new image_transport::ImageTransport(*pnh_));
    sub_ = it_->subscribeCamera("image", 1, &RecordImage::substract, this);
    ros::V_string names = boost::assign::list_of("image");
    jsk_topic_tools::warnNoRemap(names);
  }

  void RecordImage::unsubscribe()
  {
    sub_.shutdown();
  }

  void RecordImage::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "RecordImage running");
    } else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "RecordImage", vital_checker_, stat);
    }
  }

  void RecordImage::substract(const sensor_msgs::ImageConstPtr &img_msg,
                              const sensor_msgs::CameraInfoConstPtr &info_msg)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    // TODO: Save image
  }
} // namespace jsk_db_client

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_db_client::RecordImage, nodelet::Nodelet);
