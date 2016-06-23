/*
 * record_image_nodelet.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef RECORD_IMAGE_NODELET_H__
#define RECORD_IMAGE_NODELET_H__


#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_db_client/RecordImageConfig.h>

namespace jsk_db_client
{
  class RecordImage: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    RecordImage(): DiagnosticNodelet("RecordImage") {}
    typedef aem_db_client::RecordImageConfig Config;
    typedef dynamic_reconfigure::Server<Config> ConfServer;
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void updateDiagnostic(
      diagnotisc_updater::DiagnosticStatusWrapper& stat);
    virtual void substract(
      const sensor_msgs::ImageConstPtr & img_msg);
    virtual void configCallback(Config& config, uint32_t level);

    image_transport::CameraSubscriber sub_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<ConfServer> srv_;
    boost::mutex mutex_;
  private:
  }; // class RecordImage
} // namespace jsk_db_client

#endif // RECORD_IMAGE_NODELET_H__







