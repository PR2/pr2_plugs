#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_plugs_msgs/VisionOutletDetectionAction.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <outlet_pose_estimation/detector.h>
#include <outlet_pose_estimation/estimator.h>

using namespace outlet_pose_estimation;
using namespace visual_pose_estimation;

bool initFromParameters(Detector& detector, PoseEstimator& estimator)
{
  // Read all the parameters
  ros::NodeHandle local_nh("~");
  bool have_all_required = true;

  std::string outlet_template_path;
  if (!local_nh.getParam("outlet_template", outlet_template_path)) {
    ROS_ERROR("Outlet template [~outlet_template] unspecified");
    have_all_required = false;
  }

  if (!have_all_required) return false;

  // Initialize detector
  if (!detector.loadTemplate(outlet_template_path)) {
    ROS_ERROR("Could not load outlet template from file %s", outlet_template_path.c_str());
    return false;
  }

  // Initialize pose estimator with outlet object model
  estimator = createOutletEstimator(detector.getTemplate());

  return true;
}

class DetectOutletAction
{
public:
  DetectOutletAction(const std::string& name)
    : it_(nh_),
      as_(nh_, name, false),
      action_name_(name),
      update_transformed_prior_(true)
  {
    // Load configuration
    if (!initFromParameters(detector_, pose_estimator_)) {
      ROS_FATAL("%s: Error initializing from parameters, shutting down", action_name_.c_str());
      nh_.shutdown();
      return;
    }

    // Advertise visualization topics
    display_pub_ = it_.advertise("display_image", 1);
    //marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    
    // Register the goal and preempt callbacks
    as_.registerGoalCallback(boost::bind(&DetectOutletAction::goalCb, this));
    as_.registerPreemptCallback(boost::bind(&DetectOutletAction::preemptCb, this));

    as_.start();
  }

  void timeoutCb(const ros::TimerEvent& e)
  {
    if(sub_.getNumPublishers() == 0)
      ROS_INFO("%s: Aborted, there are no publishers on goal topic.", action_name_.c_str());    
    else
      ROS_INFO("%s: Aborted, there are publishers on goal topic, but detection took too long.", action_name_.c_str());    

    sub_.shutdown();
    as_.setAborted();
  }

  void goalCb()
  {
    ROS_INFO("%s: Received new goal", action_name_.c_str());
    // Accept the new goal
    typedef boost::shared_ptr<const pr2_plugs_msgs::VisionOutletDetectionGoal> GoalPtr;
    GoalPtr goal = as_.acceptNewGoal();
    /// @todo Still need to check isPreemptRequested?
    tf::poseStampedMsgToTF(goal->prior, prior_);
    tf::vector3StampedMsgToTF(goal->wall_normal, wall_normal_);
    update_transformed_prior_ = true;

    // Subscribe to the streams from the requested camera
    ros::Duration(1.0).sleep();
    std::string image_topic = goal->camera_name + "/image_rect_color";
    sub_ = it_.subscribeCamera(image_topic, 1, &DetectOutletAction::detectCb, this);

    //create a timer for how long we can actually transform images before we have to abort
    pub_timer_ = nh_.createTimer(tf_listener_.getCacheLength() - ros::Duration(1.0), boost::bind(&DetectOutletAction::timeoutCb, this, _1), true);
  }

  void preemptCb()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    pub_timer_.stop();
    sub_.shutdown();
  }

  void detectCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (!as_.isActive()) return;
    
    ROS_INFO("%s: Received image, performing detection", action_name_.c_str());

    // Convert image message
    /// @todo Try mono8 instead
    cv::Mat image = cv_bridge::toCvShare(image_msg, "bgr8")->image;

    // Detect the outlet
    std::vector<cv::Point2f> image_points;
    if (!detector_.detect(image, image_points)) {
      ROS_DEBUG("%s: Failed to detect outlet", action_name_.c_str());
      publishDisplayImage(image, image_points, false);
      /// @todo Publish feedback?
      return;
    }

    // Get the prior in the camera frame. Currently we transform the prior once, on the first
    // image received for a new goal, and assume that the camera frame does not move wrt to
    // the original frame of the prior while that goal is active.
    cam_model_.fromCameraInfo(info_msg);
    if (update_transformed_prior_) {
      // Transform the prior and wall normal into the camera coordinate frame.
      tf::Stamped<tf::Vector3> wall_normal_in_camera;
      try {
        /// @todo waitForTransform?
        tf_listener_.transformPose(cam_model_.tfFrame(), prior_, prior_in_camera_);
        tf_listener_.transformVector(cam_model_.tfFrame(), wall_normal_, wall_normal_in_camera);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("%s: TF exception\n%s", action_name_.c_str(), ex.what());
        return;
      }

      // Now we tweak the orientation of the original prior to align with the wall normal.
      // First pull out a couple prior basis vectors of interest.
      tf::Vector3 prior_x_basis = prior_in_camera_.getBasis().getColumn(0);
      tf::Vector3 prior_z_basis = prior_in_camera_.getBasis().getColumn(2);
      // New forward vector is set to the wall normal.
      tf::Vector3 adjusted_x_basis = wall_normal_in_camera.normalized();
      // We assume normal is into the wall, so positive Z in the camera frame.
      if (adjusted_x_basis.z() < 0)
        adjusted_x_basis = -adjusted_x_basis;
      // The wall normal should not differ wildly from the prior X (forward) basis vector.
      static const double ANGLE_THRESHOLD = 0.175; // ~10 degrees
      double angle = adjusted_x_basis.angle(prior_x_basis);
      if (angle > ANGLE_THRESHOLD) {
        ROS_ERROR("Wall normal differs from prior orientation by %f radians, threshold is %f. Aborting.",
                  angle, ANGLE_THRESHOLD);
        as_.setAborted();
        pub_timer_.stop();
        sub_.shutdown();
        return;
      }
      // New left vector calculated as cross product of the original up vector and the wall normal.
      tf::Vector3 adjusted_y_basis = prior_z_basis.cross(adjusted_x_basis).normalized();
      // New up vector calculated from the other two basis vectors.
      tf::Vector3 adjusted_z_basis = adjusted_x_basis.cross(adjusted_y_basis);
      // Fill in the new basis vectors as columns.
      prior_in_camera_.getBasis().setValue(adjusted_x_basis.x(), adjusted_y_basis.x(), adjusted_z_basis.x(),
                                           adjusted_x_basis.y(), adjusted_y_basis.y(), adjusted_z_basis.y(),
                                           adjusted_x_basis.z(), adjusted_y_basis.z(), adjusted_z_basis.z());

      update_transformed_prior_ = false;
    }

    // Estimate the outlet pose
    tf::Pose pose = pose_estimator_.solveWithPrior(image_points, cam_model_, prior_in_camera_);

    // Report success
    pr2_plugs_msgs::VisionOutletDetectionResult result;
    tf::poseTFToMsg(pose, result.outlet_pose.pose);
    result.outlet_pose.header.stamp = image_msg->header.stamp;
    result.outlet_pose.header.frame_id = cam_model_.tfFrame();
    as_.setSucceeded(result);
    pub_timer_.stop();

    // Publish visualization messages
    tf_broadcaster_.sendTransform(tf::StampedTransform(prior_, prior_.stamp_,
                                                       prior_.frame_id_, "outlet_prior_frame"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(prior_in_camera_, prior_in_camera_.stamp_,
                                                       prior_in_camera_.frame_id_,
                                                       "outlet_adjusted_prior_frame"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(pose, image_msg->header.stamp,
                                                       cam_model_.tfFrame(), "outlet_frame"));
    publishDisplayImage(image, image_points, true);

    // Shut down the camera subscription and wait for the next goal
    sub_.shutdown();
  }

  void publishDisplayImage(const cv::Mat& source, const std::vector<cv::Point2f>& image_points, bool success)
  {
    if (display_pub_.getNumSubscribers() == 0) return;
    detector_.getDisplayImage(source, image_points, success, display_img_cv_);
    cv_bridge::CvImage cv_image(std_msgs::Header(), "bgr8", display_img_cv_);
    display_pub_.publish(*(cv_image.toImageMsg()));
  }

protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  actionlib::SimpleActionServer<pr2_plugs_msgs::VisionOutletDetectionAction> as_;
  std::string action_name_;

  // Subscriptions
  image_transport::CameraSubscriber sub_;

  ros::Timer pub_timer_;

  // Publications
  image_transport::Publisher display_pub_;
  sensor_msgs::Image display_img_;
  cv::Mat display_img_cv_;
  ros::Publisher marker_pub_;

  // TF
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Message processing
  image_geometry::PinholeCameraModel cam_model_;
  outlet_pose_estimation::Detector detector_;
  visual_pose_estimation::PoseEstimator pose_estimator_;
  tf::Stamped<tf::Pose> prior_;
  tf::Stamped<tf::Pose> prior_in_camera_;
  tf::Stamped<tf::Vector3> wall_normal_;
  bool update_transformed_prior_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_detect_outlet");
  DetectOutletAction action(ros::this_node::getName());
  ros::spin();

  return 0;
}
