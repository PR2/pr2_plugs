#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_plugs_msgs/VisionPlugDetectionAction.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

#include <checkerboard_pose_estimation/ros_detector.h>

class DetectPlugAction
{
public:
  DetectPlugAction(const std::string& name)
    : it_(nh_),
      as_(nh_, name),
      name_(name)
  {
    // Load configuration
    if (!detector_.initFromParameters()) {
      ROS_FATAL("%s: One or more required parameters not set, shutting down", name_.c_str());
      nh_.shutdown();
      return;
    }
    
    // Register the goal and preempt callbacks
    as_.registerGoalCallback(boost::bind(&DetectPlugAction::goalCb, this));
    as_.registerPreemptCallback(boost::bind(&DetectPlugAction::preemptCb, this));
  }

  void goalCb()
  {
    ROS_INFO("%s: Received new goal", name_.c_str());
    
    // Accept the new goal
    typedef boost::shared_ptr<const pr2_plugs_msgs::VisionPlugDetectionGoal> GoalPtr;
    GoalPtr goal = as_.acceptNewGoal();
    /// @todo Still need to check isPreemptRequested?
    typedef checkerboard_pose_estimation::Detector Side;
    detector_.getDetector().setOriginSide(goal->origin_on_right ? Side::RIGHT : Side::LEFT);
    tf::poseStampedMsgToTF(goal->prior, plug_prior_);

    // Subscribe to the streams from the requested camera
    std::string image_topic = goal->camera_name + "/image_rect";
    sub_ = it_.subscribeCamera(image_topic, 1, &DetectPlugAction::detectCb, this);

    // Abort if no one is publishing images    
    ros::Duration timeout;
    ros::Time start = ros::Time::now();
    while(timeout<ros::Duration(5.0))
    {
      if(sub_.getNumPublishers()>0)
        return;
      ros::Duration(0.05).sleep();
      timeout = ros::Time::now() - start;
    }

    as_.setAborted();
    ROS_INFO("%s: Aborted, there are no publishers on %s topic", name_.c_str(), goal->camera_name.c_str());
    sub_.shutdown();
  }

  void preemptCb()
  {
    ROS_INFO("%s: Preempted", name_.c_str());
    as_.setPreempted();
    sub_.shutdown();
  }

  void detectCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (!as_.isActive()) return;
    
    ROS_INFO("%s: Received image, performing detection", name_.c_str());

    tf::Stamped<tf::Pose> plug_pose;
    if (detector_.detectObject(image_msg, info_msg, plug_prior_, tf_listener_, plug_pose)) {
      // Report success
      pr2_plugs_msgs::VisionPlugDetectionResult result;
      tf::poseStampedTFToMsg(plug_pose, result.plug_pose);
      as_.setSucceeded(result);
      
      // Shut down the camera subscription and wait for the next goal
      sub_.shutdown();
    }
  }

protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  actionlib::SimpleActionServer<pr2_plugs_msgs::VisionPlugDetectionAction> as_;
  std::string name_;
  image_transport::CameraSubscriber sub_;
  tf::TransformListener tf_listener_;
  tf::Stamped<tf::Pose> plug_prior_;
  checkerboard_pose_estimation::RosDetector detector_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_detect_plug");
  DetectPlugAction action(ros::this_node::getName());
  ros::spin();

  return 0;
}
