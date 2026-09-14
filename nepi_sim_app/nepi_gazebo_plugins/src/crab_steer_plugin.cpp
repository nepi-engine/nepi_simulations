// Purely kinematic per-wheel steering+spin animator for a 4-wheel-
// independent-steering ("crab steering" / "wheel independence") rover.
//
// This plugin does NOT drive the rover's actual body physics -- that's
// libgazebo_ros_planar_move, mounted alongside it in the same SDF model
// (see generate_model_sdf.py's buildRoverSdf), which already gives correct,
// well-tested body-frame x/y/yaw kinematics from the same cmd_vel topic.
// This plugin only makes the wheels themselves visually steer to point the
// direction of travel and spin at a rate consistent with the commanded
// speed, so the rover LOOKS like it's crab-steering instead of skidding
// sideways with static wheels. Two independent subscribers to the same
// cmd_vel topic is fine -- ROS supports multiple subscribers on one topic.
//
// Steering joints are driven with Joint::SetPosition (a kinematic
// teleport-to-angle) and spin joints with Joint::SetParam("vel"/"fmax", ...)
// (ODE's built-in velocity motor, the same mechanism
// libgazebo_ros_diff_drive itself already uses elsewhere in this codebase)
// -- deliberately, since the goal is a correct-looking animation of wheel
// corners that steer independently of the chassis, not a physically-accurate
// steering suspension model.
//
// robotNamespace matters here: without it, this plugin's ros::NodeHandle()
// resolves "cmd_vel" against the GLOBAL namespace ("/cmd_vel"), while
// planar_move (mounted alongside) constructs its own NodeHandle WITH
// robotNamespace applied and correctly subscribes to "/rover/cmd_vel" --
// omitting it here left this plugin listening on a topic nothing ever
// publishes to, silently doing nothing regardless of how the steering
// joint itself was driven (confirmed live via debug logging before this
// was found: this plugin's own OnUpdate read vx=vy=0 on every call, no
// matter what was actually published). See generate_model_sdf.py's own
// crab_steer_controller plugin block for the matching <robotNamespace>.

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <string>
#include <vector>

namespace gazebo
{

class CrabSteerPlugin : public ModelPlugin
{
public:
  CrabSteerPlugin() : wheel_radius_(0.1), vx_(0.0), vy_(0.0), last_steer_angle_(0.0) {}

  ~CrabSteerPlugin() override
  {
    update_connection_.reset();
    if (rosnode_)
    {
      queue_.clear();
      queue_.disable();
      rosnode_->shutdown();
      callback_queue_thread_.join();
    }
  }

  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override
  {
    model_ = parent;

    std::string command_topic = "cmd_vel";
    if (sdf->HasElement("commandTopic"))
    {
      command_topic = sdf->Get<std::string>("commandTopic");
    }
    std::string robot_namespace = "";
    if (sdf->HasElement("robotNamespace"))
    {
      robot_namespace = sdf->Get<std::string>("robotNamespace");
    }
    if (sdf->HasElement("wheelRadius"))
    {
      wheel_radius_ = sdf->Get<double>("wheelRadius");
    }
    if (sdf->HasElement("spinMaxForce"))
    {
      spin_max_force_ = sdf->Get<double>("spinMaxForce");
    }

    // Repeated <wheel steerJoint="..." spinJoint="..."/> elements, one per
    // wheel corner -- looked up by name against joints this SAME model
    // already declares (the steer-hub revolute + the pre-existing wheel
    // spin revolute, see buildRoverSdf), not created here.
    if (sdf->HasElement("wheel"))
    {
      sdf::ElementPtr wheelElem = sdf->GetElement("wheel");
      while (wheelElem)
      {
        std::string steerName = wheelElem->Get<std::string>("steerJoint");
        std::string spinName = wheelElem->Get<std::string>("spinJoint");
        physics::JointPtr steer = model_->GetJoint(steerName);
        physics::JointPtr spin = model_->GetJoint(spinName);
        if (steer && spin)
        {
          WheelJoints wj;
          wj.steer = steer;
          wj.spin = spin;
          wheels_.push_back(wj);
        }
        else
        {
          gzerr << "[nepi_crab_steer_plugin] could not find joint(s) '"
                << steerName << "'/'" << spinName << "' on model '"
                << model_->GetName() << "'\n";
        }
        wheelElem = wheelElem->GetNextElement("wheel");
      }
    }

    if (wheels_.empty())
    {
      gzerr << "[nepi_crab_steer_plugin] no valid <wheel> entries found -- "
            << "plugin will do nothing\n";
      return;
    }

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace));

    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        command_topic, 1,
        boost::bind(&CrabSteerPlugin::cmdVelCallback, this, _1),
        ros::VoidConstPtr(), &queue_);
    vel_sub_ = rosnode_->subscribe(so);
    callback_queue_thread_ = boost::thread(boost::bind(&CrabSteerPlugin::QueueThread, this));

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&CrabSteerPlugin::OnUpdate, this));
  }

private:
  struct WheelJoints
  {
    physics::JointPtr steer;
    physics::JointPtr spin;
  };

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
  }

  void OnUpdate()
  {
    double vx, vy;
    {
      boost::mutex::scoped_lock lock(mutex_);
      vx = vx_;
      vy = vy_;
    }
    const double speed = std::hypot(vx, vy);
    // Hold the last commanded steering angle at rest instead of re-deriving
    // atan2(0, 0) == 0 every tick -- a wheel pointed sideways shouldn't
    // visibly snap back to straight-ahead just because the rover paused.
    if (speed > 0.01)
    {
      last_steer_angle_ = std::atan2(vy, vx);
    }
    const double spin_rate = speed / wheel_radius_;
    for (auto &w : wheels_)
    {
      // Negated -- confirmed live: this joint's own measured Position()
      // comes out inverted relative to a plain atan2(vy, vx) (a commanded
      // +90 degrees, i.e. pure +Y travel, measured back as ~-90 degrees),
      // most likely ODE's own child-relative-to-parent convention for this
      // axis/parent-child pairing, not a bug in the atan2 math itself.
      // Calibrated empirically against the real simulator, not assumed
      // from SDF axis semantics on paper.
      w.steer->SetPosition(0, -last_steer_angle_);
      w.spin->SetParam("vel", 0, spin_rate);
      w.spin->SetParam("fmax", 0, spin_max_force_);
    }
  }

  void QueueThread()
  {
    static const double timeout = 0.01;
    while (rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  boost::shared_ptr<ros::NodeHandle> rosnode_;
  ros::Subscriber vel_sub_;
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  boost::mutex mutex_;

  std::vector<WheelJoints> wheels_;
  double wheel_radius_;
  double spin_max_force_ = 5.0;
  double vx_;
  double vy_;
  double last_steer_angle_;
};

GZ_REGISTER_MODEL_PLUGIN(CrabSteerPlugin)

}  // namespace gazebo
