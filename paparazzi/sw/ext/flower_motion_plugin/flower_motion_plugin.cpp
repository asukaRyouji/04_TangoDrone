#include <cmath>
#include <string>
#include <functional>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
class FlowerMotionPlugin : public ModelPlugin
{
public:
  FlowerMotionPlugin() = default;

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model = _model;
    this->world = _model->GetWorld();

    ignition::math::Pose3d pose = this->model->WorldPose();
    this->xLeft = pose.Pos().X();
    this->yLeft = pose.Pos().Y();
    this->zLeft = pose.Pos().Z();
    this->roll0 = pose.Rot().Roll();
    this->pitch0 = pose.Rot().Pitch();
    this->yaw0 = pose.Rot().Yaw();

    if (_sdf->HasElement("amplitude"))
      this->amplitude = _sdf->Get<double>("amplitude");

    if (_sdf->HasElement("frequency"))
      this->frequency = _sdf->Get<double>("frequency");

    if (_sdf->HasElement("axis"))
      this->axis = _sdf->Get<std::string>("axis");

    if (_sdf->HasElement("start_delay"))
      this->startDelay = _sdf->Get<double>("start_delay");

    this->loadTime = this->world->SimTime();

    // Keep the initial pose exactly as the leftmost pose
    this->SetPoseFromOffset(0.0);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&FlowerMotionPlugin::OnUpdate, this));
  }

private:
  double RelativeOffset(double t) const
  {
    // Starts from leftmost position at t=0:
    // offset = A * (1 - cos(2*pi*f*t))
    return this->amplitude * (1.0 - std::cos(2.0 * M_PI * this->frequency * t));
  }

  void SetPoseFromOffset(double offset)
  {
    double x = this->xLeft;
    double y = this->yLeft;
    double z = this->zLeft;

    if (this->axis == "x")
      x = this->xLeft + offset;
    else if (this->axis == "y")
      y = this->yLeft + offset;
    else if (this->axis == "z")
      z = this->zLeft + offset;

    ignition::math::Pose3d newPose(
        x, y, z,
        this->roll0, this->pitch0, this->yaw0);

    this->model->SetWorldPose(newPose);
  }

  void OnUpdate()
  {
    const common::Time currentTime = this->world->SimTime();
    const double elapsed = (currentTime - this->loadTime).Double();

    if (elapsed < this->startDelay)
    {
      this->SetPoseFromOffset(0.0);
      return;
    }

    const double motionTime = elapsed - this->startDelay;
    this->SetPoseFromOffset(this->RelativeOffset(motionTime));
  }

private:
  physics::ModelPtr model;
  physics::WorldPtr world;
  event::ConnectionPtr updateConnection;

  common::Time loadTime;

  double amplitude = 0.3;
  double frequency = 0.25;
  double startDelay = 5.0;

  double xLeft = 0.0;
  double yLeft = 0.0;
  double zLeft = 0.0;
  double roll0 = 0.0;
  double pitch0 = 0.0;
  double yaw0 = 0.0;

  std::string axis = "x";
};

GZ_REGISTER_MODEL_PLUGIN(FlowerMotionPlugin)
}