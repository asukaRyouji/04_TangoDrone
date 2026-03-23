#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

#include <fstream>
#include <functional>
#include <iomanip>
#include <string>
#include <ctime>
#include <sstream>
#include <cmath>

namespace gazebo
{
class PoseLoggerPlugin : public WorldPlugin
{
public:
  PoseLoggerPlugin() = default;

  ~PoseLoggerPlugin() override
  {
    if (this->logFile.is_open())
      this->logFile.close();
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    this->world = _world;

    // Default parameters
    this->droneModelName = "bebop";
    this->flowerModelName = "moving_flower";
    this->logDirectory = "/home/admin_user/Desktop/04_TangoDrone/Simulation_data";
    this->logRate = 100.0;  // Hz

    // Optional SDF overrides
    if (_sdf->HasElement("drone_model_name"))
      this->droneModelName = _sdf->Get<std::string>("drone_model_name");

    if (_sdf->HasElement("flower_model_name"))
      this->flowerModelName = _sdf->Get<std::string>("flower_model_name");

    if (_sdf->HasElement("log_path"))
      this->logDirectory = _sdf->Get<std::string>("log_path");

    if (_sdf->HasElement("log_rate"))
      this->logRate = _sdf->Get<double>("log_rate");

    // Generate timestamped filename
    std::time_t now_c = std::time(nullptr);
    std::tm *tm_ptr = std::localtime(&now_c);

    std::ostringstream filename;
    filename << this->logDirectory;
    if (!this->logDirectory.empty() && this->logDirectory.back() != '/')
      filename << "/";
    filename << "gazebo_pose_log_"
             << std::put_time(tm_ptr, "%Y-%m-%d_%H-%M-%S")
             << ".csv";

    this->logPath = filename.str();

    this->droneModel = this->world->ModelByName(this->droneModelName);
    this->flowerModel = this->world->ModelByName(this->flowerModelName);

    if (!this->droneModel)
      gzerr << "[PoseLoggerPlugin] Could not find drone model: "
            << this->droneModelName << "\n";

    if (!this->flowerModel)
      gzerr << "[PoseLoggerPlugin] Could not find flower model: "
            << this->flowerModelName << "\n";

    this->logFile.open(this->logPath);
    if (!this->logFile.is_open())
    {
      gzerr << "[PoseLoggerPlugin] Failed to open log file: "
            << this->logPath << "\n";
      return;
    }

    // CSV header
    this->logFile
      << "t,"
      << "drone_x,drone_y,drone_z,"
      << "drone_vel_x,drone_vel_y,drone_vel_z,"
      << "drone_roll,drone_pitch,drone_yaw,"
      << "flower_x,flower_y,flower_z,"
      << "flower_roll,flower_pitch,flower_yaw,"
      << "dx,dy,dz,distance\n";

    this->lastLogTime = this->world->SimTime();
    this->logPeriod = (this->logRate > 0.0) ? 1.0 / this->logRate : 0.01;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PoseLoggerPlugin::OnUpdate, this));

    gzmsg << "[PoseLoggerPlugin] Logging to: " << this->logPath << "\n";
    gzmsg << "[PoseLoggerPlugin] Drone model: " << this->droneModelName << "\n";
    gzmsg << "[PoseLoggerPlugin] Flower model: " << this->flowerModelName << "\n";
    gzmsg << "[PoseLoggerPlugin] Log rate: " << this->logRate << " Hz\n";
  }

private:
  void OnUpdate()
  {
    if (!this->logFile.is_open())
      return;

    // In case models were not ready at Load time, try again
    if (!this->droneModel)
      this->droneModel = this->world->ModelByName(this->droneModelName);

    if (!this->flowerModel)
      this->flowerModel = this->world->ModelByName(this->flowerModelName);

    if (!this->droneModel || !this->flowerModel)
      return;

    common::Time now = this->world->SimTime();
    const double dt = (now - this->lastLogTime).Double();

    if (dt < this->logPeriod)
      return;

    this->lastLogTime = now;

    ignition::math::Pose3d dronePose = this->droneModel->WorldPose();
    ignition::math::Vector3d droneVel = this->droneModel->WorldLinearVel();
    ignition::math::Pose3d flowerPose = this->flowerModel->WorldPose();

    const double dx = flowerPose.Pos().X() - dronePose.Pos().X();
    const double dy = flowerPose.Pos().Y() - dronePose.Pos().Y();
    const double dz = flowerPose.Pos().Z() - dronePose.Pos().Z();
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    this->logFile << std::fixed << std::setprecision(6)
                  << now.Double() << ","

                  << dronePose.Pos().X() << ","
                  << dronePose.Pos().Y() << ","
                  << dronePose.Pos().Z() << ","
                  << droneVel.X() << ","
                  << droneVel.Y() << ","
                  << droneVel.Z() << ","
                  << dronePose.Rot().Roll() << ","
                  << dronePose.Rot().Pitch() << ","
                  << dronePose.Rot().Yaw() << ","

                  << flowerPose.Pos().X() << ","
                  << flowerPose.Pos().Y() << ","
                  << flowerPose.Pos().Z() << ","
                  << flowerPose.Rot().Roll() << ","
                  << flowerPose.Rot().Pitch() << ","
                  << flowerPose.Rot().Yaw() << ","

                  << dx << ","
                  << dy << ","
                  << dz << ","
                  << distance
                  << "\n";
  }

private:
  physics::WorldPtr world;
  physics::ModelPtr droneModel;
  physics::ModelPtr flowerModel;
  event::ConnectionPtr updateConnection;

  std::ofstream logFile;

  std::string droneModelName;
  std::string flowerModelName;
  std::string logDirectory;
  std::string logPath;

  double logRate = 100.0;
  double logPeriod = 0.01;
  common::Time lastLogTime;
};

GZ_REGISTER_WORLD_PLUGIN(PoseLoggerPlugin)
}