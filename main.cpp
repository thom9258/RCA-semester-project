#include <fl/Headers.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <iostream>
#include <opencv2/opencv.hpp>

static boost::mutex mutex;

float closestObstacle_range;
float closestObstalce_angle;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      //      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
      //                << _msg->pose(i).position().x() << std::setw(6)
      //                << _msg->pose(i).position().y() << std::setw(6)
      //                << _msg->pose(i).position().z() << std::setw(6)
      //                << _msg->pose(i).orientation().w() << std::setw(6)
      //                << _msg->pose(i).orientation().x() << std::setw(6)
      //                << _msg->pose(i).orientation().y() << std::setw(6)
      //                << _msg->pose(i).orientation().z() << std::endl;
    }
  }
}

void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, cv::COLOR_RGB2BGR);

  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}

void lidarCallback(ConstLaserScanStampedPtr &msg) {
  //  std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  closestObstacle_range = range_max;
  closestObstalce_angle = 361;

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = 400;
  int height = 400;
  float px_per_m = 200 / range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);
  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);
    cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                        200.5f - range_min * px_per_m * std::sin(angle));
    cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                      200.5f - range * px_per_m * std::sin(angle));
    cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
             cv::LINE_AA, 4);
    if (range <= closestObstacle_range) {
      closestObstacle_range = range;
      closestObstalce_angle = angle;
    }
  }
  std::cout << "Angle: " << closestObstalce_angle
            << " Range: " << closestObstacle_range << std::endl;
  cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
}

/*******************************************************************************
 << VARIABLES HERE >>
*******************************************************************************/
const bool manual_control = false;

int main(int _argc, char **_argv) {
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", poseCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  gazebo::msgs::WorldControl controlMessage;
  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);

  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  const int key_esc = 27;
  const int key_space = 32;
  const int key_w = 119;
  const int key_a = 97;
  const int key_s = 115;
  const int key_d = 100;

  float speed = 0.0;
  float dir = 0.0;

  // ---- Fuzzy Engine ----
  using namespace fl;

  Engine *engine = new Engine;
  engine->setName("ObstacleAvoidance");
  engine->setDescription("");

  InputVariable *obstacle = new InputVariable;
  obstacle->setName("obstacle");
  obstacle->setDescription("");
  obstacle->setEnabled(true);
  obstacle->setRange(0.000, 1.000);
  obstacle->setLockValueInRange(false);
  obstacle->addTerm(new Ramp("left", 1.000, 0.000));
  obstacle->addTerm(new Ramp("right", 0.000, 1.000));
  engine->addInputVariable(obstacle);

  OutputVariable *fl_dir = new OutputVariable;
  fl_dir->setName("fl_dir");
  fl_dir->setDescription("");
  fl_dir->setEnabled(true);
  fl_dir->setRange(-0.4, 0.400);
  fl_dir->setLockValueInRange(false);
  fl_dir->setAggregation(new Maximum);
  fl_dir->setDefuzzifier(new Centroid(100));
  fl_dir->setDefaultValue(fl::nan);
  fl_dir->setLockPreviousValue(false);
  fl_dir->addTerm(new Ramp("left", 1.000, 0.000));
  fl_dir->addTerm(new Ramp("right", 0.000, 1.000));
  engine->addOutputVariable(fl_dir);

  RuleBlock *mamdani = new RuleBlock;
  mamdani->setName("mamdani");
  mamdani->setDescription("");
  mamdani->setEnabled(true);
  mamdani->setConjunction(fl::null);
  mamdani->setDisjunction(fl::null);
  mamdani->setImplication(new AlgebraicProduct);
  mamdani->setActivation(new General);
  mamdani->addRule(
      Rule::parse("if obstacle is left then fl_dir is right", engine));
  mamdani->addRule(
      Rule::parse("if obstacle is right then fl_dir is left", engine));
  engine->addRuleBlock(mamdani);

  std::string status;
  if (not engine->isReady(&status))
    throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
  // ---- -----
  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();
    if (manual_control) {
      /*************************************************************************
       * MANUAL CONTROLLER
       ************************************************************************/
      if (key == key_esc)
        break;

      if ((key == key_up || key == key_w) && (speed <= 1.2f))
        speed += 0.05;
      else if ((key == key_down || key == key_s) && (speed >= -1.2f))
        speed -= 0.05;
      else if ((key == key_right || key == key_d) && (dir <= 0.4f))
        dir += 0.05;
      else if ((key == key_left || key == key_a) && (dir >= -0.4f))
        dir -= 0.05;
      else if (key == key_space) {
        speed = 0;
        dir = 0;
      } else {
        // slow down
        //      speed *= 0.1;
        //      dir *= 0.1;
      }
    } else {
      /*************************************************************************
       * FUZZY CONTROLLER
       ************************************************************************/
      //      scalar location = obstacle->getMinimum() + (obstacle->range() /
      //      50);
      obstacle->setValue(closestObstacle_range);
      engine->process();
      // FL_LOG("obstacle.input = " << Op::str(location) <<
      //    " => " << "steer.output = " << Op::str(fl_dir->getValue()));

      dir = fl_dir->getValue();
      //---------
    }
    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
