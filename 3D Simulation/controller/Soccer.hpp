#ifndef SOCCER_HPP
#define SOCCER_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/GPS.hpp>
#include <map>
#include <string>

namespace managers {
  class RobotisOp2MotionManager;
  class RobotisOp2GaitManager;
  class RobotisOp2VisionManager;
}

namespace webots {
  class Motor;
  class LED;
  class Camera;
  class Accelerometer;
  class PositionSensor;
  class Gyro;
  class Speaker;
}

enum Role {
  ROLE_ATTACKER,
  ROLE_MIDFIELDER,
  ROLE_DEFENDER,
  ROLE_GOALIE
};

struct RobotInfo {
  std::string name;
  double distance;
  double gpsX, gpsY, gpsZ;
  double battery;
  bool fallenFront;
  bool fallenBack;
  Role role;
  RobotInfo()
    : distance(-1), gpsX(0), gpsY(0), gpsZ(0),
      battery(100.0), fallenFront(false), fallenBack(false),
      role(ROLE_MIDFIELDER) {}
};

class Soccer : public webots::Robot {
public:
  Soccer();
  virtual ~Soccer();
  void run();

private:
  int mTimeStep;
  void myStep();
  void wait(int ms);
  bool getBallCenter(double &x, double &y);

  // Motors, sensors
  webots::Motor *mMotors[NMOTORS];
  webots::PositionSensor *mPositionSensors[NMOTORS];
  webots::LED *mEyeLED;
  webots::LED *mHeadLED;
  webots::LED *mBackLedRed;
  webots::LED *mBackLedGreen;
  webots::LED *mBackLedBlue;
  webots::Camera *mCamera;
  webots::Accelerometer *mAccelerometer;
  webots::Gyro *mGyro;
  webots::Speaker *mSpeaker;
  webots::Compass *mCompass;;
  managers::RobotisOp2MotionManager *mMotionManager;
  managers::RobotisOp2GaitManager   *mGaitManager;
  managers::RobotisOp2VisionManager *mVisionManager;

  webots::Emitter  *mEmitter;
  webots::Receiver *mReceiver;
  webots::GPS      *mGPS;

  std::string mRobotName;
  double      mBattery;

  std::map<std::string, RobotInfo> mTeamData;
  void goToPoint(double &px, double &py, double targetX ,double targetY);
  void parseMessage(const std::string &msg);
  void assignRolesExactly4();
  void doAttackerBehavior(double &px, double &py);
  void doMidFielderBehavior(double &px, double &py);
  void doDefenderBehavior(double &px, double &py);
  void doGoalieBehavior(double &px, double &py);

};

#endif
