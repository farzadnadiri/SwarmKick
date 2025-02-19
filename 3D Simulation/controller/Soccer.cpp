#include "Soccer.hpp"

#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <RobotisOp2VisionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Emitter.hpp>
#include <webots/GPS.hpp>
#include <webots/Receiver.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Speaker.hpp>
#include <webots/Compass.hpp>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>

using namespace webots;
using namespace managers;
using namespace std;

// clamp helper
static double clamp(double value, double minv, double maxv)
{
  if (minv > maxv)
  {
    assert(0);
    return value;
  }
  if (value < minv)
    return minv;
  if (value > maxv)
    return maxv;
  return value;
}

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

static const char *motorNames[NMOTORS] = {
    "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
    "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
    "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
    "AnkleL", "FootR", "FootL", "Neck", "Head"};

Soccer::Soccer() : Robot()
{
  mTimeStep = getBasicTimeStep();
  mRobotName = getName();
  mBattery = 100.0;

  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  if (mHeadLED)
    mHeadLED->set(0x00FF00);
  mBackLedRed = getLED("BackLedRed");
  mBackLedGreen = getLED("BackLedGreen");
  mBackLedBlue = getLED("BackLedBlue");

  mCamera = getCamera("Camera");
  if (mCamera)
    mCamera->enable(2 * mTimeStep);

  mAccelerometer = getAccelerometer("Accelerometer");
  if (mAccelerometer)
    mAccelerometer->enable(mTimeStep);

  mGyro = getGyro("Gyro");
  if (mGyro)
    mGyro->enable(mTimeStep);

  mSpeaker = getSpeaker("Speaker");

  // motors
  for (int i = 0; i < NMOTORS; i++)
  {
    mMotors[i] = getMotor(motorNames[i]);
    std::string sName = motorNames[i];
    sName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sName);
    if (mPositionSensors[i])
      mPositionSensors[i]->enable(mTimeStep);

    if (mMotors[i])
    {
      minMotorPositions[i] = mMotors[i]->getMinPosition();
      maxMotorPositions[i] = mMotors[i]->getMaxPosition();
    }
  }

  // managers
  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
  mGaitManager -> setBalanceEnable(true);
  mVisionManager = new RobotisOp2VisionManager(
      (mCamera ? mCamera->getWidth() : 320),
      (mCamera ? mCamera->getHeight() : 240),
      28, 20, 50, 45, 0, 30);

  // communication
  mEmitter = getEmitter("emitter");
  mReceiver = getReceiver("receiver");
  mCompass = getCompass("compass");
  if (mCompass)
    mCompass->enable(mTimeStep);
  if (mReceiver)
    mReceiver->enable(mTimeStep);

  // gps
  mGPS = getGPS("gps");
  if (mGPS)
    mGPS->enable(mTimeStep);
}

Soccer::~Soccer()
{
  // cleanup
}

void Soccer::myStep()
{
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Soccer::wait(int ms)
{
  double start = getTime();
  double sec = (double)ms / 1000.0;
  while (getTime() < start + sec)
  {
    myStep();
  }
}

bool Soccer::getBallCenter(double &x, double &y)
{
  if (!mCamera || !mVisionManager)
    return false;
  int width = mCamera->getWidth();
  int height = mCamera->getHeight();
  const unsigned char *img = mCamera->getImage();
  bool found = mVisionManager->getBallCenter(x, y, img);
  if (!found)
  {
    x = 0;
    y = 0;
    return false;
  }
  x = 2.0 * x / width - 1.0;
  y = 2.0 * y / height - 1.0;
  return true;
}

// parse a message
void Soccer::parseMessage(const std::string &msg)
{
  RobotInfo info;

  std::istringstream iss(msg);
  std::string token;
  if (!(iss >> token))
    return;
  info.name = token;

  while (iss >> token)
  {
    if (token.find(":dist:") != std::string::npos)
    {
      size_t pos = token.find(":dist:");
      std::string val = token.substr(pos + 6);
      info.distance = std::stod(val);
    }
    else if (token.find(":gpsX:") != std::string::npos)
    {
      size_t pos = token.find(":gpsX:");
      std::string val = token.substr(pos + 6);
      info.gpsX = std::stod(val);
    }
    else if (token.find(":gpsY:") != std::string::npos)
    {
      size_t pos = token.find(":gpsY:");
      std::string val = token.substr(pos + 6);
      info.gpsY = std::stod(val);
    }
    else if (token.find(":gpsZ:") != std::string::npos)
    {
      size_t pos = token.find(":gpsZ:");
      std::string val = token.substr(pos + 6);
      info.gpsZ = std::stod(val);
    }
    else if (token.find(":battery:") != std::string::npos)
    {
      size_t pos = token.find(":battery:");
      std::string val = token.substr(pos + 9);
      info.battery = std::stod(val);
    }
    else if (token.find(":fallen:") != std::string::npos)
    {
      size_t pos = token.find(":fallen:");
      std::string val = token.substr(pos + 8);
      if (val == "front")
      {
        info.fallenFront = true;
        info.fallenBack = false;
      }
      else if (val == "back")
      {
        info.fallenFront = false;
        info.fallenBack = true;
      }
      else
      {
        info.fallenFront = false;
        info.fallenBack = false;
      }
    }
  }
  mTeamData[info.name] = info;
}

// EXACT 4 roles, one attacker, one goalie, one defender, one mid
void Soccer::assignRolesExactly4()
{
  if (mTeamData.size() < 4)
  {
    std::cout << mRobotName << mTeamData.size() << "We do not have 4 robots data yet, skipping role assignment\n";
    return;
  }

  // gather names
  std::vector<std::string> names;
  for (auto &kv : mTeamData)
    names.push_back(kv.first);
  if (names.size() < 4)
    return;

  // step 1: find Attacker
  std::string attacker;
  double minDist = 1e9;
  for (auto &kv : mTeamData)
  {
    const RobotInfo &inf = kv.second;
    if (inf.distance >= 0.0)
    {
      bool fallen = (inf.fallenFront || inf.fallenBack);
      if (!fallen && inf.distance < minDist)
      {
        minDist = inf.distance;
        attacker = kv.first;
      }
    }
  }
  if (attacker.empty())
  {
    double md2 = 1e9;
    for (auto &kv : mTeamData)
    {
      const RobotInfo &inf = kv.second;
      double d = inf.distance;
      if (d >= 0 && d < md2)
      {
        md2 = d;
        attacker = kv.first;
      }
    }
  }
  if (attacker.empty())
  {
    double maxX = -1e9;
    for (auto &kv : mTeamData)
    {
      const RobotInfo &inf = kv.second;
      if (inf.gpsX > maxX)
      {
        maxX = inf.gpsX;
        attacker = kv.first;
      }
    }
  }
  if (!attacker.empty())
  {
    mTeamData[attacker].role = ROLE_ATTACKER;
  }

  // remove attacker
  std::vector<std::string> remain;
  for (auto &nm : names)
  {
    if (nm != attacker)
      remain.push_back(nm);
  }
  if (remain.size() < 3)
    return;

  // step 2: goalie => smallest X
  std::string goalie;
  double minX = 1e9;
  for (auto &nm : remain)
  {
    const RobotInfo &inf = mTeamData[nm];
    if (inf.gpsX < minX)
    {
      minX = inf.gpsX;
      goalie = nm;
    }
  }
  mTeamData[goalie].role = ROLE_GOALIE;

  // remove goalie
  std::vector<std::string> remain2;
  for (auto &nm : remain)
  {
    if (nm != goalie)
      remain2.push_back(nm);
  }
  if (remain2.size() < 2)
    return;

  // step 3 => among last 2 => smaller X => defender, other => mid
  const RobotInfo &inf1 = mTeamData[remain2[0]];
  const RobotInfo &inf2 = mTeamData[remain2[1]];
  if (inf1.gpsX < inf2.gpsX)
  {
    mTeamData[remain2[0]].role = ROLE_DEFENDER;
    mTeamData[remain2[1]].role = ROLE_MIDFIELDER;
  }
  else
  {
    mTeamData[remain2[0]].role = ROLE_MIDFIELDER;
    mTeamData[remain2[1]].role = ROLE_DEFENDER;
  }

  // print roles
  std::cout << "---- Role assignment for 4 robots ----\n";
  for (auto &nm : names)
  {
    Role r = mTeamData[nm].role;
    std::string roleName;
    switch (r)
    {
    case ROLE_ATTACKER:
      roleName = "Attacker";
      break;
    case ROLE_GOALIE:
      roleName = "Goalie";
      break;
    case ROLE_DEFENDER:
      roleName = "Defender";
      break;
    case ROLE_MIDFIELDER:
      roleName = "MidFielder";
      break;
    }
    std::cout << "Robot " << nm << " => " << roleName << "\n";
  }
  std::cout << "---------------------------------------\n";
}

//-------------------------------------------
// ROLE-SPECIFIC functions
//-------------------------------------------
void Soccer::goToPoint(double &px, double &py,double targetX , double targetY)
{
  // Retrieve GPS values (ENU: East, North, Up)
const double *gpsVals = mGPS->getValues();
double gx = gpsVals[0]; // East coordinate
double gy = gpsVals[1]; // North coordinate
double headPosition;

// Retrieve compass values and compute current heading with 0° = North.
const double *compassVals = mCompass->getValues();
// Swap arguments so that 0° is North (i.e., compassVals[1] is the 'forward' component)
double rad = atan2(compassVals[0], compassVals[1]);
double bearing = rad * 180.0 / M_PI;
if (bearing < 0.0)
  bearing += 360.0;
if (bearing >= 360.0)
  bearing -= 360.0;
// Compute the error vector from the current position to the goal
double errorX = targetX - gx;
double errorY = targetY - gy;

// Compute the distance on the ground (horizontal) plane
double distance = sqrt(errorX * errorX + errorY * errorY);
double desiredHeading = atan2(errorY, errorX) * 180.0 / M_PI;
  // Compute the desired heading from the error vector.
  // Swap the arguments to have 0° = North.
  if (desiredHeading < 0.0)
    desiredHeading += 360.0;
  if (desiredHeading >= 360.0)
    desiredHeading -= 360.0;
  // Compute heading error
  double headingError = desiredHeading - bearing;
  
  // Normalize heading error to the range [-180, 180]
  if (headingError > 180)
    headingError -= 360;
  if (headingError < -180)
    headingError += 360;

double threshold = 0.2;
if (distance > threshold) {
  // Determine movement parameters
  double forwardAmp = (distance > 0.2) ? 1.0 : 0.5;
  double turningAmp = 0;
  
  // Adjust turning speed based on heading error magnitude
  if (headingError < -10) {
    turningAmp = -0.5; // Turn right
  } else if (headingError > 10) {
    turningAmp = 0.5; // Turn left
  }
  
  mGaitManager->setXAmplitude(forwardAmp);
  mGaitManager->setAAmplitude(turningAmp);
  mGaitManager->step(mTimeStep);
} else {
      // set eye led to red
      mEyeLED->set(0xFF0000);

      // turn round
      mGaitManager->setXAmplitude(0.0);
      mGaitManager->setAAmplitude(-0.3);
      mGaitManager->step(mTimeStep);

      // move the head vertically
      headPosition = clamp(0.7 * sin(2.0 * getTime()), minMotorPositions[19], maxMotorPositions[19]);
      mMotors[19]->setPosition(headPosition);
}
}
void Soccer::doAttackerBehavior(double &px, double &py) {
  double x, y, neckPosition, headPosition;
  bool ballInFieldOfView = getBallCenter(x, y);

  // Retrieve compass values and compute current heading (0° = North)
  const double *compassVals = mCompass->getValues();
  double rad = atan2(compassVals[0], compassVals[1]);  // swap so 0°=North
  double bearing = rad * 180.0 / M_PI;
  if (bearing < 0.0)
    bearing += 360.0;
  if (bearing >= 360.0)
    bearing -= 360.0;

  // Desired heading is 0° (facing North)
  double desiredHeading = 0.0;
  // (Normalization not really necessary for 0.0, but included for completeness)
  if (desiredHeading < 0.0)
    desiredHeading += 360.0;
  if (desiredHeading >= 360.0)
    desiredHeading -= 360.0;

  // Compute heading error and normalize to [-180, 180]
  double headingError = desiredHeading - bearing;
  if (headingError > 180)
    headingError -= 360;
  if (headingError < -180)
    headingError += 360;

  // Compute turning amplitude based on heading error.
  double turningAmp = 0;
  if (headingError < -10)
    turningAmp = 0.5; // Turn right
  else if (headingError > 10)
    turningAmp = -0.5;  // Turn left

  if (ballInFieldOfView) {
    // Set eye LED to blue (tracking ball)
    mEyeLED->set(0x0000FF);

    // Compute head movement for ball tracking (max 0.015 rad per time step)
    x = 0.015 * x + px;
    y = 0.015 * y + py;
    px = x;
    py = y;
    neckPosition = clamp(-x, minMotorPositions[18], maxMotorPositions[18]);
    headPosition = clamp(-y, minMotorPositions[19], maxMotorPositions[19]);

    // Move forward: faster if ball is far, slower if ball is near.
    if (y < 0.1)
      mGaitManager->setXAmplitude(1.0);
    else
      mGaitManager->setXAmplitude(0.5);

    // Use head direction for some turning (this can be tuned as needed)
    mGaitManager->setAAmplitude(neckPosition);
    mGaitManager->step(mTimeStep);

    // Update head motors.
    mMotors[18]->setPosition(neckPosition);
    mMotors[19]->setPosition(headPosition);

    // When the ball is very close, align with desired heading before kicking.
    if (y > 0.20) { // ball is very close, prepare to kick
    std::cout<<"// ball is very close, prepare to kick"<<std::endl;
      const double HEADING_TOLERANCE = 15.0; // degrees tolerance for alignment

      if (fabs(headingError) > HEADING_TOLERANCE) {
        // Not yet aligned: stop forward motion and turn in place.
        mGaitManager->setXAmplitude(0.0);
        mGaitManager->setAAmplitude(turningAmp);
        mGaitManager->step(mTimeStep);
        std::cout << "Aligning: Bearing = " << bearing 
                  << "°, Error = " << headingError << "°" << std::endl;
        return;  // exit this cycle and try again next time step
      }
      // Once aligned (within tolerance), kick the ball.

      mGaitManager->stop();
      wait(500);
      // Set eye LED to green to indicate kicking.
      mEyeLED->set(0x00FF00);
      if (x < 0.0)
        mMotionManager->playPage(13);  // left kick
      else
        mMotionManager->playPage(12);  // right kick
      mMotionManager->playPage(9);       // return to walk-ready position
      mGaitManager->start();
      // Reset head offset accumulators.
      px = 0.0;
      py = 0.0;
    }
    else if (y > 0.10 && y <= 0.20) {
      // When the ball is moderately close, you can adjust turning using the heading error.
    std::cout<<"// When the ball is moderately close, adjust turning using heading error"<<std::endl;

      mGaitManager->setAAmplitude(-1*turningAmp);
      mGaitManager->setYAmplitude(turningAmp);
    }
  }
  else {

    // If ball is not in field of view, set LED to red and turn to search.
    mEyeLED->set(0xFF0000);
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(-0.3);
    mGaitManager->step(mTimeStep);
    // Move head vertically for scanning.
    headPosition = clamp(0.7 * sin(2.0 * getTime()), minMotorPositions[19], maxMotorPositions[19]);
    mMotors[19]->setPosition(headPosition);
  }
}


void Soccer::doMidFielderBehavior(double &px, double &py)
{
  goToPoint(px,py,-1.0,0.0);
}

void Soccer::doDefenderBehavior(double &px, double &py)
{
  goToPoint(px,py,-1.6,-1.5);
}

void Soccer::doGoalieBehavior(double &px, double &py)
{
  goToPoint(px,py,-3.8,0.0);
}



void Soccer::run()
{
  static ofstream logFile(mRobotName + " log.txt", ios::app);

  myStep();

  if (mEyeLED)
    mEyeLED->set(0x00FF00);

  mMotionManager->playPage(1);
  mMotionManager->playPage(24);
  mMotionManager->playPage(9);
  wait(200);

  mGaitManager->start();
  mGaitManager->step(mTimeStep);

  double px = 0.0, py = 0.0;
  int fup = 0, fdown = 0;
  const double acc_tolerance = 90.0;
  const double acc_step = 10;

  while (true)
  {
    // battery drain
    if (mBattery > 0.0)
    {
      mBattery -= 0.001;
      if (mBattery < 0.0)
        mBattery = 0.0;
    }

    double x, y;
    bool ballFound = getBallCenter(x, y);
    double dist = -1.0;
    if (ballFound)
    {
      dist = 1.0 / (0.1 + y + 0.01);
      if (dist < 0)
        dist = 0.0;
    }

    double gx = 0, gy = 0, gz = 0;
    if (mGPS)
    {
      const double *vals = mGPS->getValues();
      gx = vals[0];
      gy = vals[1];
      gz = vals[2];
    }

    bool faceFallen = false, backFallen = false;
    if (mAccelerometer)
    {
      const double *acc = mAccelerometer->getValues();
      if (acc[1] < 512.0 - acc_tolerance)
        fup++;
      else
        fup = 0;
      if (acc[1] > 512.0 + acc_tolerance)
        fdown++;
      else
        fdown = 0;
      if (fup > acc_step)
        faceFallen = true;
      if (fdown > acc_step)
        backFallen = true;
    }

    // get up
    if (faceFallen)
    {
      mGaitManager->stop();
      mMotionManager->playPage(1);
      mMotionManager->playPage(10);
      mMotionManager->playPage(9);
      fup = 0;
      mGaitManager->start();
      mGaitManager->step(mTimeStep);
      continue;
    }
    else if (backFallen)
    {
      mGaitManager->stop();
      mMotionManager->playPage(1);
      mMotionManager->playPage(11);
      mMotionManager->playPage(9);
      fdown = 0;
      mGaitManager->start();
      mGaitManager->step(mTimeStep);
      continue;
    }
    std::string fallenStatus = "none";
    if (faceFallen)
      fallenStatus = "front";
    else if (backFallen)
      fallenStatus = "back";


    std::cout << "status: " << fallenStatus;
    // broadcast
    if (mEmitter)
    {
      char buf[256];
      snprintf(buf, 256, "%s :dist:%.2f :gpsX:%.2f :gpsY:%.2f :gpsZ:%.2f :battery:%.2f :fallen:%s",
               mRobotName.c_str(), dist, gx, gy, gz, mBattery, fallenStatus.c_str());
      mEmitter->send(buf, std::strlen(buf) + 1);
    }

    // receive & parse
    if (mReceiver && mReceiver->getQueueLength() > 0)
    {
      while (mReceiver->getQueueLength() > 0)
      {
        const void *data = mReceiver->getData();
        const char *msg = (const char *)data;
        parseMessage(msg);
        logFile << "[t=" << getTime() << "] " << mRobotName << " received: " << msg << "\n";
        mReceiver->nextPacket();
      }
    }

    // store local data
    {
      RobotInfo self;
      self.name = mRobotName;
      self.distance = dist;
      self.gpsX = gx;
      self.gpsY = gy;
      self.gpsZ = gz;
      self.battery = mBattery;
      self.fallenFront = faceFallen;
      self.fallenBack = backFallen;
      mTeamData[mRobotName] = self;
    }

    // role assignment
    assignRolesExactly4();

    // pick our role
    Role myRole = mTeamData[mRobotName].role;
    if (!faceFallen && !backFallen)
    {
      switch (myRole)
      {
      case ROLE_ATTACKER:
        doAttackerBehavior(px, py);
        break;
      case ROLE_MIDFIELDER:
        doMidFielderBehavior(px, py);
        break;
      case ROLE_DEFENDER:
        doDefenderBehavior(px, py);
        break;
      case ROLE_GOALIE:
        doGoalieBehavior(px, py);
        break;
      }
    }
    myStep();
  }
}
