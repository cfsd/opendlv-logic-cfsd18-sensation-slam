/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef SLAM_HPP
#define SLAM_HPP

#include <tuple>
#include <utility>
#include <thread>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include <Eigen/Dense>
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "cone.hpp"
#include "cvcones.hpp"


class Slam {


private:
 Slam(const Slam &) = delete;
 Slam(Slam &&)      = delete;
 Slam &operator=(const Slam &) = delete;
 Slam &operator=(Slam &&) = delete;
 typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;
public:
  Slam(std::map<std::string, std::string> commandlineArguments,cluon::OD4Session &a_od4);
  ~Slam();
  void nextPose(cluon::data::Envelope data);
  void nextSplitPose(cluon::data::Envelope data);
  void nextYawRate(cluon::data::Envelope data);
  void nextGroundSpeed(cluon::data::Envelope data);
  std::vector<Cone> drawCones();
  std::vector<Cone> drawRawCones();
  std::vector<Cone> drawLocalOptimizedCones();
  void setStateMachineStatus(cluon::data::Envelope data);
  bool getModuleState();
  std::vector<Eigen::Vector3d> drawPoses();
  Eigen::Vector3d drawCurrentPose();
  Eigen::Vector3d drawCurrentUKFPose();
  std::vector<std::vector<int>> drawGraph();
  void recieveCombinedMessage(cluon::data::TimeStamp currentFrameTime,std::map<int,ConePackage> currentFrame);
  void recieveCombinedCvMessage(cluon::data::TimeStamp currentFrameTime,std::map<int,ConePackage> currentFrame);
  std::vector<std::vector<int>> getPermutations(int n);
  void initializeModule();
  uint16_t getMapSize();

 private:
  void setUp(std::map<std::string, std::string> commandlineArguments);
  void setupOptimizer();
  void tearDown();
  bool isKeyframe();
  void addOdometryMeasurement(Eigen::Vector3d pose,uint32_t i);
  void fullBA();
  Eigen::Vector3d updatePoseFromGraph();
  void addPosesToGraph();
  void performSLAM(Eigen::MatrixXd Cones);
  void localizer(std::vector<std::pair<int,Eigen::Vector3d>>, Eigen::Vector3d pose);
  void createConnections(Eigen::MatrixXd cones, Eigen::Vector3d pose);
  void createFullGraph();
  void optimizeEssentialGraph(uint32_t graphIndexStart, uint32_t graphIndexEnd);
  void updateFromEssential(uint32_t poseStart, uint32_t poseEnd,uint32_t coneStart,uint32_t coneEnd, g2o::SparseOptimizer &essentialGraph);
  Eigen::Vector3d coneToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd Cone);
  int updateCurrentCone(Eigen::Vector3d pose,uint32_t currentConeIndex, uint32_t remainingIter);

  Eigen::Vector2d transformConeToCoG(double angle, double distance);
  Eigen::Vector3d Spherical2Cartesian(double azimuth, double zenimuth, double distance);
  Eigen::Vector3d Cartesian2Spherical(double x, double y, double z);
  void addConeMeasurements(int i);
  Eigen::Vector2d getConeToPoseMeasurement(int i, int j);
  Eigen::Vector2d getLocalConeToPoseMeasurement(Eigen::Vector3d pose, Eigen::Vector2d cone);
  void addConesToGraph();
  double distanceBetweenCones(Cone c1, Cone c2);
  double distanceBetweenConesOpt(Cone c1, Cone c2);
  void updateMap(uint32_t start, uint32_t end, bool updateToGlobal);
  void filterMap();
  double optimizeHeading(Eigen::MatrixXd cones,Eigen::Vector3d pose);
  std::vector<std::pair<int,Eigen::Vector3d>> matchCones(Eigen::MatrixXd cones,Eigen::Vector3d &pose);
  std::pair<double,std::vector<uint32_t>> evaluatePose(Eigen::MatrixXd cones, Eigen::Vector3d pose, std::vector<uint32_t> inMapIndex, uint32_t &conesThatFit);
  std::vector<std::pair<int,Eigen::Vector3d>> filterMatch(Eigen::MatrixXd cones, Eigen::Vector3d pose,std::pair<double,std::vector<uint32_t>> matchedCones);
  bool localizable(std::vector<std::pair<int,Eigen::Vector3d>> matchedCones);
  bool checkLocalization();
  void sendCones();
  void sendPose();
  void SendCvCones(std::vector<Cone> cones);
  void writeToPoseAndMapFile();



  /*Member variables*/
  cluon::OD4Session &od4;
  g2o::SparseOptimizer m_optimizer;
  int32_t m_timeDiffMilliseconds = 110;
  cluon::data::TimeStamp m_lastTimeStamp;
  cluon::data::TimeStamp m_lastCvTimeStamp;
  Eigen::MatrixXd m_coneCollector;
  uint32_t m_lastObjectId;
  std::mutex m_coneMutex;
  std::mutex m_sensorMutex;
  std::mutex m_mapMutex;
  std::mutex m_optimizerMutex;
  std::mutex m_yawMutex;
  std::mutex m_groundSpeedMutex;
  std::mutex m_stateMachineMutex;
  Eigen::Vector3d m_odometryData;
  std::array<double,2> m_gpsReference;
  std::vector<Cone> m_map;
  std::vector<Cone> m_essentialMap = {};
  std::vector<Eigen::Vector3d> m_poses = {};
  std::vector<std::vector<int>> m_connectivityGraph = {};
  double m_newConeThreshold= 1;
  cluon::data::TimeStamp m_keyframeTimeStamp;
  double m_timeBetweenKeyframes = 0.5;
  double m_coneMappingThreshold = 67;
  uint32_t m_currentConeIndex = 0;
  int m_currentConeDiff = 0;
  int m_lapSize = 50;
  int m_poseId = 1000;
  int m_coneRef = 0;
  double m_xOffset = 0;
  double m_yOffset = 0;
  double m_headingOffset = 0;
  uint32_t m_conesPerPacket = 20;
  bool m_sendConeData = false;
  bool m_sendPoseData = false;
  bool m_newFrame;
  bool m_loopClosing = false;
  bool m_loopClosingComplete = false;
  Eigen::Vector3d m_sendPose;
  std::mutex m_sendMutex;
  uint32_t m_senderStamp = 0;
  float m_yawRate = 0.0f;
  float m_groundSpeed = 0.0f;
  bool m_gpsCoords = false;
  cluon::data::TimeStamp m_yawReceivedTime = {};
  cluon::data::TimeStamp m_groundSpeedReceivedTime = {};
  cluon::data::TimeStamp m_geolocationReceivedTime ={};
  std::vector<Cone> m_coneList = {};
  bool m_filterMap = false;
  bool m_readyState = false;
  bool m_readyStateMachine = true;
  CVCones m_cvCones;
  
    // Constants for degree transformation
  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
  const double PI = 3.14159265f;
};


#endif
