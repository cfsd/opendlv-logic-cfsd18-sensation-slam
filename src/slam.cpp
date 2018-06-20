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

#include <iostream>

#include "slam.hpp"
#include "WGS84toCartesian.hpp"

Slam::Slam(std::map<std::string, std::string> commandlineArguments,cluon::OD4Session &a_od4) :
  od4(a_od4)
, m_optimizer()
, m_lastTimeStamp()
, m_coneCollector()
, m_lastObjectId()
, m_coneMutex()
, m_sensorMutex()
, m_mapMutex()
, m_optimizerMutex()
, m_yawMutex()
, m_odometryData()
, m_gpsReference()
, m_map()
, m_keyframeTimeStamp(cluon::time::now())
, m_newFrame()
, m_sendPose()
, m_sendMutex()
{
  setupOptimizer();
  setUp(commandlineArguments);
  m_coneCollector = Eigen::MatrixXd::Zero(4,100);
  m_lastObjectId = 0;
  m_odometryData << 0,0,0;
  m_sendPose << 0,0,0;
  m_newFrame = true;
}

void Slam::setupOptimizer(){

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > slamBlockSolver;
  typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> slamLinearSolver;
  
  auto linearSolver = g2o::make_unique<slamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  
  g2o::OptimizationAlgorithmGaussNewton* algorithmType = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<slamBlockSolver>(std::move(linearSolver)));
  m_optimizer.setAlgorithm(algorithmType); //Set optimizing method to Gauss Newton
  m_optimizer.setVerbose(true);
}

void Slam::nextSplitPose(cluon::data::Envelope data){
  std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
  if(data.dataType() == opendlv::proxy::GeodeticWgs84Reading::ID()){
    auto position = cluon::extractMessage<opendlv::proxy::GeodeticWgs84Reading>(std::move(data));

    double longitude = position.longitude();
    double latitude = position.latitude();

    //toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position)

    std::array<double,2> WGS84ReadingTemp;

    WGS84ReadingTemp[0] = latitude;
    WGS84ReadingTemp[1] = longitude;

    std::array<double,2> WGS84Reading = wgs84::toCartesian(m_gpsReference, WGS84ReadingTemp); 
    //opendlv::data::environment::WGS84Coordinate gpsCurrent = opendlv::data::environment::WGS84Coordinate(latitude, longitude);
    //opendlv::data::environment::Point3 gpsTransform = m_gpsReference.transform(gpsCurrent);

    m_odometryData(0) =  WGS84Reading[0];
    m_odometryData(1) =  WGS84Reading[1];
  }
  else if(data.dataType() == opendlv::proxy::GeodeticHeadingReading::ID()){
    auto message = cluon::extractMessage<opendlv::proxy::GeodeticHeadingReading>(std::move(data));
    double heading = message.northHeading();
    heading = heading-PI;
    heading = (heading > PI)?(heading-2*PI):(heading);
    heading = (heading < -PI)?(heading+2*PI):(heading);
    m_odometryData(2) = heading;
    //std::cout << "head: " << heading << std::endl;
  }
}

void Slam::nextPose(cluon::data::Envelope data){
    //#########################Recieve Odometry##################################
  
  std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
  m_geolocationReceivedTime = data.sampleTimeStamp();
  auto odometry = cluon::extractMessage<opendlv::logic::sensation::Geolocation>(std::move(data));

  double longitude = odometry.longitude();
  double latitude = odometry.latitude();

  //toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position)

  std::array<double,2> WGS84ReadingTemp;

  WGS84ReadingTemp[0] = latitude;
  WGS84ReadingTemp[1] = longitude;

  std::array<double,2> WGS84Reading = wgs84::toCartesian(m_gpsReference, WGS84ReadingTemp); 
  //opendlv::data::environment::WGS84Coordinate gpsCurrent = opendlv::data::environment::WGS84Coordinate(latitude, longitude);
  //opendlv::data::environment::Point3 gpsTransform = m_gpsReference.transform(gpsCurrent);

  m_odometryData << WGS84Reading[0],
                    WGS84Reading[1],
                    odometry.heading();
  std::cout << "head: " << odometry.heading() << std::endl;                   
}

void Slam::nextYawRate(cluon::data::Envelope data){

  std::lock_guard<std::mutex> lockYaw(m_yawMutex);
  auto yawRate = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(data));
  m_yawRate = yawRate.angularVelocityZ()/1;
   m_yawReceivedTime = data.sampleTimeStamp();
   //std::cout << "Yaw in message: " << m_yawRate << std::endl;
}

void Slam::recieveCombinedMessage(cluon::data::TimeStamp currentFrameTime,std::map<int,ConePackage> currentFrame){
  m_lastTimeStamp = currentFrameTime;
  if(isKeyframe()){
    Eigen::MatrixXd cones = Eigen::MatrixXd::Zero(4,currentFrame.size());
    std::map<int,ConePackage>::iterator it;
    int coneIndex = 0;
    it =currentFrame.begin();
    while(it != currentFrame.end()){
      auto direction = std::get<0>(it->second);
      auto distance = std::get<1>(it->second);
      auto type = std::get<2>(it->second);
      cones(0,coneIndex) = direction.azimuthAngle();
      cones(1,coneIndex) = direction.zenithAngle();
      cones(2,coneIndex) = distance.distance();
      cones(3,coneIndex) = type.type();
      coneIndex++;
      it++;
    }
    performSLAM(cones);
  }
}

bool Slam::isKeyframe(){
  cluon::data::TimeStamp startTime = cluon::time::now();
  double timeElapsed = fabs(static_cast<double>(cluon::time::deltaInMicroseconds(m_keyframeTimeStamp,startTime)))/1000;
  std::cout << "Time ellapsed is: " << timeElapsed << std::endl;
  if(timeElapsed>m_timeBetweenKeyframes){//Keyframe candidate is based on time difference from last keyframe
    m_keyframeTimeStamp = startTime;
    return true;
  }
  return false;
}


void Slam::performSLAM(Eigen::MatrixXd cones){
  
  if(fabs(m_odometryData(0))>200 || fabs(m_odometryData(1))>200)
  {
    return;
  }
    Eigen::Vector3d pose;
  {
    std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
    pose = m_odometryData;
    double timeElapsed = fabs(static_cast<double>(cluon::time::deltaInMicroseconds(m_yawReceivedTime, m_lastTimeStamp)))/1000000;

    {
      std::lock_guard<std::mutex> lockYaw(m_yawMutex);
      if(timeElapsed > 0 && timeElapsed < 1){
        pose(2) = pose(2) - static_cast<double>(m_yawRate)*(timeElapsed);
      }
    }  
    m_poses.push_back(pose);
  }

  if(!m_loopClosingComplete){
    {
      std::lock_guard<std::mutex> lockMap(m_mapMutex);
      createConnections(cones,pose);
    }

    uint32_t currentEndCone = m_coneList.size() - 1; 
    uint32_t coneDiff = currentEndCone - m_coneRef;

    if(coneDiff >= 10 && !m_loopClosingComplete){
      std::lock_guard<std::mutex> lockMap(m_mapMutex);
      optimizeEssentialGraph(currentEndCone-coneDiff, currentEndCone);
      m_coneRef = currentEndCone; 
    }
  }

    m_poseId++;

    //Check if there is enough loopclosing candidates
    if(!m_loopClosingComplete){
      int loopClosers = 0;
        std::lock_guard<std::mutex> lockMap(m_mapMutex);
      for(uint32_t m = 0; m < m_coneList.size(); m++){
        if(m_coneList[m].getLoopClosingState()){
          loopClosers++;
        }
      }

      if(loopClosers > 1){
        fullBA();
        m_loopClosingComplete = true;
        m_filterMap = true;
      }
    }

    //Map preprocessing
    if(m_filterMap){

      std::lock_guard<std::mutex> lockMap(m_mapMutex);
      std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
      filterMap();
      updateMap(0,m_coneList.size(),true);
      m_filterMap = false;
    }

    //Localizer
    if(m_loopClosingComplete){

      
      localizer(cones, pose, false);

      sendPose();
      sendCones();
    }

}

void Slam::localizer(Eigen::MatrixXd cones, Eigen::Vector3d pose, bool poseOptimization){

  g2o::SparseOptimizer localGraph;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > slamBlockSolver;
  typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> slamLinearSolver;
  
  auto linearSolver = g2o::make_unique<slamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  
  g2o::OptimizationAlgorithmGaussNewton* algorithmType = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<slamBlockSolver>(std::move(linearSolver)));
  localGraph.setAlgorithm(algorithmType); //Set optimizing method to Gauss Newton
  localGraph.setVerbose(true);

  std::lock_guard<std::mutex> lockMap(m_mapMutex);
  //Find match in conelist
  std::vector<int> matchedConeIndex;
  std::vector<Eigen::Vector3d> localObs;
  double shortestDistance = 10000;
  for(uint32_t i = 0; i < cones.cols(); i++){
    bool foundMatch = false;
    uint32_t j = 0;
    while(!foundMatch && j < m_coneList.size()){
      Eigen::Vector3d globalCone = coneToGlobal(pose, cones.col(i));
      Cone globalConeObject = Cone(globalCone(0), globalCone(1),0,2000);
      double distance = distanceBetweenCones(m_coneList[j],globalConeObject);

      if(distance < m_newConeThreshold){ //m_newConeThreshold
        matchedConeIndex.push_back(j);

        Eigen::Vector3d localCone = Spherical2Cartesian(cones(0,i), cones(1,i),cones(2,i));
        localObs.push_back(localCone);
        foundMatch = true;

        if(distance < shortestDistance){

          shortestDistance = distance;
          m_currentConeIndex = m_coneList[j].getId();

        }
      }

      j++;
    }

  }

  if(matchedConeIndex.size() > 0 && poseOptimization){  
    //Create graph
    //Add pose vertex
    g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
    poseVertex->setId(1000);
    poseVertex->setEstimate(pose);
    localGraph.addVertex(poseVertex);

    //Add cone vertex
    Eigen::Vector2d coneMeanXY;
    for(uint32_t i = 0; i < matchedConeIndex.size(); i++){

      g2o::EdgeSE2PointXY* coneMeasurement = new g2o::EdgeSE2PointXY;
      coneMeanXY << m_coneList[matchedConeIndex[i]].getOptX(),m_coneList[matchedConeIndex[i]].getOptY();
      g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY;
      coneVertex->setId(i);
      coneVertex->setEstimate(coneMeanXY);
      coneVertex->setFixed(true);      
      localGraph.addVertex(coneVertex);

      //Add edge between pose and cone i
      Eigen::Vector2d xyMeasurement;
      coneMeasurement->vertices()[0] = localGraph.vertex(1000);
      coneMeasurement->vertices()[1] = localGraph.vertex(i);
      xyMeasurement << localObs[i](0),localObs[i](1);
      coneMeasurement->setMeasurement(xyMeasurement);

      //Eigen::Vector2d covXY = m_coneList[matchedConeIndex[i]].getCovariance();
      Eigen::Matrix2d informationMatrix;
      informationMatrix << 1/0.1,0,
                              0,1/0.1;
      coneMeasurement->setInformation(informationMatrix);
      localGraph.addEdge(coneMeasurement);

    }



    /*g2o::VertexPointXY* firstCone = dynamic_cast<g2o::VertexPointXY*>(localGraph.vertex(0));
    firstCone->setFixed(true);*/
    std::cout << "Optimizing" << std::endl;
    localGraph.initializeOptimization();
    localGraph.optimize(10); //Add config for amount of iterations??
    std::cout << "Optimizing done." << std::endl;



  

    g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(localGraph.vertex(1000));
    g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
    Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
  
    {
      std::lock_guard<std::mutex> lockSend(m_sendMutex); 
      m_sendPose << updatedPose(0),updatedPose(1),updatedPose(2);
      std::cout << "pose: " << updatedPose(0) << " : " << updatedPose(1) << " : " << updatedPose(2) << std::endl;
    }

  }else{
    m_sendPose << pose(0),pose(1),pose(2);
  }

}


void Slam::createConnections(Eigen::MatrixXd cones, Eigen::Vector3d pose){

  //Check current cone list and find same cone, if true add pose connection and cone observation
  bool firstCone = false;
  if(m_coneList.size() == 0){
    Eigen::Vector3d localCone = Spherical2Cartesian(cones(0,0), cones(1,0),cones(2,0));
    Eigen::Vector3d globalCone = coneToGlobal(pose, cones.col(0));
    Cone cone = Cone(globalCone(0),globalCone(1),(int)globalCone(2),m_coneList.size()); //Temp id, think of system later
    cone.addObservation(localCone,globalCone,m_poseId,m_currentConeIndex);

    m_coneList.push_back(cone);
    firstCone = true;
  }

  double minDistance = 100;
  for(uint32_t i = 0; i<cones.cols(); i++){//Iterate through local cone objects
    double distanceToCar = cones(2,i);
    Eigen::Vector3d localCone = Spherical2Cartesian(cones(0,i), cones(1,i),cones(2,i));
    Eigen::Vector3d globalCone = coneToGlobal(pose, cones.col(i)); //Make local cone into global coordinate frame
    uint32_t j = 0;
    bool coneFound = false;
    while(!coneFound && j<m_coneList.size() && !m_loopClosing && !firstCone){
      if(fabs(m_coneList[j].getType() - cones(3,i))<0.0001){ //Check is same classification
    
        Cone globalConeObject = Cone(globalCone(0), globalCone(1),0,2000);
        double distance = distanceBetweenCones(m_coneList[j],globalConeObject);

        if(distance<m_newConeThreshold){ //NewConeThreshold is the accepted distance for a new cone candidate
          coneFound = true;
          m_coneList[j].addObservation(localCone, globalCone, m_poseId,m_currentConeIndex);
          
          if(distanceToCar<minDistance){//Update current cone to know where in the map we are
            m_currentConeIndex = j;
            minDistance = distanceToCar;
          }
        }
      }
      j++;
    }
    if(distanceToCar < m_coneMappingThreshold && !coneFound && !m_loopClosing && !firstCone){
      std::cout << "Trying to add cone" << std::endl;
      Cone cone = Cone(globalCone(0),globalCone(1),(int)globalCone(2),m_coneList.size()); //Temp id, think of system later
      cone.addObservation(localCone, globalCone,m_poseId,m_currentConeIndex);
      m_coneList.push_back(cone);
    }
  }
}

void Slam::createFullGraph(){


  //Add all poses to graph
  addPosesToGraph();


  //For each cone in conelist add poses

  addConesToGraph();

}

void Slam::optimizeEssentialGraph(uint32_t graphIndexStart, uint32_t graphIndexEnd){

  //Initialize graph
  g2o::SparseOptimizer essentialGraph;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > slamBlockSolver;
  typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> slamLinearSolver;
  
  auto linearSolver = g2o::make_unique<slamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  
  g2o::OptimizationAlgorithmGaussNewton* algorithmType = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<slamBlockSolver>(std::move(linearSolver)));
  essentialGraph.setAlgorithm(algorithmType); //Set optimizing method to Gauss Newton
  essentialGraph.setVerbose(true);

  std::vector<int> posesToGraph;
  //Find cones of conespan and extract poses
  for(uint32_t i = graphIndexStart; i < graphIndexEnd+1; i++){

    std::vector<int> currentConnectedPoses = m_coneList[i].getConnectedPoses();

    for(uint32_t j = 0; j < currentConnectedPoses.size(); j++){

      posesToGraph.push_back(currentConnectedPoses[j]);
    }

  }
  if(posesToGraph.size() > 0){

    uint32_t max = *std::max_element(posesToGraph.begin(), posesToGraph.end());
    uint32_t min = *std::min_element(posesToGraph.begin(), posesToGraph.end());

    //add poses to graph based on min and max
    for(uint32_t k = min; k < max+1; k++){

      //Add vertex
      g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
      poseVertex->setId(k);
      poseVertex->setEstimate(m_poses[k-1000]);

      essentialGraph.addVertex(poseVertex);

      //Add edge
      if(k > min){
        g2o::EdgeSE2* odometryEdge = new g2o::EdgeSE2;
        odometryEdge->vertices()[0] = essentialGraph.vertex(k-1);
        odometryEdge->vertices()[1] = essentialGraph.vertex(k);
        g2o::VertexSE2* prevVertex = static_cast<g2o::VertexSE2*>(essentialGraph.vertex(k-1));
        g2o::SE2 prevPose = prevVertex->estimate();
        g2o::SE2 currentPose = g2o::SE2(m_poses[k-1000](0), m_poses[k-1000](1), m_poses[k-1000](2));
        g2o::SE2 measurement = prevPose.inverse()*currentPose;
        odometryEdge->setMeasurement(measurement);
        odometryEdge->setInformation(Eigen::Matrix3d::Identity()*1/0.5); //Actual covariance should be configured
        essentialGraph.addEdge(odometryEdge);
      }
    }

    //Connect cones to poses
    for(uint32_t i = graphIndexStart; i < graphIndexEnd; i++){

        Eigen::Vector2d coneMeanXY;
        if(!m_coneList[i].isOptimized()){
          m_coneList[i].calculateMean();
          coneMeanXY << m_coneList[i].getMeanX(),m_coneList[i].getMeanY();
        }else{
          coneMeanXY << m_coneList[i].getOptX(),m_coneList[i].getOptY();

        }
      
        g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY;
        coneVertex->setId(m_coneList[i].getId());
        coneVertex->setEstimate(coneMeanXY);
        essentialGraph.addVertex(coneVertex);

        //Connect cones to POses
        g2o::EdgeSE2PointXY* coneMeasurement = new g2o::EdgeSE2PointXY;
        std::vector<int> connectedPoses = m_coneList[i].getConnectedPoses();

      for(uint32_t j = 0; j < connectedPoses.size(); j++){
          Eigen::Vector2d xyMeasurement;
          xyMeasurement = getConeToPoseMeasurement(i,j);
          std::cout << "x: " << xyMeasurement(0) << " y: " << xyMeasurement(1) << std::endl; 
          coneMeasurement->vertices()[0] = essentialGraph.vertex(connectedPoses[j]);
          coneMeasurement->vertices()[1] = essentialGraph.vertex(m_coneList[i].getId());
          coneMeasurement->setMeasurement(xyMeasurement);

          Eigen::Vector2d covXY = m_coneList[i].getCovariance();
          Eigen::Matrix2d informationMatrix;
          informationMatrix << 1/covXY(0),0,
                              0,1/covXY(1);
          coneMeasurement->setInformation(informationMatrix); //Placeholder value
          std::cout << "cX: " << covXY(0) << " cY: " << covXY(1) << std::endl;
          essentialGraph.addEdge(coneMeasurement);

        }
      }


    g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(essentialGraph.vertex(min));
    firstRobotPose->setFixed(true);

    /*g2o::VertexPointXY* firstCone = dynamic_cast<g2o::VertexPointXY*>(essentialGraph.vertex(graphIndexStart));
    firstCone->setFixed(true);*/
    std::cout << "Optimizing" << std::endl;
    essentialGraph.initializeOptimization();
    essentialGraph.optimize(10); //Add config for amount of iterations??
    std::cout << "Optimizing done." << std::endl;

    updateFromEssential(min, max, graphIndexStart,graphIndexEnd, essentialGraph);
    updateMap(graphIndexStart,graphIndexEnd,false);
  }
}

void Slam::updateFromEssential(uint32_t poseStart, uint32_t poseEnd,uint32_t coneStart,uint32_t coneEnd, g2o::SparseOptimizer &essentialGraph){

  //Update pose vector

  for(uint32_t i = poseStart; i < poseEnd; i++){
    g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(essentialGraph.vertex(i));
    g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
    Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
    m_poses[i-1000] << updatedPose(0),updatedPose(1),updatedPose(2);
  }
  //Set optimized cone positions

   Eigen::Vector2d updatedConeXY;
   g2o::VertexPointXY* updatedConeVertex;

   for(uint32_t i = coneStart; i < coneEnd; i++){
    updatedConeVertex = static_cast<g2o::VertexPointXY*>(essentialGraph.vertex(i));
    updatedConeXY = updatedConeVertex->estimate();
    m_coneList[i].setOptX(updatedConeXY(0));
    m_coneList[i].setOptY(updatedConeXY(1));
    m_coneList[i].setOptimized();    
   }
}

Eigen::Vector3d Slam::updatePoseFromGraph(){

  g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(m_optimizer.vertex(m_poseId-1));
  g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
  Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
  return updatedPose;
}

void Slam::addPosesToGraph(){

  for(uint32_t i = 0; i < m_poses.size(); i++){
    g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
    poseVertex->setId(i+1000);
    poseVertex->setEstimate(m_poses[i]);

    m_optimizer.addVertex(poseVertex);

    addOdometryMeasurement(m_poses[i],i);

    std::vector<int> poseVector;
    m_connectivityGraph.push_back(poseVector);
  }
}


void Slam::addOdometryMeasurement(Eigen::Vector3d pose, uint32_t i){
  if(i > 0){
    g2o::EdgeSE2* odometryEdge = new g2o::EdgeSE2;

    odometryEdge->vertices()[0] = m_optimizer.vertex(i+999);
    odometryEdge->vertices()[1] = m_optimizer.vertex(i+1000);
    g2o::VertexSE2* prevVertex = static_cast<g2o::VertexSE2*>(m_optimizer.vertex(i+999));
    g2o::SE2 prevPose = prevVertex->estimate();
    g2o::SE2 currentPose = g2o::SE2(pose(0), pose(1), pose(2));
    g2o::SE2 measurement = prevPose.inverse()*currentPose;
    odometryEdge->setMeasurement(measurement);
    odometryEdge->setInformation(Eigen::Matrix3d::Identity()*1/0.5); //Actual covariance should be configured
    m_optimizer.addEdge(odometryEdge);
  }
}

void Slam::fullBA(){

  createFullGraph();

  g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(m_optimizer.vertex(1000));
  firstRobotPose->setFixed(true);

  /*g2o::VertexSE2* secondRobotPose = dynamic_cast<g2o::VertexSE2*>(m_optimizer.vertex(1001));
  secondRobotPose->setFixed(true);

  g2o::VertexPointXY* firstCone = dynamic_cast<g2o::VertexPointXY*>(m_optimizer.vertex(0));
  firstCone->setFixed(true);

  g2o::VertexPointXY* secondCone = dynamic_cast<g2o::VertexPointXY*>(m_optimizer.vertex(1));
  secondCone->setFixed(true);*/


  //m_optimizer.setVerbose(true);

  std::cout << "Optimizing" << std::endl;
  m_optimizer.initializeOptimization();
  m_optimizer.optimize(10); //Add config for amount of iterations??
  std::cout << "Optimizing done." << std::endl;


  Eigen::Vector2d updatedConeXY;
  g2o::VertexPointXY* updatedConeVertex;

  for(uint32_t j = 0; j < m_coneList.size(); j++){//Iterate and replace old map landmarks with new updated ones
    updatedConeVertex = static_cast<g2o::VertexPointXY*>(m_optimizer.vertex(j));
    updatedConeXY = updatedConeVertex->estimate();
    m_coneList[j].setOptX(updatedConeXY(0));
    m_coneList[j].setOptY(updatedConeXY(1));
  }
  
  for(uint32_t i = 0; i < m_poses.size(); i++){
    g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(m_optimizer.vertex(i+1000));
    g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
    Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
    m_poses[i] << updatedPose(0),updatedPose(1),updatedPose(2);
  }
  //updateMap(0,m_coneList.size(),true);

}



Eigen::Vector3d Slam::coneToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd cones){
  Eigen::Vector3d cone = Spherical2Cartesian(cones(0), cones(1), cones(2));
  //convert from local to global coordsystem
  double newX = cone(0)*cos(pose(2))-cone(1)*sin(pose(2));
  double newY = cone(0)*sin(pose(2))+cone(1)*cos(pose(2));
  cone(0) = newX+pose(0);
  cone(1) = newY+pose(1);
  cone(2) = cones(3);
  return cone;
}

Eigen::Vector2d Slam::transformConeToCoG(double angle, double distance){
  const double lidarDistToCoG = 1.5;
  double sign = angle/std::fabs(angle);
  angle = PI - std::fabs(angle*DEG2RAD); 
  double distanceNew = std::sqrt(lidarDistToCoG*lidarDistToCoG + distance*distance - 2*lidarDistToCoG*distance*std::cos(angle));
  double angleNew = std::asin((std::sin(angle)*distance)/distanceNew )*RAD2DEG; 
  Eigen::Vector2d transformed;
  transformed << angleNew*sign,distanceNew;

  return transformed;
}

void Slam::addConesToGraph(){

  for(uint32_t i = 0; i < m_coneList.size(); i++){
    
    Eigen::Vector2d coneMeanXY;
    if(!m_coneList[i].isOptimized()){
      m_coneList[i].calculateMean();
      coneMeanXY << m_coneList[i].getMeanX(),m_coneList[i].getMeanY();
    }else{
      coneMeanXY << m_coneList[i].getOptX(),m_coneList[i].getOptY();
    }
     
    g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY;
    coneVertex->setId(m_coneList[i].getId());
    coneVertex->setEstimate(coneMeanXY);

    m_optimizer.addVertex(coneVertex);


    addConeMeasurements(i);
    }


}

void Slam::addConeMeasurements(int i){
  g2o::EdgeSE2PointXY* coneMeasurement = new g2o::EdgeSE2PointXY;

  std::vector<int> connectedPoses = m_coneList[i].getConnectedPoses();

  for(uint32_t j = 0; j < connectedPoses.size(); j++){
  Eigen::Vector2d xyMeasurement;
  xyMeasurement = getConeToPoseMeasurement(i,j);

  coneMeasurement->vertices()[0] = m_optimizer.vertex(connectedPoses[j]);
  coneMeasurement->vertices()[1] = m_optimizer.vertex(m_coneList[i].getId());
  coneMeasurement->setMeasurement(xyMeasurement);

  Eigen::Vector2d covXY = m_coneList[i].getCovariance();
  Eigen::Matrix2d informationMatrix;
  informationMatrix << 1/covXY(0),0,
                       0,1/covXY(1);
  coneMeasurement->setInformation(informationMatrix); //Placeholder value

  m_optimizer.addEdge(coneMeasurement);

  }

  //m_connectivityGraph[m_poseId-1001].push_back(cone.getId());
}

Eigen::Vector2d Slam::getConeToPoseMeasurement(int i,int j){
  
  Eigen::Vector2d cone;
  cone = m_coneList[i].getLocalConeObservation(j); 
  Eigen::Vector2d measurement;
  measurement << cone(0), cone(1);  

  return measurement;
}
Eigen::Vector2d Slam::getLocalConeToPoseMeasurement(Eigen::Vector3d pose, Eigen::Vector2d cone){
  
  Eigen::Vector2d measurement;
  measurement << cone(0)-pose(0), cone(1)-pose(1);  

  return measurement;
}
Eigen::Vector3d Slam::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  //double xyDistance = distance * cos(azimuth * static_cast<double>(DEG2RAD));
  //azimuth = (azimuth > PI)?(azimuth-2*PI):(azimuth);
  //azimuth = (azimuth < -PI)?(azimuth+2*PI):(azimuth);

  Eigen::Vector2d transformedCone = transformConeToCoG(azimuth,distance);
  azimuth = transformedCone(0);
  distance = transformedCone(1);
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(zenimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = Eigen::Vector3d::Zero();
  recievedPoint << xData,
                   yData,
                   zData;
  return recievedPoint;
}

void Slam::sendCones()
{
  Eigen::Vector3d pose;
  {
    std::lock_guard<std::mutex> lockSend(m_sendMutex); 
    pose = m_sendPose;
  }//mapmutex too
  std::lock_guard<std::mutex> lockMap(m_mapMutex);
  //std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  cluon::data::TimeStamp sampleTime = m_geolocationReceivedTime;
  for(uint32_t i = m_conesPerPacket; i>0;i--){ //Iterate through the cones ahead of time the path planning recieves
    int index = (m_currentConeIndex+i<m_map.size())?(m_currentConeIndex+i):(m_currentConeIndex+i-m_map.size()); //Check if more cones is sent than there exists
    opendlv::logic::perception::ObjectDirection directionMsg = m_map[index].getDirection(pose); //Extract cone direction
    directionMsg.objectId(i-1);
    od4.send(directionMsg,sampleTime,m_senderStamp);
    opendlv::logic::perception::ObjectDistance distanceMsg = m_map[index].getDistance(pose); //Extract cone distance
    distanceMsg.objectId(i-1);
    od4.send(distanceMsg,sampleTime,m_senderStamp);
    opendlv::logic::perception::ObjectType typeMsg;
    typeMsg.type(m_map[index].getType()); //Extract cone type
    typeMsg.objectId(i-1);
    od4.send(typeMsg,sampleTime,m_senderStamp);
  }
}

void Slam::sendPose(){
  opendlv::logic::sensation::Geolocation poseMessage;
  std::lock_guard<std::mutex> lockSend(m_sendMutex); 

  std::array<double,2> cartesianPos;
  cartesianPos[0] = m_sendPose(0);
  cartesianPos[1] = m_sendPose(1);
  std::array<double,2> sendGPS = wgs84::fromCartesian(m_gpsReference, cartesianPos);
  poseMessage.longitude(static_cast<float>(sendGPS[0]));
  poseMessage.latitude(static_cast<float>(sendGPS[1]));
  poseMessage.heading(static_cast<float>(m_sendPose(2)));
  //std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  cluon::data::TimeStamp sampleTime = m_geolocationReceivedTime;
  od4.send(poseMessage, sampleTime ,m_senderStamp);
}

double Slam::distanceBetweenCones(Cone c1, Cone c2){
  c1.calculateMean();
  c2.calculateMean();
  double distance = std::sqrt( (c1.getMeanX()-c2.getMeanX())*(c1.getMeanX()-c2.getMeanX()) + (c1.getMeanY()-c2.getMeanY())*(c1.getMeanY()-c2.getMeanY()) );
  return distance;
}
double Slam::distanceBetweenConesOpt(Cone c1, Cone c2){
  double distance = std::sqrt( (c1.getOptX()-c2.getMeanX())*(c1.getOptX()-c2.getMeanX()) + (c1.getOptY()-c2.getMeanY())*(c1.getOptY()-c2.getMeanY()) );
  return distance;
}

void Slam::updateMap(uint32_t start, uint32_t end, bool updateToGlobal){

  for(uint32_t i = start; i < end; i++){

    if(updateToGlobal && m_coneList[i].isValid()){
      m_map.push_back(m_coneList[i]);
    }else{

      m_essentialMap.push_back(m_coneList[i]);
    }
  }
}

void Slam::filterMap(){

  //Filter on mean and optimized value
  for(uint32_t i = 0; i < m_coneList.size(); i++){
    double distance = distanceBetweenConesOpt(m_coneList[i],m_coneList[i]);
    if(distance > m_newConeThreshold){
      m_coneList[i].setValidState(false);

    }
  }

  for(uint32_t i = 0; i < m_coneList.size(); i++){

      for(uint32_t j = 0; j < m_coneList.size(); j++){
        if(i != j){
          double distance = std::sqrt( (m_coneList[i].getOptX() - m_coneList[j].getOptX() )*(m_coneList[i].getOptX() - m_coneList[j].getOptX()) + (m_coneList[i].getOptY() - m_coneList[j].getOptY())*(m_coneList[i].getOptY() - m_coneList[j].getOptY()) );

          if(distance < m_newConeThreshold && m_coneList[i].isValid() && m_coneList[j].isValid()){
            m_coneList[j].setValidState(false);
          }
        } 
      }
  }  


  //Check closest pose didstance
  for(uint32_t i = 0; i < m_coneList.size(); i++){
    
    double closestPoseDistance = 10000;
    for(uint32_t j = 0; j < m_poses.size(); j++){

      double distance = std::sqrt( (m_poses[j](0)-m_coneList[i].getOptX())*(m_poses[j](0)-m_coneList[i].getOptX()) + (m_poses[j](1)-m_coneList[i].getOptY())*(m_poses[j](1)-m_coneList[i].getOptY()) );
      if(distance < closestPoseDistance){
        closestPoseDistance = distance;
      }

    }

    if(closestPoseDistance > 3){
      m_coneList[i].setValidState(false);
    }
  }

  //Check colours

  for(uint32_t i = 0; i < m_coneList.size(); i++){
    double distance = 10000;
    double azimuth = 1000;
    if(m_coneList[i].getType() == 0){
      for(uint32_t j = 0; j < m_coneList[i].getObservations(); j++ ){
        
        Eigen::Vector2d localObs = m_coneList[i].getLocalConeObservation(j);
        double localDistance = std::sqrt( localObs(0)*localObs(0) + localObs(1)*localObs(1) );
        if(localDistance < distance){
          distance = localDistance;
          azimuth = std::atan2(localObs(1),localObs(0));
        }
      }


      if(azimuth > 0.1 ){

        m_coneList[i].setType(1);

        std::cout << "New type yellow" << std::endl;
      }else if(azimuth < -0.1){
        m_coneList[i].setType(2);
        std::cout << "New type blue" << std::endl;
      }
    }


  }


}

void Slam::setUp(std::map<std::string, std::string> configuration)
{

  m_timeDiffMilliseconds = static_cast<uint32_t>(std::stoi(configuration["gatheringTimeMs"]));
  m_newConeThreshold = static_cast<double>(std::stod(configuration["sameConeThreshold"]));
  m_gpsReference[0] = static_cast<double>(std::stod(configuration["refLatitude"]));
  m_gpsReference[1] = static_cast<double>(std::stod(configuration["refLongitude"]));
  m_timeBetweenKeyframes = static_cast<double>(std::stod(configuration["timeBetweenKeyframes"]));
  m_coneMappingThreshold = static_cast<double>(std::stod(configuration["coneMappingThreshold"]));
  m_conesPerPacket = static_cast<int>(std::stoi(configuration["conesPerPacket"]));
  std::cout << "Cones per packet" << m_conesPerPacket << std::endl;
  m_senderStamp = static_cast<int>(std::stoi(configuration["id"]));
  //auto kv = getKeyValueConfiguration();
  //m_timeDiffMilliseconds = kv.getValue<double>("logic-cfsd18-perception-detectcone.timeDiffMilliseconds");
  //m_newConeThreshold = kv.getValue<double>("logic-cfsd18-sensation-slam.newConeLimit");

  //double const latitude = getKeyValueConfiguration().getValue<double>("logic-sensation-geolocator.GPSreference.latitude");
  //double const longitude = getKeyValueConfiguration().getValue<double>("logic-sensation-geolocator.GPSreference.longitude");
  //m_gpsReference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);
  
}

std::vector<Eigen::Vector3d> Slam::drawPoses(){
  std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
  return m_poses;
}

std::vector<Cone> Slam::drawCones(){
  std::lock_guard<std::mutex> lock(m_mapMutex);
  return m_map;
}
std::vector<Cone> Slam::drawRawCones(){
  std::lock_guard<std::mutex> lock(m_mapMutex);
  return m_coneList;
}
std::vector<Cone> Slam::drawLocalOptimizedCones(){
  std::lock_guard<std::mutex> lock(m_mapMutex);
  return m_essentialMap;
}
Eigen::Vector3d Slam::drawCurrentPose(){
  if(m_loopClosingComplete){
    std::lock_guard<std::mutex> lock(m_sendMutex);
    return m_sendPose;
  }
  else{
    std::lock_guard<std::mutex> lock(m_sensorMutex);
    return m_odometryData;
  }
}

std::vector<std::vector<int>> Slam::drawGraph(){
  std::lock_guard<std::mutex> lock1(m_mapMutex);
  std::lock_guard<std::mutex> lock2(m_sensorMutex);
  return m_connectivityGraph;
 
}
void Slam::writeToPoseAndMapFile()
{
  std::string filepathMap;
  filepathMap = "./map.txt";
	
		std::ofstream f;
    	f.open(filepathMap.c_str());
		for(uint32_t i = 0; i<m_map.size(); i++){

				f << std::setprecision(9) << m_map[i].getX() << "\t" << m_map[i].getY() << std::endl;
		}
		f.close();
		std::cout << "map with " << m_map.size() << " points saved" << std::endl;


    std::string filepathPose;
    filepathPose = "./pose.txt";
		std::ofstream p;
    p.open(filepathPose.c_str());
		for(uint32_t i = 0; i<m_poses.size(); i++){

				p << std::setprecision(9) << m_poses[i](0) << "\t" << m_poses[i](1) << "\t" << m_poses[i](2) << std::endl;
		}
		p.close();

}
void Slam::tearDown()
{
}
Slam::~Slam()
{

}
