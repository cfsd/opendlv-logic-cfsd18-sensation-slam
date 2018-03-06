/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "g2o/core/sparse_optimizer.h"
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "slam.hpp"
#include "cone.hpp"
#include <Eigen/Dense>

#include <cstdint>
#include <iostream>
#include <string>
#include <thread>


int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid")) {
    std::cerr << argv[0] << " is a slam implementation for the CFSD18 project." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of simulated units>] [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111" << std::endl;
    retCode = 1;
  } else {
    //uint32_t const ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    g2o::SparseOptimizer optimizer;
    (void)VERBOSE;
    // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
    cluon::data::Envelope data;
    //std::shared_ptr<Slam> slammer = std::shared_ptr<Slam>(new Slam(10));
    Slam slam;
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])),
      [&data, &slammer = slam, &od4session = od4](cluon::data::Envelope &&envelope){
        slammer.nextContainer(envelope);
        std::vector<Cone> map = slammer.getCones();
        //slam.sendCones(map,od4session);
        //Eigen::Vector3d pose = Slam.getPose();
        //sendPose(pose,od4session);  
      }
    };

    // Just sleep as this microservice is data driven.
    using namespace std::literals::chrono_literals;
    while (od4.isRunning()) {
      std::this_thread::sleep_for(1s);
      std::chrono::system_clock::time_point tp;
    }
  }
  return retCode;
}

/*void sendCones(std::vector<Cone> map,cluon::OD4Session od4){
  for(uint32_t i = 0; i<map.size(); i++){
    Cone cone = map[i];
    //opendlv
  }
}*/

