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

#ifndef COLLECTOR_HPP
#define COLLECTOR_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "opendlv-standard-message-set.hpp"
#include "slam.hpp"
#include "cvcones.hpp"
#include "cluon-complete.hpp"


typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;

class Collector{
  public:
    Collector(Slam &slam, int, int);
    ~Collector() = default;
    void CollectCones(cluon::data::Envelope data);
    void InitializeCollection();
    void GetCompleteFrame();
    void SendFrame();

  private:
    cluon::data::TimeStamp m_currentFrameTime = {}; //Timestamp of current frame
    std::map<int,ConePackage> m_currentFrame = {}; //Map containing the current cones with the Id as key
    std::map<int,int> m_envelopeCount = {}; //Counter of received envelopes
    bool m_newFrame = true; //Bool set to true if waiting for new frame
    bool m_processing = false; //not used
    uint32_t m_messageCount = 0; //number of cone envelopes received
    Slam &m_module; //main module
    uint32_t m_packetSize; //Constant given as input describing how big a cone packet is, it varies from module to module
    int m_timeDiffMilliseconds; //The timeout parameter
    uint32_t m_numberOfItems = 1; //How many cones you are expected to receive
};

#endif
