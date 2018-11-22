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

#include "collector.hpp"
/* Constructor taking the slam module object, 
the wait time for the collection and the size of the packets as input*/
Collector::Collector(Slam &slam,int timeDiffMilliseconds,int packetSize) : 
    m_module(slam),
    m_packetSize(packetSize),
    m_timeDiffMilliseconds(timeDiffMilliseconds)
{
}
/*Method called for each envelope received by the module,
Batches all envelopes related to cones and in the same frame into a map m_currentFrame*/
void Collector::CollectCones(cluon::data::Envelope data){
    cluon::data::TimeStamp ts = data.sampleTimeStamp();
    int64_t delta = cluon::time::deltaInMicroseconds(ts,m_currentFrameTime);
    if(std::abs(delta)<1){//If cones have the same timestamp they are in the same frame
        if(data.dataType() == opendlv::logic::perception::ObjectDirection::ID()){
            opendlv::logic::perception::ObjectDirection direction = cluon::extractMessage<opendlv::logic::perception::ObjectDirection>(std::move(data));
            uint32_t id = direction.objectId();
            std::map<int,ConePackage>::iterator it;
            it = m_currentFrame.find(id);
            if(it!=m_currentFrame.end()){//If a cone is already in m_currentFrame add this message
                std::get<0>(it->second) = direction;
                m_envelopeCount[id]++;
            }
            else{//Make a new entry in the map using the objectId as key
                ConePackage conePacket;
                std::get<0>(conePacket) = direction;
                m_currentFrame[id] = conePacket;
                m_envelopeCount[id] = 1;
            }
            m_numberOfItems = (m_numberOfItems<=id)?(id+1):(m_numberOfItems);
        }
        else if(data.dataType() == opendlv::logic::perception::ObjectDistance::ID()){
            opendlv::logic::perception::ObjectDistance distance = cluon::extractMessage<opendlv::logic::perception::ObjectDistance>(std::move(data));
            uint32_t id = distance.objectId();
            std::map<int,ConePackage>::iterator it;
            it = m_currentFrame.find(id);
            if(it!=m_currentFrame.end()){//If a cone is already in m_currentFrame add this message
                std::get<1>(it->second) = distance;
                m_envelopeCount[id]++;
            }
            else{//Make a new entry in the map using the objectId as key
                ConePackage conePacket;
                std::get<1>(conePacket) = distance;
                m_currentFrame[id] = conePacket;
                m_envelopeCount[id]=1;
            }
            m_numberOfItems = (m_numberOfItems<=id)?(id+1):(m_numberOfItems);
        }
        else if(data.dataType() == opendlv::logic::perception::ObjectType::ID()){
            opendlv::logic::perception::ObjectType type = cluon::extractMessage<opendlv::logic::perception::ObjectType>(std::move(data));
            uint32_t id = type.objectId();
            std::map<int,ConePackage>::iterator it;
            it = m_currentFrame.find(id);
            if(it!=m_currentFrame.end()){//If a cone is already in m_currentFrame add this message
                std::get<2>(it->second) = type;
                m_envelopeCount[id]++;
            }
            else{//Make a new entry in the map using the objectId as key
                ConePackage conePacket;
                std::get<2>(conePacket) = type;
                m_currentFrame[id] = conePacket;
                m_envelopeCount[id]=1;
            }
            m_numberOfItems = (m_numberOfItems<=id)?(id+1):(m_numberOfItems);
        }
        m_messageCount++;

    }
    else if(m_newFrame) //If the message is the first of a new frame we reset the members
    {
        m_numberOfItems = 1;
        m_currentFrame.clear();
        m_envelopeCount.clear();
        m_messageCount = 1;
        m_currentFrameTime = data.sampleTimeStamp();
        m_newFrame = false;
        if(data.dataType() == opendlv::logic::perception::ObjectDirection::ID()){//Add the message to the new frame
            opendlv::logic::perception::ObjectDirection direction = cluon::extractMessage<opendlv::logic::perception::ObjectDirection>(std::move(data));
            uint32_t id = direction.objectId();
            ConePackage conePacket;
            std::get<0>(conePacket) = direction;
            m_currentFrame[id] = conePacket;
            m_envelopeCount[id]=1;
            m_numberOfItems = (m_numberOfItems<=id)?(id+1):(m_numberOfItems);
        }
        else if(data.dataType() == opendlv::logic::perception::ObjectDistance::ID()){//Add the message to the new frame
            opendlv::logic::perception::ObjectDistance distance = cluon::extractMessage<opendlv::logic::perception::ObjectDistance>(std::move(data));
            uint32_t id = distance.objectId();
            ConePackage conePacket;
            std::get<1>(conePacket) = distance;
            m_currentFrame[id] = conePacket;
            m_envelopeCount[id]=1;
            m_numberOfItems = (m_numberOfItems<=id)?(id+1):(m_numberOfItems);
        }
        else if(data.dataType() == opendlv::logic::perception::ObjectType::ID()){//Add the message to the new frame
            opendlv::logic::perception::ObjectType type = cluon::extractMessage<opendlv::logic::perception::ObjectType>(std::move(data));
            uint32_t id = type.objectId();
            ConePackage conePacket;
            std::get<2>(conePacket) = type;
            m_currentFrame[id] = conePacket;
            m_envelopeCount[id]=1;
            m_numberOfItems = (m_numberOfItems<=id)?(id+1):(m_numberOfItems);
        }        
        std::thread coneCollector (&Collector::InitializeCollection,this); //Create a collection thread that assembles the frame
        coneCollector.detach();
    }
    else{
        std::cout << "Leaking frames wtf!!!" << std::endl;
    }

}
/*
Method that gathers all the cone objects that can be assumed to belong to the same frame
*/
void Collector::InitializeCollection(){
bool sleep = true;
auto start = std::chrono::system_clock::now();

  while(sleep)//Wait for m_timeDiffMillisecond to gather the whole frame
  {
    auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
    if(m_messageCount == m_numberOfItems*m_packetSize){//frame complete
        sleep = false;
    }
    if(elapsed.count() > m_timeDiffMilliseconds*1000){
        std::cout << "Timed out" << std::endl;
        sleep = false;
    }
  }
  GetCompleteFrame();
  SendFrame();
  m_newFrame = true;
}
/*Removes any incomplete cone packets from the frame i.e. cones without all the 3 messages*/
void Collector::GetCompleteFrame(){
    std::map<int,int>::iterator it2 = m_envelopeCount.begin();
    while(it2 != m_envelopeCount.end()){
        if(it2->second != static_cast<int>(m_packetSize)){
            m_currentFrame.erase(it2->first);
            std::cout << "Incomplete frame with id " << it2->first << " removed" << std::endl;
        }
        it2++;
    }
}

/*Sends the complete frame to the SLAM module*/
void Collector::SendFrame(){
    if(m_packetSize == 2){
        std::cout << "sending " << m_currentFrame.size() << " cones" << std::endl;
        m_module.recieveCombinedMessage(m_currentFrameTime,m_currentFrame);
    }else if(m_packetSize == 3){
        std::cout << "sending " << m_currentFrame.size() << " cones" << std::endl;
        m_module.recieveCombinedCvMessage(m_currentFrameTime,m_currentFrame);
    }
}
