


#ifndef DRAWER_HPP
#define DRAWER_HPP
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include "cone.hpp"
#include "slam.hpp"




class Drawer{
    public:
        Drawer(std::map<std::string,std::string> commandlineArgs, Slam &slam);
        //void drawRawPoints();
        //void drawROIPoints();
        //void drawRANSACPoints();
        //void drawCones();

    private:
        Slam& slam;

};
#endif