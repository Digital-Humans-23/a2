//
// Created by Dongho Kang on 07.09.22.
//

#ifndef PYLOCO_ROBOTINFO_H
#define PYLOCO_ROBOTINFO_H

#include <string>
#include <vector>

#include "crl-basic/utils/mathDefs.h"

namespace pyloco {

struct RobotInfo {
    enum class Model { Dog = 0, Go1 = 1, Bob = 2 };
    std::string name;
    std::string rbsFilePath;
    std::string rsFilePath;  // if this is an empty string, we use default joint angle
    std::map<std::string, std::string> limbNames; //fl, hl, fr, hr;
    std::vector<int> jointIdxToLock; //indices of fixed joints
    double targetHeight = 0.5;
};

const std::vector<RobotInfo> robotInfos = {
    // dog.rbs
    {
        "Dog",                                        //
        PYLOCO_DATA_FOLDER "/robots/simple/dog.rbs",  //
        "",                                           //
        {{"fl", "tibia_0"}, {"hl", "tibia_1"},
         {"fr", "tibia_2"}, {"hr", "tibia_3"}},       //
        {},                                           //
        0.437                                         //
    },
    // go1.rbs
    {
        "Go1",                                     //
        PYLOCO_DATA_FOLDER "/robots/go1/go1.rbs",  //
        PYLOCO_DATA_FOLDER "/robots/go1/go1.rs",   //
        {{"fl", "FL_calf"}, {"hl", "RL_calf"},
         {"fr", "FR_calf"}, {"hr", "RR_calf"}},    //
        {},                                        //
        0.31,                                      //
    },
    // bob.rbs
    {
        "Bob",                                                              //
        PYLOCO_DATA_FOLDER "/robots/bob/bob.rbs",                           //
        "",                                                                 //
        {{"l", "lFoot"}, {"r", "rFoot"}},                                   //
        {},  //
        0.9                                                                 //
    },
};

}  // namespace pyloco

#endif  //PYLOCO_ROBOTINFO_H