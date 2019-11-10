/* 
#Copyright (C) <September, 2019>  <Paribartan Dhakal>

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU Affero General Public License as
#published by the Free Software Foundation, either version 3 of the
#License, or (at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Affero General Public License for more details.
*/

#ifndef ENV_H
#define ENV_H

#include <string>
#include <vector>
#include <map>
#include "node.h"

namespace tree
{

struct Index
{
    Index() {}
    Index(int i, int j) : i(i), j(j) {}
    int i;
    int j;

    bool operator<(const Index &right) const
    {
        return i < right.i || (i == right.i && j < right.j);
    }
};

class EnvClass
{
private:
    char env[10][10];
    
    enum direction
    {
        EAST,
        SOUTH,
        WEST,
        NORTH
    };
    
  

    std::map<int, std::string> actionNameMap;

    int height;
    int count; //this is needed to maintain the intersection of the visible blocks

    bool targetVisible;

    std::string self_id_;

public:
    EnvClass(std::string self_id); //constructor

    direction orientation;

    std::map<int, Index> wayPointMap;
    std::map<Index, int> q_wayPointMap;

    double q_table[5][5];

    std::vector<Index> visibleBlockFunction(Index, direction, int); //a lookup table that returns
                                                                    //all the visible blocks from a
                                                                    //certain orientation, height, and
                                                                    //a location

    Index initialWayPoint;
    Index currentWayPoint;

    bool getTargetVisible() { return targetVisible; }
    std::string getDirection(direction);
    int getIntDirection(direction);

    const int NUMBER_OF_WAYPOINTS = 5;
    const int NUMBER_OF_ACTIONS = 5;

    int rotate_counter;
    ReturnStatus rotate_status;

    unsigned int current_action;
    unsigned int previous_action;

    bool reset;

    int index; //for random index
    /****************action*********************/
    ReturnStatus rotate();
    ReturnStatus elevate();
    ReturnStatus de_elevate();
    ReturnStatus waypoint_translation();
    ReturnStatus end_episode();
    /****************endOfAction*********************/

    Index prevWaypoint;

    //condition
    ReturnStatus is_target_visible();

    //utility function for mapping a function with its string value(gotten from a BT file)
    ReturnStatus call_function(std::string);
    ReturnStatus call_condition(std::string);

    bool isTargetThere(); //calls a visibleBlock function and checks if target
                          //is present in the returned indexes.

    double findMaxQValue(int); //takes in waypoint (0-4) and action (0,4)
    void settingEnvironment();
    void printEnvironment();
    void printQTable();
};

} // namespace tree

#endif