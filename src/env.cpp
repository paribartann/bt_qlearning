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

#define ROTATE_COUNT 4
#define ALPHA 0.1
#define GAMMA 0.9

#include <iostream>
#include <random>
#include "../include/env.h"

using namespace std;
using namespace tree;

std::ostream &operator<<(std::ostream &os, const Index &pt)
{
    return os << "[" << pt.i << ", " << pt.j << "]";
}

EnvClass::EnvClass(std::string self_id)
{ //CONSTRUCTOR

    settingEnvironment();

    self_id_ = self_id;

    targetVisible = false;

    reset = false;

    random_device dev;
    mt19937 rng(dev());

    uniform_int_distribution<mt19937::result_type> dist3(0, 3);
    orientation = direction(dist3(rng));
    cout << "INITIAL ORIENTATION in constructor: " << orientation << endl;

    //this portion is generating a random number between 0 and 1
    uniform_int_distribution<mt19937::result_type> dist1(0, 1);
    height = dist1(rng);
    cout << "INITIAL HEIGHT in constructor: " << height << endl;

    count = 0;

    initialWayPoint = wayPointMap[0];
    currentWayPoint = wayPointMap[0];

    rotate_status = IDLE;
    rotate_counter = 0;

    current_action = 0;
    previous_action = 0;
}

void EnvClass::settingEnvironment()
{
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            env[i][j] = '0';
        }
    }

    for (int i = 0; i < 40; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            q_table[i][j] = 0.0;
        }
    }

    //(3,7) is the goal
    env[3][7] = 'G';

    //these are the abandoned area / obstacles
    env[8][3] = 'X';
    env[5][4] = 'X';
    env[3][5] = 'X';
    env[2][6] = 'X';
    env[0][9] = 'X';

    //these are the waitpoints
    wayPointMap[0] = Index(6, 2);
    wayPointMap[1] = Index(3, 2);
    wayPointMap[2] = Index(0, 6);
    wayPointMap[3] = Index(3, 8);
    wayPointMap[4] = Index(5, 7);

    initialiseQTableMap();


    actionNameMap[0] = "RotateA";
    actionNameMap[1] = "elevate";
    actionNameMap[2] = "RotateB";
    actionNameMap[3] = "de-elevate";
    actionNameMap[4] = "wayPointTranslate";

    env[6][2] = '1';
    env[3][2] = '2';
    env[0][6] = '3';
    env[3][8] = '4';
    env[8][7] = '5';

    //printEnvironment();
}

ReturnStatus EnvClass::is_target_visible()
{
    cout << "here in is_target_visible()" << endl;
    bool flag = isTargetThere();
    if (flag)
        return tree::SUCCESS;
    else
    {
        cout << "####TARGET NOT FOUND####" << endl;
        return tree::FAILURE;
    }
}

ReturnStatus EnvClass::end_episode()
{
    cout << "here in end_episode()" << endl;
    cout << "**************** The Target has been found *****************\n";
    cout << " *************************\n";

    cout << "* Found Location: "
         << "(" << currentWayPoint.i << ", " << currentWayPoint.j << ") *\n";
    cout << "* Drone is Facing: " << getDirection(orientation) << "  *" << endl;
    cout << "* Drone's Height is:" << (height == 1 ? "high" : "low") << "  *" << endl;
    cout << "The index of the last action: " << current_action << endl;
    cout << "The last action: " << actionNameMap[current_action] << endl;
    cout << " ************************\n";

    cout << "END EP where the target was found WAYPOINT == " << currentWayPoint.i << " " << currentWayPoint.j << endl;
    /* Q-Learning Code Goes here */

    Index wayP;
    if (current_action == 4)
    {
        wayP = prevWaypoint;
    }
    else
    {
        wayP = currentWayPoint;
    }

    //this is for the wayP and current action/////////
    heightDirection.first = height;
    heightDirection.second = orientation;
    std::pair<Index, std::pair<int,direction>> state;
    state.first = wayP;
    state.second = heightDirection;

    //cout<<q_wayPointHeightDirectionMap[state];
    ///////////////////////////////////////////////////

    cout << "Updating the q-table of  WAYPOINT == " << wayP.i << " " << wayP.j << endl;

    q_table[q_wayPointHeightDirectionMap[state]][current_action] += ALPHA * (10 - q_table[q_wayPointHeightDirectionMap[state]][current_action]);


    //q_table[q_wayPointMap[wayP]][current_action] += ALPHA * (10 - q_table[q_wayPointMap[currentWayPoint]][previous_action]);

    //setting the waypoint to its initial value here
    currentWayPoint = initialWayPoint;

    cout << "CURRENT == " << currentWayPoint.i << " " << currentWayPoint.j << endl;

    random_device dev;
    mt19937 rng(dev());

    uniform_int_distribution<mt19937::result_type> dist3(0, 3);
    orientation = direction(dist3(rng));
    cout << " ORIENTATION after end_episode: " << height << endl;

    //this portion is generating a random number between 0 and 1
    uniform_int_distribution<mt19937::result_type> dist1(0, 1);
    height = dist1(rng);
    cout << " HEIGHT after end_episode: " << height << endl;

    targetVisible = true;

    reset = true;

    printQTable();

    return SUCCESS;
}

ReturnStatus EnvClass::rotate()
{
    cout << "here in rotate.\n";
    cout << "Previous Orientation is = " << getDirection(orientation) << endl;
    direction prev_orientation = orientation;
    if (rotate_status != RUNNING)
    {
        rotate_counter = ROTATE_COUNT;
    }

    if (rotate_counter > 0)
    {
        orientation = getIntDirection(orientation) < 3 ? direction(((getIntDirection(orientation)) % 3) + 1)
                                                       : direction(((getIntDirection(orientation)) % 3));
    }
    rotate_counter--;
    if (rotate_counter > 0)
        rotate_status = RUNNING;
    else
        rotate_status = SUCCESS;

    cout << "Current Orientation is = " << getDirection(orientation) << endl;

    /* Q-Learning Code Goes here */

    //this is the previous state before the orientaion is changed/////////
    cout<<"PREV ORI = "<<prev_orientation<<endl;
    heightDirection.first = height;
    heightDirection.second = prev_orientation;
    std::pair<Index, std::pair<int,direction>> prev_state;
    prev_state.first = currentWayPoint;
    prev_state.second = heightDirection;

    cout<<"Updating STATE = "<<q_wayPointHeightDirectionMap[prev_state]<<endl;
    ///////////////////////////////////////////////////


    //this is next state after the orientation is changed/////////
    cout<<"NEW ORI = "<<orientation<<endl;
    heightDirection.first = height;
    heightDirection.second = orientation;
    std::pair<Index, std::pair<int,direction>> new_state;
    new_state.first = currentWayPoint;
    new_state.second = heightDirection;

    cout<<"New STATE = "<<q_wayPointHeightDirectionMap[new_state]<<endl;


    
    ///////////////////////////////////////////////////

    
    cout << "Updating the q-table of  WAYPOINT in rotate == " << currentWayPoint.i << " " << currentWayPoint.j << " and action is " << current_action << endl;

    q_table[q_wayPointHeightDirectionMap[prev_state]][current_action] +=
        ALPHA * (0 + GAMMA * findMaxQValue(q_wayPointHeightDirectionMap[new_state]) - q_table[q_wayPointHeightDirectionMap[prev_state]][current_action]);

   
    printQTable();
    return rotate_status;
}

ReturnStatus EnvClass::elevate()
{
    cout << "here in elevate!!\n";
    cout << "Previous Drone's Height is:" << (height == 1 ? "high" : "low") << endl;
    int prev_height = height;
    if (height == 0)
        height = 1;
    cout << "Current Drone's Height is:" << (height == 1 ? "high" : "low") << endl;
    cout << "Updating the q-table of  WAYPOINT in elevate == " << currentWayPoint.i << " " << currentWayPoint.j << " and action is " << current_action << endl;



    ////this is the state before the height is changed//
    heightDirection.first = prev_height;
    heightDirection.second = orientation;
    std::pair<Index, std::pair<int,direction>> prev_state;
    prev_state.first = currentWayPoint;
    prev_state.second = heightDirection;

    cout<<q_wayPointHeightDirectionMap[prev_state];
    ///////////////////////////////////////////////////


    //this is the next state after the height is changed/////////
    heightDirection.first = height;
    heightDirection.second = orientation;
    std::pair<Index, std::pair<int,direction>> new_state;
    new_state.first = currentWayPoint;
    new_state.second = heightDirection;

    cout<<q_wayPointHeightDirectionMap[new_state];
    /////////////////////////////////////////////////////////////
    

    q_table[q_wayPointHeightDirectionMap[prev_state]][current_action] +=
        ALPHA * (0 + GAMMA * findMaxQValue(q_wayPointHeightDirectionMap[new_state]) - q_table[q_wayPointHeightDirectionMap[prev_state]][current_action]);

    
    printQTable();
    return SUCCESS;
}

tree::ReturnStatus EnvClass::de_elevate()
{
    cout << "here in de-elevate!!\n";
    cout << "Previous Drone's Height is:" << (height == 1 ? "high" : "low") << endl;
    int prev_height = height;
    if (height == 1)
        height = 0;

    cout << "Current Drone's Height is:" << (height == 1 ? "high" : "low") << endl;

    cout << "Updating the q-table of  WAYPOINT in de-elevate == " << currentWayPoint.i << " " << currentWayPoint.j << " and action is " << current_action << endl;

   //this is the state before the height is changed/////////
    heightDirection.first = prev_height;
    heightDirection.second = orientation;
    std::pair<Index, std::pair<int,direction>> prev_state;
    prev_state.first = currentWayPoint;
    prev_state.second = heightDirection;

    cout<<q_wayPointHeightDirectionMap[prev_state];
    ///////////////////////////////////////////////////


    //this is the next state after the height is changed/////////
    heightDirection.first = height;
    heightDirection.second = orientation;
    std::pair<Index, std::pair<int,direction>> new_state;
    new_state.first = currentWayPoint;
    new_state.second = heightDirection;

    cout<<q_wayPointHeightDirectionMap[new_state];
  /////////////////////////////////////////////////////
    

    q_table[q_wayPointHeightDirectionMap[prev_state]][current_action] +=
        ALPHA * (0 + GAMMA * findMaxQValue(q_wayPointHeightDirectionMap[new_state]) - q_table[q_wayPointHeightDirectionMap[prev_state]][current_action]);

    printQTable();
    return SUCCESS;
}

ReturnStatus EnvClass::waypoint_translation()
{

    cout << "here in waypoint translation\n"
         << endl;
    cout << "Previous Location: "
         << "(" << currentWayPoint.i << ", " << currentWayPoint.j << ")\n";

    //int index = 4;
    prevWaypoint = currentWayPoint; //saving the current waypoint
    if ((currentWayPoint.i == wayPointMap[0].i) && (currentWayPoint.j == wayPointMap[0].j))
    {
        currentWayPoint = wayPointMap[1];
    }
    else if ((currentWayPoint.i == wayPointMap[1].i) && (currentWayPoint.j == wayPointMap[1].j))
    {
        currentWayPoint = wayPointMap[2];
    }
    else if ((currentWayPoint.i == wayPointMap[2].i) && (currentWayPoint.j == wayPointMap[2].j))
    {
        currentWayPoint = wayPointMap[3];
    }
    else if ((currentWayPoint.i == wayPointMap[3].i) && (currentWayPoint.j == wayPointMap[3].j))
    {
        currentWayPoint = wayPointMap[4];
    }
    else
    {
        currentWayPoint = wayPointMap[0];
    }

    //this is for the wayP and current action/////////
    heightDirection.first = height;
    heightDirection.second = orientation;
    std::pair<Index, std::pair<int,direction>> prev_state;
    prev_state.first = prevWaypoint;
    prev_state.second = heightDirection;

    cout<<q_wayPointHeightDirectionMap[prev_state];
    ///////////////////////////////////////////////////


    //this is for the wayP and current action/////////
    heightDirection.first = height;
    heightDirection.second = orientation;
    std::pair<Index, std::pair<int,direction>> new_state;
    new_state.first = currentWayPoint;
    new_state.second = heightDirection;

    cout<<q_wayPointHeightDirectionMap[new_state];
    ///////////////////////////////////////////////////

    q_table[q_wayPointHeightDirectionMap[prev_state]][current_action] +=
        ALPHA * (0 + GAMMA * findMaxQValue(q_wayPointHeightDirectionMap[new_state]) - q_table[q_wayPointHeightDirectionMap[prev_state]][current_action]);


    printQTable();
    return SUCCESS;
}

//JUST A HELPER FUNCTION
bool EnvClass::isTargetThere()
{
    bool flagA = false;
    vector<Index> returnBlocks;
    cout << "*************************************************************************************************************\n";
    cout << "Visible Index and its content from current location (" << currentWayPoint.i << ", " << currentWayPoint.j << ")." << endl;
    cout << "Drone is facing " << getDirection(orientation) << " and and is in height = " << (height == 1 ? "high" : "low") << endl;
    returnBlocks = visibleBlockFunction(currentWayPoint, orientation, height);

    for (unsigned long i = 0; i < returnBlocks.size(); i++)
    {
        if ((returnBlocks[i].i >= 0 && returnBlocks[i].i <= 9) && (returnBlocks[i].j >= 0 && returnBlocks[i].j <= 9))
        {
            cout << "Coordinate: " << returnBlocks[i] << endl;
            cout << "   Content: " << env[returnBlocks[i].i][returnBlocks[i].j] << endl;
            if (env[returnBlocks[i].i][returnBlocks[i].j] == 'G')
            {
                flagA = true;
            }
        }
    }
    cout << "*************************************************************************************************************\n";
    return flagA;
}

string EnvClass::getDirection(direction ori)
{
    string dir = "";
    if (ori == 0)
    {
        dir = "EAST";
    }
    else if (ori == 1)
    {
        dir = "SOUTH";
    }
    else if (ori == 2)
    {
        dir = "WEST";
    }
    else if (ori == 3)
    {
        dir = "NORTH";
    }
    else
    {
        dir = "INVALID!!!!";
    }

    return dir;
}

int EnvClass::getIntDirection(direction ori)
{
    int dir = 0;
    if (ori == 0)
    {
        dir = 0;
    }
    else if (ori == 1)
    {
        dir = 1;
    }
    else if (ori == 2)
    {
        dir = 2;
    }
    else if (ori == 3)
    {
        dir = 3;
    }
    else
    {
        dir = -1;
    }

    return dir;
}

//utility functions
ReturnStatus EnvClass::call_function(string function_name)
{
    typedef tree::ReturnStatus (EnvClass::*F_ptr)();
    std::map<std::string, F_ptr> myMap;

    myMap["A-rotate"] = &EnvClass::rotate;
    myMap["A-elevate"] = &EnvClass::elevate;
    myMap["A-deElevate"] = &EnvClass::de_elevate;
    myMap["A-waypointTranslation"] = &EnvClass::waypoint_translation;
    myMap["A-endEpisode"] = &EnvClass::end_episode;

    F_ptr fun = myMap[function_name];

    return (this->*fun)();
}

ReturnStatus EnvClass::call_condition(string function_name)
{
    typedef tree::ReturnStatus (EnvClass::*F_ptr)();
    std::map<std::string, F_ptr> myMap;

    myMap["C-seeTarget"] = &EnvClass::is_target_visible;

    F_ptr fun = myMap[function_name];

    return (this->*fun)();
}

vector<Index> EnvClass::visibleBlockFunction(Index index, direction orientation, int height)
{

    int i = index.i;
    int j = index.j;
    count = 0;

    vector<Index> CommonVisibleBlocks;
    vector<Index> visibleBlocks;
    map<Index, int> countOccurences;

    /* ------------------- BEGINNING OF THE LOWER HEIGHT --------------  */
    if (height == 0)
    {

        if (orientation == EAST)
        { //this is east

            if (env[i][j + 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 2, j + 3));
                visibleBlocks.push_back(Index(i + 2, j + 3));
            }
            if (env[i - 1][j + 1] == 'X' || env[i - 1][j + 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 1));

                visibleBlocks.push_back(Index(i, j + 2));
                visibleBlocks.push_back(Index(i + 1, j + 2));

                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));
                visibleBlocks.push_back(Index(i + 2, j + 3));
            }
            if (env[i + 1][j + 1] == 'X' || env[i + 1][j + 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 1));

                visibleBlocks.push_back(Index(i - 1, j + 2));
                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i - 2, j + 3));
                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
            }
            if (env[i][j + 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 1));

                visibleBlocks.push_back(Index(i - 1, j + 2));
                visibleBlocks.push_back(Index(i + 1, j + 2));

                visibleBlocks.push_back(Index(i - 2, j + 3));
                visibleBlocks.push_back(Index(i + 2, j + 3));
            }
            if (env[i - 2][j + 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 1));

                visibleBlocks.push_back(Index(i - 1, j + 2));
                visibleBlocks.push_back(Index(i, j + 2));
                visibleBlocks.push_back(Index(i + 1, j + 2));

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));
                visibleBlocks.push_back(Index(i + 2, j + 3));
            }
            if (env[i - 1][j + 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 1));
                visibleBlocks.push_back(Index(i - 1, j + 2));
                visibleBlocks.push_back(Index(i, j + 2));
                visibleBlocks.push_back(Index(i + 1, j + 2));
                visibleBlocks.push_back(Index(i - 2, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));
                visibleBlocks.push_back(Index(i + 2, j + 3));
            }
            if (env[i][j + 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 1));
                visibleBlocks.push_back(Index(i - 1, j + 2));
                visibleBlocks.push_back(Index(i, j + 2));
                visibleBlocks.push_back(Index(i + 1, j + 2));
                visibleBlocks.push_back(Index(i - 2, j + 3));
                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));
                visibleBlocks.push_back(Index(i + 2, j + 3));
            }
            if (env[i + 1][j + 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 1));
                visibleBlocks.push_back(Index(i - 1, j + 2));
                visibleBlocks.push_back(Index(i, j + 2));
                visibleBlocks.push_back(Index(i + 1, j + 2));
                visibleBlocks.push_back(Index(i - 2, j + 3));
                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 2, j + 3));
            }
            if (env[i + 2][j + 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 1));
                visibleBlocks.push_back(Index(i - 1, j + 2));
                visibleBlocks.push_back(Index(i, j + 2));
                visibleBlocks.push_back(Index(i + 1, j + 2));
                visibleBlocks.push_back(Index(i - 2, j + 3));
                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));
            }
            if (count == 0)
            {
                visibleBlocks.push_back(Index(i, j + 1));
                visibleBlocks.push_back(Index(i - 1, j + 2));
                visibleBlocks.push_back(Index(i, j + 2));
                visibleBlocks.push_back(Index(i + 1, j + 2));
                visibleBlocks.push_back(Index(i - 2, j + 3));
                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));
                visibleBlocks.push_back(Index(i + 2, j + 3));
            }
        }
        else if (orientation == WEST)
        { //this is west

            if (env[i][j - 1] == 'X')
            {

                count++;
                visibleBlocks.push_back(Index(i - 2, j - 3));
                visibleBlocks.push_back(Index(i + 2, j - 3));
            }

            if (env[i - 1][j - 1] == 'X' || env[i - 1][j - 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 1));

                visibleBlocks.push_back(Index(i, j - 2));
                visibleBlocks.push_back(Index(i + 1, j - 2));

                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));
                visibleBlocks.push_back(Index(i + 2, j - 3));
            }

            if (env[i + 1][j - 1] == 'X' || env[i + 1][j - 2] == 'X')
            {
                visibleBlocks.push_back(Index(i, j - 1));

                visibleBlocks.push_back(Index(i - 1, j - 2));
                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i - 2, j - 3));
                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
            }
            if (env[i][j - 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 1));

                visibleBlocks.push_back(Index(i - 1, j - 2));
                visibleBlocks.push_back(Index(i + 1, j - 2));

                visibleBlocks.push_back(Index(i - 2, j - 3));
                visibleBlocks.push_back(Index(i + 2, j - 3));
            }
            if (env[i - 2][j - 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 1));

                visibleBlocks.push_back(Index(i - 1, j - 2));
                visibleBlocks.push_back(Index(i, j - 2));
                visibleBlocks.push_back(Index(i + 1, j - 2));

                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));
                visibleBlocks.push_back(Index(i + 2, j - 3));
            }
            if (env[i - 1][j - 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 1));

                visibleBlocks.push_back(Index(i - 1, j - 2));
                visibleBlocks.push_back(Index(i, j - 2));
                visibleBlocks.push_back(Index(i + 1, j - 2));

                visibleBlocks.push_back(Index(i - 2, j - 3));

                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));
                visibleBlocks.push_back(Index(i + 2, j - 3));
            }
            if (env[i][j - 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 1));

                visibleBlocks.push_back(Index(i - 1, j - 2));
                visibleBlocks.push_back(Index(i, j - 2));
                visibleBlocks.push_back(Index(i + 1, j - 2));

                visibleBlocks.push_back(Index(i - 2, j - 3));
                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));
                visibleBlocks.push_back(Index(i + 2, j - 3));
            }
            if (env[i + 1][j - 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 1));

                visibleBlocks.push_back(Index(i - 1, j - 2));
                visibleBlocks.push_back(Index(i, j - 2));
                visibleBlocks.push_back(Index(i + 1, j - 2));

                visibleBlocks.push_back(Index(i - 2, j - 3));
                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 2, j - 3));
            }
            if (env[i + 2][j - 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 1));

                visibleBlocks.push_back(Index(i - 1, j - 2));
                visibleBlocks.push_back(Index(i, j - 2));
                visibleBlocks.push_back(Index(i + 1, j - 2));

                visibleBlocks.push_back(Index(i - 2, j - 3));
                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));
            }
            if (count == 0)
            {
                visibleBlocks.push_back(Index(i, j - 1));

                visibleBlocks.push_back(Index(i - 1, j - 2));
                visibleBlocks.push_back(Index(i, j - 2));
                visibleBlocks.push_back(Index(i + 1, j - 2));

                visibleBlocks.push_back(Index(i - 2, j - 3));
                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));
                visibleBlocks.push_back(Index(i + 2, j - 3));
            }
        }
        else if (orientation == NORTH)
        { //this is  north

            if (env[i - 1][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 3, j - 2));
                visibleBlocks.push_back(Index(i - 3, j + 2));
            }
            if (env[i - 1][j - 1] == 'X' || env[i - 2][j - 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 1, j));

                visibleBlocks.push_back(Index(i - 2, j - 1));
                visibleBlocks.push_back(Index(i - 2, j + 1));

                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));
                visibleBlocks.push_back(Index(i - 3, j + 2));
            }
            if (env[i - 1][j + 1] == 'X' || env[i - 2][j + 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 1, j));

                visibleBlocks.push_back(Index(i - 2, j - 1));
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j - 2));
                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
            }

            if (env[i - 2][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 1, j));

                visibleBlocks.push_back(Index(i - 2, j - 1));
                visibleBlocks.push_back(Index(i - 2, j + 1));

                visibleBlocks.push_back(Index(i - 3, j - 2));
                visibleBlocks.push_back(Index(i - 3, j + 2));
            }
            if (env[i - 3][j - 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 1, j));

                visibleBlocks.push_back(Index(i - 2, j - 1));
                visibleBlocks.push_back(Index(i - 2, j));
                visibleBlocks.push_back(Index(i - 2, j + 1));

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));
                visibleBlocks.push_back(Index(i - 3, j + 2));
            }
            if (env[i - 3][j - 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 1, j));

                visibleBlocks.push_back(Index(i - 2, j - 1));
                visibleBlocks.push_back(Index(i - 2, j));
                visibleBlocks.push_back(Index(i - 2, j + 1));

                visibleBlocks.push_back(Index(i - 3, j - 2));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));
                visibleBlocks.push_back(Index(i - 3, j + 2));
            }
            if (env[i - 3][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 1, j));

                visibleBlocks.push_back(Index(i - 2, j - 1));
                visibleBlocks.push_back(Index(i - 2, j));
                visibleBlocks.push_back(Index(i - 2, j + 1));

                visibleBlocks.push_back(Index(i - 3, j - 2));
                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j + 1));
                visibleBlocks.push_back(Index(i - 3, j + 2));
            }
            if (env[i - 3][j + 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 1, j));

                visibleBlocks.push_back(Index(i - 2, j - 1));
                visibleBlocks.push_back(Index(i - 2, j));
                visibleBlocks.push_back(Index(i - 2, j + 1));

                visibleBlocks.push_back(Index(i - 3, j - 2));
                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 2));
            }
            if (env[i - 3][j + 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 1, j));

                visibleBlocks.push_back(Index(i - 2, j - 1));
                visibleBlocks.push_back(Index(i - 2, j));
                visibleBlocks.push_back(Index(i - 2, j + 1));

                visibleBlocks.push_back(Index(i - 3, j - 2));
                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));
            }
            if (count == 0)
            {
                visibleBlocks.push_back(Index(i - 1, j));

                visibleBlocks.push_back(Index(i - 2, j - 1));
                visibleBlocks.push_back(Index(i - 2, j));
                visibleBlocks.push_back(Index(i - 2, j + 1));

                visibleBlocks.push_back(Index(i - 3, j - 2));
                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));
                visibleBlocks.push_back(Index(i - 3, j + 2));
            }
        }
        else if (orientation == SOUTH)
        { //this is south

            if (env[i + 1][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 3, j - 2));
                visibleBlocks.push_back(Index(i + 3, j + 2));
            }

            if (env[i + 1][j - 1] == 'X' || env[i + 2][j - 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 1, j));

                visibleBlocks.push_back(Index(i + 2, j + 1));
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));
                visibleBlocks.push_back(Index(i + 3, j + 2));
            }

            if (env[i + 1][j + 1] == 'X' || env[i + 2][j + 1] == 'X')
            {

                count++;
                visibleBlocks.push_back(Index(i + 1, j));

                visibleBlocks.push_back(Index(i + 2, j - 1));
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j - 2));
                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
            }

            if (env[i + 2][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 1, j));

                visibleBlocks.push_back(Index(i + 2, j - 1));
                visibleBlocks.push_back(Index(i + 2, j + 1));

                visibleBlocks.push_back(Index(i + 3, j - 2));
                visibleBlocks.push_back(Index(i + 3, j + 2));
            }
            if (env[i + 3][j - 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 1, j));

                visibleBlocks.push_back(Index(i + 2, j - 1));
                visibleBlocks.push_back(Index(i + 2, j));
                visibleBlocks.push_back(Index(i + 2, j + 1));

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));
                visibleBlocks.push_back(Index(i + 3, j + 2));
            }
            if (env[i + 3][j - 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 1, j));

                visibleBlocks.push_back(Index(i + 2, j - 1));
                visibleBlocks.push_back(Index(i + 2, j));
                visibleBlocks.push_back(Index(i + 2, j + 1));

                visibleBlocks.push_back(Index(i + 3, j - 2));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));
                visibleBlocks.push_back(Index(i + 3, j + 2));
            }
            if (env[i + 3][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 1, j));

                visibleBlocks.push_back(Index(i + 2, j - 1));
                visibleBlocks.push_back(Index(i + 2, j));
                visibleBlocks.push_back(Index(i + 2, j + 1));

                visibleBlocks.push_back(Index(i + 3, j - 2));
                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j + 1));
                visibleBlocks.push_back(Index(i + 3, j + 2));
            }
            if (env[i + 3][j + 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 1, j));

                visibleBlocks.push_back(Index(i + 2, j - 1));
                visibleBlocks.push_back(Index(i + 2, j));
                visibleBlocks.push_back(Index(i + 2, j + 1));

                visibleBlocks.push_back(Index(i + 3, j - 2));
                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 2));
            }
            if (env[i + 3][j + 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 1, j));

                visibleBlocks.push_back(Index(i + 2, j - 1));
                visibleBlocks.push_back(Index(i + 2, j));
                visibleBlocks.push_back(Index(i + 2, j + 1));

                visibleBlocks.push_back(Index(i + 3, j - 2));
                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));
            }
            if (count == 0)
            {
                visibleBlocks.push_back(Index(i + 1, j));

                visibleBlocks.push_back(Index(i + 2, j - 1));
                visibleBlocks.push_back(Index(i + 2, j));
                visibleBlocks.push_back(Index(i + 2, j + 1));

                visibleBlocks.push_back(Index(i + 3, j - 2));
                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));
                visibleBlocks.push_back(Index(i + 3, j + 2));
            }
        }
    }
    else /* ------------------- BEGINNING OF THE HIGHER HEIGHT --------------  */
    {
        if (orientation == EAST)
        {

            if (env[i][j + 1] == 'X')
            {
                count++;

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));

                visibleBlocks.push_back(Index(i - 2, j + 4));
                visibleBlocks.push_back(Index(i - 1, j + 4));
                visibleBlocks.push_back(Index(i, j + 4));
                visibleBlocks.push_back(Index(i + 1, j + 4));
                visibleBlocks.push_back(Index(i + 2, j + 4));
            }
            if (env[i][j + 2] == 'X')
            {
                count++;

                visibleBlocks.push_back(Index(i - 2, j + 4));
                visibleBlocks.push_back(Index(i + 2, j + 4));
            }
            if (env[i][j + 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));

                visibleBlocks.push_back(Index(i - 2, j + 4));
                visibleBlocks.push_back(Index(i + 2, j + 4));
            }
            if (env[i - 1][j + 2] == 'X' || env[i - 1][j + 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));

                visibleBlocks.push_back(Index(i, j + 4));
                visibleBlocks.push_back(Index(i + 1, j + 4));
                visibleBlocks.push_back(Index(i + 2, j + 4));
            }
            if (env[i + 1][j + 2] == 'X' || env[i + 1][j + 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));

                visibleBlocks.push_back(Index(i - 2, j + 4));
                visibleBlocks.push_back(Index(i - 1, j + 4));
                visibleBlocks.push_back(Index(i, j + 4));
            }
            if (env[i - 2][j + 4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));

                visibleBlocks.push_back(Index(i - 1, j + 4));
                visibleBlocks.push_back(Index(i, j + 4));
                visibleBlocks.push_back(Index(i + 1, j + 4));
                visibleBlocks.push_back(Index(i + 2, j + 4));
            }
            if (env[i - 1][j + 4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));

                visibleBlocks.push_back(Index(i - 2, j + 4));
                visibleBlocks.push_back(Index(i, j + 4));
                visibleBlocks.push_back(Index(i + 1, j + 4));
                visibleBlocks.push_back(Index(i + 2, j + 4));
            }
            if (env[i][j + 4] == 'X')
            {
                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));

                visibleBlocks.push_back(Index(i - 2, j + 4));
                visibleBlocks.push_back(Index(i - 1, j + 4));
                visibleBlocks.push_back(Index(i + 1, j + 4));
                visibleBlocks.push_back(Index(i + 2, j + 4));
            }
            if (env[i + 1][j + 4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));

                visibleBlocks.push_back(Index(i - 2, j + 4));
                visibleBlocks.push_back(Index(i - 1, j + 4));
                visibleBlocks.push_back(Index(i, j + 4));
                visibleBlocks.push_back(Index(i + 2, j + 4));
            }
            if (env[i + 2][j + 4] == 'X')
            {

                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));

                visibleBlocks.push_back(Index(i - 2, j + 4));
                visibleBlocks.push_back(Index(i - 1, j + 4));
                visibleBlocks.push_back(Index(i, j + 4));
                visibleBlocks.push_back(Index(i + 1, j + 4));
            }

            if (count == 0)
            {
                visibleBlocks.push_back(Index(i, j + 2));

                visibleBlocks.push_back(Index(i - 1, j + 3));
                visibleBlocks.push_back(Index(i, j + 3));
                visibleBlocks.push_back(Index(i + 1, j + 3));

                visibleBlocks.push_back(Index(i - 2, j + 4));
                visibleBlocks.push_back(Index(i - 1, j + 4));
                visibleBlocks.push_back(Index(i, j + 4));
                visibleBlocks.push_back(Index(i + 1, j + 4));
                visibleBlocks.push_back(Index(i + 2, j + 4));
            }
        }
        else if (orientation == WEST)
        { //this is west

            if (env[i][j - 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));

                visibleBlocks.push_back(Index(i - 2, j - 4));
                visibleBlocks.push_back(Index(i - 1, j - 4));
                visibleBlocks.push_back(Index(i, j - 4));
                visibleBlocks.push_back(Index(i + 1, j - 4));
                visibleBlocks.push_back(Index(i + 2, j - 4));
            }
            if (env[i][j - 2] == 'X')
            {
                count++;

                visibleBlocks.push_back(Index(i - 2, j - 4));
                visibleBlocks.push_back(Index(i + 2, j - 4));
            }
            if (env[i][j - 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));

                visibleBlocks.push_back(Index(i - 2, j - 4));
                visibleBlocks.push_back(Index(i + 2, j - 4));
            }
            if (env[i - 1][j - 2] == 'X' || env[i - 1][j - 3] == 'X')
            {
                count++;

                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));

                visibleBlocks.push_back(Index(i, j - 4));
                visibleBlocks.push_back(Index(i + 1, j - 4));
                visibleBlocks.push_back(Index(i + 2, j - 4));
            }
            if (env[i + 1][j - 2] == 'X' || env[i + 1][j - 3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));

                visibleBlocks.push_back(Index(i - 2, j - 4));
                visibleBlocks.push_back(Index(i - 1, j - 4));
                visibleBlocks.push_back(Index(i, j - 4));
            }
            if (env[i - 1][j - 4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));

                visibleBlocks.push_back(Index(i - 1, j - 4));
                visibleBlocks.push_back(Index(i, j - 4));
                visibleBlocks.push_back(Index(i + 1, j - 4));
                visibleBlocks.push_back(Index(i + 2, j - 4));
            }
            if (env[i - 1][j - 4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));

                visibleBlocks.push_back(Index(i - 2, j - 4));
                visibleBlocks.push_back(Index(i, j - 4));
                visibleBlocks.push_back(Index(i + 1, j - 4));
                visibleBlocks.push_back(Index(i + 2, j - 4));
            }
            if (env[i][j - 4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));

                visibleBlocks.push_back(Index(i - 2, j - 4));
                visibleBlocks.push_back(Index(i - 1, j - 4));
                visibleBlocks.push_back(Index(i + 1, j - 4));
                visibleBlocks.push_back(Index(i + 2, j - 4));
            }
            if (env[i + 1][j - 4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));

                visibleBlocks.push_back(Index(i - 2, j - 4));
                visibleBlocks.push_back(Index(i - 1, j - 4));
                visibleBlocks.push_back(Index(i, j - 4));
                visibleBlocks.push_back(Index(i + 2, j - 4));
            }
            if (env[i + 2][j - 4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));

                visibleBlocks.push_back(Index(i - 2, j - 4));
                visibleBlocks.push_back(Index(i - 1, j - 4));
                visibleBlocks.push_back(Index(i, j - 4));
                visibleBlocks.push_back(Index(i + 1, j - 4));
            }
            if (count == 0)
            {
                visibleBlocks.push_back(Index(i, j - 2));

                visibleBlocks.push_back(Index(i - 1, j - 3));
                visibleBlocks.push_back(Index(i, j - 3));
                visibleBlocks.push_back(Index(i + 1, j - 3));

                visibleBlocks.push_back(Index(i - 2, j - 4));
                visibleBlocks.push_back(Index(i - 1, j - 4));
                visibleBlocks.push_back(Index(i, j - 4));
                visibleBlocks.push_back(Index(i + 1, j - 4));
                visibleBlocks.push_back(Index(i + 2, j - 4));
            }
        }
        else if (orientation == NORTH)
        { //this is  north
            if (env[i - 1][j] == 'X')
            {

                count++;

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j + 1));

                visibleBlocks.push_back(Index(i - 4, j - 2));
                visibleBlocks.push_back(Index(i - 4, j - 1));
                visibleBlocks.push_back(Index(i - 4, j));
                visibleBlocks.push_back(Index(i - 4, j + 1));
                visibleBlocks.push_back(Index(i - 4, j + 2));
            }

            else if (env[i - 2][j] == 'X')
            {

                count++;

                visibleBlocks.push_back(Index(i - 4, j - 2));
                visibleBlocks.push_back(Index(i - 4, j + 2));
            }

            else if (env[i - 3][j] == 'X')
            {

                count++;
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j + 1));

                visibleBlocks.push_back(Index(i - 4, j - 2));
                visibleBlocks.push_back(Index(i - 4, j + 2));
            }

            else if (env[i - 2][j - 1] == 'X' || env[i - 3][j - 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));

                visibleBlocks.push_back(Index(i - 4, j));
                visibleBlocks.push_back(Index(i - 4, j + 1));
                visibleBlocks.push_back(Index(i - 4, j + 2));
            }

            else if (env[i - 2][j + 1] == 'X' || env[i - 3][j + 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));

                visibleBlocks.push_back(Index(i - 4, j - 2));
                visibleBlocks.push_back(Index(i - 4, j - 1));
                visibleBlocks.push_back(Index(i - 4, j));
            }
            if (env[i - 4][j - 2] == 'X')
            {

                count++;
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));

                visibleBlocks.push_back(Index(i - 4, j - 1));
                visibleBlocks.push_back(Index(i - 4, j));
                visibleBlocks.push_back(Index(i - 4, j + 1));
                visibleBlocks.push_back(Index(i - 4, j + 2));
            }
            if (env[i - 4][j - 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));

                visibleBlocks.push_back(Index(i - 4, j - 2));
                visibleBlocks.push_back(Index(i - 4, j));
                visibleBlocks.push_back(Index(i - 4, j + 1));
                visibleBlocks.push_back(Index(i - 4, j + 2));
            }
            if (env[i - 4][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));

                visibleBlocks.push_back(Index(i - 4, j - 2));
                visibleBlocks.push_back(Index(i - 4, j - 1));
                visibleBlocks.push_back(Index(i - 4, j + 1));
                visibleBlocks.push_back(Index(i - 4, j + 2));
            }
            if (env[i - 4][j + 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));

                visibleBlocks.push_back(Index(i - 4, j - 2));
                visibleBlocks.push_back(Index(i - 4, j - 1));
                visibleBlocks.push_back(Index(i - 4, j));
                visibleBlocks.push_back(Index(i - 4, j + 2));
            }
            if (env[i - 4][j + 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));

                visibleBlocks.push_back(Index(i - 4, j - 2));
                visibleBlocks.push_back(Index(i - 4, j - 1));
                visibleBlocks.push_back(Index(i - 4, j));
                visibleBlocks.push_back(Index(i - 4, j + 1));
            }

            if (count == 0)
            {
                visibleBlocks.push_back(Index(i - 2, j));

                visibleBlocks.push_back(Index(i - 3, j - 1));
                visibleBlocks.push_back(Index(i - 3, j));
                visibleBlocks.push_back(Index(i - 3, j + 1));

                visibleBlocks.push_back(Index(i - 4, j - 2));
                visibleBlocks.push_back(Index(i - 4, j - 1));
                visibleBlocks.push_back(Index(i - 4, j));
                visibleBlocks.push_back(Index(i - 4, j + 1));
                visibleBlocks.push_back(Index(i - 4, j + 2));
            }
        }
        else if (orientation == SOUTH)
        { //this is south

            if (env[i + 1][j] == 'X')
            {

                count++;

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j + 1));

                visibleBlocks.push_back(Index(i + 4, j - 2));
                visibleBlocks.push_back(Index(i + 4, j - 1));
                visibleBlocks.push_back(Index(i + 4, j));
                visibleBlocks.push_back(Index(i + 4, j + 1));
                visibleBlocks.push_back(Index(i + 4, j + 2));
            }

            if (env[i + 2][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 4, j - 2));
                visibleBlocks.push_back(Index(i + 4, j + 2));
            }

            if (env[i + 3][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j + 1));

                visibleBlocks.push_back(Index(i + 4, j - 2));
                visibleBlocks.push_back(Index(i + 4, j + 2));
            }

            if (env[i + 2][j - 1] == 'X' || env[i + 3][j - 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));

                visibleBlocks.push_back(Index(i + 4, j));
                visibleBlocks.push_back(Index(i + 4, j + 1));
                visibleBlocks.push_back(Index(i + 4, j + 2));
            }

            if (env[i + 2][j + 1] == 'X' || env[i + 3][j + 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));

                visibleBlocks.push_back(Index(i + 4, j - 2));
                visibleBlocks.push_back(Index(i + 4, j - 1));
                visibleBlocks.push_back(Index(i + 4, j));
            }

            if (env[i + 4][j - 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));

                visibleBlocks.push_back(Index(i + 4, j - 1));
                visibleBlocks.push_back(Index(i + 4, j));
                visibleBlocks.push_back(Index(i + 4, j + 1));
                visibleBlocks.push_back(Index(i + 4, j + 2));
            }
            if (env[i + 4][j - 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));

                visibleBlocks.push_back(Index(i + 4, j - 2));
                visibleBlocks.push_back(Index(i + 4, j));
                visibleBlocks.push_back(Index(i + 4, j + 1));
                visibleBlocks.push_back(Index(i + 4, j + 2));
            }
            if (env[i + 4][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));

                visibleBlocks.push_back(Index(i + 4, j - 2));
                visibleBlocks.push_back(Index(i + 4, j - 1));
                visibleBlocks.push_back(Index(i + 4, j + 1));
                visibleBlocks.push_back(Index(i + 4, j + 2));
            }
            if (env[i + 4][j + 1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));

                visibleBlocks.push_back(Index(i + 4, j - 2));
                visibleBlocks.push_back(Index(i + 4, j - 1));
                visibleBlocks.push_back(Index(i + 4, j));
                visibleBlocks.push_back(Index(i + 4, j + 2));
            }
            if (env[i + 4][j + 2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));

                visibleBlocks.push_back(Index(i + 4, j - 2));
                visibleBlocks.push_back(Index(i + 4, j - 1));
                visibleBlocks.push_back(Index(i + 4, j));
                visibleBlocks.push_back(Index(i + 4, j + 1));
            }

            if (count == 0)
            {
                visibleBlocks.push_back(Index(i + 2, j));

                visibleBlocks.push_back(Index(i + 3, j - 1));
                visibleBlocks.push_back(Index(i + 3, j));
                visibleBlocks.push_back(Index(i + 3, j + 1));

                visibleBlocks.push_back(Index(i + 4, j - 2));
                visibleBlocks.push_back(Index(i + 4, j - 1));
                visibleBlocks.push_back(Index(i + 4, j));
                visibleBlocks.push_back(Index(i + 4, j + 1));
                visibleBlocks.push_back(Index(i + 4, j + 2));
            }

        } //end of else for HIGH SOUTH FACING
    }     //end of else for HIGH HEIGHT

    if (count > 1)
    {

        for (const auto &p : visibleBlocks)
        {
            ++countOccurences[p];
        }

        for (const auto &p : countOccurences)
        {
            const auto &p1 = p.first;

            int countO = p.second;
            if (countO == count)
            {
                CommonVisibleBlocks.push_back(p1);
            }
        }
        return CommonVisibleBlocks;
    }

    return visibleBlocks;
}

double EnvClass::findMaxQValue(int state)
{
    double max_val = q_table[state][0];
    //cout<<"NUM_ACTIONS == "<<NUM_ACTIONS<<endl;
    for (int i = 1; i < NUMBER_OF_ACTIONS; i++)
    {
        if (max_val < q_table[state][i])
        {
            max_val = q_table[state][i];
        }
    }
    return max_val;
}

void EnvClass::printEnvironment()
{
    cout << "\n";

    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            cout << env[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl
         << endl;
}

void EnvClass::printQTable()
{
    cout << "\n";

    cout << "Q_TABLE :::" << endl;
    for (int i = 0; i < 40; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            cout << q_table[i][j] << " ";
        }
        cout << endl;
        if(i % 4 == 3) cout<<endl;
        if(i % 8 == 7) cout<<endl<<endl;
    }
    cout << endl
         << endl;
}


void EnvClass::initialiseQTableMap()
{


    //Index(6,2)
    heightDirection.first = 0;      //height = low
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(6,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 0;

    heightDirection.first = 0;      //height = low
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(6,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 1;


    heightDirection.first = 0;      //height = low
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(6,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 2;

    heightDirection.first = 0;      //height = low
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(6,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 3;


    //height = high
    heightDirection.first = 1;      //height = high
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(6,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 4;

    heightDirection.first = 1;      //height = high
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(6,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 5;


    heightDirection.first = 1;      //height = high
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(6,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 6;

    heightDirection.first = 1;      //height = high
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(6,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 7;


    //Index(3,2)
    heightDirection.first = 0;      //height = low
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(3,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 8;

    heightDirection.first = 0;      //height = low
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(3,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 9;


    heightDirection.first = 0;      //height = low
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(3,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 10;

    heightDirection.first = 0;      //height = low
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(3,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 11;


    //height = high
    heightDirection.first = 1;      //height = high
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(3,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 12;

    heightDirection.first = 1;      //height = high
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(3,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 13;


    heightDirection.first = 1;      //height = high
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(3,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 14;

    heightDirection.first = 1;      //height = high
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(3,2);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 15;


    //Index(0,6)

    heightDirection.first = 0;      //height = low
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(0,6);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 16;

    heightDirection.first = 0;      //height = low
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(0,6);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 17;


    heightDirection.first = 0;      //height = low
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(0,6);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 18;

    heightDirection.first = 0;      //height = low
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(0,6);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 19;


    //height = high
    heightDirection.first = 1;      //height = high
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(0,6);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 20;

    heightDirection.first = 1;      //height = high
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(0,6);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 21;


    heightDirection.first = 1;      //height = high
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(0,6);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 22;

    heightDirection.first = 1;      //height = high
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(0,6);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 23;


    //Index(3,8)

    heightDirection.first = 0;      //height = low
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(3,8);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 24;

    heightDirection.first = 0;      //height = low
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(3,8);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 25;


    heightDirection.first = 0;      //height = low
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(3,8);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 26;

    heightDirection.first = 0;      //height = low
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(3,8);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 27;

    //height = high
    heightDirection.first = 1;      //height = high
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(3,8);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 28;

    heightDirection.first = 1;      //height = high
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(3,8);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 29;


    heightDirection.first = 1;      //height = high
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(3,8);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 30;

    heightDirection.first = 1;      //height = high
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(3,8);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 31; 

    //Index(5,7)
    heightDirection.first = 0;      //height = low
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(5,7);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 32;

    heightDirection.first = 0;      //height = low
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(5,7);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 33;


    heightDirection.first = 0;      //height = low
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(5,7);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 34;

    heightDirection.first = 0;      //height = low
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(5,7);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 35;


    //height = high
    heightDirection.first = 1;      //height = high
    heightDirection.second = EAST;     //direction = east
    wayPointHeightDirection.first = Index(5,7);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 36;

    heightDirection.first = 1;      //height = high
    heightDirection.second = SOUTH;     //direction = south
    wayPointHeightDirection.first = Index(5,7);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 37;


    heightDirection.first = 1;      //height = high
    heightDirection.second = WEST;     //direction = west
    wayPointHeightDirection.first = Index(5,7);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 38;

    heightDirection.first = 1;      //height = high
    heightDirection.second = NORTH;     //direction = north
    wayPointHeightDirection.first = Index(5,7);
    wayPointHeightDirection.second = heightDirection;
    q_wayPointHeightDirectionMap[wayPointHeightDirection] = 39; 

}
