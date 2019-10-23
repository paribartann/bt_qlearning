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
#include "../include/env.h"

using namespace std;
using namespace tree;



std::ostream& operator<< (std::ostream& os, const Index& pt)
{
    return os << "[" << pt.i << ", " << pt.j << "]";
}


EnvClass::EnvClass( std::string self_id )
{

    settingEnvironment();

    self_id_ = self_id;
    targetVisible = false;
    orientation = WEST;
    height = 0;

    count = 0;

    initialWayPoint = wayPointMap[4];
    currentWayPoint = wayPointMap[4];

    rotate_status = IDLE;
    rotate_counter = 0;

    curr_child = 0;
    prev_child = 0;

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

    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            q_table[i][j] = 0;
        }
    }

    //(3,7) is the goal
    env[3][7] = 'G';

    //these are the abandoned area / obstacles
    env[8][3] = 'X';
    env[5][4] = 'X';
    env[3][5] = 'X';
    env[5][7] = 'X';
    env[2][6] = 'X';
    env[0][9] = 'X';

    //these are the waitpoints
    wayPointMap[0] = Index(6,2);
    wayPointMap[1] = Index(3,2);
    wayPointMap[2] = Index(0,6);
    wayPointMap[3] = Index(3,8);
    wayPointMap[4] = Index(8,7);


    // q_wayPointMap[Index(6,2]) = 0;
    // q_wayPointMap[Index(3,2)] = 1;
    // q_wayPointMap[Index(0,6)] = 2;
    // q_wayPointMap[Index(3,8)] = 3;
    // q_wayPointMap[Index(8,7)] = 4;


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

void EnvClass::printEnvironment()
{   
    cout<<"\n";
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            cout<<env[i][j]<<" ";
        }
        cout<<endl;
    }
        
}

ReturnStatus EnvClass::is_target_visible()
{
    cout<<"here in is_target_visible()"<<endl;
    bool flag = isTargetThere();
    if (flag) 
        return tree::SUCCESS;
    else
    {
        cout<<"####TARGET NOT FOUND####"<<endl;
        return tree::FAILURE;
    }
}

ReturnStatus EnvClass::end_episode()
{
    cout<<"here in end_episode()"<<endl;
    cout<<"**************** The Target has been found *****************\n";
    cout<<" *************************\n";

    cout<<"* Found Location: "<<"("<<currentWayPoint.i<<", "<<currentWayPoint.j<<") *\n";
    cout<<"* Drone is Facing: "<<getDirection(orientation)<<"  *"<<endl;
    cout<<"* Drone's Height is:"<<(height == 1 ? "high" : "low")<<"  *"<<endl;
    cout<<"The index of the last action: "<<prev_child<<endl;
    cout<<"The last action: "<<actionNameMap[prev_child]<<endl;
    cout<<" ************************\n";


    /* Q-Learning Code Goes here */

    /*
    q[s][a]
    q_table[currentWayPoint.key][prev_child] += ALPHA(10 - q_table[currentWayPoint.key][prev_child] );
    */

   //q_table[q_wayPointMap[currentWayPoint]][prev_child] += ALPHA * ( 10 - q_table[q_wayPointMap[currentWayPoint]][prev_child] );

    currentWayPoint = initialWayPoint;

    height = (height == 1 ? 0 : 1);

    targetVisible = true;

    return SUCCESS;
}

ReturnStatus EnvClass::rotate() 
{
   cout<<"here in rotate.\n";
   cout<<"Previous Orientation is = "<<getDirection(orientation)<<endl;
   
    if (rotate_status != RUNNING) {
	    rotate_counter = ROTATE_COUNT;
    }
    
    if (rotate_counter > 0) {

        if(orientation == EAST)
        {
            orientation = SOUTH;
        }
        else if(orientation == SOUTH)
        {
            orientation = WEST;
        }
        else if(orientation == WEST)
        {
            orientation = NORTH;
        }
        else if (orientation == NORTH)
        {
            orientation = EAST;
        }
        else
        {
            cout<<"INVALID ORIENTATION"<<endl;
            exit(0);
        }
    }
    rotate_counter--;
    if (rotate_counter > 0)
        rotate_status = RUNNING;
    else
	    rotate_status = SUCCESS;

 
    cout<<"Current Orientation is = "<<getDirection(orientation)<<endl;

    /* Q-Learning Code Goes here */

    /*
    q_table[q_wayPointMap[currentWayPoint]][curr_child] += 
                        ALPHA * (0 + GAMMA * maxof(q[q_wayPointMap[currentWayPoint]][b]) - q_table[q_wayPointMap[currentWayPoint]][curr_child]) ;

    */

    
    

    return rotate_status;
   
}


ReturnStatus EnvClass::elevate() 
{
   cout<<"here in elevate!!\n";
   cout<<"Previous Drone's Height is:"<<(height == 1 ? "high" : "low")<<endl;
    if (height == 0)
        height = 1;
   cout<<"Current Drone's Height is:"<<(height == 1 ? "high" : "low")<<endl;

   /* Q-Learning Code Goes here */

   /*
    q_table[q_wayPointMap[currentWayPoint]][curr_child] += 
                        ALPHA * (0 + GAMMA * maxof(q[q_wayPointMap[currentWayPoint]][b]) - q_table[q_wayPointMap[currentWayPoint]][curr_child]) ;

    */


   return SUCCESS;
}

tree::ReturnStatus EnvClass::de_elevate() 
{
   cout<<"here in de-elevate!!\n";
   cout<<"Previous Drone's Height is:"<<(height == 1 ? "high" : "low")<<endl;
    if (height == 1)
        height = 0;
   cout<<"Current Drone's Height is:"<<(height == 1 ? "high" : "low")<<endl;

   /* Q-Learning Code Goes here */

    /*
    q_table[q_wayPointMap[currentWayPoint]][curr_child] += 
                        ALPHA * (0 + GAMMA * maxof(q[q_wayPointMap[currentWayPoint]][b]) - q_table[q_wayPointMap[currentWayPoint]][curr_child]) ;

    */

   return SUCCESS;
}


ReturnStatus EnvClass::waypoint_translation() 
{

    //save current_waypoint -> s

    cout<<"here in waypoint translation\n"<<endl;
    cout<<"Previous Location: "<<"("<<currentWayPoint.i<<", "<<currentWayPoint.j<<")\n";

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

    cout<<"Current Location: "<<"("<<currentWayPoint.i<<", "<<currentWayPoint.j<<")\n";

    /* Q-Learning Code Goes here */

    /*
    
    q_table[s][idx] += ALPHA(0 + GAMMA * maxof(q[currentWayPoint.key][b]) - q_table[s][idx])
    */
    
    return SUCCESS;
}


//JUST A HELPER FUNCTION
bool EnvClass::isTargetThere() 
{
    bool flagA = false;
    vector<Index> returnBlocks;
    cout<<"*************************************************************************************************************\n";
    cout<<"Visible Index and its content from current location ("<<currentWayPoint.i<<", "<<currentWayPoint.j<<")."<<endl;
    cout<<"Drone is facing "<<getDirection(orientation)<<" and and is in height = "<<(height == 1 ? "high" : "low")<<endl;
    returnBlocks = visibleBlockFunction(currentWayPoint, orientation, height);  
    
    for(unsigned long i = 0; i < returnBlocks.size(); i++)
    {
        if( (returnBlocks[i].i >=0 && returnBlocks[i].i<=9) && (returnBlocks[i].j >=0 && returnBlocks[i].j<=9) )
        {
            cout<<"Coordinate: "<<returnBlocks[i]<<endl;
            cout<<"   Content: "<<env[returnBlocks[i].i][returnBlocks[i].j]<<endl;
            if(env[returnBlocks[i].i][returnBlocks[i].j] == 'G')
            {
                flagA = true;
            }
        }
        
    }
    cout<<"*************************************************************************************************************\n";
    return flagA;
}

string EnvClass::getDirection(direction ori)
{
    string dir = "";
    if(ori == 0){
        dir = "EAST";
    }
    else if(ori == 1){
        dir = "WEST";
    }
    else if(ori == 2){
        dir = "NORTH";
    }
    else if(ori == 3){
        dir = "SOUTH";
    }
    else
    {
        dir = "INVALID!!!!";
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
     if (height == 0){

        if(orientation == EAST) { //this is east

            if(env[i][j+1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-2,j+3));
                visibleBlocks.push_back(Index(i+2,j+3));
            }
            if(env[i-1][j+1] == 'X' || env[i-1][j+2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+1));

                visibleBlocks.push_back(Index(i,j+2));
                visibleBlocks.push_back(Index(i+1,j+2));

                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));
                visibleBlocks.push_back(Index(i+2,j+3));
            }
            if(env[i+1][j+1] == 'X' || env[i+1][j+2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+1));

                visibleBlocks.push_back(Index(i-1,j+2));
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i-2, j+3));
                visibleBlocks.push_back(Index(i-1, j+3));
                visibleBlocks.push_back(Index(i, j+3));
            }
            if(env[i][j+2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+1));

                visibleBlocks.push_back(Index(i-1,j+2));
                visibleBlocks.push_back(Index(i+1,j+2));

                visibleBlocks.push_back(Index(i-2,j+3));
                visibleBlocks.push_back(Index(i+2,j+3));
            }
            if(env[i-2][j+3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+1));
                
                visibleBlocks.push_back(Index(i-1,j+2));
                visibleBlocks.push_back(Index(i,j+2));
                visibleBlocks.push_back(Index(i+1,j+2));

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));
                visibleBlocks.push_back(Index(i+2,j+3));
            }
            if(env[i-1][j+3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+1));
                visibleBlocks.push_back(Index(i-1,j+2));
                visibleBlocks.push_back(Index(i,j+2));
                visibleBlocks.push_back(Index(i+1,j+2));
                visibleBlocks.push_back(Index(i-2,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));
                visibleBlocks.push_back(Index(i+2,j+3));
            }
            if(env[i][j+3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+1));
                visibleBlocks.push_back(Index(i-1,j+2));
                visibleBlocks.push_back(Index(i,j+2));
                visibleBlocks.push_back(Index(i+1,j+2));
                visibleBlocks.push_back(Index(i-2,j+3));
                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));
                visibleBlocks.push_back(Index(i+2,j+3));
            }
            if(env[i+1][j+3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+1));
                visibleBlocks.push_back(Index(i-1,j+2));
                visibleBlocks.push_back(Index(i,j+2));
                visibleBlocks.push_back(Index(i+1,j+2));
                visibleBlocks.push_back(Index(i-2,j+3));
                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+2,j+3));
            }
            if(env[i+2][j+3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+1));
                visibleBlocks.push_back(Index(i-1,j+2));
                visibleBlocks.push_back(Index(i,j+2));
                visibleBlocks.push_back(Index(i+1,j+2));
                visibleBlocks.push_back(Index(i-2,j+3));
                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));
            }
            if (count == 0)
            {
                visibleBlocks.push_back(Index(i,j+1));
                visibleBlocks.push_back(Index(i-1,j+2));
                visibleBlocks.push_back(Index(i,j+2));
                visibleBlocks.push_back(Index(i+1,j+2));
                visibleBlocks.push_back(Index(i-2,j+3));
                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));
                visibleBlocks.push_back(Index(i+2,j+3));
            }

        }
        else if (orientation == WEST) { //this is west

            if(env[i][j-1] == 'X'){

                count++;
                visibleBlocks.push_back(Index(i-2,j-3));
                visibleBlocks.push_back(Index(i+2,j-3));
                
            }

            if(env[i-1][j-1] == 'X' || env[i-1][j-2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-1));

                visibleBlocks.push_back(Index(i,j-2));
                visibleBlocks.push_back(Index(i+1,j-2));

                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));
                visibleBlocks.push_back(Index(i+2,j-3));
            }

            if(env[i+1][j-1] == 'X' || env[i+1][j-2] == 'X')
            {
                visibleBlocks.push_back(Index(i,j-1));

                visibleBlocks.push_back(Index(i-1,j-2));
                visibleBlocks.push_back(Index(i,j-2));
                
                visibleBlocks.push_back(Index(i-2,j-3));
                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
              
            }
            if(env[i][j-2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-1));

                visibleBlocks.push_back(Index(i-1,j-2));
                visibleBlocks.push_back(Index(i+1,j-2));

                visibleBlocks.push_back(Index(i-2,j-3));
                visibleBlocks.push_back(Index(i+2,j-3));
            }
            if(env[i-2][j-3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-1));

                visibleBlocks.push_back(Index(i-1,j-2));
                visibleBlocks.push_back(Index(i,j-2));
                visibleBlocks.push_back(Index(i+1,j-2));

           
                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));
                visibleBlocks.push_back(Index(i+2,j-3));
            }
             if(env[i-1][j-3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-1));

                visibleBlocks.push_back(Index(i-1,j-2));
                visibleBlocks.push_back(Index(i,j-2));
                visibleBlocks.push_back(Index(i+1,j-2));

                visibleBlocks.push_back(Index(i-2,j-3));
                
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));
                visibleBlocks.push_back(Index(i+2,j-3));
            }
             if(env[i][j-3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-1));

                visibleBlocks.push_back(Index(i-1,j-2));
                visibleBlocks.push_back(Index(i,j-2));
                visibleBlocks.push_back(Index(i+1,j-2));

                visibleBlocks.push_back(Index(i-2,j-3));
                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));
                visibleBlocks.push_back(Index(i+2,j-3));
            }
             if(env[i+1][j-3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-1));

                visibleBlocks.push_back(Index(i-1,j-2));
                visibleBlocks.push_back(Index(i,j-2));
                visibleBlocks.push_back(Index(i+1,j-2));

                visibleBlocks.push_back(Index(i-2,j-3));
                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+2,j-3));
            }
             if(env[i+2][j-3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-1));

                visibleBlocks.push_back(Index(i-1,j-2));
                visibleBlocks.push_back(Index(i,j-2));
                visibleBlocks.push_back(Index(i+1,j-2));

                visibleBlocks.push_back(Index(i-2,j-3));
                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));
                
            }
            if (count == 0)
            {
                visibleBlocks.push_back(Index(i,j-1));

                visibleBlocks.push_back(Index(i-1,j-2));
                visibleBlocks.push_back(Index(i,j-2));
                visibleBlocks.push_back(Index(i+1,j-2));

                visibleBlocks.push_back(Index(i-2,j-3));
                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));
                visibleBlocks.push_back(Index(i+2,j-3));
            }
        }
         else if (orientation == NORTH) {    //this is  north

            if (env[i-1][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-3,j-2));
                visibleBlocks.push_back(Index(i-3,j+2));
            }
            if (env[i-1][j-1] == 'X' || env[i-2][j-1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-1,j));

                visibleBlocks.push_back(Index(i-2,j-1));
                visibleBlocks.push_back(Index(i-2,j+1));

                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));
                visibleBlocks.push_back(Index(i-3,j+2));
            }
            if (env[i-1][j+1] == 'X' || env[i-2][j+1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-1,j));

                visibleBlocks.push_back(Index(i-2,j-1));
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j-2));
                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
            }

            if (env[i-2][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-1,j));

                visibleBlocks.push_back(Index(i-2,j-1));
                visibleBlocks.push_back(Index(i-2,j+1));

                visibleBlocks.push_back(Index(i-3,j-2));
                visibleBlocks.push_back(Index(i-3,j+2));

            }
            if (env[i-3][j-2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-1,j));

                visibleBlocks.push_back(Index(i-2,j-1));
                visibleBlocks.push_back(Index(i-2,j));
                visibleBlocks.push_back(Index(i-2,j+1));

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));
                visibleBlocks.push_back(Index(i-3,j+2));
            }
            if (env[i-3][j-1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-1,j));

                visibleBlocks.push_back(Index(i-2,j-1));
                visibleBlocks.push_back(Index(i-2,j));
                visibleBlocks.push_back(Index(i-2,j+1));

                visibleBlocks.push_back(Index(i-3,j-2));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));
                visibleBlocks.push_back(Index(i-3,j+2));
            }
            if (env[i-3][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-1,j));

                visibleBlocks.push_back(Index(i-2,j-1));
                visibleBlocks.push_back(Index(i-2,j));
                visibleBlocks.push_back(Index(i-2,j+1));

                visibleBlocks.push_back(Index(i-3,j-2));
                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j+1));
                visibleBlocks.push_back(Index(i-3,j+2));
            }
            if (env[i-3][j+1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-1,j));

                visibleBlocks.push_back(Index(i-2,j-1));
                visibleBlocks.push_back(Index(i-2,j));
                visibleBlocks.push_back(Index(i-2,j+1));

                visibleBlocks.push_back(Index(i-3,j-2));
                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+2));
            }
            if (env[i-3][j+2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-1,j));

                visibleBlocks.push_back(Index(i-2,j-1));
                visibleBlocks.push_back(Index(i-2,j));
                visibleBlocks.push_back(Index(i-2,j+1));

                visibleBlocks.push_back(Index(i-3,j-2));
                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));
            }
            if(count == 0)
            {
                visibleBlocks.push_back(Index(i-1,j));

                visibleBlocks.push_back(Index(i-2,j-1));
                visibleBlocks.push_back(Index(i-2,j));
                visibleBlocks.push_back(Index(i-2,j+1));

                visibleBlocks.push_back(Index(i-3,j-2));
                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));
                visibleBlocks.push_back(Index(i-3,j+2));

            }
        }
         else if (orientation == SOUTH)
          {    //this is south

            if (env[i+1][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+3,j-2));
                visibleBlocks.push_back(Index(i+3,j+2));

            }

            if (env[i+1][j-1] == 'X' || env[i+2][j-1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+1,j));

                visibleBlocks.push_back(Index(i+2,j+1));
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));
                visibleBlocks.push_back(Index(i+3,j+2));

            }

            
            if (env[i+1][j+1] == 'X' || env[i+2][j+1] == 'X')
            {

                count++;
                visibleBlocks.push_back(Index(i+1,j));

                visibleBlocks.push_back(Index(i+2,j-1));
                visibleBlocks.push_back(Index(i+2,j+1));

                visibleBlocks.push_back(Index(i+3,j-2));
                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));

            }

            if (env[i+2][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+1,j));

                visibleBlocks.push_back(Index(i+2,j-1));
                visibleBlocks.push_back(Index(i+2,j+1));

                visibleBlocks.push_back(Index(i+3,j-2));
                visibleBlocks.push_back(Index(i+3,j+2));

            }
            if (env[i+3][j-2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+1,j));

                visibleBlocks.push_back(Index(i+2,j-1));
                visibleBlocks.push_back(Index(i+2,j));
                visibleBlocks.push_back(Index(i+2,j+1));

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));
                visibleBlocks.push_back(Index(i+3,j+2));
            }
            if (env[i+3][j-1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+1,j));

                visibleBlocks.push_back(Index(i+2,j-1));
                visibleBlocks.push_back(Index(i+2,j));
                visibleBlocks.push_back(Index(i+2,j+1));

                visibleBlocks.push_back(Index(i+3,j-2));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));
                visibleBlocks.push_back(Index(i+3,j+2));
            }
            if (env[i+3][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+1,j));

                visibleBlocks.push_back(Index(i+2,j-1));
                visibleBlocks.push_back(Index(i+2,j));
                visibleBlocks.push_back(Index(i+2,j+1));

                visibleBlocks.push_back(Index(i+3,j-2));
                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j+1));
                visibleBlocks.push_back(Index(i+3,j+2));
            }
            if (env[i+3][j+1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+1,j));

                visibleBlocks.push_back(Index(i+2,j-1));
                visibleBlocks.push_back(Index(i+2,j));
                visibleBlocks.push_back(Index(i+2,j+1));

                visibleBlocks.push_back(Index(i+3,j-2));
                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+2));
            }
            if (env[i+3][j+2] == 'X')
            {   
                count++;
                visibleBlocks.push_back(Index(i+1,j));

                visibleBlocks.push_back(Index(i+2,j-1));
                visibleBlocks.push_back(Index(i+2,j));
                visibleBlocks.push_back(Index(i+2,j+1));

                visibleBlocks.push_back(Index(i+3,j-2));
                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));
            }
            if (count == 0 )
            {
                visibleBlocks.push_back(Index(i+1,j));

                visibleBlocks.push_back(Index(i+2,j-1));
                visibleBlocks.push_back(Index(i+2,j));
                visibleBlocks.push_back(Index(i+2,j+1));

                visibleBlocks.push_back(Index(i+3,j-2));
                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));
                visibleBlocks.push_back(Index(i+3,j+2));

            }
         }

     }
    else    /* ------------------- BEGINNING OF THE HIGHER HEIGHT --------------  */
    {
         if(orientation == EAST)
        {

            if(env[i][j+1] == 'X')
            {
                count++;
                

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));

                visibleBlocks.push_back(Index(i-2,j+4));
                visibleBlocks.push_back(Index(i-1,j+4));
                visibleBlocks.push_back(Index(i,j+4));
                visibleBlocks.push_back(Index(i+1,j+4));
                visibleBlocks.push_back(Index(i+2,j+4));
            }
            if(env[i][j+2] == 'X')
            {
                count++;
                
                visibleBlocks.push_back(Index(i-2,j+4));
                visibleBlocks.push_back(Index(i+2,j+4));

            }
            if(env[i][j+3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));

                visibleBlocks.push_back(Index(i-2,j+4));
                visibleBlocks.push_back(Index(i+2,j+4));


            }
            if(env[i-1][j+2] == 'X' || env[i-1][j+3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));

                visibleBlocks.push_back(Index(i,j+4));
                visibleBlocks.push_back(Index(i+1,j+4));
                visibleBlocks.push_back(Index(i+2,j+4));
            }
            if(env[i+1][j+2] == 'X' || env[i+1][j+3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));

                visibleBlocks.push_back(Index(i-2,j+4));
                visibleBlocks.push_back(Index(i-1,j+4));
                visibleBlocks.push_back(Index(i,j+4));
            }
            if(env[i-2][j+4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));

                visibleBlocks.push_back(Index(i-1,j+4));
                visibleBlocks.push_back(Index(i,j+4));
                visibleBlocks.push_back(Index(i+1,j+4));
                visibleBlocks.push_back(Index(i+2,j+4));
            }
            if(env[i-1][j+4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));

                visibleBlocks.push_back(Index(i-2,j+4));
                visibleBlocks.push_back(Index(i,j+4));
                visibleBlocks.push_back(Index(i+1,j+4));
                visibleBlocks.push_back(Index(i+2,j+4));
            }
            if(env[i][j+4] == 'X')
            {
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));

                visibleBlocks.push_back(Index(i-2,j+4));
                visibleBlocks.push_back(Index(i-1,j+4));
                visibleBlocks.push_back(Index(i+1,j+4));
                visibleBlocks.push_back(Index(i+2,j+4));
            }
            if(env[i+1][j+4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));

                visibleBlocks.push_back(Index(i-2,j+4));
                visibleBlocks.push_back(Index(i-1,j+4));
                visibleBlocks.push_back(Index(i,j+4));
                visibleBlocks.push_back(Index(i+2,j+4));
            }
            if(env[i+2][j+4] == 'X')
            {
                
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));

                visibleBlocks.push_back(Index(i-2,j+4));
                visibleBlocks.push_back(Index(i-1,j+4));
                visibleBlocks.push_back(Index(i,j+4));
                visibleBlocks.push_back(Index(i+1,j+4));
            }

            if(count == 0 )
            {
                visibleBlocks.push_back(Index(i,j+2));

                visibleBlocks.push_back(Index(i-1,j+3));
                visibleBlocks.push_back(Index(i,j+3));
                visibleBlocks.push_back(Index(i+1,j+3));

                visibleBlocks.push_back(Index(i-2,j+4));
                visibleBlocks.push_back(Index(i-1,j+4));
                visibleBlocks.push_back(Index(i,j+4));
                visibleBlocks.push_back(Index(i+1,j+4));
                visibleBlocks.push_back(Index(i+2,j+4));
            
            }
        }
        else if (orientation == WEST)
        { //this is west

            if(env[i][j-1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));

                visibleBlocks.push_back(Index(i-2,j-4));
                visibleBlocks.push_back(Index(i-1,j-4));
                visibleBlocks.push_back(Index(i,j-4));
                visibleBlocks.push_back(Index(i+1,j-4));
                visibleBlocks.push_back(Index(i+2,j-4));
                
            }
            if(env[i][j-2] == 'X')
            {
                count++;
                
                visibleBlocks.push_back(Index(i-2,j-4));
                visibleBlocks.push_back(Index(i+2,j-4));
            }
            if(env[i][j-3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-2));

                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));

                visibleBlocks.push_back(Index(i-2,j-4));
                visibleBlocks.push_back(Index(i+2,j-4));

            }
            if(env[i-1][j-2] == 'X' || env[i-1][j-3] == 'X')
            {
                count++;

                visibleBlocks.push_back(Index(i,j-2));

                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));

                visibleBlocks.push_back(Index(i,j-4));
                visibleBlocks.push_back(Index(i+1,j-4));
                visibleBlocks.push_back(Index(i+2,j-4));

            }
            if(env[i+1][j-2] == 'X' || env[i+1][j-3] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-2));

                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));

                visibleBlocks.push_back(Index(i-2,j-4));
                visibleBlocks.push_back(Index(i-1,j-4));
                visibleBlocks.push_back(Index(i,j-4));
            }
            if(env[i-1][j-4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-2));

                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));

                visibleBlocks.push_back(Index(i-1,j-4));
                visibleBlocks.push_back(Index(i,j-4));
                visibleBlocks.push_back(Index(i+1,j-4));
                visibleBlocks.push_back(Index(i+2,j-4));
            }
            if(env[i-1][j-4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-2));

                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));

                visibleBlocks.push_back(Index(i-2,j-4));
                visibleBlocks.push_back(Index(i,j-4));
                visibleBlocks.push_back(Index(i+1,j-4));
                visibleBlocks.push_back(Index(i+2,j-4));
            }
            if(env[i][j-4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-2));

                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));

                visibleBlocks.push_back(Index(i-2,j-4));
                visibleBlocks.push_back(Index(i-1,j-4));
                visibleBlocks.push_back(Index(i+1,j-4));
                visibleBlocks.push_back(Index(i+2,j-4));
            }
            if(env[i+1][j-4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-2));

                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));

                visibleBlocks.push_back(Index(i-2,j-4));
                visibleBlocks.push_back(Index(i-1,j-4));
                visibleBlocks.push_back(Index(i,j-4));
                visibleBlocks.push_back(Index(i+2,j-4));
            }
            if(env[i+2][j-4] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i,j-2));

                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));

                visibleBlocks.push_back(Index(i-2,j-4));
                visibleBlocks.push_back(Index(i-1,j-4));
                visibleBlocks.push_back(Index(i,j-4));
                visibleBlocks.push_back(Index(i+1,j-4));
            }
            if (count == 0)
            {
                visibleBlocks.push_back(Index(i,j-2));

                visibleBlocks.push_back(Index(i-1,j-3));
                visibleBlocks.push_back(Index(i,j-3));
                visibleBlocks.push_back(Index(i+1,j-3));

                visibleBlocks.push_back(Index(i-2,j-4));
                visibleBlocks.push_back(Index(i-1,j-4));
                visibleBlocks.push_back(Index(i,j-4));
                visibleBlocks.push_back(Index(i+1,j-4));
                visibleBlocks.push_back(Index(i+2,j-4));
            }

        }
        else if (orientation == NORTH)
        {    //this is  north
            if(env[i-1][j] == 'X')
            {
                
                count++;

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j+1));

                visibleBlocks.push_back(Index(i-4,j-2));
                visibleBlocks.push_back(Index(i-4,j-1));
                visibleBlocks.push_back(Index(i-4,j));
                visibleBlocks.push_back(Index(i-4,j+1));
                visibleBlocks.push_back(Index(i-4,j+2));
            
            }

            else if(env[i-2][j] == 'X')
            {  

                count++;

                visibleBlocks.push_back(Index(i-4,j-2));
                visibleBlocks.push_back(Index(i-4,j+2));
            }

            else if(env[i-3][j] == 'X')
            {

                count++;
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j+1));

                visibleBlocks.push_back(Index(i-4,j-2));
                visibleBlocks.push_back(Index(i-4,j+2));
            }


            else if(env[i-2][j-1] == 'X' || env[i-3][j-1] == 'X')
            {   
                count++;
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));

                visibleBlocks.push_back(Index(i-4,j));
                visibleBlocks.push_back(Index(i-4,j+1));
                visibleBlocks.push_back(Index(i-4,j+2));

            }

            
            else if(env[i-2][j+1] == 'X' || env[i-3][j+1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));

                visibleBlocks.push_back(Index(i-4,j-2));
                visibleBlocks.push_back(Index(i-4,j-1));
                visibleBlocks.push_back(Index(i-4,j));
              
            }
            if(env[i-4][j-2] == 'X')
            {

                count++;
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));

                visibleBlocks.push_back(Index(i-4,j-1));
                visibleBlocks.push_back(Index(i-4,j));
                visibleBlocks.push_back(Index(i-4,j+1));
                visibleBlocks.push_back(Index(i-4,j+2));

            }
             if(env[i-4][j-1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));

                visibleBlocks.push_back(Index(i-4,j-2));
                visibleBlocks.push_back(Index(i-4,j));
                visibleBlocks.push_back(Index(i-4,j+1));
                visibleBlocks.push_back(Index(i-4,j+2));
            }
             if(env[i-4][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));

                visibleBlocks.push_back(Index(i-4,j-2));
                visibleBlocks.push_back(Index(i-4,j-1));
                visibleBlocks.push_back(Index(i-4,j+1));
                visibleBlocks.push_back(Index(i-4,j+2));
            }
             if(env[i-4][j+1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));

                visibleBlocks.push_back(Index(i-4,j-2));
                visibleBlocks.push_back(Index(i-4,j-1));
                visibleBlocks.push_back(Index(i-4,j));
                visibleBlocks.push_back(Index(i-4,j+2));
            }
            if(env[i-4][j+2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));

                visibleBlocks.push_back(Index(i-4,j-2));
                visibleBlocks.push_back(Index(i-4,j-1));
                visibleBlocks.push_back(Index(i-4,j));
                visibleBlocks.push_back(Index(i-4,j+1));
            }

            if (count == 0)
            {
                visibleBlocks.push_back(Index(i-2,j));

                visibleBlocks.push_back(Index(i-3,j-1));
                visibleBlocks.push_back(Index(i-3,j));
                visibleBlocks.push_back(Index(i-3,j+1));

                visibleBlocks.push_back(Index(i-4,j-2));
                visibleBlocks.push_back(Index(i-4,j-1));
                visibleBlocks.push_back(Index(i-4,j));
                visibleBlocks.push_back(Index(i-4,j+1));
                visibleBlocks.push_back(Index(i-4,j+2));

            }

        }
        else if (orientation == SOUTH) {    //this is south

            if(env[i+1][j] == 'X')
            {
                
                count++;

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j+1));

                visibleBlocks.push_back(Index(i+4,j-2));
                visibleBlocks.push_back(Index(i+4,j-1));
                visibleBlocks.push_back(Index(i+4,j));
                visibleBlocks.push_back(Index(i+4,j+1));
                visibleBlocks.push_back(Index(i+4,j+2));

            }

            if(env[i+2][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+4,j-2));
                visibleBlocks.push_back(Index(i+4,j+2));
            }

            if(env[i+3][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j+1));

                visibleBlocks.push_back(Index(i+4,j-2));
                visibleBlocks.push_back(Index(i+4,j+2));
            }


            if(env[i+2][j-1] == 'X' || env[i+3][j-1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));

                visibleBlocks.push_back(Index(i+4,j));
                visibleBlocks.push_back(Index(i+4,j+1));
                visibleBlocks.push_back(Index(i+4,j+2));

            }

            
            if(env[i+2][j+1] == 'X' || env[i+3][j+1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));

                visibleBlocks.push_back(Index(i+4,j-2));
                visibleBlocks.push_back(Index(i+4,j-1));
                visibleBlocks.push_back(Index(i+4,j));
            }

            if (env[i+4][j-2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));

                visibleBlocks.push_back(Index(i+4,j-1));
                visibleBlocks.push_back(Index(i+4,j));
                visibleBlocks.push_back(Index(i+4,j+1));
                visibleBlocks.push_back(Index(i+4,j+2));
            }
            if (env[i+4][j-1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));

                visibleBlocks.push_back(Index(i+4,j-2));
                visibleBlocks.push_back(Index(i+4,j));
                visibleBlocks.push_back(Index(i+4,j+1));
                visibleBlocks.push_back(Index(i+4,j+2));
            }
            if (env[i+4][j] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));

                visibleBlocks.push_back(Index(i+4,j-2));
                visibleBlocks.push_back(Index(i+4,j-1));
                visibleBlocks.push_back(Index(i+4,j+1));
                visibleBlocks.push_back(Index(i+4,j+2));
            }
            if (env[i+4][j+1] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));

                visibleBlocks.push_back(Index(i+4,j-2));
                visibleBlocks.push_back(Index(i+4,j-1));
                visibleBlocks.push_back(Index(i+4,j));
                visibleBlocks.push_back(Index(i+4,j+2));
            }
            if (env[i+4][j+2] == 'X')
            {
                count++;
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));

                visibleBlocks.push_back(Index(i+4,j-2));
                visibleBlocks.push_back(Index(i+4,j-1));
                visibleBlocks.push_back(Index(i+4,j));
                visibleBlocks.push_back(Index(i+4,j+1));
            }

            if (count == 0)
            {
                visibleBlocks.push_back(Index(i+2,j));

                visibleBlocks.push_back(Index(i+3,j-1));
                visibleBlocks.push_back(Index(i+3,j));
                visibleBlocks.push_back(Index(i+3,j+1));

                visibleBlocks.push_back(Index(i+4,j-2));
                visibleBlocks.push_back(Index(i+4,j-1));
                visibleBlocks.push_back(Index(i+4,j));
                visibleBlocks.push_back(Index(i+4,j+1));
                visibleBlocks.push_back(Index(i+4,j+2));
            }

        }   //end of else for HIGH SOUTH FACING
    }   //end of else for HIGH HEIGHT



    if(count > 1)
    {
        
        for (const auto& p : visibleBlocks) {
            ++countOccurences[p]; 
        }

        for (const auto& p : countOccurences) {
            const auto& p1 = p.first;
            
            int countO = p.second;
            if(countO == count )
            {   
                CommonVisibleBlocks.push_back(p1);
            }
        }
        return CommonVisibleBlocks;
    }
   
    return visibleBlocks;
}
       


    
