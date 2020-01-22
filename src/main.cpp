/* 
#Copyright (C) <June, 2019>  <Paribartan Dhakal>

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU Affero General Public License as
#published by the Free Software Foundation, either version 3 of the
#License, or (at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Affero General Public License for more details.
*/


#include "../include/tree.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main()
{
    
    ifstream infile("../files/waypoints1");
    int num = 0;
    infile >> num;  //number of elements
    int wayp_array[num];
    for (int i = 0; i < num; i++)
    {
        wayp_array[i] = 0;
    }
    
    for (int i = 0; i < num; i++)
    {
        infile>>wayp_array[i];
    }
    
    tree::BT bt1("/uav1", wayp_array);


    // tree::BT bt2("/uav2");

    tree::Node* root = bt1.buildBT("../files/BT_Fig8");
    
    bt1.execute(root, 200000);


    // tree::Node* root_ = bt2.buildBT("../files/BT_Fig8");
    
    // bt2.execute(root_, 200000);



    return 0;
}
