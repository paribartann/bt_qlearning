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


#ifndef LEARNING_SELECTOR_STAR_NODE_H
#define LEARNING_SELECTOR_STAR_NODE_H


#include "node.h"
#include <string>
#include <vector>
#include "../include/env.h"

namespace tree{

    class LearningSelectorStarNode : public Node
    {
        private:
            unsigned int current_child_index_;
            Index current_state;
            
            
        public:
            LearningSelectorStarNode(std::string key, unsigned int, EnvClass&);
            ReturnStatus Tick();
            EnvClass* envObject;
            int returnedArray[5] = {0, 1, 2, 3, 4};
            int* SortingArray(double[][5], int[]);
            int* getActionsArray(int[], double, int);
            int* randomize (int arr[], int n);  
    };
}

#endif