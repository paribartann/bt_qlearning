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

#include "../include/learningSelectorStar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <random>
#include <cstdlib>
#include <ctime>

#define EPS 0.05

using namespace std;

void swap(int *a, int *b);

tree::LearningSelectorStarNode::LearningSelectorStarNode(string key, unsigned int num, EnvClass &envObject_)
{
    this->key = key;
    current_child_index_ = 0;
    set_type(LEARNINGSELECTORSTAR);
    set_num_children(num);
    envObject = &envObject_;
    current_child = 0;
    previous_child = 0;
}

tree::ReturnStatus tree::LearningSelectorStarNode::Tick()
{

    cout << "LearningSelectorStar Node ticked" << endl;

    if (envObject->reset)
    {
        current_child_index_ = 0;
        /****************** GENERATING RANDOM NUMBER ***************************/
        random_device rd;
        srand(rd());
        double random_number;
        random_number = (double)rand() / (double)RAND_MAX; //float in range 0 to 1
        //random_number = (double)rand() / (double)((unsigned) RAND_MAX + 1) //1 excluded
        cout << "RANDOM NUMBER = " << random_number << endl;
        int ActionIndexes[] = {0, 1, 2, 3, 4};
        int n = sizeof(ActionIndexes) / sizeof(ActionIndexes[0]);
        /***********************************************************************/

        current_state = envObject->currentWayPoint; //observing the current state

        //cout<<"X: "<<current_state.i<<" Y: "<s<current_state.j<<endl;

        int *returnedActionArray = getActionsArray(ActionIndexes, random_number, n); //get the action based on the random number generated
        for (unsigned int i = 0; i < get_num_children(); i++)
        {
            returnedArray[i] = returnedActionArray[i];
        }
        envObject->reset = false;
    }

    for (unsigned int i = 0; i < get_num_children(); i++)
    {
        cout << returnedArray[i] << " " << endl;
    }

    while (current_child_index_ < get_num_children())
    {

        envObject->previous_action = current_child;
        envObject->current_action = returnedArray[current_child_index_];
        current_child = returnedArray[current_child_index_];

        cout << returnedArray[current_child_index_] << " number child is being ticked" << endl;

        //TICK
        child_i_status_ = child[returnedArray[current_child_index_]]->Tick();

        if (child_i_status_ == tree::FAILURE)
        {
            child[current_child_index_]->set_status(tree::IDLE); //just to say that it's been completed

            if (current_child_index_ != (get_num_children() - 1)) //if not the last child
            {
                std::cout << current_child_index_ + 1 << "'th child of LearningSelectorStar returned FAILURE. Going for other one" << std::endl;
                current_child_index_++;
            }
            else //if it;s the last child
            {
                current_child_index_ = 0;
                std::cout << "Last child of LearningSelectorStar returned FAILURE. Going for other one" << std::endl;
                return child_i_status_;
            }
        }
        else
        {
            if (child_i_status_ == tree::SUCCESS) //success
            {
                child[current_child_index_]->set_status(tree::IDLE); //just to say that it's been completed
                std::cout << current_child_index_ + 1 << "'th child of LearningSelectorStar returned SUCCESS. Going for other one" << std::endl;
                current_child_index_ = 0;
            }
            else //RUNNING
            {
                child[current_child_index_]->set_status(child_i_status_);
                std::cout << current_child_index_ + 1 << "'th child of LearningSelectorStar returned RUNNING." << std::endl;
            }
            return child_i_status_;
        }
    }
    return child_i_status_;
}

int *tree::LearningSelectorStarNode::SortingArray(double array[][5], int actionArray[])
{

    int max_index;
    double index_val;

    for (unsigned int i = 0; i < get_num_children() - 1; i++)
    {
        max_index = i;
        for (unsigned int j = i + 1; j < get_num_children(); j++)
        {

            if (array[envObject->q_wayPointMap[current_state]][j] > array[envObject->q_wayPointMap[current_state]][max_index])
            {
                max_index = j;
            }
        }
        index_val = actionArray[max_index];
        actionArray[max_index] = actionArray[i];
        actionArray[i] = index_val;
    }
    return actionArray;
}

int *tree::LearningSelectorStarNode::getActionsArray(int *arr, double random_number, int n)
{

    if (random_number < EPS)
    {
        cout << "RANDOM" << endl;
        return randomize(arr, n);
    }
    else
    {
        cout << "SORTED" << endl;
        return SortingArray(envObject->q_table, arr);
    }
}

int *tree::LearningSelectorStarNode::randomize(int arr[], int n)
{

    random_device rd;
    srand(rd());
    for (int i = n - 1; i > 0; i--)
    {
        int j = rand() % (i + 1);
        swap(&arr[i], &arr[j]);
    }

    return arr;
}

void swap(int *a, int *b)
{
    int temp = *a;
    *a = *b;
    *b = temp;
}
