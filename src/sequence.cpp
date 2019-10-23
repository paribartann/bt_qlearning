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


#include "../include/sequence.h"
#include <iostream>
#include <string>



tree::SequenceNode::SequenceNode(std::string key, unsigned int num)
{
    //SelectorNode* temp = new SelectorNode; 
    this->key = key; 
    set_type(SEQUENCE);
    set_num_children(num);
}

tree::ReturnStatus tree::SequenceNode::Tick()
{

    std::cout<<"Sequence Node ticked"<<std::endl;

    for (unsigned int i = 0; i < get_num_children(); i++)
    {       
        child_i_status_ = child[i]->Tick();

        if (child_i_status_ == tree::SUCCESS)
        {
            set_status(child_i_status_);
            if (i == (get_num_children() - 1))
            {
                std::cout<<"Last child of SEQUENCE node returned SUCCESS. Going for other one"<<std::endl;
                return tree::SUCCESS;
            }
            else
            {
                std::cout<<i+1<<"'th child of SEQUENCE node returned SUCCESS. Going for other one"<<std::endl;
            }
        }
        else
        {
            if (child_i_status_ == tree::FAILURE)
            {
                std::cout<<i+1<<"'th child of SEQUENCE node returned FAILURE. Returning back to the parent" <<std::endl;
            }
            else
            {
                std::cout<<i+1<<"'th child of SELECTOR node returned RUNNNING. Returning back to the parent" <<std::endl;
            }
            return child_i_status_;
        }
        
        
           
    }
    return child_i_status_; 
}