/* 
 * File:   related to path_modification_fixtureTest.h
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 6:12 PM
 */

#ifndef MAIN_PROCESS_H
#define	MAIN_PROCESS_H

#include "ros/ros.h"
#include "ramp_msgs/Range.h"
#include "utility.h"
#include "modifier.h"

Utility u;

bool handleRequest(ramp_msgs::ModificationRequest::Request& req,
                   ramp_msgs::ModificationRequest::Response& res)
{
  Modifier mod(req);
  res.mod_paths = mod.perform();
  
   return true;
}



#endif	/* MAIN_PROCESS_H */

