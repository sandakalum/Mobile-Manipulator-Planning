#!/usr/bin/env python3
import rospy
import rospkg 
import sys
from ktmpb.srv import *
rospack =rospkg.RosPack()
import xml.etree.ElementTree as ET 


# Global variables
directory=''

def main():
    # Initialise code
    #check for arguments
    if len(sys.argv)<2:
        print("Number of parameters is not correct")
        print ("Arguments should be ff_client_py ppdl_files")
       
    rospy.loginfo ("Starting FF Python Client")
    rospy.init_node("ff_client_python_node")
    #Open config file
    ROSpackage_path= rospack.get_path("ktmpb")
    config_tree = ET.parse(ROSpackage_path+sys.argv[1])
    config_root = config_tree.getroot()
    #Get data for Problem files
    pddldomainfile=config_root.find('Problemfiles').find('pddldomain').get('name')
    pddlfactsfile=config_root.find('Problemfiles').find('pddlproblem').get('name')
    DIRECTORY =config_root.find('Problemfiles').find('directory').get('name')
    print("Using pddl facts file", pddlfactsfile)

    #Setting problem files
    global directory
    directory=ROSpackage_path + DIRECTORY#"/demos/OMPL_geo_demos/Table_Rooms_R2/" 
    pddlDomainFile = directory + pddldomainfile
    pddlProblemFile = directory + pddlfactsfile
    
    #Solving Task Planning Problem
    rospy.wait_for_service("/FFPlan")
    ff_srv = Plan()
    ff_client = rospy.ServiceProxy("/FFPlan",Plan)
    
    #Loading the request for the ff service: the problem file and the domain file
    ff_srv.problem = pddlProblemFile
    ff_srv.domain = pddlDomainFile
    rospy.loginfo_once(pddlDomainFile)
    rospy.loginfo_once(pddlProblemFile)
    
    #Calling the ff service
    ff_response= ff_client(ff_srv.problem,ff_srv.domain)
    
    #Printing the response
    rospy.loginfo("The computed plan is:")
    if ff_response.response:
        rospy.loginfo(str(ff_response.plan))
        taskPlan= ff_response.plan
    else:
        rospy.loginfo("Unable to compute task plan")
    
    print("****************************************")
    for line in taskPlan:
        print(line)
        
    print("****************************************")



if __name__ == '__main__':
    try:
        #Run the main function 
        main()
    except rospy.ROSInterruptException:
        pass
