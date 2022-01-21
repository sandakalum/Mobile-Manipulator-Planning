#!/usr/bin/env python3
import rospy
import rospkg 
import sys
from std_msgs.msg import String, Time
from geometry_msgs.msg import Pose
from ktmpb.msg import fVector
from ktmpb.srv import *
import random
rospack =rospkg.RosPack()
import xml.etree.ElementTree as ET 
from collections import defaultdict
#Import python module with functions necessary fotrr interfacing with kautham
import kautham_py.kautham_python_interface as kautham
import math
import transformations
import pytransform3d.transformations as pt
import pytransform3d.rotations as pr

#Function to write to xml file in Conf tag
def writePath(taskfile,tex):
    taskfile.write("\t\t<Conf> %s </Conf>\n" % tex)
    return True

#Function to move robot to the init position 
def Move_to_init(init,rob):
    global Robot_pos
    pos=Robot_pos
    #Set Robot Control
    #kauthamSetRobControlsNoQuery(Robot_move_control)
    #Set the move query in Kautham
    kauthamSetQuery(pos,init)
    path=kauthamGetPath(1)
    if path:
        if not graspedobject:
            taskfile.write("\t<Transit>\n")
        k= sorted(list(path.keys()))[-1][1]+1
        for i in range(int(len(path.keys())/k)-1):
            tex=''
            for j in range(0,k):
                tex=tex + str(path[i,j]) + " "  
            writePath(taskfile,tex)
        if not graspedobject:
            taskfile.write("\t</Transit>\n")
        kauthamMoveRobot(init)


#initialising data structure to store scene info 
class knowledge:
    pass

# Global variables
directory=''
graspit_world_file_name=''
Robot_move_control= ''
Robot_pos=[]
taskfile=''
graspedobject= False
graspTransfUsed=''
graspControlsUsed=''

#Function to convert joint angles to kautham control values
def JointAngleToKauthamControl (pose):
    Pose=[]
    Pose.append((pose[0]+0)/0.35) #torso_lift
    Pose.append((pose[1]+0.02)/(0.02+2.728893571890)) #arm_1
    Pose.append((pose[2]+1.55079632679)/(1.55079632679+1.0708307825)) #arm_2
    Pose.append((pose[3]+3.51429173529)/(3.51429173529+1.55079632679)) #arm_3
    Pose.append((pose[4]+0.372699081699)/(0.372699081699+2.33619449019)) #arm_4
    Pose.append((pose[5]+2.07439510239)/(2.07439510239+2.07439510239)) #arm_5
    Pose.append((pose[6]+1.39371669412)/(1.39371669412+1.39371669412)) #arm_6
    Pose.append((pose[7]+2.07439510239)/(2.07439510239+2.07439510239)) #arm_7
    Pose.append(pose[8])#gripper

    return Pose

#Function to call Tiago Ik service to get configuration of Tiago (In joint angles in radian)
def Tiago_IK(pose):
    print("wait for service /tiago_Ik")
    #define server client
    rospy.wait_for_service("/tiago_Ik")
    Tiago_IK_srv=TiagoIk()
    Tiago_IK_srv.Pose=pose
    Tiago_IK_client= rospy.ServiceProxy("/tiago_Ik",TiagoIk)
    #Send request and recieve response
    print("send request for service /tiago_Ik: ",Tiago_IK_srv.Pose)
    r= Tiago_IK_client(Tiago_IK_srv.Pose)
    if list(r.Conf)==[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]:
        print('Failed to compute Inverse Kinematics for the given position')
        return False
    conf=list(r.Conf)
    return JointAngleToKauthamControl(conf)

#Function to generate GraspIt World file 
def GraspIt_worldFile_Generator(pose):
    global graspit_world_file_name
    #Read Graspit XML file
    tree = ET.parse(graspit_world_file_name)
    root = tree.getroot()
    abc=root.find('graspableBody').find('transform').find('fullTransform')#.get('name')
    #Modify pose in XML file
    abc.text = "("+str(pose[6])+ " "+str(pose[3])+ " "+str(pose[4])+ " "+str(pose[5])+ " ){"+str(pose[0]*1000)+" "+ str(pose[1]*1000)+" "+str(pose[2]*1000)+")"
    tree.write(graspit_world_file_name)
    return(graspit_world_file_name)




#Function to call GraspIt! and get gripper pose wrt object
def GraspIt(pose):
    #define server client
    rospy.wait_for_service("graspit_ros_node/grasp_planning")
    #Set request variables
    GraspIt_srv=GraspPlanning()
    GraspIt_srv.world_file_path=GraspIt_worldFile_Generator(pose)
    GraspIt_srv.output_path=""
    GraspIt_srv.maxPlanningSteps=70000
    GraspIt_srv.repeatPlanning=1
    GraspIt_srv.keepMaxPlanningResults=1
    GraspIt_srv.finishWithAutograsp=False
    GraspIt_srv.saveVisualResults=True
    GraspIt_client= rospy.ServiceProxy("graspit_ros_node/grasp_planning",GraspPlanning)
    #Send request and recieve response
    r= GraspIt_client(GraspIt_srv.world_file_path,GraspIt_srv.output_path,GraspIt_srv.maxPlanningSteps,GraspIt_srv.repeatPlanning,GraspIt_srv.keepMaxPlanningResults,GraspIt_srv.finishWithAutograsp,GraspIt_srv.saveVisualResults)
    print(r.grasping_poses)
    Pose=[r.grasping_poses[0].translation.x,r.grasping_poses[0].translation.y,r.grasping_poses[0].translation.z,r.grasping_poses[0].rotation.w,r.grasping_poses[0].rotation.x,r.grasping_poses[0].rotation.y,r.grasping_poses[0].rotation.z]

    return Pose

#Function to compute grasp config
def computeGraspControls(pose, trygrasp):
    print("computing graspControl")
    #Check for pose/poseregion
    if len(pose)>7:
        #For Poseregion
        obj_pose=[random.uniform(pose[0], pose[1]), random.uniform(pose[2], pose[3]),pose[4], pose[5],pose[6],pose[7],pose[8]]
    else:
        #For pose
        obj_pose=pose
    Rob_pose= list(kautham.kGetRobotPos(0))
    print("obj_pose: ", obj_pose)
  
    #Object wrt to world frame
    q_obj=pr.matrix_from_quaternion([obj_pose[6],obj_pose[3],obj_pose[4],obj_pose[5]])
    P_obj=obj_pose[0:3]
    T_world_object= pt.transform_from(q_obj,P_obj)

    #Use the grasp transf given or compute it with graspIt
    if trygrasp:
        Pose = trygrasp
    else:
        Pose=GraspIt(obj_pose)
    q_gripper=pr.matrix_from_quaternion(Pose[3:7])#([0.707,0.707,0,0])
    P_gripper=Pose[0:3]#[-0.165,0,0.06]    
    T_object_gripper= pt.transform_from(q_gripper,P_gripper)
    
    #Robot wrt to world frame
    q_rob=pr.matrix_from_quaternion([Rob_pose[6],Rob_pose[3],Rob_pose[4],Rob_pose[5]])
    P_rob=Rob_pose[0:3]
    T_world_robot= pt.transform_from(q_rob,P_rob)
    #its inverse
    T_robot_world = pt.invert_transform(T_world_robot)

    #Gripper wrt Robot
    T_robot_object= pt.concat(T_world_object,T_robot_world)
    T_robot_gripper= pt.concat(T_object_gripper,T_robot_object)
    #convert T_robot_gripper to (pos+quaterion_xyzw)
    Pose_Final_i= pt.pq_from_transform(T_robot_gripper)
    q_pose=pr.quaternion_xyzw_from_wxyz(pr.check_quaternion(Pose_Final_i[3:7]))
    Pose_Final=[Pose_Final_i[0],Pose_Final_i[1],Pose_Final_i[2]]
    Pose_Final.extend(q_pose)

    #round
    Pose_rounded = [ round(elem, 3) for elem in Pose_Final ]
    print("Pose_rounded: ",Pose_rounded)
    #compute IK
    return Tiago_IK(Pose_rounded)

def main():
    # Initialise code
    #check for arguments
    if len(sys.argv)<4:
        print("Number of parameters is not correct")
        print ("Arguments should be ktmp-interface  tampconfig_file")
       
    rospy.loginfo ("Starting Task and Motion Planning Interface Python Client")
    rospy.init_node("ktmpb_python_interface")
    #Open config file
    ROSpackage_path= rospack.get_path("ktmpb")
    config_tree = ET.parse(ROSpackage_path+sys.argv[1])
    config_root = config_tree.getroot()
    #Get data from config file
    #Get data for Problem files
    pddldomainfile=config_root.find('Problemfiles').find('pddldomain').get('name')
    pddlfactsfile=config_root.find('Problemfiles').find('pddlproblem').get('name')
    kauthamproblem = config_root.find('Problemfiles').find('kautham').get('name')
    DIRECTORY =config_root.find('Problemfiles').find('directory').get('name')
    graspit_world_file =config_root.find('Problemfiles').find('graspit').get('name')
    print("Using kautham problem",kauthamproblem)
    print("Using pddl facts file", pddlfactsfile)
    #Get data for states 
    Object_pose={}
    Object_kthindex={}
    Robot_pose={}
    Robot_control={}
    for val in config_root.find('States').findall('Initial'):
        object_name = val.find('Object').get('name')
        object_kthindex = val.find('Object').get('kthindex')
        object_pose= val.find('Object').text
        object_pose=[float(f) for f in object_pose.split()]
        Object_pose[object_name]= object_pose
        Object_kthindex[object_name]= object_kthindex
        robot_name= val.find('Robot').get('name')
        robot_controlfile= val.find('Robot').get('controlfile')
        robot_pose= val.find('Robot').text
        robot_pose=[float(f) for f in robot_pose.split()]
        Robot_pose[robot_name]= robot_pose
        Robot_control[robot_name]= robot_controlfile

    #Get data for Actions
    #Get data for PICK action
    pick=defaultdict(lambda: defaultdict(dict))
    for val in config_root.find('Actions').findall('Pick'):
        robot_name= val.get('robot')
        obj=val.get('object')
        region= val.get('region')
        Rob= int(val.find('Rob').text)
        link=int(val.find('Link').text)
        Cont=str(val.find('Cont').text).strip()
        ObjName= int(val.find('Obj').text)
        Regioncontrols= val.find('Regioncontrols').text
        Regioncontrols= [float(f) for f in Regioncontrols.split()]
        grasp={}
        for grasps in val.findall("Graspcontrols"):
            grasp_name = grasps.get('grasp')
            graspcontrol= grasps.text
            graspcontrol=[float(f) for f in graspcontrol.split()]
            grasp[grasp_name]= graspcontrol
            pick[robot_name][obj][region]={'Regioncontrols': Regioncontrols,'Graspcontrols': grasp, 'Object_Name':ObjName, 'Robot_name':Rob, 'Link':link, 'Cont':Cont}
        if len(grasp)==0:
            Pose= val.find('Pose').text
            Pose= [float(f) for f in Pose.split()]
            for grasps in val.findall("Grasptransf"):
                grasp_name = grasps.get('grasp')
                grasptransf= grasps.text
                grasptransf=[float(f) for f in grasptransf.split()]
                grasp[grasp_name]= grasptransf
            if len(grasp)==0:
                print('No grasp transf found - This may be a problem')
            else:
                print('grasp = ', grasp)
                pick[robot_name][obj][region]={'Regioncontrols': Regioncontrols,'Pose':Pose, 'Grasptransf':grasp, 'Object_Name':ObjName, 'Robot_name':Rob, 'Link':link,'Cont':Cont}

    #Get data for PLACE action
    place=defaultdict(lambda: defaultdict(dict))
    for val in config_root.find('Actions').findall('Place'):
        robot_name= val.get('robot')
        obj=val.get('object')
        region= val.get('region') 
        Rob= int(val.find('Rob').text)
        Cont=str(val.find('Cont').text).strip()
        ObjName= int(val.find('Obj').text)
        Regioncontrols= val.find('Regioncontrols').text
        Regioncontrols= [float(f) for f in Regioncontrols.split()]
        grasp={}
        for grasps in val.findall("Graspcontrols"):
            grasp_name = grasps.get('grasp')
            graspcontrol= grasps.text
            graspcontrol=[float(f) for f in graspcontrol.split()]
            grasp[grasp_name]= graspcontrol
        try:
            Poseregion= val.find('Poseregion').text
            Poseregion= [float(f) for f in Poseregion.split()]
            if len(grasp):
                place[robot_name][obj][region]={'Regioncontrols': Regioncontrols,'Poseregion': Poseregion,'Graspcontrols': grasp, 'Object_Name':ObjName, 'Robot_name':Rob,'Cont':Cont}
            else:
                place[robot_name][obj][region]={'Regioncontrols': Regioncontrols,'Poseregion': Poseregion, 'Object_Name':ObjName, 'Robot_name':Rob,'Cont':Cont}
        except:
            pass
        try:
            Pose= val.find('Pose').text
            Pose= [float(f) for f in Pose.split()]
            if len(grasp):
                place[robot_name][obj][region]={'Regioncontrols': Regioncontrols,'Pose':Pose,'Graspcontrols': grasp, 'Object_Name':ObjName, 'Robot_name':Rob,'Cont':Cont,}

        except:
            place[robot_name][obj][region]={'Regioncontrols': Regioncontrols,'Graspcontrols': grasp, 'Object_Name':ObjName, 'Robot_name':Rob,'Cont':Cont,}

    #Get data for MOVE action
    move=defaultdict(lambda: defaultdict(dict))
    for val in config_root.find('Actions').findall('Move'):
        robot_name= val.get('robot')
        region_from=val.get('region_from')
        region_to= val.get('region_to') 
        Rob= val.find('Rob').text
        Cont=str(val.find('Cont').text).strip()
        #Home= val.find('Home').text
        #Home= [float(f.replace("−", "-")) for f in Home.split()]
        Initcontrols= val.find('InitControls').text
        Initcontrols=[float(f) for f in Initcontrols.split()]
        Goalcontrols= val.find('GoalControls').text
        Goalcontrols=[float(f) for f in Goalcontrols.split()]
        global Robot_move_control
        Robot_move_control= Cont
        #move[robot_name][region_from][region_to]={'Initcontrols':Initcontrols, 'Goalcontrols':Goalcontrols,'Robot_name':Rob,'Cont':Cont,'Home':Home}
        move[robot_name][region_from][region_to]={'Initcontrols':Initcontrols, 'Goalcontrols':Goalcontrols,'Robot_name':Rob,'Cont':Cont}

    #Setting problem files
    modelFolder = ROSpackage_path + "/demos/models/"
    global directory
    global graspit_world_file_name
    directory=ROSpackage_path + DIRECTORY#"/demos/OMPL_geo_demos/Table_Rooms_R2/" 
    if graspit_world_file == "":
        print('No GraspIt folder provided - GraspIt is not going to be used');
        graspit_world_file_name = ""
    else:
        print('GraspIt folder provided', graspit_world_file);
        graspit_world_file_name=ROSpackage_path+graspit_world_file
        
    kauthamProblemFile= directory + kauthamproblem
    pddlDomainFile = directory + pddldomainfile
    pddlProblemFile = directory + pddlfactsfile
    
    #Solving Task Planning Problem
    rospy.wait_for_service("/FFPlan")
    ff_srv = Plan()
    ff_client = rospy.ServiceProxy("/FFPlan",Plan)
    #Loading the request for the ff service: the problem file and the domain file
    ff_srv.problem = pddlProblemFile
    ff_srv.domain = pddlDomainFile

    rospy.loginfo_once(kauthamProblemFile)
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
    
    ##Solving the motion planning problem
    #Open kautham problem
    kautham.kOpenProblem(modelFolder,kauthamProblemFile)
    #Set obsctacle from tampconfig file
    for key in Object_pose.keys():
        index = int(Object_kthindex[key])
        kautham.kSetObstaclePos(index,Object_pose[key])
    #Set robot from tampconfig file
    index=0
    global Robot_pos
    for key in Robot_pose.keys():
        print(Robot_pose[key])
        #kauthamSetRobotPos(index,Robot_pose[key])
        kautham.kSetRobControls(Robot_control[key],[0,0,0],Robot_pose[key])
        kautham.kMoveRobot(Robot_pose[key])
        Robot_pos=Robot_pose[key]
        index=index+1
    #Save to file
    global taskfile
    tampconfig_file= ROSpackage_path+sys.argv[1]
    tampconfig_file= tampconfig_file.replace(directory,'')
    tfile =directory+'taskfile_'+tampconfig_file
    taskfile = open(tfile, "w+")
    taskfile.write("<?xml version=\"1.0\"?>\n")

    #Write Initial states to config file
    taskfile.write("<Task name= \"%s\" >\n" % kauthamproblem)
    taskfile.write("\t<Initialstate>\n")
    for keys in Object_pose.keys():
        pos=''
        for j in range(7):
            pos=pos + str(Object_pose[keys][j]) + " "
        #taskfile.write("\t\t<Object object=\"%s\"> %s </Object>\n"%(object_index[keys],pos))
        taskfile.write("\t\t<Object object=\"%s\"> %s </Object>\n"%(Object_kthindex[keys],pos))
    taskfile.write("\t</Initialstate>\n")

    #Initialising variables
    global graspedobject
    graspedobject= False
    for line in taskPlan:
        #print(line)
        Line=line.split(" ")
        action=Line[0]
        rob= Line[1]    
        #Move action    
        if action =="MOVE":
            print("**************************************************************************")
            print("  MOVE ACTION  ")
            print("**************************************************************************")
            fromLocation = Line[2]
            toLocation = Line[3]
            print(action +" "+rob+" "+fromLocation+" "+toLocation)
            try:
                init = move[rob][fromLocation][toLocation]['Initcontrols']
                goal= move[rob][fromLocation][toLocation]['Goalcontrols']
                Robot_control=move[rob][fromLocation][toLocation]['Cont']
            except:
                try:
                    print ("Tag for "+ action +" "+rob+" "+fromLocation+" "+toLocation+" Not defined in config file \nHence using "+ action +" "+rob+" "+toLocation+" "+fromLocation+" In reverse instead")
                    goal = move[rob][toLocation][fromLocation]['Initcontrols']
                    init = move[rob][toLocation][fromLocation]['Goalcontrols']
                    Robot_control=move[rob][toLocation][fromLocation]['Cont']
                except:
                    print(action +" "+rob+" "+fromLocation+" "+toLocation+" Not defined in config file ")
            print("Searching path to Move robot " )
            print("Init= ", fromLocation)
            print("Goal= ", toLocation)
            print("Init= ", init)
            print("Goal=", goal)
            #Home=move[rob][fromLocation][toLocation]['Home']
            Robot_move_control= Robot_control
            print("Robot control=",Robot_control)
            #Set Robot Control
            kautham.kSetRobControlsNoQuery(Robot_control)
            #Set the move query in Kautham
            kautham.kSetQuery(init,goal)
            #Solve query
            print("Solving Query")
            path=kautham.kGetPath(1)
            #Write path to taskfile
            if path:
                print("-------- Path found: Moving robot " )
                if graspedobject is False:
                    taskfile.write("\t<Transit>\n")
                    k= sorted(list(path.keys()))[-1][1]+1
                    for i in range(sorted(list(path.keys()))[-1][0]):
                        tex=''
                        for j in range(0,k):
                            tex=tex + str(path[i,j]) + " "  
                        writePath(taskfile,tex)
                    taskfile.write("\t</Transit>\n")
                else:
                    k= sorted(list(path.keys()))[-1][1]+1
                    for i in range(sorted(list(path.keys()))[-1][0]):
                        tex=''
                        for j in range(0,k):
                            tex=tex + str(path[i,j]) + " "  
                        writePath(taskfile,tex)

                kautham.kMoveRobot(goal)
            else:
                print("**************************************************************************")
                print("Get path Failed! No Move possible, Infeasible Task Plan")
                print("**************************************************************************")
                break
        #Pick Action
        elif action == "PICK":
            print("**************************************************************************")
            print("  PICK ACTION  ")
            print("**************************************************************************")
            obstacle = Line[2]
            fromLocation = Line[3]
            print(action +" "+rob+" "+obstacle+" "+fromLocation)
            obsIndex= pick[rob][obstacle][fromLocation]['Object_Name']
            robotIndex =pick[rob][obstacle][fromLocation]['Robot_name']
            linkIndex =pick[rob][obstacle][fromLocation]['Link']
            init = pick[rob][obstacle][fromLocation]['Regioncontrols']
            Robot_control=pick[rob][obstacle][fromLocation]['Cont']
            #Set robot control
            kautham.kSetRobControlsNoQuery(Robot_control)
            if 'Graspcontrols' in pick[rob][obstacle][fromLocation].keys():
                grasp_control =pick[rob][obstacle][fromLocation]['Graspcontrols']
                print("Start looping along graspcontrols")
                for grasp in grasp_control.keys():
                    #Set the move query in Kautham
                    goal=grasp_control[str(grasp)]
                    print("Searching path to Move to object position " )
                    print("Init= ", init)
                    print("Goal= ", goal)
                    print("Robot Control=",Robot_control)
                    #Set robot control
                    kautham.kSetRobControlsNoQuery(Robot_control)
                    #Set the move query in Kautham
                    kautham.kSetQuery(init,goal)
                    #Solve query
                    print("Solving Query to pick object")
                    path=kautham.kGetPath(1)
                    if path :
                        print("-------- Path found: Moving to object position " )
                        print('Storing Grasp Controls Used = ',grasp)
                        graspControlsUsed=grasp
                        break

                    else:
                        print("**************************************************************************")
                        print("Get path Failed! No Move possible, Infeasible Task Plan\nTrying next graspcontrol")
                        print("**************************************************************************")
                  
            else:
                #Loop to iterate through IK solutions
                for i in range(10):
                    #Do not use GraspIt
                    if graspit_world_file_name == "":
                        if 'Grasptransf' in pick[rob][obstacle][fromLocation].keys():
                            grasp_transf =pick[rob][obstacle][fromLocation]['Grasptransf']
                            print("Start looping along grasp_transfs")
                            for grasp in grasp_transf.keys():
                                #Set the move query in Kautham
                                trygrasp=grasp_transf[str(grasp)]
                                print("grasp: ",grasp)
                                print("trygrasp: ",trygrasp)
                                print("pick: ",pick[rob][obstacle][fromLocation]['Pose'])
                                goal= computeGraspControls(pick[rob][obstacle][fromLocation]['Pose'], trygrasp)
                                print("Searching path to Move to object position " )
                                print("Init= ", init)
                                print("Goal= ", goal)
                                print("Robot Control=",Robot_control)
                                print("Trying Grasp transf=",trygrasp)
                                try:
                                    #Set the move query in Kautham
                                    kautham.kSetQuery(init,goal)
                                    #Solve query
                                    print("Solving Query to pick object")
                                    path=kautham.kGetPath(1)
                                    if path:
                                        print("-------- Path found: Moving to object position " )
                                        print('Storing Grasp Transf Used = ',trygrasp)
                                        graspTransfUsed=trygrasp
                                        break
                                    else:
                                        print('--------- No path found in trial ', i)
                                except:
                                    if i >8:
                                        print("failed to compute grasp with viable solution")
                        else:
                            print("Error: Do not know how to grasp - No Grasptranf found and no GraspIt path defined")
                        if path:
                            break
                    #Use GraspIt
                    else:
                        trygrasp=""
                        goal= computeGraspControls(pick[rob][obstacle][fromLocation]['Pose'], trygrasp)
                        #Set the move query in Kautham
                        print("Searching path to Move to object position " )
                        print("Init= ", init)
                        print("Goal= ", goal)
                        print("Robot Control=",Robot_control)
                        print("Using GraspIt")
                        try:
                            #Set the move query in Kautham
                            kautham.kSetQuery(init,goal)
                            #Solve query
                            print("Solving Query to pick object")
                            path=kautham.kGetPath(1)
                            if path:
                                print("-------- Path found: Moving to object position " )
                                break
                        except:
                            if i >8:
                                print("failed to compute grasp with viable solution")
            #Write path to taskfile
            if path:
                taskfile.write("\t<Transit>\n")
                k= sorted(list(path.keys()))[-1][1]+1
                for i in range(sorted(list(path.keys()))[-1][0]):
                    tex=''
                    for j in range(0,k):
                        tex=tex + str(path[i,j]) + " "  
                    writePath(taskfile,tex)
                taskfile.write("\t</Transit>\n")
                kautham.kMoveRobot(goal)

            else:
                print("**************************************************************************")
                print("Get path Failed! No Move possible, Infeasible Task Plan")
                print("**************************************************************************")
                break
            #Send pick query to kautham
            print ("Picking object",obsIndex)
            kautham.kAttachObject(robotIndex, linkIndex, obsIndex)

            #Move back to the home configuration of the region with the picked object
            print("Searching path to Move back to the home configuration of the region")
            print("Init= ", goal)
            print("Goal= ", init)
            print("Robot Control=",Robot_control)
            #Set robot control
            kautham.kSetRobControlsNoQuery(Robot_control)
            kautham.kSetQuery(goal,init)
            path= kautham.kGetPath(1)
            if path:
                print("-------- Path found: Moving to the home configuration of the region " )
                #start transfer
                graspedobject= True
                taskfile.write("\t<Transfer object = \"%d\" robot = \"0\" link = \"%d\">\n" % (obsIndex, linkIndex))
                k= sorted(list(path.keys()))[-1][1]+1
                for i in range(sorted(list(path.keys()))[-1][0]):
                    tex=''
                    for j in range(0,k):
                        tex=tex + str(path[i,j]) + " "  
                    writePath(taskfile,tex)
                kautham.kMoveRobot(init)
            else:
                print("**************************************************************************")
                print("Get path Failed! No Move after pick possible, Infeasible Task Plan")
                print("**************************************************************************")
                break
        #PLACE Action
        elif action =="PLACE":
            print("**************************************************************************")
            print("  PLACE ACTION  ")
            print("**************************************************************************")
            obstacle = Line[2]
            toLoaction = Line[3]
            print(action +" "+rob+" "+obstacle+" "+toLoaction)
            obsIndex= place[rob][obstacle][toLocation]['Object_Name']
            robotIndex =place[rob][obstacle][toLocation]['Robot_name']
            init =place[rob][obstacle][toLocation]['Regioncontrols']
            Robot_control=place[rob][obstacle][toLocation]['Cont']
            #Set robot control
            kautham.kSetRobControlsNoQuery(Robot_control)
            if 'Graspcontrols' in place[rob][obstacle][toLocation].keys():
                #grasp_control= place[rob][obstacle][toLocation]['Graspcontrols']
                #for grasp in grasp_control.keys():
                goal = place[rob][obstacle][toLocation]['Graspcontrols'][graspControlsUsed]
                #Set the move query in Kautham
                print("Searching path to Move to the location where object must be placed - using graspcontrols = ", graspControlsUsed)
                print("Init= ", init)
                print("Goal= ", goal)
                print("Robot Control=",Robot_control)
                #Move robot to init
                #Move_to_init(init,rob)
                #Set robot control
                kautham.kSetRobControlsNoQuery(Robot_control)
                kautham.kSetQuery(init,goal)
                #Solve query
                print("Solving query to place object")
                path=kautham.kGetPath(1)
                #Write path to task file
                if path:
                    print("-------- Path found: Moving to the location where object must be placed " )
                    #finish transfer
                    k= sorted(list(path.keys()))[-1][1]+1
                    for i in range(sorted(list(path.keys()))[-1][0]):
                        tex=''
                        for j in range(0,k):
                            tex=tex + str(path[i,j]) + " "  
                        writePath(taskfile,tex)
                    taskfile.write("\t</Transfer>\n")
                    kautham.kMoveRobot(goal)
                    graspedobject= False

                else:
                    print("**************************************************************************")
                    print("Get Path failed. No MOVE TO PLACE possible. Infeasible TASK PLAN\nTrying next Grasp Control")
                    print("**************************************************************************")

            else:
                #Loop to iterate through IK solutions
                for i in range(10):
                    if 'Poseregion' in place[rob][obstacle][toLocation].keys():
                        goal=computeGraspControls(place[rob][obstacle][toLocation]['Poseregion'],graspTransfUsed)
                    else :
                        goal=computeGraspControls(place[rob][obstacle][toLocation]['Pose'],graspTransfUsed)
                    #Set the move query in Kautham
                    print("Searching path to Move to the location where object must be placed - using grasp tranf")
                    print("Init= ", init)
                    print("Goal= ", goal)
                    print("Robot Control=",Robot_control)
                    try:
                        #Set the move query in Kautham
                        kautham.kSetQuery(init,goal)
                        #Solve query
                        print("Solving Query to place object")
                        path=kautham.kGetPath(1)
                        if path:
                            print("-------- Path found: Moving to the location where object must be placed " )
                            break
                        else:
                            print('--------- No path found in trial ', i)
                    except:
                        if i >8:
                            print("failed to compute grasp with viable slution")

                if path:
                    #finish transfer
                    k= sorted(list(path.keys()))[-1][1]+1
                    for i in range(sorted(list(path.keys()))[-1][0]):
                        tex=''
                        for j in range(0,k):
                            tex=tex + str(path[i,j]) + " "  
                        writePath(taskfile,tex)
                    taskfile.write("\t</Transfer>\n")
                    kautham.kMoveRobot(goal)
                    graspedobject= False

            if not path:
                print("**************************************************************************")
                print("Get Path failed. No MOVE TO PLACE possible. Infeasible TASK PLAN")
                print("**************************************************************************")
                break
            #Set Place query to kautham
            print("Placing object ",obsIndex)
            kautham.kDetachObject(obsIndex)
            #Move back to the home configuration of the region without the object
            print("-Searching path to Move back to the home configuration of the region")
            print("Init= ", goal)
            print("Goal= ", init)
            print("Robot Control=",Robot_control)
            #Set robot control
            kautham.kSetRobControlsNoQuery(Robot_control)
            kautham.kSetQuery(goal,init)
            path=kautham.kGetPath(1)
            if path:
                print("-------- Path found: Moving to the home configuration of the region " )
                kautham.kMoveRobot(init)
                k= sorted(list(path.keys()))[-1][1]+1
                taskfile.write("\t<Transit>\n")
                for i in range(sorted(list(path.keys()))[-1][0]):
                    tex=''
                    for j in range(0,k):
                        tex=tex + str(path[i,j]) + " "  
                    writePath(taskfile,tex)
                taskfile.write("\t</Transit>\n")
            else:
                print("**************************************************************************")
                print("Get path Failed! No Move possible after Place process, Infeasible Task Plan")
                print("**************************************************************************")
                break

        else:
            print("Action has not been defined")

    #Close kautham problem
    kautham.kCloseProblem()
    #Close and save XML document
    taskfile.write("</Task>")
    taskfile.close()
    print("Results saved in ", taskfile)
    

if __name__ == '__main__':
    try:
        #Run the main function 
        main()
    except rospy.ROSInterruptException:
        pass
