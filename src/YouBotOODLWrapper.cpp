/******************************************************************************
 * Copyright (c) 2011
 * Locomotec
 *
 * Author:
 * Sebastian Blumenthal
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include "youbot_driver_ros_interface/YouBotOODLWrapper.h"
#include "youbot_driver_ros_interface/joint_state_observer_oodl.h"

#include <youbot_trajectory_action_server/joint_trajectory_action.h>
#include <sstream>

namespace youBot
{

YouBotOODLWrapper::YouBotOODLWrapper()
{
}

YouBotOODLWrapper::YouBotOODLWrapper(ros::NodeHandle n) :
node(n)
{

    youBotConfiguration.hasBase = false;
    youBotConfiguration.hasArms = false;
    areBaseMotorsSwitchedOn = false;	
    areArmMotorsSwitchedOn = false;

    youBotChildFrameID = "base_link"; //holds true for both: base and arm
    armJointStateMessages.clear();
///////////////////////////////////////////////////////////////////
    armConfigMessages.clear();

    n.param("youBotDriverCycleFrequencyInHz", youBotDriverCycleFrequencyInHz, 200.0);
    //n.param("trajectoryActionServerEnable", trajectoryActionServerEnable, false);
    //n.param("trajectoryVelocityGain", trajectoryVelocityGain, 0.0);
    //n.param("trajectoryPositionGain", trajectoryPositionGain, 5.0);
    n.param("youBotI",youBotI,0);
    gripperCycleCounter = 0;
    diagnosticNameArm = "platform_Arm";
    diagnosticNameBase = "platform_Base";
    dashboardMessagePublisher = n.advertise<pr2_msgs::PowerBoardState>("/dashboard/platform_state", 1);
    diagnosticArrayPublisher = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
}

YouBotOODLWrapper::~YouBotOODLWrapper()
{
    this->stop();
    dashboardMessagePublisher.shutdown();
    diagnosticArrayPublisher.shutdown();
}

void YouBotOODLWrapper::initializeBase(std::string baseName)
{

    try
    {
        ROS_INFO("Configuration file path: %s", youBotConfiguration.configurationFilePath.c_str());
        youBotConfiguration.baseConfiguration.youBotBase = new youbot::YouBotBase(baseName, youBotConfiguration.configurationFilePath);
        youBotConfiguration.baseConfiguration.youBotBase->doJointCommutation();
    }
    catch (std::exception& e)
    {
        std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Base \"%s\" could not be initialized.", baseName.c_str());
        youBotConfiguration.hasBase = false;
        return;
    }

    /* setup input/output communication */
    youBotConfiguration.baseConfiguration.baseCommandSubscriber = node.subscribe("cmd_vel", 1000, &YouBotOODLWrapper::baseCommandCallback, this);
    youBotConfiguration.baseConfiguration.baseOdometryPublisher = node.advertise<nav_msgs::Odometry > ("odom", 1);
    youBotConfiguration.baseConfiguration.baseJointStatePublisher = node.advertise<sensor_msgs::JointState > ("base/joint_states", 1);

    /* setup services*/
    youBotConfiguration.baseConfiguration.switchOffMotorsService = node.advertiseService("base/switchOffMotors", &YouBotOODLWrapper::switchOffBaseMotorsCallback, this);
    youBotConfiguration.baseConfiguration.switchONMotorsService = node.advertiseService("base/switchOnMotors", &YouBotOODLWrapper::switchOnBaseMotorsCallback, this);

    /* setup frame_ids */
    youBotOdometryFrameID = "odom";
    youBotOdometryChildFrameID = "base_footprint";

    ROS_INFO("Base is initialized.");
    youBotConfiguration.hasBase = true;
    areBaseMotorsSwitchedOn = true;
}

void YouBotOODLWrapper::initializeArm(std::string armName, bool enableStandardGripper)
{
    int armIndex;
    youbot::JointName jointNameParameter;
    std::string jointName;
    stringstream topicName;
    stringstream serviceName;

    try
    {
        ROS_INFO("Configuration file path: %s", youBotConfiguration.configurationFilePath.c_str());   
        YouBotArmConfiguration tmpArmConfig;
        youBotConfiguration.youBotArmConfigurations.push_back(tmpArmConfig);
        armIndex = static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()) - 1;
        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm = new youbot::YouBotManipulator(armName, youBotConfiguration.configurationFilePath);
        youBotConfiguration.youBotArmConfigurations[armIndex].armID = armName;
        topicName.str("");
        topicName << "arm_" << (armIndex + 1) << "/";
        youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName = topicName.str(); // e.g. arm_1/
        youBotConfiguration.youBotArmConfigurations[armIndex].parentFrameIDName = "base_link";
        youBotConfiguration.armNameToArmIndexMapping.insert(make_pair(armName, static_cast<int> (youBotConfiguration.youBotArmConfigurations.size())));

        /* take joint names form configuration files */
        youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.clear();
        for (int i = 0; i < youBotArmDoF; ++i)
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getConfigurationParameter(jointNameParameter);
            jointNameParameter.getParameter(jointName);
            youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.push_back(jointName);
            ROS_INFO("Joint %i for arm %s has name: %s", i + 1, youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str(), youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i].c_str());

        }


        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->doJointCommutation();
        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->calibrateManipulator();
        if (enableStandardGripper)
        {
        	youbot::GripperBarName barName;
        	std::string gripperBarName;

            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar1().getConfigurationParameter(barName);
            barName.getParameter(gripperBarName);
            youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX] = gripperBarName;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 1, youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str(), gripperBarName.c_str());

            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar2().getConfigurationParameter(barName);
            barName.getParameter(gripperBarName);
            youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX] = gripperBarName;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 2, youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str(), gripperBarName.c_str());

            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->calibrateGripper();
        }
    }
    catch (std::exception& e)
    {
        youBotConfiguration.youBotArmConfigurations.pop_back();
        std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Arm \"%s\" could not be initialized.", armName.c_str());
        ROS_INFO("System has %i initialized arm(s).", static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));
        return;
    }


    /* setup input/output communication */


   /* topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/pwm_command"; // e.g. arm_1/arm_controller/positionCommand
    youBotConfiguration.youBotArmConfigurations[armIndex].armPWMCommandSubscriber = node.subscribe<youbot_driver_ros_interface::JointPWMs > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armPWMsCommandCallback, this, _1, armIndex));*/


     topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/current_command"; // e.g. arm_1/arm_controller/positionCommand
    youBotConfiguration.youBotArmConfigurations[armIndex].armCurrentCommandSubscriber = node.subscribe<youbot_driver_ros_interface::JointCurrents > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armCurrentsCommandCallback, this, _1, armIndex));


    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/position_command"; // e.g. arm_1/arm_controller/positionCommand
    youBotConfiguration.youBotArmConfigurations[armIndex].armPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armPositionsCommandCallback, this, _1, armIndex));

    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/velocity_command";
    youBotConfiguration.youBotArmConfigurations[armIndex].armVelocityCommandSubscriber = node.subscribe<brics_actuator::JointVelocities > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armVelocitiesCommandCallback, this, _1, armIndex));

    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/torques_command";
    youBotConfiguration.youBotArmConfigurations[armIndex].armTorquesCommandSubscriber = node.subscribe<brics_actuator::JointTorques > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armTorquesCommandCallback, this, _1, armIndex));

	topicName.str("");
	topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/follow_joint_trajectory";
	// topicName.str("/arm_1/arm_controller/follow_joint_trajectory");
	youBotConfiguration.youBotArmConfigurations[armIndex].armJointTrajectoryAction = new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction > (
					node, topicName.str(),
					boost::bind(&YouBotOODLWrapper::armJointTrajectoryGoalCallback, this, _1, armIndex),
					boost::bind(&YouBotOODLWrapper::armJointTrajectoryCancelCallback, this, _1, armIndex), false);


    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "joint_states";
    youBotConfiguration.youBotArmConfigurations[armIndex].armJointStatePublisher = node.advertise<sensor_msgs::JointState > (topicName.str(), 1); //TODO different names or one topic?

    topicName.str("");
    topicName <<youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/gains/arm_joint_1/parameter_updates";
    youBotConfiguration.youBotArmConfigurations[armIndex].armjoint1ParameterCommandPublisher = node.advertise<sensor_msgs::JointState> (topicName.str(), 1);

   
    /*topicName.str("");
    topicName <<youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/gains/arm_joint_2/parameter_updates";
    youBotConfiguration.youBotArmConfigurations[armIndex].armjoint2ParemterCommandPublisher = node.advertise<dynamic_reconfigure::Config > (topicName.str(), 1);

    topicName.str("");
    topicName <<youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/gains/arm_joint_3/parameter_updates";
    youBotConfiguration.youBotArmConfigurations[armIndex].armjoint3ParemterCommandPublisher = node.advertise<dynamic_reconfigure::Config > (topicName.str(), 1);

    topicName.str("");
    topicName <<youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/gains/arm_joint_4/parameter_updates";
    youBotConfiguration.youBotArmConfigurations[armIndex].armjoint4ParemterCommandPublisher = node.advertise<dynamic_reconfigure::Config > (topicName.str(), 1);

    topicName.str("");
    topicName <<youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/gains/arm_joint_5/parameter_updates";
    youBotConfiguration.youBotArmConfigurations[armIndex].armjoint5ParemterCommandPublisher = node.advertise<dynamic_reconfigure::Config > (topicName.str(), 1);*/

   

    if (enableStandardGripper)
    {
        topicName.str("");
        topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "gripper_controller/position_command";
        youBotConfiguration.youBotArmConfigurations[armIndex].gripperPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::gripperPositionsCommandCallback, this, _1, armIndex));
        youBotConfiguration.youBotArmConfigurations[armIndex].lastGripperCommand = 0.0; //This is true if the gripper is calibrated.
    }



    /* setup services*/
    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "switchOffMotors"; // e.g. "arm_1/switchOffMotors"
    youBotConfiguration.youBotArmConfigurations[armIndex].switchOffMotorsService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::switchOffArmMotorsCallback, this, _1, _2, armIndex));


    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "setParam"; // e.g. "arm_1/switchOffMotors"
    youBotConfiguration.youBotArmConfigurations[armIndex].killService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::ParametersControlCallback, this, _1, _2, armIndex));



    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "switchOnMotors"; // e.g. "arm_1/switchOnMotors"
    youBotConfiguration.youBotArmConfigurations[armIndex].switchONMotorsService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::switchOnArmMotorsCallback, this, _1, _2, armIndex));

    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "calibrate"; // e.g. "arm_1/calibrate"
    youBotConfiguration.youBotArmConfigurations[armIndex].calibrateService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::calibrateArmCallback, this, _1, _2, armIndex));
/*
    if (trajectoryActionServerEnable)
    {
        JointStateObserver* jointStateObserver = new JointStateObserverOODL(this, armIndex);
        topicName.str("");
        topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "action";
        youBotConfiguration.youBotArmConfigurations[armIndex].jointTrajectoryAction = new JointTrajectoryAction(jointStateObserver,
                                                                                                                trajectoryPositionGain,
                                                                                                                trajectoryVelocityGain,
                                                                                                                youBotDriverCycleFrequencyInHz);

        serviceName.str("");
        serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/joint_trajectory_action"; // e.g. "arm_1/switchOnMotors"
        
        youBotConfiguration.youBotArmConfigurations[armIndex].trajectoryActionServer = new Server(serviceName.str(),
                                                                                                  boost::bind(&YouBotOODLWrapper::executeActionServer, this, _1, armIndex),
                                                                                                  false);
        youBotConfiguration.youBotArmConfigurations[armIndex].trajectoryActionServer->start();
    }
*/
    /* initialize message vector for arm joint states */
    sensor_msgs::JointState dummyMessage;
    armJointStateMessages.push_back(dummyMessage);
   //////////////////////////////////////////////////////////////////////////////////////////////
    sensor_msgs::JointState dummyConfigMessage;
    armConfigMessages.push_back(dummyConfigMessage);

    /* setup frame_ids */
    youBotArmFrameID = "arm"; //TODO find default topic name
    ROS_INFO("Arm \"%s\" is initialized.", armName.c_str());
    ROS_INFO("System has %i initialized arm(s).", static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));
    youBotConfiguration.hasArms = true;
    areArmMotorsSwitchedOn = true;

    // currently no action is running
	armHasActiveJointTrajectoryGoal = false;

    //tracejoint = 4;
    //myTrace = new youbot::DataTrace(youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(tracejoint), "Joint4TrajectoryTrace");

    // we can handle actionlib requests only after the complete initialization has been performed
	youBotConfiguration.youBotArmConfigurations[armIndex].armJointTrajectoryAction->start();
}

/*
void YouBotOODLWrapper::executeActionServer(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, int armIndex)
{

    JointTrajectoryAction* jointTrajectoryAction = youBotConfiguration.youBotArmConfigurations[armIndex].jointTrajectoryAction;
    if (jointTrajectoryAction != NULL)
    {

        jointTrajectoryAction->execute(goal, youBotConfiguration.youBotArmConfigurations[armIndex].trajectoryActionServer);
    }
}
*/
void YouBotOODLWrapper::stop()
{

    if (youBotConfiguration.baseConfiguration.youBotBase)
    {
        delete youBotConfiguration.baseConfiguration.youBotBase;
        youBotConfiguration.baseConfiguration.youBotBase = 0;
    }

    youBotConfiguration.baseConfiguration.baseCommandSubscriber.shutdown();
    youBotConfiguration.baseConfiguration.baseJointStatePublisher.shutdown();
    youBotConfiguration.baseConfiguration.baseOdometryPublisher.shutdown();
    youBotConfiguration.baseConfiguration.switchONMotorsService.shutdown();
    youBotConfiguration.baseConfiguration.switchOffMotorsService.shutdown();
    // youBotConfiguration.baseConfiguration.odometryBroadcaster.
    youBotConfiguration.hasBase = false;
    areBaseMotorsSwitchedOn = false;


    for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++) //delete each arm
    {
        if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm)
        {
            delete youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm;
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm = 0;
        }
         //////////////////////////////////////////////////////////
        youBotConfiguration.youBotArmConfigurations[armIndex].armjoint1ParameterCommandPublisher.shutdown();
        youBotConfiguration.youBotArmConfigurations[armIndex].armJointStatePublisher.shutdown();
        youBotConfiguration.youBotArmConfigurations[armIndex].armPositionCommandSubscriber.shutdown();
        youBotConfiguration.youBotArmConfigurations[armIndex].armVelocityCommandSubscriber.shutdown();
        youBotConfiguration.youBotArmConfigurations[armIndex].armTorquesCommandSubscriber.shutdown();
        youBotConfiguration.youBotArmConfigurations[armIndex].calibrateService.shutdown();
        youBotConfiguration.youBotArmConfigurations[armIndex].gripperPositionCommandSubscriber.shutdown();
        youBotConfiguration.youBotArmConfigurations[armIndex].switchONMotorsService.shutdown();
        youBotConfiguration.youBotArmConfigurations[armIndex].switchOffMotorsService.shutdown();
    }

    youBotConfiguration.hasArms = false;
    areArmMotorsSwitchedOn = false;
    youBotConfiguration.youBotArmConfigurations.clear();
    armJointStateMessages.clear();
//////////////////////////////////////////////////////////////////////////
    armConfigMessages.clear();


    youbot::EthercatMaster::destroy();
}

void YouBotOODLWrapper::baseCommandCallback(const geometry_msgs::Twist& youbotBaseCommand)
{

    if (youBotConfiguration.hasBase)
    { // in case stop has been invoked
        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;

        /*
         * Frame in OODL:
         *
         *		 FRONT
         *
         *         X
         *         ^
         *         |
         *         |
         *         |
         * Y <-----+
         *
         *        BACK
         *
         * Positive angular velocity means turning counterclockwise
         *
         */

        longitudinalVelocity = youbotBaseCommand.linear.x * meter_per_second;
        transversalVelocity = youbotBaseCommand.linear.y * meter_per_second;
        angularVelocity = youbotBaseCommand.angular.z * radian_per_second;

        try
        {
            youBotConfiguration.baseConfiguration.youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
        }

    }
    else
    {
        ROS_ERROR("No base initialized!");
    }
}



/////////////////////////////////////////////////////////////////
/////////////          armCurrentsCommandCallback    ///////////
///////////////////////////////////////////////////////////////


void YouBotOODLWrapper::armCurrentsCommandCallback(const youbot_driver_ros_interface::JointCurrentsConstPtr& youbotArmCommand, int armIndex)
{
    ROS_DEBUG("Command for arm%i received", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) // in case stop has been invoked
    {

        ROS_DEBUG("Arm ID is: %s", youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str());
        if (youbotArmCommand->currents.size() < 1)
        {
            ROS_WARN("youBot driver received an invalid joint positions command.");
            return;
        }

        youbot::JointCurrentSetpoint jointCurrent;
        string unit = boost::units::to_string(boost::units::si::ampere);

        /* populate mapping between joint names and values  */
        std::map<string, double> jointNameToValueMapping;
        for (int i = 0; i < static_cast<int> (youbotArmCommand->currents.size()); ++i)
        {
            if (unit == youbotArmCommand->currents[i].unit)
            {
                jointNameToValueMapping.insert(make_pair(youbotArmCommand->currents[i].joint_uri, youbotArmCommand->currents[i].value));
            }
            else
            {
                ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotArmCommand->currents[i].unit.c_str(), unit.c_str());
            }
        }

        /* loop over all youBot arm joints and check if something is in the received message that requires action */
        ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
        for (int i = 0; i < youBotArmDoF; ++i)
        {

            /* check what is in map */
            map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]);
            if (jointIterator != jointNameToValueMapping.end())
            {

                /* set the desired joint value */
                ROS_DEBUG("Trying to set joint %s to new position value %f", (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]).c_str(), jointIterator->second);
                jointCurrent.current = jointIterator->second * ampere;
                try
                {
                    youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).setData(jointCurrent); //youBot joints start with 1 not with 0 -> i + 1
                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: %s", i + 1, errorMessage.c_str());
                }
            }
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
    }
    else
    {
        ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
    }

}
















void YouBotOODLWrapper::armPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotArmCommand, int armIndex)
{
    ROS_DEBUG("Command for arm%i received", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) // in case stop has been invoked
    {

        ROS_DEBUG("Arm ID is: %s", youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str());
        if (youbotArmCommand->positions.size() < 1)
        {
            ROS_WARN("youBot driver received an invalid joint positions command.");
            return;
        }

        youbot::JointAngleSetpoint desiredAngle;
        string unit = boost::units::to_string(boost::units::si::radian);

        /* populate mapping between joint names and values  */
        std::map<string, double> jointNameToValueMapping;
        for (int i = 0; i < static_cast<int> (youbotArmCommand->positions.size()); ++i)
        {
            if (unit == youbotArmCommand->positions[i].unit)
            {
                jointNameToValueMapping.insert(make_pair(youbotArmCommand->positions[i].joint_uri, youbotArmCommand->positions[i].value));
            }
            else
            {
                ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotArmCommand->positions[i].unit.c_str(), unit.c_str());
            }
        }

        /* loop over all youBot arm joints and check if something is in the received message that requires action */
        ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
        for (int i = 0; i < youBotArmDoF; ++i)
        {

            /* check what is in map */
            map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]);
            if (jointIterator != jointNameToValueMapping.end())
            {

                /* set the desired joint value */
                ROS_DEBUG("Trying to set joint %s to new position value %f", (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]).c_str(), jointIterator->second);
                desiredAngle.angle = jointIterator->second * radian;
                try
                {
                    youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).setData(desiredAngle); //youBot joints start with 1 not with 0 -> i + 1
                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: %s", i + 1, errorMessage.c_str());
                }
            }
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
    }
    else
    {
        ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
    }

}

void YouBotOODLWrapper::armVelocitiesCommandCallback(const brics_actuator::JointVelocitiesConstPtr& youbotArmCommand, int armIndex)
{
    ROS_DEBUG("Command for arm%i received", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0)
    { // in case stop has been invoked


        if (youbotArmCommand->velocities.size() < 1)
        {
            ROS_WARN("youBot driver received an invalid joint velocities command.");
            return;
        }

        youbot::JointVelocitySetpoint desiredAngularVelocity;
        string unit = boost::units::to_string(boost::units::si::radian_per_second);

        /* populate mapping between joint names and values  */
        std::map<string, double> jointNameToValueMapping;
        for (int i = 0; i < static_cast<int> (youbotArmCommand->velocities.size()); ++i)
        {
            if (unit == youbotArmCommand->velocities[i].unit)
            {
                jointNameToValueMapping.insert(make_pair(youbotArmCommand->velocities[i].joint_uri, youbotArmCommand->velocities[i].value));
            }
            else
            {
                ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotArmCommand->velocities[i].unit.c_str(), unit.c_str());
            }

        }

        /* loop over all youBot arm joints and check if something is in the received message that requires action */
        ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
        for (int i = 0; i < youBotArmDoF; ++i)
        {

            /* check what is in map */
            map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]);
            if (jointIterator != jointNameToValueMapping.end())
            {

                /* set the desired joint value */
                ROS_DEBUG("Trying to set joint %s to new velocity value %f", (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]).c_str(), jointIterator->second);
                desiredAngularVelocity.angularVelocity = jointIterator->second * radian_per_second;
                try
                {
                    youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).setData(desiredAngularVelocity); //youBot joints start with 1 not with 0 -> i + 1

                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: %s", i + 1, errorMessage.c_str());
                }
            }
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
    }
    else
    {
        ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
    }
}

void YouBotOODLWrapper::armTorquesCommandCallback(const brics_actuator::JointTorquesConstPtr& youbotArmCommand, int armIndex)
{
  ROS_DEBUG("Command for arm%i received", armIndex + 1);
      ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

      if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) // in case stop has been invoked
      {

          ROS_DEBUG("Arm ID is: %s", youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str());
          if (youbotArmCommand->torques.size() < 1)
          {
              ROS_WARN("youBot driver received an invalid joint torques command.");
              return;
          }

          youbot::JointTorqueSetpoint desiredTorque;
          string unit = boost::units::to_string(boost::units::si::newton_meter);

          /* populate mapping between joint names and values  */
          std::map<string, double> jointNameToValueMapping;
          for (int i = 0; i < static_cast<int> (youbotArmCommand->torques.size()); ++i)
          {
              if (unit == youbotArmCommand->torques[i].unit)
              {
                  jointNameToValueMapping.insert(make_pair(youbotArmCommand->torques[i].joint_uri, youbotArmCommand->torques[i].value));
              }
              else
              {
                  ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotArmCommand->torques[i].unit.c_str(), unit.c_str());
              }
          }

          /* loop over all youBot arm joints and check if something is in the received message that requires action */
          ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
          youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
          for (int i = 0; i < youBotArmDoF; ++i)
          {

              /* check what is in map */
              map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]);
              if (jointIterator != jointNameToValueMapping.end())
              {

                  /* set the desired joint value */
                  ROS_DEBUG("Trying to set joint %s to new torque value %f", (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]).c_str(), jointIterator->second);
                  desiredTorque.torque = jointIterator->second * newton_meter;
                  try
                  {
                      youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).setData(desiredTorque); //youBot joints start with 1 not with 0 -> i + 1
                  }
                  catch (std::exception& e)
                  {
                      std::string errorMessage = e.what();
                      ROS_WARN("Cannot set arm joint %i: %s", i + 1, errorMessage.c_str());
                  }
              }
          }
          youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
      }
      else
      {
          ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
      }
}

void YouBotOODLWrapper::armJointTrajectoryGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex) {
	ROS_INFO("Goal for arm%i received", armIndex + 1);
	ROS_ASSERT(armIndex < youBotConfiguration.youBotArmConfigurations.size());

	if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm == 0) {
		ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
		youbotArmGoal.setRejected();
		return;
	}


	trajectory_msgs::JointTrajectory trajectory = youbotArmGoal.getGoal()->trajectory;

	// validate that the correct number of joints is provided in the goal
	if (trajectory.joint_names.size() != static_cast<unsigned int> (youBotArmDoF)) {
		ROS_ERROR("Trajectory is malformed! Goal has %i joint names, but only %i joints are supported", static_cast<int> (trajectory.joint_names.size()), youBotArmDoF);
		youbotArmGoal.setRejected();
		return;
	}

	// compare the joint names of the youBot configuration and joint names provided with the trajectory
	for (unsigned int i = 0; i < youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size(); i++) {
		bool jointNameFound = false;
		for (unsigned int j = 0; j < trajectory.joint_names.size(); j++) {
			if (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i] == trajectory.joint_names[j]) {
				jointNameFound = true;
				break;
			}
		}

		if (!jointNameFound) {
			ROS_ERROR("Trajectory is malformed! Joint %s is missing in the goal", youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i].c_str());
			youbotArmGoal.setRejected();
			return;
		}
	}

  std::vector<youbot::JointTrajectory> jointTrajectories(youBotArmDoF);
  
	// convert from the ROS trajectory representation to the controller's representation
	std::vector<std::vector< quantity<plane_angle> > > positions(youBotArmDoF);
	std::vector<std::vector< quantity<angular_velocity> > > velocities(youBotArmDoF);
	std::vector<std::vector< quantity<angular_acceleration> > > accelerations(youBotArmDoF);
  youbot::TrajectorySegment segment;
	for (unsigned int i = 0; i < trajectory.points.size(); i++) {
		trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
		// validate the trajectory point
		if ((point.positions.size() != static_cast<unsigned int> (youBotArmDoF)
						|| point.velocities.size() != static_cast<unsigned int> (youBotArmDoF)
						|| point.accelerations.size() != static_cast<unsigned int> (youBotArmDoF))) {
			ROS_ERROR("A trajectory point is malformed! %i positions, velocities and accelerations must be provided", youBotArmDoF);
			youbotArmGoal.setRejected();
			return;
		}
    
		for (int j = 0; j < youBotArmDoF; j++) {
      segment.positions = point.positions[j]*radian;
      segment.velocities = point.velocities[j]*radian_per_second;
      segment.accelerations = point.accelerations[j] * radian_per_second/second;
      segment.time_from_start = boost::posix_time::microsec(point.time_from_start.toNSec()/1000);
      jointTrajectories[j].segments.push_back(segment);
		}
	}
  for (int j = 0; j < youBotArmDoF; j++) {
      jointTrajectories[j].start_time = boost::posix_time::microsec_clock::local_time(); //TODO is this correct to set the trajectory start time to now
  }

  
  

	// cancel the old goal
  /*
	if (armHasActiveJointTrajectoryGoal) {
		armActiveJointTrajectoryGoal.setCanceled();
		armHasActiveJointTrajectoryGoal = false;
		for (int i = 0; i < youBotArmDoF; ++i) {
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).cancelTrajectory();
		}
	}
  */

	// replace the old goal with the new one
	youbotArmGoal.setAccepted();
	armActiveJointTrajectoryGoal = youbotArmGoal;
	armHasActiveJointTrajectoryGoal = true;

  
 // myTrace->startTrace();

	// send the trajectory to the controller
	for (int i = 0; i < youBotArmDoF; ++i) {
		try {
			// youBot joints start with 1 not with 0 -> i + 1
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).trajectoryController.setTrajectory(jointTrajectories[i]);
      ROS_INFO("set trajectories %d", i);
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot set trajectory for joint %i: %s", i + 1, errorMessage.c_str());
		}
	}
	ROS_INFO("set all trajectories");
}

void YouBotOODLWrapper::armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex) {
	ROS_DEBUG("Goal for arm%i received", armIndex + 1);
	ROS_ASSERT(armIndex < youBotConfiguration.youBotArmConfigurations.size());

	// stop the controller
	for (int i = 0; i < youBotArmDoF; ++i) {
		try {
			// youBot joints start with 1 not with 0 -> i + 1
      //TODO cancel trajectory
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).trajectoryController.cancelCurrentTrajectory();
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).stopJoint();
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot stop joint %i: %s", i + 1, errorMessage.c_str());
		}
	}

	if (armActiveJointTrajectoryGoal == youbotArmGoal) {
		// Marks the current goal as canceled.
		youbotArmGoal.setCanceled();
		armHasActiveJointTrajectoryGoal = false;
	}
}

void YouBotOODLWrapper::gripperPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotGripperCommand, int armIndex)
{
	ROS_DEBUG("Command for gripper%i received", armIndex + 1);
	ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));

	if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) { // in case stop has been invoked

		if (youbotGripperCommand->positions.size() < 1){
			ROS_WARN("youBot driver received an invalid gripper positions command.");
			return;
		}

		map<string, double>::const_iterator gripperIterator;
		youbot::GripperBarPositionSetPoint leftGripperFingerPosition;
		youbot::GripperBarPositionSetPoint rightGripperFingerPosition;
		string unit = boost::units::to_string(boost::units::si::meter);

		/* populate mapping between joint names and values */
		std::map<string, double> jointNameToValueMapping;
		for (int i = 0; i < static_cast<int>(youbotGripperCommand->positions.size()); ++i) {
			if (unit == youbotGripperCommand->positions[i].unit) {
				jointNameToValueMapping.insert(make_pair(youbotGripperCommand->positions[i].joint_uri, youbotGripperCommand->positions[i].value));
			} else {
				ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotGripperCommand->positions[i].unit.c_str(), unit.c_str());
			}
		}

		youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time

		/* check if something is in the received message that requires action for the left finger gripper */
		gripperIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX]);
		if (gripperIterator != jointNameToValueMapping.end()) {
			ROS_DEBUG("Trying to set the left gripper finger to new value %f", gripperIterator->second);

			leftGripperFingerPosition.barPosition = gripperIterator->second * meter;
			try {
				youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar1().setData(leftGripperFingerPosition);
			} catch (std::exception& e) {
				std::string errorMessage = e.what();
				ROS_WARN("Cannot set the left gripper finger: %s", errorMessage.c_str());
			}
		}

		/* check if something is in the received message that requires action for the right finger gripper */
		gripperIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX]);
		if (gripperIterator != jointNameToValueMapping.end()) {
			ROS_DEBUG("Trying to set the right gripper to new value %f", gripperIterator->second);

			rightGripperFingerPosition.barPosition = gripperIterator->second * meter;
			try {
				youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar2().setData(rightGripperFingerPosition);
			} catch (std::exception& e) {
				std::string errorMessage = e.what();
				ROS_WARN("Cannot set the right gripper finger: %s", errorMessage.c_str());
			}
		}

		youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
	} else {
		ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
	}
}








/////////////////////////////////////////////////////////////////////
/*void YouBotOODLWrapper::armPWMsCommandCallback(const youbot_driver_ros_interface::JointPWMsConstPtr& youbotArmCommand, int armIndex)
{
    ROS_DEBUG("Command for arm%i received", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) // in case stop has been invoked
    {

        ROS_DEBUG("Arm ID is: %s", youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str());
        if (youbotArmCommand->pwms.size() < 1)
        {
            ROS_WARN("youBot driver received an invalid joint positions command.");
            return;
        }

        youbot::JointPWMSetpoint jointpwms;
       // string unit = boost::units::to_string(boost::units::si::int);         

        //
        std::map<string, double> jointNameToValueMapping;
        for (int i = 0; i < static_cast<int> (youbotArmCommand->pwms.size()); ++i)
        {
           // if (unit == youbotArmCommand->pwms[i].unit)
           // {
                jointNameToValueMapping.insert(make_pair(youbotArmCommand->pwms[i].joint_uri, youbotArmCommand->pwms[i].value));
           // }
           // else
           // {
             //   ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotArmCommand->pwms[i].unit.c_str(), unit.c_str());
            //}
        }

       
        ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
        for (int i = 0; i < youBotArmDoF; ++i)
        {

           
            map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]);
            if (jointIterator != jointNameToValueMapping.end())
            {

               
                ROS_DEBUG("Trying to set joint %s to new position value %f", (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]).c_str(), jointIterator->second);
               // jointpwms.pwm = jointIterator->second ;
                try
                {
                    youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getData(jointpwms); //youBot joints start with 1 not with 0 -> i + 1
                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: %s", i + 1, errorMessage.c_str());
                }
            }
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
    }
    else
    {
        ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
    }

}*/


/////////////////////////////////////////////////////////////////////////

void YouBotOODLWrapper::updateParameter()
{
     try
  {
        currentTime = ros::Time::now();
        youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);

       if (youBotConfiguration.hasArms == true)
    {

       for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++) 
        {
            ROS_ASSERT(youBotConfiguration.youBotArmConfigurations.size() == armConfigMessages.size());
            
            //armConfigMessages[armIndex].bools.resize(1);
            //armConfigMessages[armIndex].ints.resize(1);
            //armConfigMessages[armIndex].strs.resize(1);
            //armConfigMessages[armIndex].doubles.resize(1);
            //armConfigMessages[armIndex].groups.resize(1);
            armConfigMessages[armIndex].header.stamp = currentTime;
            armConfigMessages[armIndex].name.resize(1);
            armConfigMessages[armIndex].position.resize(youBotArmDoF);
            //armConfigMessages[armIndex].velocity.resize(youBotArmDoF + youBotNumberOfFingers);
           // armConfigMessages[armIndex].effort.resize(youBotArmDoF + youBotNumberOfFingers);
            
           if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm == 0)
            {
                ROS_ERROR("Arm%i is not correctly initialized! Cannot publish data.", armIndex + 1);
                continue;
            }
           
             armConfigMessages[armIndex].name[0] = "Joint_1"; //TODO no unique names for URDF yet


            //std::vector<dynamic_reconfigure::DoubleParameter>  doubles;
            //doubles.resize(5);

           // doubles[0].name = "p";
           // doubles[0].value = pidController[0].getGain(p);
            //doubles[1].name = "i";
           // doubles[1]
            //doubles[2].name = "d";
           // doubles[3].name = "i_clamp_min";
           // doubles[4].name = "i_clamp_max";
            double p;
            double i;
            double d;
            double i1;
            double i2;
 
    //std::vector<double> p;
 
    //std::vector<double> i;
    //std::vector<double> d;
    //std::vector<double> i1;
    //std::vector<double> i2;

       control_toolbox::Pid pid;
        pid.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
        double position_desi_ = 0.5;
        ...
        ros::Time last_time = ros::Time::now();
        while (true) {
        ros::Time time = ros::Time::now();
        double effort = pid.updatePid(currentPosition() - position_desi_, time - last_time);
        last_time = time;
        }
        
	    pidController.resize(youBotArmDoF);
            pidController[0].getGains(p, i, d,i1,i2);

            armConfigMessages[armIndex].position[0] = 5.0;
            armConfigMessages[armIndex].position[1] = i;
            armConfigMessages[armIndex].position[2] = d;
            armConfigMessages[armIndex].position[3] = i1;
            armConfigMessages[armIndex].position[4] = i2;


           // armConfigMessages[armIndex].doubles = doubles;

           }
     } 
     youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true); // ensure that all joint values will be received at the same time
  }catch (youbot::EtherCATConnectionException& e)
  {
      ROS_WARN("%s", e.what());
      
      youBotConfiguration.hasArms = false;
  }
     catch (std::exception& e)
	{
		ROS_WARN_ONCE("%s", e.what());
	}       
               
}



void YouBotOODLWrapper::computeOODLSensorReadings()
{

	try{
    currentTime = ros::Time::now();
    youbot::JointSensedAngle currentAngle;
    youbot::JointSensedVelocity currentVelocity;
    youbot::JointSensedTorque currentTorque;
////////////////////////////
    

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false); // ensure that all joint values will be received at the same time

    if (youBotConfiguration.hasBase == true)
    {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;

        double vx = 0.0;
        double vy = 0.0;
        double vtheta = 0.0;

        quantity<si::length> longitudinalPosition;
        quantity<si::length> transversalPosition;
        quantity<plane_angle> orientation;

        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;

        youBotConfiguration.baseConfiguration.youBotBase->getBasePosition(longitudinalPosition, transversalPosition, orientation);
        x = longitudinalPosition.value();
        y = transversalPosition.value();
        theta = orientation.value();

        youBotConfiguration.baseConfiguration.youBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        vx = longitudinalVelocity.value();
        vy = transversalVelocity.value();
        vtheta = angularVelocity.value();
        //ROS_DEBUG("Perceived odometric values (x,y,tetha, vx,vy,vtetha): %f, %f, %f \t %f, %f, %f", x, y, theta, vx, vy, vtheta);


        /* Setup odometry tf frame */
        odometryQuaternion = tf::createQuaternionMsgFromYaw(theta);

        odometryTransform.header.stamp = currentTime;
        odometryTransform.header.frame_id = youBotOdometryFrameID;
        odometryTransform.child_frame_id = youBotOdometryChildFrameID;

        odometryTransform.transform.translation.x = x;
        odometryTransform.transform.translation.y = y;
        odometryTransform.transform.translation.z = 0.0;
        odometryTransform.transform.rotation = odometryQuaternion;

        /* Setup odometry Message */
        odometryMessage.header.stamp = currentTime;
        odometryMessage.header.frame_id = youBotOdometryFrameID;

        odometryMessage.pose.pose.position.x = x;
        odometryMessage.pose.pose.position.y = y;
        odometryMessage.pose.pose.position.z = 0.0;
        odometryMessage.pose.pose.orientation = odometryQuaternion;

        odometryMessage.child_frame_id = youBotOdometryChildFrameID;
        //		odometryMessage.child_frame_id = youBotOdometryFrameID;
        odometryMessage.twist.twist.linear.x = vx;
        odometryMessage.twist.twist.linear.y = vy;
        odometryMessage.twist.twist.angular.z = vtheta;

        /* Set up joint state message for the wheels */
        baseJointStateMessage.header.stamp = currentTime;
        baseJointStateMessage.name.resize(youBotNumberOfWheels * 2); // *2 because of virtual wheel joints in the URDF description
        baseJointStateMessage.position.resize(youBotNumberOfWheels * 2);
        baseJointStateMessage.velocity.resize(youBotNumberOfWheels * 2);
        baseJointStateMessage.effort.resize(youBotNumberOfWheels * 2);

        ROS_ASSERT((youBotConfiguration.baseConfiguration.wheelNames.size() == static_cast<unsigned int> (youBotNumberOfWheels)));
        for (int i = 0; i < youBotNumberOfWheels; ++i)
        {
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i + 1).getData(currentVelocity);
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i + 1).getData(currentTorque);

            baseJointStateMessage.name[i] = youBotConfiguration.baseConfiguration.wheelNames[i];
            baseJointStateMessage.position[i] = currentAngle.angle.value();
            baseJointStateMessage.velocity[i] = currentVelocity.angularVelocity.value();
            baseJointStateMessage.effort[i] = currentTorque.torque.value();
        }

        /*
         * Here we add values for "virtual" rotation joints in URDF - robot_state_publisher can't
         * handle non-aggregated jointState messages well ...
         */
        baseJointStateMessage.name[4] = "caster_joint_fl";
        baseJointStateMessage.position[4] = 0.0;

        baseJointStateMessage.name[5] = "caster_joint_fr";
        baseJointStateMessage.position[5] = 0.0;

        baseJointStateMessage.name[6] = "caster_joint_bl";
        baseJointStateMessage.position[6] = 0.0;

        baseJointStateMessage.name[7] = "caster_joint_br";
        baseJointStateMessage.position[7] = 0.0;

        /*
         * Yet another hack to make the published values compatible with the URDF description.
         * We actually flipp the directions of the wheel on the right side such that the standard ROS controllers
         * (e.g. for PR2) can be used for the youBot
         */
        baseJointStateMessage.position[0] = -baseJointStateMessage.position[0];
        baseJointStateMessage.position[2] = -baseJointStateMessage.position[2];

    }

    if (youBotConfiguration.hasArms == true)
    {

        for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++)
        {
            ROS_ASSERT(youBotConfiguration.youBotArmConfigurations.size() == armJointStateMessages.size());

            /* fill joint state message */
            armJointStateMessages[armIndex].header.stamp = currentTime;
            armJointStateMessages[armIndex].name.resize(youBotArmDoF + youBotNumberOfFingers);
            armJointStateMessages[armIndex].position.resize(youBotArmDoF + youBotNumberOfFingers);
            armJointStateMessages[armIndex].velocity.resize(youBotArmDoF + youBotNumberOfFingers);
            armJointStateMessages[armIndex].effort.resize(youBotArmDoF + youBotNumberOfFingers);

            

            ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
            for (int i = 0; i < youBotArmDoF; ++i)
            {
                youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1 //FIXME might segfault if only 1eout of 2 arms are initialized.
                youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getData(currentVelocity);
                youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getData(currentTorque);

                armJointStateMessages[armIndex].name[i] = youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]; //TODO no unique names for URDF yet
                armJointStateMessages[armIndex].position[i] = currentAngle.angle.value();
                armJointStateMessages[armIndex].velocity[i] = currentVelocity.angularVelocity.value();
                armJointStateMessages[armIndex].effort[i] = currentTorque.torque.value();
            }

            // check if trajectory controller is finished
			bool areTrajectoryControllersDone = true;
			for (int i = 0; i < youBotArmDoF; ++i) {
				if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).trajectoryController.isTrajectoryControllerActive()) {
					areTrajectoryControllersDone = false;
					break;
				}
			}
			if (areTrajectoryControllersDone && armHasActiveJointTrajectoryGoal) {
                armHasActiveJointTrajectoryGoal = false;
				control_msgs::FollowJointTrajectoryResult trajectoryResult;
				trajectoryResult.error_code = trajectoryResult.SUCCESSFUL;
				armActiveJointTrajectoryGoal.setSucceeded(trajectoryResult, "trajectory successful");
                // ROS_INFO("trajectory successful");
                // myTrace->stopTrace();
                // myTrace->plotTrace();
			}

            /*
             * NOTE: gripper slide rails are always symmetric, but the fingers can be screwed in different
             * positions! The published values account for the distance between the gripper slide rails, not the fingers
             * themselves. Of course if the finger are screwed to the most inner position (i.e. the can close completely),
             * than it is correct.
             */
            /*try 
            {
                youbot::YouBotGripperBar& gripperBar1 = youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar1();
                youbot::YouBotGripperBar& gripperBar2 = youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar2();

                if  (gripperCycleCounter == 0) { //workaround: avoid congestion of mailbox message by querying only every ith iteration
            	    gripperCycleCounter = youBotDriverCycleFrequencyInHz/5; //approx. 5Hz here
            	    gripperBar1.getData(gripperBar1Position);
            	    gripperBar2.getData(gripperBar2Position);
                }
                gripperCycleCounter--;

                armJointStateMessages[armIndex].name[youBotArmDoF + 0] = youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX];
                double leftGipperFingerPosition = gripperBar1Position.barPosition.value();
                armJointStateMessages[armIndex].position[youBotArmDoF + 0] = leftGipperFingerPosition;

                double rightGipperFingerPosition = gripperBar2Position.barPosition.value();
                armJointStateMessages[armIndex].name[youBotArmDoF + 1] = youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX];
                armJointStateMessages[armIndex].position[youBotArmDoF + 1] = rightGipperFingerPosition;
            }
            catch (std::exception& e)
            {
                std::string errorMessage = e.what();
                ROS_WARN("Cannot read gripper values: %s", errorMessage.c_str());
            }*/
/*
            if (trajectoryActionServerEnable)
            {
                // updating joint states in trajectory action 
                youBotConfiguration.youBotArmConfigurations[armIndex].jointTrajectoryAction->jointStateCallback(armJointStateMessages[armIndex]);
            }
*/
        }
    }

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true); // ensure that all joint values will be received at the same time
  }catch (youbot::EtherCATConnectionException& e)
  {
      ROS_WARN("%s", e.what());
      youBotConfiguration.hasBase = false;
      youBotConfiguration.hasArms = false;
  }
	catch (std::exception& e)
	{
		ROS_WARN_ONCE("%s", e.what());
	}

}


void YouBotOODLWrapper::publishOODLSensorReadings()
{
      
    if (youBotConfiguration.hasBase)
    {
        youBotConfiguration.baseConfiguration.odometryBroadcaster.sendTransform(odometryTransform);
        youBotConfiguration.baseConfiguration.baseOdometryPublisher.publish(odometryMessage);
        youBotConfiguration.baseConfiguration.baseJointStatePublisher.publish(baseJointStateMessage);
    }

    if (youBotConfiguration.hasArms)
    {
        for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++)
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].armJointStatePublisher.publish(armJointStateMessages[armIndex]);
            youBotConfiguration.youBotArmConfigurations[armIndex].armjoint1ParameterCommandPublisher.publish(armConfigMessages[armIndex]);
        }
    }


}



bool YouBotOODLWrapper::switchOffBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	ROS_INFO("Switch off the base motors");
	if (youBotConfiguration.hasBase) { // in case stop has been invoked

        youbot::JointCurrentSetpoint currentStopMovement;
        currentStopMovement.current = 0.0 * ampere;
		try {
      youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(1).setData(currentStopMovement);
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(2).setData(currentStopMovement);
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(3).setData(currentStopMovement);
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(4).setData(currentStopMovement);
      youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot switch off the base motors: %s", errorMessage.c_str());
			return false;
		}
	} else {
		ROS_ERROR("No base initialized!");
		return false;
	}
  	areBaseMotorsSwitchedOn = false;
	return true;
}

bool YouBotOODLWrapper::switchOnBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Switch on the base motors");
    if (youBotConfiguration.hasBase)
    { // in case stop has been invoked
        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;

        longitudinalVelocity = 0.0 * meter_per_second;
        transversalVelocity = 0.0 * meter_per_second;
        angularVelocity = 0.0 * radian_per_second;

        try
        {
            youBotConfiguration.baseConfiguration.youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("No base initialized!");
        return false;
    }
    areBaseMotorsSwitchedOn = true;
    return true;
}



bool YouBotOODLWrapper::ParametersControlCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex)
{
    
     ROS_INFO("Parameter on the Arm");
     ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));
     
     if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) { 
      //P-pos_sec 
            PParameterSecondParametersPositionControl_actual.resize(youBotArmDoF);
            IParameterSecondParametersPositionControl_actual.resize(youBotArmDoF);
            DParameterSecondParametersPositionControl_actual.resize(youBotArmDoF);
            IClippingParameterSecondParametersPositionControl_actual.resize(youBotArmDoF);

            PParameterSecondParametersSpeedControl_actual.resize(youBotArmDoF);
            IParameterSecondParametersSpeedControl_actual.resize(youBotArmDoF);
            DParameterSecondParametersSpeedControl_actual.resize(youBotArmDoF);
            IClippingParameterSecondParametersSpeedControl_actual.resize(youBotArmDoF);

            PParameterCurrentControl_actual.resize(youBotArmDoF);
            IParameterCurrentControl_actual.resize(youBotArmDoF);
            DParameterCurrentControl_actual.resize(youBotArmDoF);
            IClippingParameterCurrentControl_actual.resize(youBotArmDoF);

            PParameterFirstParametersPositionControl_actual.resize(youBotArmDoF);
            IParameterFirstParametersPositionControl_actual.resize(youBotArmDoF);
            DParameterFirstParametersPositionControl_actual.resize(youBotArmDoF);
            IClippingParameterFirstParametersPositionControl_actual.resize(youBotArmDoF);

            PParameterFirstParametersSpeedControl_actual.resize(youBotArmDoF);
            IParameterFirstParametersSpeedControl_actual.resize(youBotArmDoF);
            DParameterFirstParametersSpeedControl_actual.resize(youBotArmDoF);
            IClippingParameterFirstParametersSpeedControl_actual.resize(youBotArmDoF);
      
      
            ros::param::get("P_Sec_Pos_joint_1",PParameterSecondParametersPositionControl_actual[0]);
            ros::param::get("P_Sec_Pos_joint_2",PParameterSecondParametersPositionControl_actual[1]);
            ros::param::get("P_Sec_Pos_joint_3",PParameterSecondParametersPositionControl_actual[2]);
            ros::param::get("P_Sec_Pos_joint_4",PParameterSecondParametersPositionControl_actual[3]);
            ros::param::get("P_Sec_Pos_joint_5",PParameterSecondParametersPositionControl_actual[4]);
          //I-pos_sec
            ros::param::get("I_Sec_Pos_joint_1",IParameterSecondParametersPositionControl_actual[0]);
            ros::param::get("I_Sec_Pos_joint_2",IParameterSecondParametersPositionControl_actual[1]);
            ros::param::get("I_Sec_Pos_joint_3",IParameterSecondParametersPositionControl_actual[2]);
            ros::param::get("I_Sec_Pos_joint_4",IParameterSecondParametersPositionControl_actual[3]);
            ros::param::get("I_Sec_Pos_joint_5",IParameterSecondParametersPositionControl_actual[4]);
         //D-pos_sec
            ros::param::get("D_Sec_Pos_joint_1",DParameterSecondParametersPositionControl_actual[0]);
            ros::param::get("D_Sec_Pos_joint_2",DParameterSecondParametersPositionControl_actual[1]);
            ros::param::get("D_Sec_Pos_joint_3",DParameterSecondParametersPositionControl_actual[2]);
            ros::param::get("D_Sec_Pos_joint_4",DParameterSecondParametersPositionControl_actual[3]);
            ros::param::get("D_Sec_Pos_joint_5",DParameterSecondParametersPositionControl_actual[4]);
        // ICli_Pos_sec

            ros::param::get("IClip_Sec_Pos_joint_1",IClippingParameterSecondParametersPositionControl_actual[0]);
            ros::param::get("IClip_Sec_Pos_joint_2",IClippingParameterSecondParametersPositionControl_actual[1]);
            ros::param::get("IClip_Sec_Pos_joint_3",IClippingParameterSecondParametersPositionControl_actual[2]);
            ros::param::get("IClip_Sec_Pos_joint_4",IClippingParameterSecondParametersPositionControl_actual[3]);
            ros::param::get("IClip_Sec_Pos_joint_5",IClippingParameterSecondParametersPositionControl_actual[4]);

        //IClip_speed_sec
            ros::param::get("IClip_Sec_Speed_joint_1",IClippingParameterSecondParametersSpeedControl_actual[0]);
            ros::param::get("IClip_Sec_Speed_joint_2",IClippingParameterSecondParametersSpeedControl_actual[1]);
            ros::param::get("IClip_Sec_Speed_joint_3",IClippingParameterSecondParametersSpeedControl_actual[2]);
            ros::param::get("IClip_Sec_Speed_joint_4",IClippingParameterSecondParametersSpeedControl_actual[3]);
            ros::param::get("IClip_Sec_Speed_joint_5",IClippingParameterSecondParametersSpeedControl_actual[4]);
         //P_speed_sec
            ros::param::get("P_Sec_Speed_joint_1",PParameterSecondParametersSpeedControl_actual[0]);
            ros::param::get("P_Sec_Speed_joint_2",PParameterSecondParametersSpeedControl_actual[1]);
            ros::param::get("P_Sec_Speed_joint_3",PParameterSecondParametersSpeedControl_actual[2]);
            ros::param::get("P_Sec_Speed_joint_4",PParameterSecondParametersSpeedControl_actual[3]);
            ros::param::get("P_Sec_Speed_joint_5",PParameterSecondParametersSpeedControl_actual[4]);
        //I_speed_sec
            ros::param::get("I_Sec_Speed_joint_1",IParameterSecondParametersSpeedControl_actual[0]);
            ros::param::get("I_Sec_Speed_joint_2",IParameterSecondParametersSpeedControl_actual[1]);
            ros::param::get("I_Sec_Speed_joint_3",IParameterSecondParametersSpeedControl_actual[2]);
            ros::param::get("I_Sec_Speed_joint_4",IParameterSecondParametersSpeedControl_actual[3]);
            ros::param::get("I_Sec_Speed_joint_5",IParameterSecondParametersSpeedControl_actual[4]);
         //D_speed_sec
            ros::param::get("D_Sec_Speed_joint_1",DParameterSecondParametersSpeedControl_actual[0]);
            ros::param::get("D_Sec_Speed_joint_2",DParameterSecondParametersSpeedControl_actual[1]);
            ros::param::get("D_Sec_Speed_joint_3",DParameterSecondParametersSpeedControl_actual[2]);
            ros::param::get("D_Sec_Speed_joint_4",DParameterSecondParametersSpeedControl_actual[3]);
            ros::param::get("D_Sec_Speed_joint_5",DParameterSecondParametersSpeedControl_actual[4]);
         // PID Current
            ros::param::get("P_Current_joint_1",PParameterCurrentControl_actual[0]);
            ros::param::get("P_Current_joint_2",PParameterCurrentControl_actual[1]);
            ros::param::get("P_Current_joint_3",PParameterCurrentControl_actual[2]);
            ros::param::get("P_Current_joint_4",PParameterCurrentControl_actual[3]);
            ros::param::get("P_Current_joint_5",PParameterCurrentControl_actual[4]);

            ros::param::get("I_Current_joint_1",IParameterCurrentControl_actual[0]);
            ros::param::get("I_Current_joint_2",IParameterCurrentControl_actual[1]);
            ros::param::get("I_Current_joint_3",IParameterCurrentControl_actual[2]);
            ros::param::get("I_Current_joint_4",IParameterCurrentControl_actual[3]);
            ros::param::get("I_Current_joint_5",IParameterCurrentControl_actual[4]);

             ros::param::get("D_Current_joint_1",DParameterCurrentControl_actual[0]);
             ros::param::get("D_Current_joint_2",DParameterCurrentControl_actual[1]);
             ros::param::get("D_Current_joint_3",DParameterCurrentControl_actual[2]);
             ros::param::get("D_Current_joint_4",DParameterCurrentControl_actual[3]);
             ros::param::get("D_Current_joint_5",DParameterCurrentControl_actual[4]);

            ros::param::get("IClip_Current_joint_1",IClippingParameterCurrentControl_actual[0]);
            ros::param::get("IClip_Current_joint_2",IClippingParameterCurrentControl_actual[1]);
            ros::param::get("IClip_Current_joint_3",IClippingParameterCurrentControl_actual[2]);
            ros::param::get("IClip_Current_joint_4",IClippingParameterCurrentControl_actual[3]);
            ros::param::get("IClip_Current_joint_5",IClippingParameterCurrentControl_actual[4]);

       //PID_First
          // P_pos
            ros::param::get("P_First_Pos_joint_1",PParameterFirstParametersPositionControl_actual[0]);
            ros::param::get("P_First_Pos_joint_2",PParameterFirstParametersPositionControl_actual[1]);
            ros::param::get("P_First_Pos_joint_3",PParameterFirstParametersPositionControl_actual[2]);
            ros::param::get("P_First_Pos_joint_4",PParameterFirstParametersPositionControl_actual[3]);
            ros::param::get("P_First_Pos_joint_5",PParameterFirstParametersPositionControl_actual[4]);
         // P_Pos
            ros::param::get("I_First_Pos_joint_1",IParameterFirstParametersPositionControl_actual[0]);
            ros::param::get("I_First_Pos_joint_2",IParameterFirstParametersPositionControl_actual[1]);
            ros::param::get("I_First_Pos_joint_3",IParameterFirstParametersPositionControl_actual[2]);
            ros::param::get("I_First_Pos_joint_4",IParameterFirstParametersPositionControl_actual[3]);
            ros::param::get("I_First_Pos_joint_5",IParameterFirstParametersPositionControl_actual[4]);
        //P_pos
            ros::param::get("D_First_Pos_joint_1",DParameterFirstParametersPositionControl_actual[0]);
            ros::param::get("D_First_Pos_joint_2",DParameterFirstParametersPositionControl_actual[1]);
            ros::param::get("D_First_Pos_joint_3",DParameterFirstParametersPositionControl_actual[2]);
            ros::param::get("D_First_Pos_joint_4",DParameterFirstParametersPositionControl_actual[3]);
            ros::param::get("D_First_Pos_joint_5",DParameterFirstParametersPositionControl_actual[4]);
        // Iclip  pos
            ros::param::get("IClip_First_Pos_joint_1",IClippingParameterFirstParametersPositionControl_actual[0]);
            ros::param::get("IClip_First_Pos_joint_2",IClippingParameterFirstParametersPositionControl_actual[1]);
            ros::param::get("IClip_First_Pos_joint_3",IClippingParameterFirstParametersPositionControl_actual[2]);
            ros::param::get("IClip_First_Pos_joint_4",IClippingParameterFirstParametersPositionControl_actual[3]);
            ros::param::get("IClip_First_Pos_joint_5",IClippingParameterFirstParametersPositionControl_actual[4]);



        // P_pos
            ros::param::get("P_First_Speed_joint_1",PParameterFirstParametersSpeedControl_actual[0]);
            ros::param::get("P_First_Speed_joint_2",PParameterFirstParametersSpeedControl_actual[1]);
            ros::param::get("P_First_Speed_joint_3",PParameterFirstParametersSpeedControl_actual[2]);
            ros::param::get("P_First_Speed_joint_4",PParameterFirstParametersSpeedControl_actual[3]);
            ros::param::get("P_First_Speed_joint_5",PParameterFirstParametersSpeedControl_actual[4]);
         // P_Pos
            ros::param::get("I_First_Speed_joint_1",IParameterFirstParametersSpeedControl_actual[0]);
            ros::param::get("I_First_Speed_joint_2",IParameterFirstParametersSpeedControl_actual[1]);
            ros::param::get("I_First_Speed_joint_3",IParameterFirstParametersSpeedControl_actual[2]);
            ros::param::get("I_First_Speed_joint_4",IParameterFirstParametersSpeedControl_actual[3]);
            ros::param::get("I_First_Speed_joint_5",IParameterFirstParametersSpeedControl_actual[4]);
       //P_pos
            ros::param::get("D_First_Speed_joint_1",DParameterFirstParametersSpeedControl_actual[0]);
            ros::param::get("D_First_Speed_joint_2",DParameterFirstParametersSpeedControl_actual[1]);
            ros::param::get("D_First_Speed_joint_3",DParameterFirstParametersSpeedControl_actual[2]);
            ros::param::get("D_First_Speed_joint_4",DParameterFirstParametersSpeedControl_actual[3]);
            ros::param::get("D_First_Speed_joint_5",DParameterFirstParametersSpeedControl_actual[4]);
       // Iclip  pos
            ros::param::get("IClip_First_Speed_joint_1",IClippingParameterFirstParametersSpeedControl_actual[0]);
            ros::param::get("IClip_First_Speed_joint_2",IClippingParameterFirstParametersSpeedControl_actual[1]);
            ros::param::get("IClip_First_Speed_joint_3",IClippingParameterFirstParametersSpeedControl_actual[2]);
            ros::param::get("IClip_First_Speed_joint_4",IClippingParameterFirstParametersSpeedControl_actual[3]);
            ros::param::get("IClip_First_Speed_joint_5",IClippingParameterFirstParametersSpeedControl_actual[4]); 

            PParameterSecondParametersPositionControl_Parameter.resize(youBotArmDoF);
            IParameterSecondParametersPositionControl_Parameter.resize(youBotArmDoF);
            DParameterSecondParametersPositionControl_Parameter.resize(youBotArmDoF);
            IClippingParameterSecondParametersPositionControl_Parameter.resize(youBotArmDoF);

            PParameterSecondParametersSpeedControl_Parameter.resize(youBotArmDoF);
            IParameterSecondParametersSpeedControl_Parameter.resize(youBotArmDoF);
            DParameterSecondParametersSpeedControl_Parameter.resize(youBotArmDoF);
            IClippingParameterSecondParametersSpeedControl_Parameter.resize(youBotArmDoF);

            PParameterFirstParametersPositionControl_Parameter.resize(youBotArmDoF);
            IParameterFirstParametersPositionControl_Parameter.resize(youBotArmDoF);
            DParameterFirstParametersPositionControl_Parameter.resize(youBotArmDoF);
            IClippingParameterFirstParametersPositionControl_Parameter.resize(youBotArmDoF);

            PParameterFirstParametersSpeedControl_Parameter.resize(youBotArmDoF);
            IParameterFirstParametersSpeedControl_Parameter.resize(youBotArmDoF);
            DParameterFirstParametersSpeedControl_Parameter.resize(youBotArmDoF);
            IClippingParameterFirstParametersSpeedControl_Parameter.resize(youBotArmDoF);


            PParameterCurrentControl_Parameter.resize(youBotArmDoF);
            IParameterCurrentControl_Parameter.resize(youBotArmDoF);
            DParameterCurrentControl_Parameter.resize(youBotArmDoF);
            IClippingParameterCurrentControl_Parameter.resize(youBotArmDoF);

            for (int i = 0; i< youBotArmDoF; ++i)
            {
          // PID-pos_sec
                  PParameterSecondParametersPositionControl_Parameter[i].setParameter(PParameterSecondParametersPositionControl_actual[i]); 
                  IParameterSecondParametersPositionControl_Parameter[i].setParameter(IParameterSecondParametersPositionControl_actual[i]); 
                  DParameterSecondParametersPositionControl_Parameter[i].setParameter(DParameterSecondParametersPositionControl_actual[i]); 
                  IClippingParameterSecondParametersPositionControl_Parameter[i].setParameter(IClippingParameterSecondParametersPositionControl_actual[i]);
          //PID-speed_sec
                  PParameterSecondParametersSpeedControl_Parameter[i].setParameter(PParameterSecondParametersSpeedControl_actual[i]); 
                  IParameterSecondParametersSpeedControl_Parameter[i].setParameter(IParameterSecondParametersSpeedControl_actual[i]); 
                  DParameterSecondParametersSpeedControl_Parameter[i].setParameter(DParameterSecondParametersSpeedControl_actual[i]); 
                  IClippingParameterSecondParametersSpeedControl_Parameter[i].setParameter(IClippingParameterSecondParametersSpeedControl_actual[i]);
        
           // PID  Current
                  PParameterCurrentControl_Parameter[i].setParameter(PParameterCurrentControl_actual[i]);
                  IParameterCurrentControl_Parameter[i].setParameter(IParameterCurrentControl_actual[i]);
                  DParameterCurrentControl_Parameter[i].setParameter(DParameterCurrentControl_actual[i]); 
                  IClippingParameterCurrentControl_Parameter[i].setParameter(IClippingParameterCurrentControl_actual[i]);
       

          //pID_first _Speed      
                   PParameterFirstParametersPositionControl_Parameter[i].setParameter(PParameterFirstParametersPositionControl_actual[i]);
                   IParameterFirstParametersPositionControl_Parameter[i].setParameter(IParameterFirstParametersPositionControl_actual[i]);
                   DParameterFirstParametersPositionControl_Parameter[i].setParameter(DParameterFirstParametersPositionControl_actual[i]);
                   IClippingParameterFirstParametersPositionControl_Parameter[i].setParameter(IClippingParameterFirstParametersPositionControl_actual[i]);
       
         // speed_PID_Speed

                   PParameterFirstParametersSpeedControl_Parameter[i].setParameter(PParameterFirstParametersSpeedControl_actual[i]);
                   IParameterFirstParametersSpeedControl_Parameter[i].setParameter(IParameterFirstParametersSpeedControl_actual[i]);
                   DParameterFirstParametersSpeedControl_Parameter[i].setParameter(DParameterFirstParametersSpeedControl_actual[i]);
                   IClippingParameterFirstParametersSpeedControl_Parameter[i].setParameter(IClippingParameterFirstParametersSpeedControl_actual[i]);
            }



           try{
                 youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time


                        for (int i = 0; i < youBotArmDoF; ++i) {
                       //PID_pos_sec
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(PParameterSecondParametersPositionControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IParameterSecondParametersPositionControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(DParameterSecondParametersPositionControl_Parameter[i]);
			 youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IClippingParameterSecondParametersPositionControl_Parameter[i]);
			
                        //PID_speed_sec
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(PParameterSecondParametersSpeedControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IParameterSecondParametersSpeedControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(DParameterSecondParametersSpeedControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IClippingParameterSecondParametersSpeedControl_Parameter[i]);
			
                      // PID Current
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(PParameterCurrentControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IParameterCurrentControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(DParameterCurrentControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IClippingParameterCurrentControl_Parameter[i]);	

                      // PID First  pos
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(PParameterFirstParametersPositionControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IParameterFirstParametersPositionControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(DParameterFirstParametersPositionControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IClippingParameterFirstParametersPositionControl_Parameter[i]);
                   
                      // PID First  speed
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(PParameterFirstParametersSpeedControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IParameterFirstParametersSpeedControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(DParameterFirstParametersSpeedControl_Parameter[i]);
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i+1).setConfigurationParameter(IClippingParameterFirstParametersSpeedControl_Parameter[i]);
			
                     }



                youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot  configuration the arm : %s", errorMessage.c_str());
			return false;
		}


	} else {
		ROS_ERROR("Arm%i not initialized!", armIndex+1);
		return false;
       

     } 
	return true; 

}


bool YouBotOODLWrapper::switchOffArmMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex) {
	ROS_INFO("Switch off the arm%i motors", armIndex+1);
	ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));

	if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) { // in case stop has been invoked

        youbot::JointCurrentSetpoint currentStopMovement;
        currentStopMovement.current = 0.0 * ampere;
		try{
      youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(1).setData(currentStopMovement);
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(2).setData(currentStopMovement);
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(3).setData(currentStopMovement);
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(4).setData(currentStopMovement);
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(5).setData(currentStopMovement);
      youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot switch off the arm motors: %s", errorMessage.c_str());
			return false;
		}
	} else {
		ROS_ERROR("Arm%i not initialized!", armIndex+1);
		return false;
	}
  	areArmMotorsSwitchedOn = false;
	return true;
}

bool YouBotOODLWrapper::switchOnArmMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex)
{
    ROS_INFO("Switch on the arm%i motors", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0)
    {   
      try
        {
            std::vector<youbot::JointSensedAngle> sensedJointAngleVector;
            std::vector<youbot::JointAngleSetpoint> desiredJointAngleVector;
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getJointData(sensedJointAngleVector);
            youbot::JointAngleSetpoint desiredJointAngle;
            for(unsigned int i = 0; i < sensedJointAngleVector.size(); i++){
              desiredJointAngle = sensedJointAngleVector[i].angle;
              desiredJointAngleVector.push_back(desiredJointAngle);
            }        
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->setJointData(desiredJointAngleVector);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot switch on the arm motors: %s", errorMessage.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Arm%i not initialized!", armIndex + 1);
        return false;
    }
    areArmMotorsSwitchedOn = true;
    return true;
}

bool YouBotOODLWrapper::calibrateArmCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex)
{
    ROS_INFO("Calibrate the arm%i", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0)
    {

        try
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->calibrateManipulator(true);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot calibrate the arm: %s", errorMessage.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Arm%i not initialized!", armIndex + 1);
        return false;
    }
    return true;
}

bool YouBotOODLWrapper::reconnectCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

	this->stop();

	/* configuration */
	bool youBotHasBase;
	bool youBotHasArms;
	node.param("youBotHasBase", youBotHasBase, false);
	node.param("youBotHasArms", youBotHasArms, false);
	std::vector<std::string> armNames;

	// Retrieve all defined arm names from the launch file params
	int i = 1;
	std::stringstream armNameParam;
	armNameParam << "youBotArmName" << i; // youBotArmName1 is first checked param... then youBotArmName2, etc.
	while (node.hasParam(armNameParam.str())) {
		std::string armName;
		node.getParam(armNameParam.str(), armName);
		armNames.push_back(armName);
		armNameParam.str("");
		armNameParam << "youBotArmName" <<  (++i);
	}

	ROS_ASSERT((youBotHasBase == true) || (youBotHasArms == true)); // At least one should be true, otherwise nothing to be started.
	if (youBotHasBase == true)
	{
		this->initializeBase(this->youBotConfiguration.baseConfiguration.baseID);
	}

	if (youBotHasArms == true)
	{
		std::vector<std::string>::iterator armNameIter;
		for (armNameIter = armNames.begin(); armNameIter != armNames.end(); ++armNameIter) {
			this->initializeArm(*armNameIter);
		}
	}
	return true;
}

void YouBotOODLWrapper::publishArmAndBaseDiagnostics(double publish_rate_in_secs) {
    // only publish every X seconds
    if ((ros::Time::now() - lastDiagnosticPublishTime).toSec() < publish_rate_in_secs)
      return;

    lastDiagnosticPublishTime = ros::Time::now();

    platformStateMessage.header.stamp = ros::Time::now();
    diagnosticArrayMessage.header.stamp = ros::Time::now();
    diagnosticArrayMessage.status.clear();

    // diagnostics message
    // base status
    diagnosticStatusMessage.name = diagnosticNameBase;
    if (youBotConfiguration.hasBase) {
      diagnosticStatusMessage.message = "base is present";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::OK;
    } else {
      diagnosticStatusMessage.message = "base is not connected or switched off";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);

    // arm status
    diagnosticStatusMessage.name = diagnosticNameArm;
    if (youBotConfiguration.hasArms) {
      diagnosticStatusMessage.message = "arm is present";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::OK;
    } else {
      diagnosticStatusMessage.message = "arm is not connected or switched off";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);


    // EtherCAT status
    diagnosticStatusMessage.name = "platform_EtherCAT";
    if (youbot::EthercatMaster::getInstance().isEtherCATConnectionEstablished()) {
      diagnosticStatusMessage.message = "EtherCAT connnection is established";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::OK;
      platformStateMessage.run_stop = false;
    }
    else {
      diagnosticStatusMessage.message = "EtherCAT connnection lost";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      platformStateMessage.run_stop = true;
    }
    diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);        


    // dashboard message
    if (youBotConfiguration.hasBase && areBaseMotorsSwitchedOn)
      platformStateMessage.circuit_state[0] = pr2_msgs::PowerBoardState::STATE_ENABLED;
    else if (youBotConfiguration.hasBase && !areBaseMotorsSwitchedOn)
      platformStateMessage.circuit_state[0] = pr2_msgs::PowerBoardState::STATE_STANDBY;
    else
      platformStateMessage.circuit_state[0] = pr2_msgs::PowerBoardState::STATE_DISABLED;

    if (youBotConfiguration.hasArms && areArmMotorsSwitchedOn)
      platformStateMessage.circuit_state[1] = pr2_msgs::PowerBoardState::STATE_ENABLED;
    else if (youBotConfiguration.hasArms && !areArmMotorsSwitchedOn)
      platformStateMessage.circuit_state[1] = pr2_msgs::PowerBoardState::STATE_STANDBY;
    else
      platformStateMessage.circuit_state[1] = pr2_msgs::PowerBoardState::STATE_DISABLED;


    // publish established messages
    dashboardMessagePublisher.publish(platformStateMessage);
    diagnosticArrayPublisher.publish(diagnosticArrayMessage);
  }

} // namespace youBot

/* EOF */
