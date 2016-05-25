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

#ifndef YOUBOTOODLWRAPPER_H_
#define YOUBOTOODLWRAPPER_H_

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include <diagnostic_msgs/DiagnosticArray.h>

#include <pr2_msgs/PowerBoardState.h>

#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

#include "dynamic_reconfigure/Config.h"
#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointVelocities.h"
#include "brics_actuator/JointTorques.h"
#include "youbot_driver_ros_interface/JointCurrents.h"
#include "youbot_driver_ros_interface/JointPWMs.h"


/* OODL includes */
#include "YouBotConfiguration.h"
#include <youbot_driver/youbot/JointTrajectoryController.hpp>
#include <youbot_driver/youbot/DataTrace.hpp>
#include <youbot_driver/generic/PidController.hpp>
#include <control_toolbox/pid.h>

#include "ros/single_subscriber_publisher.h"
#include "ros/subscriber_link.h"

//#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <actionlib/server/simple_action_server.h>

//typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

namespace youBot
{

/**
 * @brief Wrapper class to map ROS messages to OODL method calls for the youBot platform.
 */
class YouBotOODLWrapper
{
public:

    /**
     * @brief Constructor with a ROS handle.
     * @param n ROS handle
     */
    YouBotOODLWrapper(ros::NodeHandle n);

    /**
     * @brief DEfault constructor.
     */
    virtual ~YouBotOODLWrapper();


    /* Coordination: */

    /**
     * @brief Initializes a youBot base.
     * @param baseName Name of the base. Used to open the configuration file e.g. youbot-base.cfg
     */
    void initializeBase(std::string baseName);

    /**
     * @brief Initializes a youBot base.
     * @param armName Name of the base. Used to open the configuration file e.g. youbot-manipulator.cfg
     * @param enableStandardGripper If set to true, then the default gripper of the youBot will be initialized.
     */
    void initializeArm(std::string armName, bool enableStandardGripper = true);

    /**
     * @brief Stops all initialized elements.
     * Stops arm and/or base (if initialized).
     */
    void stop();

    /* Communication: */

    /**
     * @brief Callback that is executed when a commend for the base comes in.
     * @param youbotBaseCommand Message that contains the desired translational and rotational velocity for the base.
     */
    void baseCommandCallback(const geometry_msgs::Twist& youbotBaseCommand);

    /**
     * @deprecated
     * @brief Callback that is executed when a commend for the arm comes in.
     * @param youbotArmCommand Message that contains the desired joint configuration.
     *
     * Currently only the first configuration (JointTrajectoryPoint) per message is processed.
     * Velocity and acceleration values are ignored.
     */
    void armCommandCallback(const trajectory_msgs::JointTrajectory& youbotArmCommand);

  
    void armPWMsCommandCallback(const brics_actuator::JointValueConstPtr& youbotArmCommand, int armIndex);

   
   void armCurrentsCommandCallback(const youbot_driver_ros_interface::JointCurrentsConstPtr& youbotArmCommand, int armIndex);



    /**
     * @brief Callback that is executed when a position command for the arm comes in.
     * @param youbotArmCommand Message that contains the desired joint configuration.
     * @param armIndex Index that identifies the arm
     */

    void armPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotArmCommand, int armIndex);

    /**
     * @brief Callback that is executed when a velocity command for the arm comes in.
     * @param youbotArmCommand Message that contains the desired joint configuration.
     * @param armIndex Index that identifies the arm
     */
    void armVelocitiesCommandCallback(const brics_actuator::JointVelocitiesConstPtr& youbotArmCommand, int armIndex);
    
    /**
     * @brief Callback that is executed when a velocity command for the arm comes in.
     * @param youbotArmCommand Message that contains the desired joint configuration.
     * @param armIndex Index that identifies the arm
     */
    void armTorquesCommandCallback(const brics_actuator::JointTorquesConstPtr& youbotArmCommand, int armIndex);
    
    /**
	 * @brief Callback that is executed when an action goal to perform a joint trajectory with the arm comes in.
	 * @param youbotArmGoal Actionlib goal that contains the trajectory.
	 * @param armIndex Index that identifies the arm
	 */
        void armJointTrajectoryGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex);

	/**
	 * @brief Callback that is executed when an action goal of a joint trajectory is canceled.
	 * @param youbotArmGoal Actionlib goal that contains the trajectory.
	 * @param armIndex Index that identifies the arm
	 */
	void armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex);
    
    /**
     * @brief Callback that is executed when a position command for the gripper comes in.
     * @param youbotGripperCommand Message that contains the desired joint configuration.
     * @param armIndex Index that identifies the arm
     */
    void gripperPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotGripperCommand, int armIndex);

    /**
     * @brief Publishes all sensor measurements. Both for base and arm.
     *
     * Depending on what has been initialized before, either odometry and/or joint state valiues are published.
     * computeOODLSensorReadings needs to be executed before.

     */

    void connectCallback(const ros::SingleSubscriberPublisher& pub); 
    void updateParameter();
    void publishOODLSensorReadings();
    void publishPID();
    
    /**
    * @brief Publishes status of base and arm as diagnostic and dashboard messages continuously
    */
    void publishArmAndBaseDiagnostics(double publish_rate_in_secs);

    /* Computation: */

    /**
     * @brief Mapps OODL values to ROS messages
     */
    void computeOODLSensorReadings();
    
    

    bool ParametersControlCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex);

    bool ParametersReadControlCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex);

    bool switchOffBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool switchOnBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool switchOffArmMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex);

    bool switchOnArmMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex);

    bool calibrateArmCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex);

    bool reconnectCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

 

    /* Configuration: */

    /// Handle the aggregates all parts of a youBot system
    YouBotConfiguration youBotConfiguration;
    // jointTrajectoryAction;
   // JointTrajectoryAction trajectoryaction;

private:

    YouBotOODLWrapper(); //forbid default constructor
    

    /// Degrees of freedom for the youBot manipulator
    static const int youBotArmDoF = 5;

    /// Number of finger mounted on the gripper.
    static const int youBotNumberOfFingers = 2;

    /// Number of wheels attached to the base.
    static const int youBotNumberOfWheels = 4;


    std::string youBotChildFrameID;
    std::string youBotOdometryFrameID;
    std::string youBotOdometryChildFrameID;
    std::string youBotArmFrameID;

    int numSubscribersConnected;

       //PID_Sec_Pos/Speed
      std::vector <youbot::PParameterSecondParametersPositionControl>  PParameterSecondParametersPositionControl_Parameter;
      std::vector <int>  PParameterSecondParametersPositionControl_actual ;
      
      std::vector<youbot::PParameterSecondParametersSpeedControl>  PParameterSecondParametersSpeedControl_Parameter;
      std::vector <int>  PParameterSecondParametersSpeedControl_actual;
      
     // I
      std::vector<youbot::IParameterSecondParametersPositionControl>  IParameterSecondParametersPositionControl_Parameter;
      std::vector <int>  IParameterSecondParametersPositionControl_actual;
      
      std::vector <youbot::IParameterSecondParametersSpeedControl>  IParameterSecondParametersSpeedControl_Parameter;
      std::vector <int> IParameterSecondParametersSpeedControl_actual;
      
     //D
      std::vector <youbot::DParameterSecondParametersPositionControl>  DParameterSecondParametersPositionControl_Parameter;
      std::vector <int> DParameterSecondParametersPositionControl_actual;
      
      std::vector <youbot::DParameterSecondParametersSpeedControl>  DParameterSecondParametersSpeedControl_Parameter;
      std::vector <int> DParameterSecondParametersSpeedControl_actual;
      
     //IClipp    youbot::IClippingParameterCurrentControl
      std::vector <youbot::IClippingParameterSecondParametersPositionControl>  IClippingParameterSecondParametersPositionControl_Parameter;
      std::vector <int>  IClippingParameterSecondParametersPositionControl_actual;
     
      std::vector <youbot::IClippingParameterSecondParametersSpeedControl>  IClippingParameterSecondParametersSpeedControl_Parameter;
      std::vector <int> IClippingParameterSecondParametersSpeedControl_actual;
      
  //PID_Current 
      //D_Current
      std::vector <youbot::DParameterCurrentControl>   DParameterCurrentControl_Parameter;
      std::vector <int> DParameterCurrentControl_actual;
      
      //P_Current     youbot::PParameterCurrentControl
      std::vector <youbot::PParameterCurrentControl>   PParameterCurrentControl_Parameter;
      std::vector <int> PParameterCurrentControl_actual;
      
      //I_Current
      std::vector <youbot::IParameterCurrentControl>   IParameterCurrentControl_Parameter;
       std::vector <int> IParameterCurrentControl_actual;
      
     //IClipp_Current      youbot::IClippingParameterCurrentControl
      std::vector <youbot::IClippingParameterCurrentControl>   IClippingParameterCurrentControl_Parameter;
       std::vector <int> IClippingParameterCurrentControl_actual;
     

   //PID_Firs_Pos/Speed
       std::vector <youbot::PParameterFirstParametersPositionControl>  PParameterFirstParametersPositionControl_Parameter;
        std::vector <int>  PParameterFirstParametersPositionControl_actual;
      
   
     std::vector < youbot::PParameterFirstParametersSpeedControl>  PParameterFirstParametersSpeedControl_Parameter;
     std::vector <int> PParameterFirstParametersSpeedControl_actual;
     
     // I
       std::vector<youbot::IParameterFirstParametersPositionControl>  IParameterFirstParametersPositionControl_Parameter;
      std::vector<int>  IParameterFirstParametersPositionControl_actual ;
       
  
      std::vector<youbot::IParameterFirstParametersSpeedControl>  IParameterFirstParametersSpeedControl_Parameter;
       std::vector<int> IParameterFirstParametersSpeedControl_actual;
      
     //D
       std::vector<youbot::DParameterFirstParametersPositionControl>  DParameterFirstParametersPositionControl_Parameter;
        std::vector<int>  DParameterFirstParametersPositionControl_actual ;
       

      std::vector<youbot::DParameterFirstParametersSpeedControl>  DParameterFirstParametersSpeedControl_Parameter;
       std::vector<int> DParameterFirstParametersSpeedControl_actual;
      
      //IClipp    youbot::IClippingParameterCurrentControl
      std::vector<youbot::IClippingParameterFirstParametersPositionControl>  IClippingParameterFirstParametersPositionControl_Parameter;
       std::vector<int>  IClippingParameterFirstParametersPositionControl_actual;
      
  
      std::vector<youbot::IClippingParameterFirstParametersSpeedControl>  IClippingParameterFirstParametersSpeedControl_Parameter;
       std::vector<int> IClippingParameterFirstParametersSpeedControl_actual;
     
    //std::vector<youbot::JointTrajectoryController> jointTrajectoryController;
    
    /// The ROS node handle
    ros::NodeHandle node;

    /// ROS timestamp
    ros::Time currentTime;


    /// The published odometry message with distances in [m], angles in [RAD] and velocities in [m/s] and [RAD/s]
    nav_msgs::Odometry odometryMessage;

    /// The published odometry tf frame with distances in [m]
    geometry_msgs::TransformStamped odometryTransform;

    /// The quaternion inside the tf odometry frame with distances in [m]
    geometry_msgs::Quaternion odometryQuaternion;

    /// The published joint state of the base (wheels) with angles in [RAD] and velocities in [RAD/s]
    sensor_msgs::JointState baseJointStateMessage;

    /// Vector of the published joint states of per arm with angles in [RAD]
    vector<sensor_msgs::JointState> armJointStateMessages;
   
    vector<dynamic_reconfigure::Config> armConfigMessages;
   // vector<sensor_msgs::JointState> armConfigMessages;
    vector<youbot_driver_ros_interface::JointCurrents> armCurrentMessages;

    vector<youbot_driver_ros_interface::JointPWMs>  armJointPWMMessages;
     

    /// The joint trajectory goal that is currently active.
	actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle armActiveJointTrajectoryGoal;

	/// Tell if a goal is currently active.
	bool armHasActiveJointTrajectoryGoal;

	youbot::GripperSensedBarPosition gripperBar1Position;
	youbot::GripperSensedBarPosition gripperBar2Position;
	int gripperCycleCounter;

    //void executeActionServer(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal,  int armIndex);
    
    //bool trajectoryActionServerEnable;
    //double trajectoryVelocityGain;
    //double trajectoryPositionGain;
    double youBotDriverCycleFrequencyInHz;
    int youBotI;
        
    /// diagnostic msgs
    ros::Time lastDiagnosticPublishTime;

    ros::Publisher dashboardMessagePublisher;
    pr2_msgs::PowerBoardState platformStateMessage;

    ros::Publisher diagnosticArrayPublisher;
    diagnostic_msgs::DiagnosticArray diagnosticArrayMessage;
    diagnostic_msgs::DiagnosticStatus diagnosticStatusMessage;
    std::string diagnosticNameArm;
    std::string diagnosticNameBase;

    bool areBaseMotorsSwitchedOn;
    bool areArmMotorsSwitchedOn;
};

} // namespace youBot

#endif /* YOUBOTOODLWRAPPER_H_ */

/* EOF */
