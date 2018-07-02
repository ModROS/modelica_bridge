/**
 * @brief Example controller for use with modbridge_node
 * @file modbridge_ex_controller.cpp
 * 
 * Receives teleop input from modbridge_ex_teleop, and feedback from model via modbridge_node.
 * Provides basic controller output to move feedback values to the input values.
 * @author Shashank Swaminathan <sh.swami235@gmail.com>
 */
/*
 * Copyright 2018 Shashank Swaminathan.
 *
 * This file is part of the modelica_bridge_example package and subject to the license terms
 * in the top-level LICENSE file of the modelica_bridge repository.
 * https://github.com/ModROS/modelica_bridge/blob/master/LICENSE
 */
#include "ros/ros.h"
#include "modelica_bridge/ModComm.h"

/** @def MAX_ARRAY
 *  @brief Defines the maximum size of internal data arrays
 */
#define MAX_ARRAY 100

ros::Publisher control_pub; ///< publisher for controller values

double springSetPoints[MAX_ARRAY] = {0.0}; ///< array to store control values

/**
 * Receives feedback from the model; calculates and publishes control values.
 * Reads from topic /model_values. Publishes to topic /control_values
 * Only reads MAX_ARRAY length of incoming data - rest is not stored.
 * @param [in] feedback_val ModComm message holding model feedback values
 * @see springSetPoints()
 * @see control_pub()
 * @see joyCallback()
 * @return none
 */
void controlCallback(const modelica_bridge::ModComm::ConstPtr& feedback_val) {

    modelica_bridge::ModComm control_val;
    for(int i = 0; i < feedback_val->size; i++) {
        control_val.data.push_back(5 * (springSetPoints[i] - feedback_val->data[i]));
    }
    control_val.size = control_val.data.size();

    control_pub.publish(control_val);
}

/**
 * Reads input from teleop node, stores as controller's set points
 * Reads on topic /modbridge_joy, of modelica_bridge_example's example teleop node
 * @param [in] inJoy
 * @see springSetPoints()
 * @see controlCallback()
 * @return none
 */
void joyCallback(const modelica_bridge::ModComm::ConstPtr& inJoy) {
    for(int i = 0; i<inJoy->size; i++) {
        springSetPoints[i] = inJoy->data[i];
    }
}

/**
 * Initializes node, pub, and subs. Spin to keep the node running
 * @see control_pub()
 * @see controlCallback()
 * @see joyCallback()
 * @return 0
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "modbridge_examples_controller");
    ros::NodeHandle n;
    
    control_pub = n.advertise<modelica_bridge::ModComm>("control_values", 1);
    ros::Subscriber feedback_sub = n.subscribe("model_values", 1, controlCallback);
    ros::Subscriber joy_sub = n.subscribe("modbridge_joy", 1, joyCallback);

    ros::spin();

    return 0;
}