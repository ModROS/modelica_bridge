#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from pyfmi import load_fmu
# from std_msgs.msg import UInt8
# from geometry_msgs.msg import Pose2D

"""
This ROS node subscribes to the output node of the Airmar sensor, on /sensors/airmar/wind/rel and 
/sensors/airmar/wind/true, and stores on a rolling basis the most recent 10 values into a deque (a 
rolling queue). An average function is performed on that deque, and the resulting values are published
to /weather/wind/rel and /weather/wind/true
"""

class FmuSim:
    def __init__(self):
        
        # Initialize the ROS node
        rospy.init_node("fmu_simulator", anonymous=True)
        rospy.loginfo("Starting FMU simulator node ...")
        rospy.loginfo("--------------")
        
        # Initialize feedback publisher
        self.pub_feedback = rospy.Publisher('/feedback_values', Pose2D, queue_size=10)
        
        # Create deques to store the last 10 values
        self.mmodel = load_fmu('/home/sswaminathan/Desktop/' + 
                            'FMU_Bridge/FMU_Bridge_Test/' +
                            './FMU_Bridge_Test_TestV1.fmu')

        self.Tstart = 0.0
        self.Tend = 10.0

        self.mmodel.setup_experiment(start_time = self.Tstart)
        self.mmodel.enter_initialization_mode()
        self.mmodel.exit_initialization_mode()

        self.eInfo = self.mmodel.get_event_info()
        self.eInfo.newDiscreteStatesNeeded = True

        self.mmodel.enter_continuous_time_mode()

        self.x = self.mmodel.continuous_states

        self.x_nominal = self.mmodel.nominal_continuous_states

        self.event_ind = self.mmodel.get_event_indicators()

        self.vref  = [self.mmodel.get_variable_valueref('positionSensor.s')] + \
                [self.mmodel.get_variable_valueref('positionSensor1.s')]

        self.in_ref = [self.mmodel.get_variable_valueref('u[1]')] + [self.mmodel.get_variable_valueref('u[2]')]

        self.t_sol = [self.Tstart]

        self.sol = [self.mmodel.get_real(self.vref)]

        self.time = self.Tstart
        self.Tnext = self.Tend
        self.dt = 0.01
                
    def run(self):
        """
        This function loops until the node is shut down, reading messages sent from the Airmar
        and publishing their contents to the appropriate ROS topics.
        """

        """
        while not rospy.is_shutdown():
            # Subscribe to airmar node's topics
            rospy.Subscriber("/sensors/airmar/wind/rel", Pose2D, self.received_rel_wind_msg)
            rospy.Subscriber("/sensors/airmar/wind/true", Pose2D, self.received_true_filter_msg)
            # Subscribe to the filter's window size controller topic
            rospy.Subscriber("/control/wind_filter/window_size", UInt8, self.change_window_size)
            
            # Run callbacks
            rospy.spin()
        """

        while not rospy.is_shutdown() and not self.mmodel.get_event_info().terminateSimulation:
            dx = self.mmodel.get_derivatives()

            h = min(self.dt, self.Tnext-self.time)
            self.time = self.time + h

            self.mmodel.time = self.time

            self.mmodel.set_real(self.in_ref, self.input_function(self.mmodel.time))

            self.x = self.x + h*dx 
            self.mmodel.continuous_states = self.x

            event_ind_new = self.mmodel.get_event_indicators()

            step_event = self.mmodel.completed_integrator_step()

            time_event = abs(self.time-self.Tnext) <= 1.e-10
            state_event = True if True in ((event_ind_new>0.0) != (self.event_ind>0.0)) else False

            if step_event or time_event or state_event:
                self.mmodel.enter_event_mode()
                self.eInfo = self.mmodel.get_event_info()
                self.eInfo.newDiscreteStatesNeeded = True

                while self.eInfo.newDiscreteStatesNeeded:
                    self.mmodel.event_update('0')
                    self.eInfo = self.mmodel.get_event_info()
                    if self.eInfo.newDiscreteStatesNeeded:
                        pass
                if self.eInfo.valuesOfContinuousStatesChanged:
                    self.x = self.mmodel.continuous_states

                if self.eInfo.nominalsOfContinuousStatesChanged:
                    atol = 0.01*rtol*self.mmodel.nominal_continuous_states

                if self.eInfo.nextEventTimeDefined:
                    self.Tnext = min(self.eInfo.nextEventTime, self.Tend)
                else:
                    self.Tnext = self.Tend
                self.mmodel.enter_continuous_time_mode()
            self.event_ind = event_ind_new

            self.t_sol += [self.time]
            self.sol += [self.mmodel.get_real(self.vref)]

    def input_function(self, t):
        return np.array([np.sin(t), np.cos(t)]).tolist()
    
    def plot_function(self):
        plt.figure(1)
        plt.plot(self.t_sol, np.array(self.sol)[:,0])
        plt.title(self.mmodel.get_name())
        plt.ylabel('Height (m)')
        plt.xlabel('Time (s)')

        plt.figure(2)
        plt.plot(self.t_sol,np.array(self.sol)[:,1])
        plt.title(self.mmodel.get_name())
        plt.ylabel('Velocity (m/s)')
        plt.xlabel('Time (s)')
        plt.show()

#     def received_rel_wind_msg(self, rel_wind):
#         """
#             Takes Pose2D message of raw relative wind speed and heading data, and uses a rolling average filter.
#             Publishes filtered relative wind and heading data as a Pose2D message to ROS.
#         """
#         # Append new value to deque
#         self.wind_rel_deque['x'].append(rel_wind.x)
#         self.wind_rel_deque['theta'].append(rel_wind.theta)
# 
#         # Check number of stored values in window
#         len_check = self.wind_rel_deque['x'].maxlen
# 
#         # Only run filter if the deque has already stored 10 values (passed initialization period)
#         if (len(self.wind_rel_deque['x']) == len_check and len(self.wind_rel_deque['theta']) == len_check):
#             filtered_rel_wind = Pose2D() # Create Pose2D message
#     
#             # Take means of the deque by converting to a numpy array
#             # Stores mean values into outgoing message
#             filtered_rel_wind.x = np.array(self.wind_rel_deque['x']).mean()
#             filtered_rel_wind.theta = np.array(self.wind_rel_deque['theta']).mean()
#     
#             # Publish averaged values of wind data
#             logmsg = 'Relative Wind Speed\nx      ' + str(filtered_rel_wind.x) + 'Theta  ' + str(filtered_rel_wind.theta)
#             rospy.loginfo('\n' + logmsg + '\n')
#             self.pub_rel.publish(filtered_rel_wind)

if __name__ == '__main__':
    crash_count = 0
    while crash_count < 5:
        try:
            core = FmuSim()
            core.run()
            core.plot_function()
        # except rospy.ROSInterruptException:
        except:
            crash_count += 1
            print('FMU simulator node was interrupted.')
    print('FMU simulator node crashed.')
