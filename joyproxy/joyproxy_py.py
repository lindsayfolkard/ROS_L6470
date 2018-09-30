# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
#from l6470_srvs import GoToPosition
#from l6470_srvs import StopAll
#from l6470_srvs import GoSpeed
from l6470_msgs.msg import ManualSpeed
#from  import 

class JoyProxy(Node):
    """
    A node with a single subscriber.
    This class creates a node which prints messages it receives on a topic. Creating a node by
    inheriting from Node is recommended because it allows it to be imported and used by
    other scripts.
    """

    def __init__(self):
        # Calls Node.__init__('listener')
        # Print hello
        print("Initialising...")
        super().__init__('joyproxy')
        self.sub                              = self.create_subscription(String, 'chatter', self.chatter_callback)
        self.subJoy                       = self.create_subscription(Joy, 'joy'    , self.joystick_callback)
        self.speedPublisher      = self.create_publisher(ManualSpeed, 'manual_speed')
        #timer_period = 0.05  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.speedClient   = self.create_client(GoSpeed,'go_speed')
        #self.stopAllClient = self.create_client(StopAll,'stop_all')
        #while not self.speedClient.wait_for_service(timeout_sec=1.0):
        #        self.get_logger().info('service not available, waiting again...')
        print("Initialised.")
     
    #def timer_callback(self):
        #msg = NodeMsg;
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1

    def chatter_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def joystick_callback(self,msg):
        #self.get_logger().info('I heard joystick: "%s"' % msg.header)
        #self.get_logger().info('Axes 0 : %f' % msg.axes[0])
        #self.print_joy_state(msg)
        
        # Are we stopping ?
        #req = GoSpeed.Request()
        
        # Determine our joystick mapping
        # Axes 1 = J1 = Vertical          = big nema23 = motor 0
        # Axes 4 = J2 = Horizontal 1 = small nema23 = motor1
        # Axes 2 = J3 = Horizontal 2 = Nema17 = motor2
        
        # Scaling
        # Well - the joystick values are normalised in the range -1.0:1.0
        maxStepsPerSecond = 800
        
        # Generate the speed request command according to the above mapping
        manualSpeedMsg = ManualSpeed()
        manualSpeedMsg.speed.append(maxStepsPerSecond*msg.axes[1])
        manualSpeedMsg.speed.append(maxStepsPerSecond*msg.axes[4])
        manualSpeedMsg.speed.append(maxStepsPerSecond*msg.axes[2])
        
        # Accelerations ? - fuck it
        
        # Publish the speed command
        #self.speedClient.call_async(req)
        self.speedPublisher.publish(manualSpeedMsg)

       # if future.result() is not None:
       #    node.get_logger().info(
       #     'Result of add_two_ints: for %d + %d = %d' %
       #     (req.a, req.b, future.result().sum))
       # else:
       #     node.get_logger().info('Service call failed %r' % (future.exception(),))
        
    def print_joy_state(self,msg):
        print("Axes : ",end='')
        axesNum=0
        for axesValue in msg.axes:
            print('{axes} = {value}, '.format(axes=axesNum,value=axesValue),end='')
            ++axesNum
        print(" ")
        
        print("Buttons : ") 
        buttonNum=0
        for buttonValue in msg.buttons:
            print('{button} = {value}, '.format(button=buttonNum,value=buttonValue),end='')
            ++buttonNum
        print(" ")

def main(args=None):
    """
    Run a Listener node standalone.
    This function is called directly when using an entrypoint. Entrypoints are configured in
    setup.py. This along with the script installation in setup.cfg allows a listener node to be run
    with the command `ros2 run examples_rclpy_executors listener`.
    :param args: Arguments passed in from the command line.
    """
    rclpy.init(args=args)
    try:
        joyProxy = JoyProxy()
        rclpy.spin(joyProxy)
    finally:
        joyProxy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()
