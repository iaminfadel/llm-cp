import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int8MultiArray
import time
from geometry_msgs.msg import Vector3

class Commander(Node):
        def __init__(self):
            super().__init__('commander')
            self.commandSub = self.create_subscription(Int8MultiArray, 'moves', self.command_callback, 10)
            self.responsePub = self.create_publisher(Float32MultiArray, 'response', 10)

            self.holdTime = 2 # seconds

            self.angle = 0
            self.distance = 0
            
            self.leftMotorPub = self.create_publisher(Float32, 'leftMotorSpeed', 10)
            self.rightMotorPub = self.create_publisher(Float32, 'rightMotorSpeed', 10)

            self.sub = self.create_subscription(Vector3, 'distance', self.callback, 10)

            self.speed = 0.833

            self.sendResponse = False

        def callback (self, msg):
            self.angle = msg.y
            self.distance = msg.x

            if self.sendResponse:
                response = Float32MultiArray()
                response.data = [self.distance, self.angle]
                self.responsePub.publish(response)
                self.sendResponse = False


        def MoveForward(self):
            self.leftMotorPub.publish(Float32(data=2*self.speed))
            self.rightMotorPub.publish(Float32(data=2*self.speed))
            # wait for holdTime
            time.sleep(self.holdTime)
            # stop
            self.leftMotorPub.publish(Float32(data=0.0))
            self.rightMotorPub.publish(Float32(data=0.0)) 

        def MoveBackward(self):
            self.leftMotorPub.publish(Float32(data=-2*self.speed))
            self.rightMotorPub.publish(Float32(data=-2*self.speed))
            # wait for holdTime
            time.sleep(self.holdTime)
            # stop
            self.leftMotorPub.publish(Float32(data=0.0))
            self.rightMotorPub.publish(Float32(data=0.0))

        def TurnLeft(self):
            self.leftMotorPub.publish(Float32(data=-self.speed))
            self.rightMotorPub.publish(Float32(data=self.speed))
            # wait for holdTime
            time.sleep(self.holdTime)
            # stop
            self.leftMotorPub.publish(Float32(data=0.0))
            self.rightMotorPub.publish(Float32(data=0.0))

        def TurnRight(self):
            self.leftMotorPub.publish(Float32(data=self.speed))
            self.rightMotorPub.publish(Float32(data=-self.speed))
            # wait for holdTime
            time.sleep(self.holdTime)
            # stop
            self.leftMotorPub.publish(Float32(data=0.0))
            self.rightMotorPub.publish(Float32(data=0.0))

        def command_callback(self,request):
            # request is an Int8MultiArray
            self.get_logger().info('Received command: %s' % request.data)
            for i in request.data:
                if i == 0:
                    self.MoveForward()
                elif i == 1:
                    self.MoveBackward()
                elif i == 2:
                    self.TurnLeft()
                elif i == 3:
                    self.TurnRight()
                else:
                    self.get_logger().info('Invalid command: %d' % i)
            self.sendResponse = True


def main(args=None):
    rclpy.init(args=args)

    commander = Commander()

    rclpy.spin(commander)

    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
