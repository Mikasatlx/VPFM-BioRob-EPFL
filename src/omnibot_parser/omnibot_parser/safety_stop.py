import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

ROBOT_FLOCK_SIZE = 5

class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')

        # Create publisher for all robots
        self.velocity_pub = []
        for i in range(ROBOT_FLOCK_SIZE):
            self.velocity_pub.append(self.create_publisher(Twist, '/R' + str(i+1) + '/velocity', 1))

        # Timer for checking if parser is running
        self.stop_send = False
        timer_freq = 4
        _ = self.create_timer(1/timer_freq, self.stop_timer_callback)

        self.get_logger().info('Node running ...')

    def stop_timer_callback(self):
        active_nodes = self.get_node_names()
            
        if "automatic_parser" not in active_nodes:
            if not self.stop_send:
                stop_vel = Twist()
                for vel_pub in self.velocity_pub:
                    vel_pub.publish(stop_vel)
                    self.get_logger().warning(f'Sending stop on {vel_pub.topic_name}')
                self.stop_send = True
        elif self.stop_send :
            self.stop_send = False
            self.get_logger().info('... stop rearmed ...')




def main(args=None):
    rclpy.init(args=args)
    safety_stop = SafetyStop()

    try:
        rclpy.spin(safety_stop)
    except KeyboardInterrupt:
        pass
    finally:
        safety_stop.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
