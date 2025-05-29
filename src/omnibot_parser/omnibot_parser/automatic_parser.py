import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assistive_furniture_interfaces.msg import FurnitureList
import subprocess


ROBOT_FLOCK_SIZE = 5

class AutomaticParser(Node):
    def __init__(self):
        super().__init__('automatic_parser')

        # Create publisher for all robots
        self.velocity_pub = []
        for i in range(ROBOT_FLOCK_SIZE):
            self.velocity_pub.append(self.create_publisher(Twist, '/R' + str(i+1) + '/velocity', 1))

        # Subscriber to velocity_cmd
        _ = self.create_subscription(FurnitureList, '/DSM_coordinator', self.DSM_coordinator_callback, 1)

        # Timer for checking connection with DSM; sending stop if no cmd received
        self.stamp_last_cmd = None
        self.stamp_current_cmd = 0.
        self.stop_send = False
        timer_freq = 4.
        _ = self.create_timer(1/timer_freq, self.stop_timer_callback)

        self.get_logger().info('Node running ...')

    def DSM_coordinator_callback(self, msg):
        self.stamp_last_cmd = self.stamp_current_cmd
        for f in msg.furniture:
            f_name = f.header.frame_id.split('_')
            robot_id = int(f_name[1])-1
            twist_cmd = f.twist
            self.velocity_pub[robot_id].publish(twist_cmd)
        self.stamp_current_cmd = f.header.stamp
    
    def stop_timer_callback(self):
        if self.stamp_last_cmd == self.stamp_current_cmd:
            if not self.stop_send:
                self.get_logger().info('... not receiving cmds ...')
                stop_vel = Twist()
                for vel_pub in self.velocity_pub:
                    vel_pub.publish(stop_vel)
                    self.get_logger().warning(f'Sending stop on {vel_pub.topic_name}')
                self.stop_send = True
        elif self.stop_send :
            self.stop_send = False
            self.get_logger().info('... receiving cmds again...')
        self.stamp_last_cmd = self.stamp_current_cmd

def main(args=None):
    rclpy.init(args=args)
    automatic_parser = AutomaticParser()

    try:
        rclpy.spin(automatic_parser)
    except KeyboardInterrupt:
        pass
    finally:
        automatic_parser.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
