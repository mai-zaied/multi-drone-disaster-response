import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus


class FogServer(Node):
    def __init__(self):
        super().__init__('fog_server')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.drones = {
            'drone0': '/fmu/out/vehicle_status_v1',
            'drone1': '/px4_1/fmu/out/vehicle_status_v1',
            'drone2': '/px4_2/fmu/out/vehicle_status_v1'
        }

        self.drone_subscribers = []
        self.decision_publishers = {}

        for drone_id, status_topic in self.drones.items():
            sub = self.create_subscription(
                VehicleStatus,
                status_topic,
                lambda msg, d=drone_id: self.status_callback(msg, d),
                qos_profile
            )
            self.drone_subscribers.append(sub)

            decision_topic = f'/fog/{drone_id}/decision'
            pub = self.create_publisher(String, decision_topic, 10)
            self.decision_publishers[drone_id] = pub

            self.get_logger().info(f'[FOG] Listening to {drone_id} on {status_topic}')
            self.get_logger().info(f'[FOG] Publishing decisions for {drone_id} on {decision_topic}')

    def status_callback(self, msg, drone_id):
        self.get_logger().info(
            f'[FOG] {drone_id} status received | nav_state={msg.nav_state}, arming_state={msg.arming_state}'
        )

        decision = String()

        if msg.arming_state == 2:
            decision.data = f'{drone_id}: COMMAND_MONITOR (ARMED)'
            self.get_logger().warn(f'[FOG ALERT] {drone_id} is ARMED')

        elif msg.nav_state == 4:
            decision.data = f'{drone_id}: COMMAND_HOLD_POSITION'
            self.get_logger().info(f'[FOG INFO] {drone_id} HOLD POSITION')

        else:
            decision.data = f'{drone_id}: COMMAND_NORMAL_OPERATION'

        time.sleep(1)

        self.decision_publishers[drone_id].publish(decision)

        self.get_logger().info(f'[FOG DECISION] Published: {decision.data}')


def main(args=None):
    rclpy.init(args=args)
    node = FogServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()