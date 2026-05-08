#!/usr/bin/env python3
"""
Gazebo → ROS 2 Thermal Camera Bridge (Float32 version)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import gz.transport13
import gz.msgs10.image_pb2 as image_pb2

TEMP_MIN_K = 280.0   # 7°C – cold background
TEMP_MAX_K = 320.0   # 47°C – human body range

def apply_iron_colormap(gray_frame):
    """Apply IRON/INFERNO false-color map"""
    try:
        import cv2
        bgr = cv2.applyColorMap(gray_frame, cv2.COLORMAP_INFERNO)
        return bgr[:, :, ::-1].copy()
    except ImportError:
        # Simple heat map fallback
        h, w = gray_frame.shape
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        norm = gray_frame / 255.0
        rgb[:, :, 0] = (np.clip(norm * 2 - 1, 0, 1) * 255).astype(np.uint8)  # R
        rgb[:, :, 1] = (np.clip(norm * 2 - 0.5, 0, 1) * 255).astype(np.uint8)  # G
        rgb[:, :, 2] = (np.clip(1 - norm * 2, 0, 1) * 255).astype(np.uint8)   # B
        return rgb

class ThermalBridgeNode(Node):
    def __init__(self, drone_id=0, world_name='baylands_collapsed_fixed'):
        super().__init__(f'thermal_bridge_drone{drone_id}')
        
        ns = f'/drone{drone_id}/thermal'
        self.pub_raw = self.create_publisher(Image, f'{ns}/image_raw', 10)
        self.pub_color = self.create_publisher(Image, f'{ns}/image_colormap', 10)
        
        # Subscribe to Gazebo thermal camera topic
        gz_topic = f'/world/{world_name}/model/x500_depth_{drone_id}/link/thermal_camera_link/sensor/thermal_camera/image'
        
        self.gz_node = gz.transport13.Node()
        self.gz_node.subscribe(image_pb2.Image, gz_topic, self._on_thermal_image)
        
        self.get_logger().info(f'Subscribed to: {gz_topic}')
        self.frame_count = 0
    
    def _on_thermal_image(self, gz_img):
        h, w = gz_img.height, gz_img.width
        
        # Handle R_FLOAT32 format (4 bytes per pixel)
        raw_bytes = bytes(gz_img.data)
        expected = h * w * 4  # float32 = 4 bytes
        
        if len(raw_bytes) < expected:
            self.get_logger().warn(f'Short frame: {len(raw_bytes)} < {expected}')
            return
        
        # Convert to float32 array (temperature in Kelvin)
        temp_k = np.frombuffer(raw_bytes[:expected], dtype=np.float32).reshape(h, w)
        
        # Normalize to uint8
        norm = np.clip((temp_k - TEMP_MIN_K) / (TEMP_MAX_K - TEMP_MIN_K), 0.0, 1.0)
        gray = (norm * 255).astype(np.uint8)
        
        # Apply colormap
        color_rgb = apply_iron_colormap(gray)
        
        now = self.get_clock().now().to_msg()
        frame_id = f'drone{self.drone_id}_thermal_camera_link'
        
        # Publish mono8
        raw_msg = Image()
        raw_msg.header.stamp = now
        raw_msg.header.frame_id = frame_id
        raw_msg.height = h
        raw_msg.width = w
        raw_msg.encoding = 'mono8'
        raw_msg.step = w
        raw_msg.data = gray.tobytes()
        self.pub_raw.publish(raw_msg)
        
        # Publish RGB8
        color_msg = Image()
        color_msg.header.stamp = now
        color_msg.header.frame_id = frame_id
        color_msg.height = h
        color_msg.width = w
        color_msg.encoding = 'rgb8'
        color_msg.step = w * 3
        color_msg.data = color_rgb.tobytes()
        self.pub_color.publish(color_msg)
        
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: {w}×{h}, '
                f'temp {temp_k.min()-273.15:.1f}°C to {temp_k.max()-273.15:.1f}°C'
            )

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone-id', type=int, default=0)
    parser.add_argument('--world', type=str, default='baylands_collapsed_fixed')
    args = parser.parse_args()
    
    rclpy.init()
    node = ThermalBridgeNode(drone_id=args.drone_id, world_name=args.world)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
