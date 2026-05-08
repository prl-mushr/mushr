import rclpy
from lifecycle_msgs.srv import GetState

received = False

def callback(future):
    global received
    received = True

def main():
    rclpy.init()
    node = rclpy.create_node('wait_for_map_server')
    client = node.create_client(GetState, '/map_server/get_state')
    client.wait_for_service()
    future = client.call_async(GetState.Request())
    future.add_done_callback(callback)
    while not received:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
