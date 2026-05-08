import importlib
from threading import Thread
import yaml
import numpy as np
import rclpy
import rclpy.action
from rclpy.node import Node
import sensor_msgs.msg
from rosidl_runtime_py import set_message_fields
import ros2service.api


class JoyTeleopException(Exception):
    pass

class JoyTeleop(Node):

    def __init__(self):
        super().__init__('joy_teleop')

        self.declare_parameter("car_name", "/car")
        self.declare_parameter("teleop_config", "")

        self.CAR_NAME = self.get_parameter("car_name").value
        if not self.CAR_NAME.endswith("/"):
            self.CAR_NAME += "/"

        self.topic_publishers = {}
        self.al_clients = dict()
        self.srv_clients = dict()
        self.service_types = dict()
        self.message_types = dict()
        self.command_list = dict()
        self.offline_actions = []
        self.offline_services = []

        self.old_buttons = []

        teleop_cfg = self._load_teleop_config(
            self.get_parameter("teleop_config").value
        )

        for i in teleop_cfg:
            if i in self.command_list:
                self.get_logger().log("command {} was duplicated".format(i))
                continue
            action_type = teleop_cfg[i]["type"]
            self.add_command(i, teleop_cfg[i])
            if action_type == "topic":
                self.register_topic(i, teleop_cfg[i])
            elif action_type == "action":
                self.register_action(i, teleop_cfg[i])
            elif action_type == "service":
                self.register_service(i, teleop_cfg[i])
            else:
                self.get_logger().log("unknown type '{}' for command '{}'".format(action_type, i))


        # Don't subscribe until everything has been initialized.
        self.create_subscription(sensor_msgs.msg.Joy, "joy", self.joy_callback, 10)

        # Run a low-freq action updater
        self.create_timer(2.0, self.update_actions)

    def _load_teleop_config(self, config_path):
        if not config_path:
            self.get_logger().fatal("no teleop configuration file was provided")
            raise JoyTeleopException("no config path")

        with open(config_path, "r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle) or {}

        # Support both flat format (teleop: ...) and ROS2 param format
        # (joy_teleop: ros__parameters: teleop: ...)
        teleop_cfg = config.get("teleop")
        if teleop_cfg is None:
            ros_params = config.get("joy_teleop", {}).get("ros__parameters", {})
            teleop_cfg = ros_params.get("teleop")
        if teleop_cfg is None:
            self.get_logger().fatal("no teleop configuration was found")
            raise JoyTeleopException("no config")

        return teleop_cfg

    def joy_callback(self, data):
        try:
            for c in self.command_list:
                if self.match_command(c, data.buttons):
                    self.run_command(c, data)
                    # Only run 1 command at a time
                    break
        except JoyTeleopException as e:
            self.get_logger().log("error while parsing joystick input: {}".format(str(e)))
        self.old_buttons = data.buttons

    def get_message_type(self, type_name):
        if type_name not in self.message_types:
            try:
                package, message = type_name.split("/")
                mod = importlib.import_module(package + ".msg")
                self.message_types[type_name] = getattr(mod, message)
            except ValueError:
                raise JoyTeleopException("message type format error")
            except ImportError:
                raise JoyTeleopException(
                    "module {} could not be loaded".format(package)
                )
            except AttributeError:
                raise JoyTeleopException(
                    "message {} could not be loaded from module {}".format(
                        package, message
                    )
                )
        return self.message_types[type_name]
    
    class AsyncServiceProxy(Node):

        def __init__(self, name, service_class, persistent=True):
            super().__init__('minimal_client_async')

            self.cli = self.create_client(service_class, name, persistent)
            
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
    
            self._thread = Thread(target=self.cli, args=[])

        def __del__(self):
            # try to join our thread - no way I know of to interrupt a service
            # request
            if self._thread.is_alive():
                self._thread.join(1.0)

        def __call__(self, request):
            if self._thread.is_alive():
                self._thread.join(0.01)
                if self._thread.is_alive():
                    return False
        
            self._thread = Thread(target=self.cli, args=[request])
            self._thread.start()
            return True
                
    def register_service(self, name, command):
        """ Add an AsyncServiceProxy for a joystick command """
        service_name = command["service_name"]
        try:
            service_type = self.get_service_type(service_name)
            self.srv_clients[service_name] = self.AsyncServiceProxy(
                service_name, service_type
            )

            if service_name in self.offline_services:
                self.offline_services.remove(service_name)
        except JoyTeleopException:
            if service_name not in self.offline_services:
                self.offline_services.append(service_name)

    def run_service(self, c, joy_state):
        cmd = self.command_list[c]
        request = self.get_service_type(cmd["service_name"])._request_class()
        # should work for requests, too
        set_message_fields(request, [cmd["service_request"]])
        if not self.srv_clients[cmd["service_name"]](request):
            rclpy.loginfo(
                "Not sending new service request for command {} because previous request has not finished".format(
                    c
                )
            )

    def get_service_type(self, service_name):
        if service_name not in self.service_types:
            try:
                self.service_types[service_name] = ros2service.api.get_service_class(service_name=service_name)
            except Exception as e:
                raise JoyTeleopException(
                    "service {} could not be loaded: {}".format(service_name, str(e))
                )
        return self.service_types[service_name]
    
    def match_command(self, c, buttons):
        """Find a command matching a joystick configuration"""
        # Buttons is a vector of the shape [0,1,0,1....
        # Turn it into a vector of form [1, 3...
        button_indexes = np.argwhere(buttons).flatten()

        # Check if the pressed buttons match the commands exactly.
        buttons_match = np.array_equal(self.command_list[c]["buttons"], button_indexes)

        # print button_indexes
        if buttons_match:
            return True

        # This might also be a default command.
        # We need to check if ANY commands match this set of pressed buttons.
        any_commands_matched = np.any(
            [
                np.array_equal(command["buttons"], button_indexes)
                for name, command in self.command_list.items()
            ]
        )

        # Return the final result.
        return (buttons_match) or (
            not any_commands_matched and self.command_list[c]["is_default"]
        )

    def add_command(self, name, command):
        """Add a command to the command list"""
        # Check if this is a default command
        if "is_default" not in command:
            command["is_default"] = False

        if command["type"] == "topic":
            if "deadman_buttons" not in command:
                command["deadman_buttons"] = []
            command["buttons"] = command["deadman_buttons"]
        elif command["type"] == "action":
            if "action_goal" not in command:
                command["action_goal"] = {}
        elif command["type"] == "service":
            if "service_request" not in command:
                command["service_request"] = {}
        self.command_list[name] = command

    def run_command(self, command, joy_state):
        """Run a joystick command"""
        cmd = self.command_list[command]
        if cmd["type"] == "topic":
            self.run_topic(command, joy_state)
        elif cmd["type"] == "action":
            if cmd["action_name"] in self.offline_actions:
                self.get_logger().log(
                    "command {} was not played because the action "
                    "server was unavailable. Trying to reconnect...".format(
                        cmd["action_name"]
                    )
                )
                self.register_action(command, self.command_list[command])
            else:
                if joy_state.buttons != self.old_buttons:
                    self.run_action(command, joy_state)
        elif cmd["type"] == "service":
            if cmd["service_name"] in self.offline_services:
                self.get_logger().log(
                    "command {} was not played because the service "
                    "server was unavailable. Trying to reconnect...".format(
                        cmd["service_name"]
                    )
                )
                self.register_service(command, self.command_list[command])
            else:
                if joy_state.buttons != self.old_buttons:
                    self.run_service(command, joy_state)
        else:
            raise JoyTeleopException(
                "command {} is neither a topic publisher nor an action or service client".format(
                    command
                )
            )
        
    def register_topic(self, name, command):
        """Add a topic publisher for a joystick command"""
        if command["topic_name"].startswith("/"):  # For /dev/null
            topic_name = command["topic_name"]
        else:
            topic_name = self.CAR_NAME + command["topic_name"]
            self.get_logger().info(
            "command {} is publishing to {}, which is outside of the car namespace".format(
                name, topic_name                )
            )



        try:
            topic_type = self.get_message_type(command["message_type"])
            self.topic_publishers[topic_name] = self.create_publisher(
                topic_type, topic_name, 1
            )
        except JoyTeleopException as e:
            self.get_logger().log(
                "could not register topic for command {}: {}".format(name, str(e))
            )

    def run_topic(self, c, joy_state):
        cmd = self.command_list[c]
        msg = self.get_message_type(cmd["message_type"])()

        if "message_value" in cmd:
            for param in cmd["message_value"]:
                self.set_member(msg, param["target"], param["value"])

        else:
            for mapping in cmd["axis_mappings"]:
                if len(joy_state.axes) <= mapping["axis"]:
                    self.get_logger().log(
                        "Joystick has only {} axes (indexed from 0), but #{} was referenced in config.".format(
                            len(joy_state.axes), mapping["axis"]
                        )
                    )
                    val = 0.0
                else:
                    val = joy_state.axes[mapping["axis"]] * mapping.get(
                        "scale", 1.0
                    ) + mapping.get("offset", 0.0)

                self.set_member(msg, mapping["target"], val)

        if cmd["topic_name"] != "/dev/null":
            topic_name = self.CAR_NAME + cmd["topic_name"]
        else:
            topic_name = cmd["topic_name"]
        self.topic_publishers[topic_name].publish(msg)


    def set_member(self, msg, member, value):
        ml = member.split(".")
        if len(ml) < 1:
            return
        target = msg
        for i in ml[:-1]:
            target = getattr(target, i)
        setattr(target, ml[-1], value)


    def register_action(self, name, command):
        """Add an action client for a joystick command"""
        action_name = command["action_name"]
        try:
            action_type = self.get_message_type(self.get_action_type(action_name))

            self.al_clients[action_name] = rclpy.action.ActionClient(self, action_type, action_name=action_name)
            
            if action_name in self.offline_actions:
                self.offline_actions.remove(action_name)

        except JoyTeleopException:
            if action_name not in self.offline_actions:
                self.offline_actions.append(action_name)

    def run_action(self, c, joy_state):
        cmd = self.command_list[c]
        goal = self.get_message_type(
            self.get_action_type(cmd["action_name"])[:-6] + "Goal")()
        set_message_fields(goal, [cmd["action_goal"]])
        self._action_client.wait_for_server()
        return self.al_clients.send_goal_async(goal)
        

    def update_actions(self, evt=None):
        for name, cmd in self.command_list.items():
            if cmd["type"] != "action":
                continue
            if cmd["action_name"] in self.offline_actions:
                self.register_action(name, cmd)


    def get_action_type(self, action_name):
        try:
            topics_and_types = self.get_topic_names_and_types()

            if topics_and_types.__contains__(action_name):
                return ros2topic.api(rclpy.resolve_name(action_name) + "/goal")[0][:-4]
        except TypeError:
            raise JoyTeleopException("could not find action {}".format(action_name))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JoyTeleop()
        rclpy.spin(node)
    except JoyTeleopException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
