import rclpy, threading, time
import rclpy.action
import rclpy.duration
from turtlesim.srv import Kill, Spawn
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from custom_interfaces.action import MoveTurtle
from geometry_msgs.msg import Twist

class Controller(LifecycleNode):
    def __init__(self):
        super().__init__("controller_node")
        self.grup = ReentrantCallbackGroup()
        self.is_server_active = False
        self.goal_obj: ServerGoalHandle = None
        self.goal_lock = threading.Lock()
        self.get_logger().warn("Turtlesim node started. Current state: UNCONFIGURED")
        self.declare_parameter("turtle_name", "arda")
        self.turt_name = self.get_parameter("turtle_name").value

    def on_configure(self, state: LifecycleState):
        self.spawner_client = self.create_client(Spawn, "spawn", callback_group=self.grup)
        self.killer_client = self.create_client(Kill, "kill", callback_group=self.grup)
        self.cmdvel_pubber = self.create_lifecycle_publisher(Twist, f'{self.turt_name}/cmd_vel', 10, callback_group=self.grup)
        self.turtlebot_cmdvel_pubber = self.create_lifecycle_publisher(Twist, "cmd_vel", 10, callback_group=self.grup)
        self.movement_server = ActionServer(
            self,
            MoveTurtle,
            f'{self.turt_name}_move',
            goal_callback = self.goalCallback,
            cancel_callback = self.cancelCallback,
            execute_callback = self.execCallback,
            callback_group=self.grup)
        
        self.spawnTurtle()
        self.get_logger().warn("Turtle "+self.turt_name+" spawned in Turtlesim. Current state: CONFIGURED")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.killTurtle()
        self.movement_server.destroy()
        self.destroy_client(self.spawner_client)
        self.destroy_client(self.killer_client)
        self.destroy_lifecycle_publisher(self.cmdvel_pubber)
        self.destroy_lifecycle_publisher(self.turtlebot_cmdvel_pubber)
        self.get_logger().warn("Switching back to state: UNCONFIGURED")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.is_server_active = True
        self.get_logger().warn("Current state: ACTIVE")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.is_server_active = False
        self.get_logger().warn("Current state: INACTIVE")
        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState):
        self.movement_server.destroy()
        self.is_server_active = False
        self.get_logger().warn("Server is shutting down... Current state: FINALIZED")
        self.killTurtle()
        self.destroy_client(self.spawner_client)
        self.destroy_client(self.killer_client)
        self.destroy_lifecycle_publisher(self.cmdvel_pubber)
        self.destroy_lifecycle_publisher(self.turtlebot_cmdvel_pubber)
        return TransitionCallbackReturn.SUCCESS

    def spawnTurtle(self):
        while not self.spawner_client.wait_for_service(0.5):
            self.get_logger().info("Waiting for spawn service become avaiasdukjld")
        
        req = Spawn.Request()
        req.name = self.turt_name
        req.x = 4.5
        req.y = 4.5
        futur = self.spawner_client.call_async(req)
        self.get_logger().info(f"Turtle {req.name} spawned")    # {futur.name} will result error because response is an uncompleted object
 
    def killTurtle(self):
        while not self.killer_client.wait_for_service(0.5):
            self.get_logger().info("Waiting for kill server become available...")

        req = Kill.Request()
        req.name = self.turt_name
        self.get_logger().info("Attempt to kill turtle: "+ str(req.name))
        futur = self.killer_client.call_async(req)
        self.get_logger().info('Turtle '+str(req.name)+' has been removed')

    def goalCallback(self, goal_req: MoveTurtle.Goal):
        self.get_logger().warn("Received a new goal...")
        if not self.is_server_active:
            self.get_logger().warn("Goal rejected -reason: SERVER INACTIVE")
            return GoalResponse.REJECT
        
        with self.goal_lock:
            if self.goal_obj is not None and self.goal_obj.is_active:
                self.get_logger().warn("Goal rejected -reason: CURRENTLY ACTIVE GOAL")
                return GoalResponse.REJECT
            
            if (abs(goal_req.linear_vel_x) > 10 or 
                abs(goal_req.angular_vel_z) > 7 or
                goal_req.duration <= 0):
                self.get_logger().warn("Goal rejected -reason: INVALID GOAL PARAM")
                return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancelCallback(self, goal_to_exec: ServerGoalHandle):
        self.get_logger().warn("Received a cancel request")
        return CancelResponse.ACCEPT

    def execCallback(self, goal_to_exec: ServerGoalHandle):
        with self.goal_lock:
            self.goal_obj = goal_to_exec

        velx = goal_to_exec.request.linear_vel_x
        angz = goal_to_exec.request.angular_vel_z
        durasyon = goal_to_exec.request.duration

        res = MoveTurtle.Result()
        meşaz = Twist()
        start_time = self.get_clock().now()

        while rclpy.ok():
            elapsed_time = self.get_clock().now() - start_time
            if self.is_server_active and elapsed_time.nanoseconds/pow(10,9) > durasyon: # then stop the turtle and succeed
                meşaz.linear.x = float(0)
                meşaz.angular.z = float(0)
                self.cmdvel_pubber.publish(meşaz)
                self.turtlebot_cmdvel_pubber.publish(meşaz)
                res.success = True
                res.message = 'Success'
                goal_to_exec.succeed()
                return res

            if not self.is_server_active:
                meşaz.linear.x = float(0)
                meşaz.angular.z = float(0)
                self.cmdvel_pubber.publish(meşaz)
                self.turtlebot_cmdvel_pubber.publish(meşaz)
                res.success = False
                res.message = 'Aborted because server is inactive'
                goal_to_exec.abort()
                return res

            meşaz.linear.x = velx
            meşaz.angular.z = angz
            self.cmdvel_pubber.publish(meşaz)
            self.turtlebot_cmdvel_pubber.publish(meşaz)
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    nod = Controller()
    executeur = MultiThreadedExecutor()
    executeur.add_node(nod)
    executeur.spin()
    rclpy.shutdown()

if __name__== "__main__":
    main()