#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#


# Third Party
import torch

a = torch.zeros(4, device="cuda:0")

# Standard Library
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless_mode",
    type=str,
    default=None,
    help="To run headless, use one of [native, websocket], webrtc might not work.",
)
parser.add_argument("--robot", type=str, default="franka.yml", help="robot configuration to load")
parser.add_argument(
    "--external_asset_path",
    type=str,
    default=None,
    help="Path to external assets when loading an externally located robot",
)
parser.add_argument(
    "--external_robot_configs_path",
    type=str,
    default=None,
    help="Path to external robot config when loading an external robot",
)

parser.add_argument(
    "--visualize_spheres",
    action="store_true",
    help="When True, visualizes robot spheres",
    default=False,
)
parser.add_argument(
    "--reactive",
    action="store_true",
    help="When True, runs in reactive mode",
    default=False,
)

parser.add_argument(
    "--constrain_grasp_approach",
    action="store_true",
    help="When True, approaches grasp with fixed orientation and motion only along z axis.",
    default=False,
)

parser.add_argument(
    "--reach_partial_pose",
    nargs=6,
    metavar=("qx", "qy", "qz", "x", "y", "z"),
    help="Reach partial pose",
    type=float,
    default=None,
)
parser.add_argument(
    "--hold_partial_pose",
    nargs=6,
    metavar=("qx", "qy", "qz", "x", "y", "z"),
    help="Hold partial pose while moving to goal",
    type=float,
    default=None,
)

# Custom argument to specificy if the contact locations are requested from the tactile extension or not
parser.add_argument(
    "--no_extension",
    action="store_true",
    help="When True, uses the tactile extension to get contact locations",
    default=False,
)


args = parser.parse_args()

############################################################

# Third Party
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }
)
# Standard Library
from typing import Dict

# Third Party
import carb
import numpy as np
from helper import add_extensions, add_robot_to_scene
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid, sphere

########### OV #################
from omni.isaac.core.utils.types import ArticulationAction

# CuRobo
# from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig, Cuboid
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as cuPose
from curobo.types.robot import JointState
from curobo.types.state import JointState
from curobo.util.logger import log_error, setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import (
    get_assets_path,
    get_filename,
    get_path_of_dir,
    get_robot_configs_path,
    get_world_configs_path,
    join_path,
    load_yaml,
)
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    PoseCostMetric,
)

############################################################

########### Contact Extension #################
# Import ROS2 to listen for geometry mssages
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Float64
from rclpy.executors import SingleThreadedExecutor
import asyncio

# Listener node
# class ContactListenerNode(Node):
#     def __init__(self):
#         super().__init__("contact_listener_node")
#         self.create_subscription(Vector3, "touch_sensor_pos", self.contact_callback, 10)

#         self.pos = []
#         self.listen_count = 0

#     def contact_callback(self, msg):
#         print("Contact detected")
#         self.pos = [msg.x, msg.y, msg.z]
#         self.listen_count += 1

# Added by Caleb to publish joint position solutions
##########################
class HIROJointPublisher(Node):
    def __init__(self):
        super().__init__('hiro_joint_pub')
        self.publisher = self.create_publisher(Float32MultiArray, 'cu_joint_solution', 1)
        self.i = 0

    def publish(self, joint_positions):
        msg = Float32MultiArray()
        msg.data = joint_positions
        self.publisher.publish(msg)

# Object Spawner Node
# Directly places a new object at the contact location for every contact detected
class ObjectSpawnerNode(Node):
    def __init__(self, spawned_viz_objects, world_collision_checker, tensor_args):
        super().__init__("object_spawner_node")
        self.create_subscription(Vector3, "touch_sensor_pos", self.place_at_contact, 10)

        self.spawned_viz_objects = spawned_viz_objects # List of objects that have been spawned
        self.world_collision_checker = world_collision_checker # Collision checker for the world
        self.tensor_args = tensor_args

        self.next_id = 0

    def place_at_contact(self, msg):
        # Take the already pre-spawned objets and place them at the contact locations to adjust motion planning
        print("Contact detected, spawning object")
        new_pose = cuPose.from_list([msg.x,msg.y,msg.z,1,0,0,0], tensor_args=self.tensor_args)
        self.world_collision_checker.update_obstacle_pose("contact_obstacle_" + str(self.next_id), new_pose)
        self.spawned_viz_objects[self.next_id].set_world_pose(position=np.array([msg.x, msg.y, msg.z]))
        self.next_id = (self.next_id + 1) % len(self.spawned_viz_objects)

# Contact List Listener Node
class ContactListListenerNode(Node):
    def __init__(self):
        super().__init__("contact_list_listener_node")
        self.create_subscription(Int16MultiArray, "contact_list", self.parse_list_callback, 1)

        self.list = []

    def parse_list_callback(self, msg):
        print("Contact list received: " + str(msg.data))
        self.list = msg.data

    def clear_list(self):
        self.list = []
        
# Requests a location from the tactile extension and places an obstacle at the location recieved
from tactile_msgs.srv import IndexToPos
class ContactLocationClient(Node):
    def __init__(self, spawned_viz_objects, world_collision_checker, tensor_args):
        super().__init__('contact_location_client')
        self.client = self.create_client(IndexToPos, 'index_to_pos')


        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.req = IndexToPos.Request()

        self.spawned_viz_objects = spawned_viz_objects # List of objects that have been spawned
        self.world_collision_checker = world_collision_checker # Collision checker for the world
        self.tensor_args = tensor_args

        self.next_id = 0

    def send_request(self, index):
        #self.get_logger().info('Sending request...')
        self.req.index = index
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.future_callback)
        return self.future

    def future_callback(self, future):
        #self.get_logger().info('Future callback called')
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info('Contact location: {%0.2f, %0.2f, %0.2f}' % (response.position.x, response.position.y, response.position.z))
            new_pose = cuPose.from_list([response.position.x, response.position.y, response.position.z,1,0,0,0], tensor_args=self.tensor_args)
            self.world_collision_checker.update_obstacle_pose("contact_obstacle_" + str(self.next_id), new_pose)
            self.spawned_viz_objects[self.next_id].set_world_pose(position=np.array([response.position.x, response.position.y, response.position.z]))
            self.next_id = (self.next_id + 1) % len(self.spawned_viz_objects)

# Requests a location from the tactile extension and places an obstacle at the location recieved
from tactile_msgs.srv import IndexToPose
class ContactPoseClient(Node):
    def __init__(self, spawned_viz_objects, world_collision_checker, tensor_args):
        super().__init__('contact_pose_client')
        self.client = self.create_client(IndexToPose, 'index_to_pose')


        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.req = IndexToPose.Request()

        self.spawned_viz_objects = spawned_viz_objects # List of objects that have been spawned
        self.world_collision_checker = world_collision_checker # Collision checker for the world
        self.tensor_args = tensor_args

        self.next_id = 0
        self.dist = 0.0

    def send_request(self, index):
        #self.get_logger().info('Sending request...')
        self.req.index = index
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.future_callback)
        return self.future

    def future_callback(self, future):
        #self.get_logger().info('Future callback called')
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info('Contact location: {%0.2f, %0.2f, %0.2f}' % (response.pose.position.x, response.pose.position.y, response.pose.position.z))

            #Add an arbitrary distance from the contact location on the object's local z axis
            response.pose.position.z += self.dist

            # #Update the obstacle pose in the world collision checker
            new_pose = cuPose.from_list([response.pose.position.x, response.pose.position.y, response.pose.position.z,response.pose.orientation.w, response.pose.orientation.x, response.pose.orientation.y, response.pose.orientation.z], tensor_args=self.tensor_args)
            self.world_collision_checker.update_obstacle_pose("contact_obstacle_" + str(self.next_id), new_pose)
            self.spawned_viz_objects[self.next_id].set_world_pose(position=np.array([response.pose.position.x, response.pose.position.y, response.pose.position.z]))
            self.next_id = (self.next_id + 1) % len(self.spawned_viz_objects)

    def update_dist(self, dist):
        self.dist = dist

class ContactDistSubsciber(Node):
    def __init__(self):
        super().__init__('set_contact_dist_pub')
        self.create_subscription(Float64, 'contact_dist', self.contact_dist_callback, 10)
        self.dist = 1.0

    def contact_dist_callback(self, msg):
        print("Contact distance received: " + str(msg.data))
        self.dist = msg.data


################### End Effector Teleop ####################
class EndEffectorTeleop(Node):
    def __init__(self, target_prim):
        super().__init__("end_effector_teleop")
        self.create_subscription(Pose, "end_effector_pose", self.ee_pos_callback, 10) # Switch to self.ee_callback for orientaiton as well

        self.target_prim = target_prim
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.listen_count = 0

    def ee_callback(self, msg):
        #print("End effector position received")
        self.position = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.target_prim.set_world_pose(position=self.position, orientation=self.orientation)
        self.listen_count += 1

    def ee_pos_callback(self, msg):
        #print("End effector position received")
        self.position = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.target_prim.set_world_pose(position=self.position)
        self.listen_count += 1


# Asynchronous function to listen for contact
# async def contact_listener(executor, node):
#     while True:
#         executor.spin_once(timeout_sec=0.1)
#         print("Listened for contact " + str(node.listen_count) + " times")
#         await asyncio.sleep(0)

###############################################

################### Contact Obstacle Spawning ####################
#class ContactObstacleSpawner(Node):

########### OV #################;;;;;


def main():

    # create a curobo motion gen instance:
    num_targets = 0
    # assuming obstacles are in objects_path:
    my_world = World(stage_units_in_meters=1.0)
    stage = my_world.stage

    xform = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(xform)
    stage.DefinePrim("/curobo", "Xform")
    # my_world.stage.SetDefaultPrim(my_world.stage.GetPrimAtPath("/World"))
    stage = my_world.stage
    # stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    # Make a target to follow
    target = cuboid.VisualCuboid(
        "/World/target",
        position=np.array([0.5, 0, 0.5]),
        orientation=np.array([0, 1, 0, 0]),
        color=np.array([0, 1.0, 0]),
        size=0.02,
    )

    setup_curobo_logger("warn")
    past_pose = None
    n_obstacle_cuboids = 30
    n_obstacle_mesh = 100

    ######## Initialize ROS2 node #########
    rclpy.init(args=None)
    ee_teleop_node = EndEffectorTeleop(target)
    contact_listener_node = ContactListListenerNode()
    hiro_joint_pub = HIROJointPublisher()
    executor = SingleThreadedExecutor()
    executor.add_node(ee_teleop_node)
    executor.add_node(contact_listener_node)
    #######################################

    # warmup curobo instance
    usd_help = UsdHelper()
    target_pose = None

    tensor_args = TensorDeviceType()
    robot_cfg_path = get_robot_configs_path()
    if args.external_robot_configs_path is not None:
        robot_cfg_path = args.external_robot_configs_path
    robot_cfg = load_yaml(join_path(robot_cfg_path, args.robot))["robot_cfg"]

    if args.external_asset_path is not None:
        robot_cfg["kinematics"]["external_asset_path"] = args.external_asset_path
    if args.external_robot_configs_path is not None:
        robot_cfg["kinematics"]["external_robot_configs_path"] = args.external_robot_configs_path
    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]

    robot, robot_prim_path = add_robot_to_scene(robot_cfg, my_world)
    articulation_controller = None

    world_cfg_table = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
    )
    world_cfg_table.cuboid[0].pose[2] -= 0.02
    world_cfg1 = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
    ).get_mesh_world()
    world_cfg1.mesh[0].name += "_mesh"
    world_cfg1.mesh[0].pose[2] = -10.5

    ################## Add Contact Objects ##################

    touch_cuboids = []
    touch_cuboids_viz = []
    for i in range(3):
        new_cuboid = Cuboid(
            name="contact_obstacle_" + str(i),
            pose=[0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0],
            dims=[0.1, 0.1, 0.1],
            color=[0.8, 0.0, 0.0, 1.0],
            scale=0.1,
        )
        touch_cuboids.append(new_cuboid)

        # Visual double that can be seen in the simulation
        cuboid_visual = cuboid.VisualCuboid(
            "/World/contact_obstacle_" + str(i) + "_viz",
            position=np.array([0.0, 0.0, 1.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            color=np.array([0.8, 0.0, 0.0]),
            size=0.1,
        )
        touch_cuboids_viz.append(cuboid_visual)

    world_cfg = WorldConfig(cuboid=touch_cuboids + world_cfg_table.cuboid, mesh=world_cfg1.mesh)

    trajopt_dt = None
    optimize_dt = True
    trajopt_tsteps = 32
    trim_steps = None
    max_attempts = 4
    interpolation_dt = 0.05
    if args.reactive:
        trajopt_tsteps = 40
        trajopt_dt = 0.04
        optimize_dt = False
        max_attempts = 1
        trim_steps = [1, None]
        interpolation_dt = trajopt_dt
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_cfg,
        world_cfg,
        tensor_args,
        collision_checker_type=CollisionCheckerType.MESH,
        num_trajopt_seeds=12,
        num_graph_seeds=12,
        interpolation_dt=interpolation_dt,
        collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
        optimize_dt=optimize_dt,
        trajopt_dt=trajopt_dt,
        trajopt_tsteps=trajopt_tsteps,
        trim_steps=trim_steps,
    )
    motion_gen = MotionGen(motion_gen_config)

    # Add the cuboid to the scene
    #motion_gen.world_coll_checker.add_obb(new_cuboid)
    #motion_gen.world_coll_checker.add_obb(new_cuboid_2)

    # Instantiate the appropriate class based on the --no_extension argument
    # This defaults to requesting contact locations from the tactile extension
    if args.no_extension:
        contact_updator = ObjectSpawnerNode(touch_cuboids_viz, motion_gen.world_coll_checker, tensor_args)
    else:
        contact_updator = ContactPoseClient(touch_cuboids_viz, motion_gen.world_coll_checker, tensor_args)

    dist_sub = ContactDistSubsciber()
    curr_dist = 0.0

    spawner_executor = SingleThreadedExecutor()
    spawner_executor.add_node(contact_updator)
    spawner_executor.add_node(dist_sub)
    #########################################################


    print("warming up...")
    motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False, parallel_finetune=True)

    print("Curobo is Ready")

    add_extensions(simulation_app, args.headless_mode)

    # Add the contact extension to the simulation
    if not args.no_extension:
        try:
            enable_extension("contact_ext")
        except:
            print("Contact extension not found")

    plan_config = MotionGenPlanConfig(
        enable_graph=False,
        enable_graph_attempt=2,
        max_attempts=max_attempts,
        enable_finetune_trajopt=True,
        parallel_finetune=True,
    )

    usd_help.load_stage(my_world.stage)
    usd_help.add_world_to_stage(world_cfg, base_frame="/World")

    cmd_plan = None
    cmd_idx = 0
    my_world.scene.add_default_ground_plane()
    i = 0
    spheres = None
    past_cmd = None
    target_orientation = None
    past_orientation = None
    pose_metric = None

    # Main loop
    while simulation_app.is_running():
        my_world.step(render=True)
        if not my_world.is_playing():
            if i % 1000 == 0:
                print("**** Click Play to start simulation *****")
            i += 1
            # if step_index == 0:
            #    my_world.play()
            continue
        
        step_index = my_world.current_time_step_index
        # print(step_index)
        if articulation_controller is None:
            # robot.initialize()
            articulation_controller = robot.get_articulation_controller()
        if step_index < 2:
            my_world.reset()
            robot._articulation_view.initialize()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            robot.set_joint_positions(default_config, idx_list)

            robot._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(idx_list))]), joint_indices=idx_list
            )
        if step_index < 20:
            continue

        # if step_index == 50 or step_index % 1000 == 0.0:
        #     print("Updating world, reading w.r.t.", robot_prim_path)
        #     obstacles = usd_help.get_obstacles_from_stage(
        #         # only_paths=[obstacles_path],
        #         reference_prim_path=robot_prim_path,
        #         ignore_substring=[
        #             robot_prim_path,
        #             "/World/target",
        #             "/World/defaultGroundPlane",
        #             "/curobo",
        #         ],
        #     ).get_collision_check_world()
        #     print(len(obstacles.objects))

        #     motion_gen.update_world(obstacles)
        #     print("Updated World")
        #     carb.log_info("Synced CuRobo world from stage.")

        ##################### End Effector Teleop #####################
        # Place the target at wherever the contact is detected
        if step_index % 20 == 0:
            executor.spin_once(timeout_sec=0.0)

            # Update the spawining distance if it has changed
            if dist_sub.dist != curr_dist:
                print("Updating contact distance to " + str(dist_sub.dist))
                curr_dist = dist_sub.dist
                contact_updator.update_dist(curr_dist)
                

            # Spawn an obstacle for every index returned by the list node
            for index in contact_listener_node.list:
                #print("Requesting location for index " + str(index))
                future = contact_updator.send_request(str(index))
                spawner_executor.spin_until_future_complete(future, timeout_sec=0.1) 
                #response = future.result()

            # Clear the list after all the objects have been spawned
            contact_listener_node.clear_list()

        ###############################################################

        # position and orientation of target virtual cube:
        cube_position, cube_orientation = target.get_world_pose()

        if past_pose is None:
            past_pose = cube_position
        if target_pose is None:
            target_pose = cube_position
        if target_orientation is None:
            target_orientation = cube_orientation
        if past_orientation is None:
            past_orientation = cube_orientation

        sim_js = robot.get_joints_state()
        sim_js_names = robot.dof_names
        if np.any(np.isnan(sim_js.positions)):
            log_error("isaac sim has returned NAN joint position values.")
        cu_js = JointState(
            position=tensor_args.to_device(sim_js.positions),
            velocity=tensor_args.to_device(sim_js.velocities),  # * 0.0,
            acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        )

        if not args.reactive:
            cu_js.velocity *= 0.0
            cu_js.acceleration *= 0.0

        if args.reactive and past_cmd is not None:
            cu_js.position[:] = past_cmd.position
            cu_js.velocity[:] = past_cmd.velocity
            cu_js.acceleration[:] = past_cmd.acceleration
        cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)

        if args.visualize_spheres and step_index % 2 == 0:
            sph_list = motion_gen.kinematics.get_robot_as_spheres(cu_js.position)

            if spheres is None:
                spheres = []
                # create spheres:

                for si, s in enumerate(sph_list[0]):
                    sp = sphere.VisualSphere(
                        prim_path="/curobo/robot_sphere_" + str(si),
                        position=np.ravel(s.position),
                        radius=float(s.radius),
                        color=np.array([0, 0.8, 0.2]),
                    )
                    spheres.append(sp)
            else:
                for si, s in enumerate(sph_list[0]):
                    if not np.isnan(s.position[0]):
                        spheres[si].set_world_pose(position=np.ravel(s.position))
                        spheres[si].set_radius(float(s.radius))

        robot_static = False
        if (np.max(np.abs(sim_js.velocities)) < 0.2) or args.reactive:
            robot_static = True
        if (
            (
                np.linalg.norm(cube_position - target_pose) > 1e-3
                or np.linalg.norm(cube_orientation - target_orientation) > 1e-3
            )
            and np.linalg.norm(past_pose - cube_position) == 0.0
            and np.linalg.norm(past_orientation - cube_orientation) == 0.0
            and robot_static
        ):
            # Set EE teleop goals, use cube for simple non-vr init:
            ee_translation_goal = cube_position
            ee_orientation_teleop_goal = cube_orientation

            # compute curobo solution:
            ik_goal = cuPose(
                position=tensor_args.to_device(ee_translation_goal),
                quaternion=tensor_args.to_device(ee_orientation_teleop_goal),
            )
            plan_config.pose_cost_metric = pose_metric
            result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)
            # ik_result = ik_solver.solve_single(ik_goal, cu_js.position.view(1,-1), cu_js.position.view(1,1,-1))

            succ = result.success.item()  # ik_result.success.item()
            if num_targets == 1:
                if args.constrain_grasp_approach:
                    pose_metric = PoseCostMetric.create_grasp_approach_metric()
                if args.reach_partial_pose is not None:
                    reach_vec = motion_gen.tensor_args.to_device(args.reach_partial_pose)
                    pose_metric = PoseCostMetric(
                        reach_partial_pose=True, reach_vec_weight=reach_vec
                    )
                if args.hold_partial_pose is not None:
                    hold_vec = motion_gen.tensor_args.to_device(args.hold_partial_pose)
                    pose_metric = PoseCostMetric(hold_partial_pose=True, hold_vec_weight=hold_vec)
            if succ:
                num_targets += 1
                cmd_plan = result.get_interpolated_plan()
                cmd_plan = motion_gen.get_full_js(cmd_plan)
                # get only joint names that are in both:
                idx_list = []
                common_js_names = []
                for x in sim_js_names:
                    if x in cmd_plan.joint_names:
                        idx_list.append(robot.get_dof_index(x))
                        common_js_names.append(x)
                # idx_list = [robot.get_dof_index(x) for x in sim_js_names]

                cmd_plan = cmd_plan.get_ordered_joint_state(common_js_names)

                cmd_idx = 0

            else:
                carb.log_warn("Plan did not converge to a solution.  No action is being taken.")
            target_pose = cube_position
            target_orientation = cube_orientation
        past_pose = cube_position
        past_orientation = cube_orientation
        if cmd_plan is not None:
            cmd_state = cmd_plan[cmd_idx]
            past_cmd = cmd_state.clone()
            # get full dof state
            art_action = ArticulationAction(
                cmd_state.position.cpu().numpy(),
                joint_indices=idx_list,
            )


            # Publish the joint positions to the ROS2 network
            #hiro_joint_data = list(cmd_plan.position.cpu().numpy()[0][:-2])
            hiro_joint_data = list(cmd_state.position.cpu().numpy()[:-2])
            hiro_joint_data = [x.item() for x in hiro_joint_data]
            # print(type(hiro_joint_data))
            # print(type(hiro_joint_data[0]))
            msg = Float32MultiArray()
            # print(hiro_joint_data, "\n\n\n")
            msg.data = hiro_joint_data
            hiro_joint_pub.publisher.publish(msg)


            # set desired joint angles obtained from IK:
            articulation_controller.apply_action(art_action)
            cmd_idx += 1
            for _ in range(2):
                my_world.step(render=False)
            if cmd_idx >= len(cmd_plan.position):
                cmd_idx = 0
                cmd_plan = None
                past_cmd = None

    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":

    main()
