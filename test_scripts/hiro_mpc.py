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
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
# from curobo.wrap.model import robot_world
# from curobo.wrap.model.robot_segmenter import RobotSegmenter


a = torch.zeros(4, device="cuda:0")

# Third Party
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": False,
        "width": "1920",
        "height": "1080",
    }
)
# Third Party
import cv2
import numpy as np
import torch
from matplotlib import cm
from nvblox_torch.datasets.realsense_dataset import RealsenseDataloader

# CuRobo
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.types.state import JointState
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

simulation_app.update()
# Standard Library
import argparse

# Third Party
import carb
from helper import VoxelManager, add_robot_to_scene
from omni.isaac.core import World
from omni.isaac.core.materials import OmniPBR
from omni.isaac.core.objects import cuboid, sphere
from omni.isaac.core.utils.types import ArticulationAction

# CuRobo
from curobo.rollout.rollout_base import Goal
from curobo.util.usd_helper import UsdHelper
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

parser = argparse.ArgumentParser()


parser.add_argument("--robot", type=str, default="franka.yml", help="robot configuration to load")
parser.add_argument(
    "--waypoints", action="store_true", help="When True, sets robot in static mode", default=True
)
parser.add_argument(
    "--use-debug-draw",
    action="store_true",
    help="When True, sets robot in static mode",
    default=False,
)
args = parser.parse_args()

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



def draw_rollout_points(rollouts: torch.Tensor, clear: bool = False):
    if rollouts is None:
        return
    # Standard Library
    import random

    # Third Party
    from omni.isaac.debug_draw import _debug_draw

    draw = _debug_draw.acquire_debug_draw_interface()
    N = 100
    if clear:
        draw.clear_points()
    cpu_rollouts = rollouts.cpu().numpy()
    b, h, _ = cpu_rollouts.shape
    point_list = []
    colors = []
    for i in range(b):
        point_list += [
            (cpu_rollouts[i, j, 0], cpu_rollouts[i, j, 1], cpu_rollouts[i, j, 2]) for j in range(h)
        ]
        colors += [(1.0 - (i + 1.0 / b), 0.3 * (i + 1.0 / b), 1.0, 0.2) for _ in range(h)]
    sizes = [10.0 for _ in range(b * h)]
    draw.draw_points(point_list, colors, sizes)


def draw_points(voxels):
    # Third Party

    # Third Party
    from omni.isaac.debug_draw import _debug_draw

    draw = _debug_draw.acquire_debug_draw_interface()
    # if draw.get_num_points() > 0:
    draw.clear_points()
    if len(voxels) == 0:
        return

    jet = cm.get_cmap("plasma").reversed()

    cpu_pos = voxels[..., :3].view(-1, 3).cpu().numpy()
    z_val = cpu_pos[:, 0]

    jet_colors = jet(z_val)

    b, _ = cpu_pos.shape
    point_list = []
    colors = []
    for i in range(b):
        # get list of points:
        point_list += [(cpu_pos[i, 0], cpu_pos[i, 1], cpu_pos[i, 2])]
        colors += [(jet_colors[i][0], jet_colors[i][1], jet_colors[i][2], 0.8)]
    sizes = [20.0 for _ in range(b)]

    draw.draw_points(point_list, colors, sizes)


def clip_camera(camera_data):
    # clip camera image to bounding box:
    h_ratio = 0.05
    w_ratio = 0.05
    depth = camera_data["raw_depth"]
    depth_tensor = camera_data["depth"]
    h, w = depth_tensor.shape
    depth[: int(h_ratio * h), :] = 0.0
    depth[int((1 - h_ratio) * h) :, :] = 0.0
    depth[:, : int(w_ratio * w)] = 0.0
    depth[:, int((1 - w_ratio) * w) :] = 0.0

    depth_tensor[: int(h_ratio * h), :] = 0.0
    depth_tensor[int(1 - h_ratio * h) :, :] = 0.0
    depth_tensor[:, : int(w_ratio * w)] = 0.0
    depth_tensor[:, int(1 - w_ratio * w) :] = 0.0

def filter_robot(data):
    h_ratio = 0.05
    w_ratio = 0.05
    depth = data["raw_depth"]
    depth_tensor = data["depth"]
    # print(depth_tensor)
    h, w = depth_tensor.shape
    # depth[: int(h_ratio * h), :] = 0.0
    # depth[int((1 - h_ratio) * h) :, :] = 0.0
    # depth[:, : int(w_ratio * w)] = 0.0
    # depth[:, int((1 - w_ratio) * w) :] = 0.0

    # depth_tensor[: int(h_ratio * h), :] = 0.0
    # depth_tensor[int(1 - h_ratio * h) :, :] = 0.0
    # depth_tensor[:, : int(w_ratio * w)] = 0.0
    # depth_tensor[:, int(1 - w_ratio * w) :] = 0.0
    depth_image = data["raw_depth"]
    color_image = data["raw_rgb"]
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=100), cv2.COLORMAP_VIRIDIS
    )
    color_image = cv2.flip(color_image, 1)
    (b ,g, r) = cv2.split(color_image)
    color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
    depth_colormap = cv2.flip(depth_colormap, 1)
    depth_colormap = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2GRAY)
    (thresh, im_bw_depth) = cv2.threshold(depth_colormap, 170, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    (thresh, im_bw_color) = cv2.threshold(g, 230, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    
    # kernel = np.ones((9, 9), np.uint8) 
    # kernel2 = np.ones((3, 3), np.uint8) 
    # im_bw_color = cv2.dilate(im_bw_color, kernel, 15)
    # im_bw_color = cv2.dilate(im_bw_color, kernel, 5)
    # im_bw_color = cv2.dilate(im_bw_color, kernel2, 5)
    # im_bw_color = cv2.erode(im_bw_color, kernel, 5)
    color_masked = cv2.bitwise_and(color_image, color_image, mask=im_bw_depth)
    im_bw_color = cv2.bitwise_not(im_bw_color)
    depth_masked = cv2.bitwise_and(im_bw_depth, im_bw_depth, mask=im_bw_color)
    # depth_masked = cv2.bitwise_not(depth_masked)
    # a = cv2.findNonZero(depth_masked)
    images = np.hstack((color_image, color_masked, im_bw_color, depth_masked))    
    cv2.namedWindow("NVBLOX Example", cv2.WINDOW_NORMAL)
    cv2.imshow("NVBLOX Example", images)
    key = cv2.waitKey(1)

def draw_line(start, gradient):
    # Third Party
    from omni.isaac.debug_draw import _debug_draw

    draw = _debug_draw.acquire_debug_draw_interface()
    # if draw.get_num_points() > 0:
    draw.clear_lines()
    start_list = [start]
    end_list = [start + gradient]

    colors = [(0.0, 0, 0.8, 0.9)]

    sizes = [10.0]
    draw.draw_lines(start_list, end_list, colors, sizes)


if __name__ == "__main__":
    radius = 0.05
    act_distance = 0.4
    voxel_size = 0.05
    render_voxel_size = 0.02
    clipping_distance = 0.9

    my_world = World(stage_units_in_meters=1.0)
    stage = my_world.stage

    stage = my_world.stage
    my_world.scene.add_default_ground_plane()

    xform = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(xform)
    target_material = OmniPBR("/World/looks/t", color=np.array([0, 1, 0]))
    target_material_2 = OmniPBR("/World/looks/t2", color=np.array([0, 1, 0]))
    if not args.waypoints:
        target = cuboid.VisualCuboid(
            "/World/target_1",
            position=np.array([0.5, 0.0, 0.7]),
            orientation=np.array([0, 1.0, 0, 0]),
            size=0.04,
            visual_material=target_material,
        )

    else:
        target = cuboid.VisualCuboid(
            "/World/target_1",
            position=np.array([0.3, -0.4, 0.5]),
            orientation=np.array([0, 1.0, 0, 0]),
            size=0.04,
            visual_material=target_material,
        )

    # Make a target to follow
    target_2 = cuboid.VisualCuboid(
        "/World/target_2",
        position=np.array([0.3, 0.4, 0.5]),
        orientation=np.array([0.0, 1, 0.0, 0.0]),
        size=0.04,
        visual_material=target_material_2,
    )

    # Make a target to follow
    camera_marker = cuboid.VisualCuboid(
        "/World/camera_nvblox",
        position=np.array([0.444, -0.605, 0.4]),
        # orientation=np.array([0.793, 0, 0.609,0.0]),
        orientation=np.array([0.69, -0.7100, -0.057, -0.08255]),
        # orientation=np.array([0.138, -0.1032, 0.649,-0.74]),
        # orientation=np.array([0.561, -0.561, 0.431,-0.431]),
        color=np.array([0, 0, 1]),
        size=0.01,
    )
    camera_marker.set_visibility(False)
    collision_checker_type = CollisionCheckerType.BLOX
    world_cfg = WorldConfig.from_dict(
        {
            "blox": {
                "world": {
                    "pose": [0, 0, 0, 1, 0, 0, 0],
                    "integrator_type": "occupancy",
                    "voxel_size": 0.03,
                }
            }
        }
    )
    tensor_args = TensorDeviceType()

    robot_cfg = load_yaml(join_path(get_robot_configs_path(), args.robot))["robot_cfg"]

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]
    robot_cfg["kinematics"]["collision_sphere_buffer"] = 0.02
    robot, _ = add_robot_to_scene(robot_cfg, my_world, "/World/world_robot/")

    world_cfg_table = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_wall.yml"))
    )

    world_cfg_table.cuboid[0].pose[2] -= 0.01
    usd_help = UsdHelper()

    usd_help.load_stage(my_world.stage)
    usd_help.add_world_to_stage(world_cfg_table.get_mesh_world(), base_frame="/World")
    world_cfg.add_obstacle(world_cfg_table.cuboid[0])
    world_cfg.add_obstacle(world_cfg_table.cuboid[1])

    mpc_config = MpcSolverConfig.load_from_robot_config(
        robot_cfg,
        world_cfg,
        use_cuda_graph=True,
        use_cuda_graph_metrics=True,
        use_cuda_graph_full_step=False,
        self_collision_check=True,
        collision_checker_type=CollisionCheckerType.BLOX,
        use_mppi=True,
        use_lbfgs=False,
        use_es=False,
        store_rollouts=True,
        step_dt=0.02,
    )

    mpc = MpcSolver(mpc_config)

    retract_cfg = mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
    joint_names = mpc.rollout_fn.joint_names

    state = mpc.rollout_fn.compute_kinematics(
        JointState.from_position(retract_cfg, joint_names=joint_names)
    )
    current_state = JointState.from_position(retract_cfg, joint_names=joint_names)
    retract_pose = Pose(state.ee_pos_seq, quaternion=state.ee_quat_seq)
    goal = Goal(
        current_state=current_state,
        goal_state=JointState.from_position(retract_cfg, joint_names=joint_names),
        goal_pose=retract_pose,
    )

    goal_buffer = mpc.setup_solve_single(goal, 1)
    mpc.update_goal(goal_buffer)

    world_model = mpc.world_collision
    realsense_data = RealsenseDataloader(clipping_distance_m=clipping_distance)
    data = realsense_data.get_data()

    camera_pose = Pose.from_list([0.444, -0.805, 8.78, -0.74, 0.138, 0, 0])
    i = 0
    tensor_args = TensorDeviceType()
    target_list = [target, target_2]
    target_material_list = [target_material, target_material_2]
    for material in target_material_list:
        material.set_color(np.array([0.1, 0.1, 0.1]))
    target_idx = 0
    cmd_idx = 0
    cmd_plan = None
    articulation_controller = robot.get_articulation_controller()
    cmd_state_full = None

    cmd_step_idx = 0
    current_error = 0.0
    error_thresh = 0.06
    first_target = False
    if not args.use_debug_draw:
        voxel_viewer = VoxelManager(100, size=render_voxel_size)

    rclpy.init()
    hiro_joint_pub = HIROJointPublisher()
    config = RobotWorldConfig.load_from_config(
        robot_cfg,
        load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")),
        collision_activation_distance=0.0,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,
        n_cuboids=50,
    )
    # robot_world = RobotWorld(config)
    # robot_segmenter = RobotSegmenter(robot_world)
    while simulation_app.is_running():
        my_world.step(render=True)

        if not my_world.is_playing():
            if i % 100 == 0:
                print("**** Click Play to start simulation *****")
            i += 1
            # if step_index == 0:
            #    my_world.play()
            continue
        step_index = my_world.current_time_step_index
        if cmd_step_idx == 0:
            draw_rollout_points(mpc.get_visual_rollouts(), clear=not args.use_debug_draw)

        if step_index < 2:
            my_world.reset()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            robot.set_joint_positions(default_config, idx_list)

            robot._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(idx_list))]), joint_indices=idx_list
            )
        sim_js = robot.get_joints_state()
        sim_js_names = robot.dof_names
        cu_js = JointState(
            position=tensor_args.to_device(sim_js.positions),
            velocity=tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        )
        cu_js = cu_js.get_ordered_joint_state(mpc.rollout_fn.joint_names)
        if step_index % 1 == 0.0:
            # camera data updation
            world_model.decay_layer("world")
            data = realsense_data.get_data()
            clip_camera(data)
            filter_robot(data)
            cube_position, cube_orientation = camera_marker.get_local_pose()
            camera_pose = Pose(
                position=tensor_args.to_device(cube_position),
                quaternion=tensor_args.to_device(cube_orientation),
            )
            # print(type(data['depth']))
            data_camera = CameraObservation(  # rgb_image = data["rgba_nvblox"],
                depth_image=data["depth"], intrinsics=data["intrinsics"], pose=camera_pose
            )
            data_camera = data_camera.to(device=tensor_args.device)
            # robot_segmenter.update_camera_projection(data_camera)
            # _, data_camera.depth_image = robot_segmenter.get_robot_mask_from_active_js(data_camera, cu_js)
            world_model.add_camera_frame(data_camera, "world")
            world_model.process_camera_frames("world", False)
            torch.cuda.synchronize()
            world_model.update_blox_hashes()

            bounding = Cuboid("t", dims=[1, 1, 1.0], pose=[0, 0, 0, 1, 0, 0, 0])
            voxels = world_model.get_voxels_in_bounding_box(bounding, voxel_size)
            if voxels.shape[0] > 0:
                voxels = voxels[voxels[:, 2] > voxel_size]
                voxels = voxels[voxels[:, 0] > 0.0]
                if args.use_debug_draw:
                    draw_points(voxels)

                else:
                    voxels = voxels.cpu().numpy()
                    voxel_viewer.update_voxels(voxels[:, :3])
            else:
                if not args.use_debug_draw:
                    voxel_viewer.clear()

        

        if cmd_state_full is None:
            current_state.copy_(cu_js)
        else:
            current_state_partial = cmd_state_full.get_ordered_joint_state(
                mpc.rollout_fn.joint_names
            )
            current_state.copy_(current_state_partial)
            current_state.joint_names = current_state_partial.joint_names

        if current_error <= error_thresh and (not first_target or args.waypoints):
            first_target = True
            # motion generation:
            for ks in range(len(target_material_list)):
                if ks == target_idx:
                    target_material_list[ks].set_color(np.ravel([0, 1.0, 0]))
                else:
                    target_material_list[ks].set_color(np.ravel([0.1, 0.1, 0.1]))

            cube_position, cube_orientation = target_list[target_idx].get_world_pose()

            # Set EE teleop goals, use cube for simple non-vr init:
            ee_translation_goal = cube_position
            ee_orientation_teleop_goal = cube_orientation

            # compute curobo solution:
            ik_goal = Pose(
                position=tensor_args.to_device(ee_translation_goal),
                quaternion=tensor_args.to_device(ee_orientation_teleop_goal),
            )
            goal_buffer.goal_pose.copy_(ik_goal)
            mpc.update_goal(goal_buffer)
            target_idx += 1
            if target_idx >= len(target_list):
                target_idx = 0

        if cmd_step_idx == 0:
            mpc_result = mpc.step(current_state, max_attempts=2)
            current_error = mpc_result.metrics.pose_error.item()
        cmd_state_full = mpc_result.js_action
        common_js_names = []
        idx_list = []
        for x in sim_js_names:
            if x in cmd_state_full.joint_names:
                idx_list.append(robot.get_dof_index(x))
                common_js_names.append(x)

        cmd_state = cmd_state_full.get_ordered_joint_state(common_js_names)
        cmd_state_full = cmd_state

        art_action = ArticulationAction(
            cmd_state.position.cpu().numpy(),
            # cmd_state.velocity.cpu().numpy(),
            joint_indices=idx_list,
        )

        hiro_joint_data = list(cmd_state.position.cpu().numpy()[0][:-2])
        hiro_joint_data = [x.item() for x in hiro_joint_data]
        # print(type(hiro_joint_data))
        # print(type(hiro_joint_data[0]))
        msg = Float32MultiArray()
        # print(hiro_joint_data, "\n\n\n")
        msg.data = hiro_joint_data
        hiro_joint_pub.publisher.publish(msg)
        # print("command state", cmd_state, "\n\n\n")
        articulation_controller.apply_action(art_action)

        if cmd_step_idx == 2:
            cmd_step_idx = 0

        # positions_goal = a
        # if cmd_plan is not None:
        #     cmd_state = cmd_plan[cmd_idx]

        #     # get full dof state
        #     art_action = ArticulationAction(
        #         cmd_state.position.cpu().numpy(),
        #         # cmd_state.velocity.cpu().numpy(),
        #         joint_indices=idx_list,
        #     )
        #     # set desired joint angles obtained from IK:
        #     articulation_controller.apply_action(art_action)
        #     cmd_step_idx += 1
        #     # for _ in range(2):
        #     #    my_world.step(render=False)
        #     if cmd_idx >= len(cmd_plan.position):
        #         cmd_idx = 0
        #         cmd_plan = None
    realsense_data.stop_device()
    print("finished program")

    simulation_app.close()
