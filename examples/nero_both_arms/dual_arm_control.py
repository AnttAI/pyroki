"""NERO Dual Arm Control - Both Arms with ROS Support

This script demonstrates dual arm control where:
- Left arm: Both simulation (green/transparent) and real robot (original colors) via ROS
- Right arm: Both simulation (green/transparent) and real robot (original colors) via ROS

Both robots show:
- Sim robot: Planned trajectory (green/transparent)
- Real robot: Actual robot state from ROS feedback (original colors, solid)

ROS Control Options:
- Publish LEFT ARM trajectory only
- Publish RIGHT ARM trajectory only
- Publish BOTH ARMS trajectories simultaneously

USAGE:
    1. Start the dual arm ROS controller:
       ros2 launch agx_arm_ctrl start_double_agx_arm.launch.py \
           left_can_port:=can2 right_can_port:=can1 \
           left_arm_type:=nero right_arm_type:=nero \
           left_effector_type:=none right_effector_type:=none

    2. Run this script from the pyroki root directory:
       python examples/nero_both_arms/dual_arm_control.py

    3. Use the GUI buttons to publish trajectories to the real robots
"""

import sys
import os
from pathlib import Path

# Add parent directory to path to import pyroki_snippets
sys.path.insert(0, str(Path(__file__).parent.parent))

import time
import numpy as np
import pyroki as pk
import viser
import yourdfpy
from viser.extras import ViserUrdf
from scipy.spatial.transform import Rotation as R

import pyroki_snippets as pks

# ROS2 imports (optional)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    ROS_AVAILABLE = True

    class DualArmROSController(Node):
        """ROS2 node to control both arms and receive feedback"""
        def __init__(self):
            super().__init__('pyroki_dual_arm_controller')

            # Publishers for both arms
            self.left_publisher = self.create_publisher(
                JointState,
                '/left_arm/control/move_j',
                10
            )
            self.right_publisher = self.create_publisher(
                JointState,
                '/right_arm/control/move_j',
                10
            )
            self.publish_rate = 100.0  # Hz - Matches Nero controller update_rate (see ros2_controllers.yaml)

            # Subscribe to both arms joint states (real robot feedback)
            self.left_current_joint_positions = None
            self.right_current_joint_positions = None

            self.left_subscription = self.create_subscription(
                JointState,
                '/left_arm/feedback/joint_states',
                self.left_joint_state_callback,
                10
            )

            self.right_subscription = self.create_subscription(
                JointState,
                '/right_arm/feedback/joint_states',
                self.right_joint_state_callback,
                10
            )

            self.get_logger().info('Dual Arm Controller initialized')
            self.get_logger().info('LEFT ARM: Publishing to /left_arm/control/move_j')
            self.get_logger().info('LEFT ARM: Subscribing to /left_arm/feedback/joint_states')
            self.get_logger().info('RIGHT ARM: Publishing to /right_arm/control/move_j')
            self.get_logger().info('RIGHT ARM: Subscribing to /right_arm/feedback/joint_states')

        def left_joint_state_callback(self, msg):
            """Store the latest left arm joint positions from the real robot"""
            # Extract only the arm joint positions (first 7 joints)
            if len(msg.position) >= 7:
                self.left_current_joint_positions = np.array(msg.position[:7])

        def right_joint_state_callback(self, msg):
            """Store the latest right arm joint positions from the real robot"""
            # Extract only the arm joint positions (first 7 joints)
            if len(msg.position) >= 7:
                self.right_current_joint_positions = np.array(msg.position[:7])

        def get_left_current_position(self):
            """Get current left arm joint positions"""
            if self.left_current_joint_positions is not None:
                return self.left_current_joint_positions.copy()
            else:
                return None

        def get_right_current_position(self):
            """Get current right arm joint positions"""
            if self.right_current_joint_positions is not None:
                return self.right_current_joint_positions.copy()
            else:
                return None

        def publish_left_trajectory(self, trajectory):
            """Publish trajectory to URDF left arm via ROS"""
            self.get_logger().info(f'Publishing {len(trajectory)} waypoints to LEFT ARM...')

            for i, positions in enumerate(trajectory):
                msg = JointState()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['joint1', 'joint2', 'joint3', 'joint4',
                           'joint5', 'joint6', 'joint7']
                msg.position = positions.tolist()
                msg.velocity = []
                msg.effort = []

                self.left_publisher.publish(msg)
                self.get_logger().info(f'Published LEFT ARM waypoint {i + 1}/{len(trajectory)}')
                time.sleep(1.0 / self.publish_rate)

            self.get_logger().info('LEFT ARM trajectory publishing complete!')

        def publish_right_trajectory(self, trajectory):
            """Publish trajectory to URDF right arm via ROS"""
            self.get_logger().info(f'Publishing {len(trajectory)} waypoints to RIGHT ARM...')

            for i, positions in enumerate(trajectory):
                msg = JointState()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['joint1', 'joint2', 'joint3', 'joint4',
                           'joint5', 'joint6', 'joint7']
                msg.position = positions.tolist()
                msg.velocity = []
                msg.effort = []

                self.right_publisher.publish(msg)
                self.get_logger().info(f'Published RIGHT ARM waypoint {i + 1}/{len(trajectory)}')
                time.sleep(1.0 / self.publish_rate)

            self.get_logger().info('RIGHT ARM trajectory publishing complete!')

        def publish_both_trajectories(self, visual_left_trajectory, visual_right_trajectory):
            """Publish trajectories to both arms simultaneously"""
            length = min(len(visual_left_trajectory), len(visual_right_trajectory))
            self.get_logger().info(f'Publishing {length} synchronized waypoints to BOTH ARMS...')

            for i in range(length):
                left_msg = JointState()
                left_msg.header = Header()
                left_msg.header.stamp = self.get_clock().now().to_msg()
                left_msg.name = ['joint1', 'joint2', 'joint3', 'joint4',
                               'joint5', 'joint6', 'joint7']
                left_msg.position = visual_left_trajectory[i].tolist()
                left_msg.velocity = []
                left_msg.effort = []

                right_msg = JointState()
                right_msg.header = Header()
                right_msg.header.stamp = self.get_clock().now().to_msg()
                right_msg.name = ['joint1', 'joint2', 'joint3', 'joint4',
                               'joint5', 'joint6', 'joint7']
                right_msg.position = visual_right_trajectory[i].tolist()
                right_msg.velocity = []
                right_msg.effort = []

                # Publish to both arms simultaneously
                self.left_publisher.publish(left_msg)
                self.right_publisher.publish(right_msg)

                self.get_logger().info(f'Published BOTH ARMS waypoint {i + 1}/{length}')
                time.sleep(1.0 / self.publish_rate)

            self.get_logger().info('BOTH ARMS trajectory publishing complete!')

except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS2 (rclpy) not available. ROS publishing features will be disabled.")

import threading


def main():
    """Generate trajectories and visualize dual arms (left with ROS, right sim-only)."""

    # Initialize ROS2 (if available)
    ros_node = None
    if ROS_AVAILABLE:
        rclpy.init()
        ros_node = DualArmROSController()

        # Spin ROS node in background thread
        ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
        ros_thread.start()

    # Load dual arm URDF
    urdf_path = "examples/nero_both_arms/antt_t1.urdf"
    print(f"Loading dual arm URDF from: {urdf_path}")

    urdf = yourdfpy.URDF.load(urdf_path)

    # Define end effector links for both arms
    left_end_effector = "left_link8"
    right_end_effector = "right_link8"

    # Create robot
    print("Creating robot model...")
    robot = pk.Robot.from_urdf(urdf)
    print(f"Robot has {len(robot.joints.names)} total joints")
    print(f"Joint names: {robot.joints.names}")

    # IMPORTANT MAPPING NOTE:
    # The URDF naming and visual positions are swapped:
    # - URDF "left_joint" names appear on the RIGHT side visually
    # - URDF "right_joint" names appear on the LEFT side visually
    #
    # To keep code clean, we use VISUAL position naming throughout:
    # - visual_left_* refers to what appears on the left in visualization
    # - visual_right_* refers to what appears on the right in visualization

    # Get URDF joint names
    urdf_left_joint_names = [name for name in robot.joints.names if 'left_joint' in name and name != 'left_joint8']
    urdf_right_joint_names = [name for name in robot.joints.names if 'right_joint' in name and name != 'right_joint8']

    # Map to visual positions (URDF left → visual right, URDF right → visual left)
    visual_right_joint_indices = [robot.joints.names.index(name) for name in urdf_left_joint_names]
    visual_left_joint_indices = [robot.joints.names.index(name) for name in urdf_right_joint_names]

    print(f"\nVisual LEFT arm (URDF right) joints (7-DOF): {urdf_right_joint_names}")
    print(f"Visual LEFT arm joint indices: {visual_left_joint_indices}")
    print(f"Visual RIGHT arm (URDF left) joints (7-DOF): {urdf_left_joint_names}")
    print(f"Visual RIGHT arm joint indices: {visual_right_joint_indices}")

    # IK solution structure: [urdf_left_joint1-7, urdf_right_joint1-7] (14 values)
    # Map to visual positions
    visual_right_ik_indices = list(range(0, 7))  # URDF left → visual right
    visual_left_ik_indices = list(range(7, 14))  # URDF right → visual left

    # =====================================================
    # DEFINE TARGET POINTS FOR BOTH ARMS (MIRRORED)
    # =====================================================

    print("\n=== Target Point Configuration (Bimanual - Same Orientation) ===")

    # SHARED ORIENTATION - Both arms use the SAME orientation regardless of position
    # This ensures both arms approach their targets in the same way
    shared_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion (w, x, y, z)

    # LEFT ARM TARGET - independent position
    left_target_x = 0.3
    left_target_y = 0.3
    left_target_z = 1.0
    left_target_position = np.array([left_target_x, left_target_y, left_target_z])
    left_target_orientation = shared_orientation  # Same orientation as right arm

    # RIGHT ARM TARGET - independent position (can be anywhere)
    right_target_x = -0.3
    right_target_y = 0.3
    right_target_z = 1.0
    right_target_position = np.array([right_target_x, right_target_y, right_target_z])
    right_target_orientation = shared_orientation  # Same orientation as left arm

    print(f"\nLEFT ARM target position (x, y, z): {left_target_position}")
    print(f"RIGHT ARM target position (x, y, z): {right_target_position}")
    print(f"SHARED orientation (w, x, y, z): {shared_orientation}")

    # =====================================================
    # SOLVE INVERSE KINEMATICS FOR BOTH ARMS SIMULTANEOUSLY
    # =====================================================

    print("\nSolving bimanual inverse kinematics (same orientation)...")

    # Use bimanual IK solver for simultaneous control with SAME orientation
    dual_target_config = pks.solve_ik_with_multiple_targets(
        robot=robot,
        target_link_names=[left_end_effector, right_end_effector],
        target_positions=np.array([left_target_position, right_target_position]),
        target_wxyzs=np.array([shared_orientation, shared_orientation]),
    )

    print(f"\n✓ DUAL ARM IK Solution (full): {dual_target_config}")

    # Extract only the arm joints (7-DOF) from the IK solution
    # IK solution contains only revolute joints: [urdf_left_joint1-7, urdf_right_joint1-7]
    # Map to visual positions: urdf_left → visual_right, urdf_right → visual_left
    visual_right_target_joints = np.array([dual_target_config[i] for i in visual_right_ik_indices])
    visual_left_target_joints = np.array([dual_target_config[i] for i in visual_left_ik_indices])

    print(f"\n✓ Visual LEFT ARM joints (7-DOF): {visual_left_target_joints}")
    print(f"✓ Visual RIGHT ARM joints (7-DOF): {visual_right_target_joints}")

    # Verify the achieved end-effector poses
    # Get link indices
    left_ee_idx = robot.links.names.index(left_end_effector)
    right_ee_idx = robot.links.names.index(right_end_effector)

    left_achieved_T = robot.forward_kinematics(dual_target_config, left_ee_idx)
    right_achieved_T = robot.forward_kinematics(dual_target_config, right_ee_idx)

    # Convert JAX arrays to numpy
    left_achieved_T = np.array(left_achieved_T)
    right_achieved_T = np.array(right_achieved_T)

    print(f"\n=== Verification: Achieved End-Effector Poses ===")
    print(f"LEFT ARM transformation matrix shape: {left_achieved_T.shape}")
    print(f"LEFT ARM transformation matrix:\n{left_achieved_T}")
    print(f"RIGHT ARM transformation matrix shape: {right_achieved_T.shape}")
    print(f"RIGHT ARM transformation matrix:\n{right_achieved_T}")

    # For now, just print the positions to verify the IK worked
    if left_achieved_T.shape == (4, 4):
        left_achieved_pos = left_achieved_T[:3, 3]
        right_achieved_pos = right_achieved_T[:3, 3]
        print(f"\nLEFT ARM achieved position: {left_achieved_pos}")
        print(f"RIGHT ARM achieved position: {right_achieved_pos}")
        print(f"Position difference (norm): {np.linalg.norm(left_achieved_pos - right_achieved_pos):.6f}")

    # =====================================================
    # GENERATE SMOOTH TRAJECTORIES FOR BOTH ARMS
    # =====================================================

    print("\nGenerating smooth trajectories for both arms...")

    timesteps = 200  # More steps for smoother trajectory
    dt = 0.01  # Matches controller update rate (100 Hz)

    # Start from current positions or zero
    if ROS_AVAILABLE and ros_node is not None:
        time.sleep(0.5)  # Wait for joint states
        left_start_config = ros_node.get_left_current_position()
        if left_start_config is not None:
            print(f"LEFT ARM: Starting from current robot position")
        else:
            left_start_config = np.zeros(7)
            print(f"LEFT ARM: No feedback yet, starting from home position")
    else:
        left_start_config = np.zeros(7)
        print(f"LEFT ARM: Starting from home position")

    # Right arm always starts from zero (simulation only)
    right_start_config = np.zeros(7)
    print(f"RIGHT ARM: Starting from home position")

    # Generate trajectories using minimum-jerk (quintic polynomial)
    def generate_trajectory(start_config, end_config, steps):
        trajectory = []
        for i in range(steps):
            s = i / (steps - 1)
            alpha = 10 * s**3 - 15 * s**4 + 6 * s**5
            current_config = (1 - alpha) * start_config + alpha * end_config
            trajectory.append(current_config)
        return np.array(trajectory)

    visual_left_trajectory = generate_trajectory(left_start_config, visual_left_target_joints, timesteps)
    visual_right_trajectory = generate_trajectory(right_start_config, visual_right_target_joints, timesteps)

    print(f"✓ Generated trajectories with {timesteps} steps")
    print(f"  Total time: {timesteps * dt} seconds")

    # =====================================================
    # SAVE TRAJECTORIES
    # =====================================================

    np.save("examples/nero_both_arms/left_arm_trajectory.npy", visual_left_trajectory)
    np.save("examples/nero_both_arms/right_arm_trajectory.npy", visual_right_trajectory)
    print(f"\n✓ Trajectories saved")

    # =====================================================
    # VISUALIZATION SETUP
    # =====================================================

    print("\nStarting dual arm visualizer...")
    server = viser.ViserServer()

    # We need to visualize:
    # 1. SIMULATION robot (green/transparent) - shows planned trajectory for BOTH arms
    # 2. REAL robot (original colors) - shows actual robot state from ROS feedback

    # Simulation robot (green/transparent) - shows planned trajectory
    urdf_sim = ViserUrdf(
        server,
        urdf,
        root_node_name="/robot_sim",
        mesh_color_override=(0.2, 0.8, 0.2, 0.7)  # Green with 70% opacity
    )

    # Real robot (original URDF colors/meshes) - shows actual robot state from ROS
    urdf_real = ViserUrdf(
        server,
        urdf,
        root_node_name="/robot_real"
    )

    server.scene.add_grid("/ground", width=3, height=3, cell_size=0.2)

    # Visualize target points as spheres
    left_target_sphere = server.scene.add_icosphere(
        "/left_target_point",
        radius=0.05,
        color=(0.0, 1.0, 0.0),  # Green
        position=tuple(left_target_position),
    )

    right_target_sphere = server.scene.add_icosphere(
        "/right_target_point",
        radius=0.05,
        color=(0.0, .0, 1.0),  # Blue
        position=tuple(right_target_position),
    )

    # Visualize target orientations as axes
    left_target_frame = server.scene.add_frame(
        "/left_target_frame",
        position=tuple(left_target_position),
        wxyz=tuple(left_target_orientation),
        axes_length=0.15,
        axes_radius=0.01,
    )

    right_target_frame = server.scene.add_frame(
        "/right_target_frame",
        position=tuple(right_target_position),
        wxyz=tuple(right_target_orientation),
        axes_length=0.15,
        axes_radius=0.01,
    )

    # =====================================================
    # GUI CONTROLS
    # =====================================================

    server.gui.add_text("=== Dual Arm Visualization (Same Orientation) ===", initial_value="", disabled=True)
    server.gui.add_text("Green/Transparent = Planned Trajectory (Sim)", initial_value="", disabled=True)
    server.gui.add_text("Original Colors = Actual Robot (ROS Feedback)", initial_value="", disabled=True)
    server.gui.add_text("BOTH ARMS: ROS enabled", initial_value="", disabled=True)
    server.gui.add_text("Both arms use SAME orientation at targets", initial_value="", disabled=True)

    # Left arm controls
    server.gui.add_text("=== LEFT ARM Controls ===", initial_value="", disabled=True)
    left_slider = server.gui.add_slider(
        "Left Step",
        min=0,
        max=len(visual_left_trajectory) - 1,
        step=1,
        initial_value=0,
    )

    # Right arm controls
    server.gui.add_text("=== RIGHT ARM Controls ===", initial_value="", disabled=True)
    right_slider = server.gui.add_slider(
        "Right Step",
        min=0,
        max=len(visual_right_trajectory) - 1,
        step=1,
        initial_value=0,
    )

    # Playback controls
    server.gui.add_text("=== Playback Controls ===", initial_value="", disabled=True)
    playing = server.gui.add_checkbox("Play Both", initial_value=False)
    speed = server.gui.add_slider("Speed", min=0.1, max=3.0, step=0.1, initial_value=1.0)
    loop = server.gui.add_checkbox("Loop", initial_value=True)

    # Left arm target sliders
    server.gui.add_text("=== LEFT ARM Target Position ===", initial_value="", disabled=True)
    left_target_x_slider = server.gui.add_slider(
        "Left Target X",
        min=-0.5,
        max=0.5,
        step=0.01,
        initial_value=left_target_x,
    )
    left_target_y_slider = server.gui.add_slider(
        "Left Target Y",
        min=-0.5,
        max=0.5,
        step=0.01,
        initial_value=left_target_y,
    )
    left_target_z_slider = server.gui.add_slider(
        "Left Target Z",
        min=0.3,
        max=1.5,
        step=0.01,
        initial_value=left_target_z,
    )

    # Right arm target sliders
    server.gui.add_text("=== RIGHT ARM Target Position ===", initial_value="", disabled=True)
    right_target_x_slider = server.gui.add_slider(
        "Right Target X",
        min=-0.5,
        max=0.5,
        step=0.01,
        initial_value=right_target_x,
    )
    right_target_y_slider = server.gui.add_slider(
        "Right Target Y",
        min=-0.5,
        max=0.5,
        step=0.01,
        initial_value=right_target_y,
    )
    right_target_z_slider = server.gui.add_slider(
        "Right Target Z",
        min=0.3,
        max=1.5,
        step=0.01,
        initial_value=right_target_z,
    )

    # Regenerate buttons
    regenerate_left_button = server.gui.add_button("Regenerate LEFT ARM Trajectory")
    regenerate_right_button = server.gui.add_button("Regenerate RIGHT ARM Trajectory")

    # ROS publishing for both arms
    if ROS_AVAILABLE:
        server.gui.add_text("=== ROS Control ===", initial_value="", disabled=True)
        publish_left_button = server.gui.add_button("Publish LEFT ARM to ROS")
        publish_right_button = server.gui.add_button("Publish RIGHT ARM to ROS")
        publish_both_button = server.gui.add_button("Publish BOTH ARMS to ROS")

        left_ros_status = server.gui.add_text("Left ROS Status", initial_value="Ready to publish", disabled=True)
        right_ros_status = server.gui.add_text("Right ROS Status", initial_value="Ready to publish", disabled=True)
        both_ros_status = server.gui.add_text("Both Arms ROS Status", initial_value="Ready to publish", disabled=True)

        left_real_robot_status = server.gui.add_text("Left Real Robot Status",
                                                     initial_value="Waiting for feedback...",
                                                     disabled=True)
        right_real_robot_status = server.gui.add_text("Right Real Robot Status",
                                                      initial_value="Waiting for feedback...",
                                                      disabled=True)

    status_text = server.gui.add_text("Status", initial_value="Ready", disabled=True)

    print(f"\n✓ Visualizer ready at http://localhost:8080")
    print("  - Blue sphere = left arm target")
    print("  - Green sphere = right arm target")
    print("  - Green/Transparent = Sim trajectory for BOTH arms")
    print("  - Original colors = Real robot feedback from ROS")

    if ROS_AVAILABLE:
        print("\n  ROS Control Options:")
        print("    - Click 'Publish LEFT ARM to ROS' to send left arm trajectory")
        print("    - Click 'Publish RIGHT ARM to ROS' to send right arm trajectory")
        print("    - Click 'Publish BOTH ARMS to ROS' to send synchronized dual-arm trajectory")

    # Track current trajectories
    current_visual_left_trajectory = visual_left_trajectory.copy()
    current_visual_right_trajectory = visual_right_trajectory.copy()

    # =====================================================
    # BUTTON CALLBACKS
    # =====================================================

    @regenerate_left_button.on_click
    def _(_):
        nonlocal current_visual_left_trajectory, current_visual_right_trajectory

        # Get new left target
        new_left_target = np.array([
            left_target_x_slider.value,
            left_target_y_slider.value,
            left_target_z_slider.value,
        ])

        # Keep right target as-is (no automatic mirroring)
        new_right_target = np.array([
            right_target_x_slider.value,
            right_target_y_slider.value,
            right_target_z_slider.value,
        ])

        status_text.value = "Solving BIMANUAL IK (same orientation)..."
        print(f"\nRegenerating trajectories with same orientation:")
        print(f"  LEFT:  {new_left_target}")
        print(f"  RIGHT: {new_right_target}")

        try:
            # Solve bimanual IK simultaneously - both arms use SAME orientation
            new_joint_config_full = pks.solve_ik_with_multiple_targets(
                robot=robot,
                target_link_names=[left_end_effector, right_end_effector],
                target_positions=np.array([new_left_target, new_right_target]),
                target_wxyzs=np.array([shared_orientation, shared_orientation]),
            )

            # Extract arm joints (7-DOF each) from IK solution
            # IK solution: [urdf_left(0-6), urdf_right(7-13)] → [visual_right, visual_left]
            urdf_left_config = np.array([new_joint_config_full[i] for i in visual_right_ik_indices])
            urdf_right_config = np.array([new_joint_config_full[i] for i in visual_left_ik_indices])

            # Get starting position from ROS feedback
            if ROS_AVAILABLE and ros_node is not None:
                urdf_right_start = ros_node.get_right_current_position()
                if urdf_right_start is None:
                    urdf_right_start = right_start_config
            else:
                urdf_right_start = right_start_config

            # Generate trajectories: urdf_right → visual_left, urdf_left → visual_right
            current_visual_left_trajectory = generate_trajectory(urdf_right_start, urdf_right_config, timesteps)
            current_visual_right_trajectory = generate_trajectory(left_start_config, urdf_left_config, timesteps)

            # Update visualization for left target
            left_target_sphere.position = tuple(new_left_target)
            left_target_frame.position = tuple(new_left_target)
            left_target_frame.wxyz = tuple(shared_orientation)  # Ensure same orientation

            # Also update right frame to ensure consistent shared orientation
            right_target_frame.wxyz = tuple(shared_orientation)

            # Reset sliders
            left_slider.value = 0
            left_slider.max = len(current_visual_left_trajectory) - 1
            right_slider.value = 0
            right_slider.max = len(current_visual_right_trajectory) - 1

            # Save
            np.save("examples/nero_both_arms/left_arm_trajectory.npy", current_visual_left_trajectory)
            np.save("examples/nero_both_arms/right_arm_trajectory.npy", current_visual_right_trajectory)

            status_text.value = "✓ Trajectories updated (same orientation)!"
            print("✓ Bimanual trajectories regenerated with same orientation!")

        except Exception as e:
            status_text.value = f"Error: {str(e)}"
            print(f"Error: {e}")

    @regenerate_right_button.on_click
    def _(_):
        nonlocal current_visual_left_trajectory, current_visual_right_trajectory

        # Get new right target
        new_right_target = np.array([
            right_target_x_slider.value,
            right_target_y_slider.value,
            right_target_z_slider.value,
        ])

        # Keep left target as-is (no automatic mirroring)
        new_left_target = np.array([
            left_target_x_slider.value,
            left_target_y_slider.value,
            left_target_z_slider.value,
        ])

        status_text.value = "Solving BIMANUAL IK (same orientation)..."
        print(f"\nRegenerating trajectories with same orientation:")
        print(f"  LEFT:  {new_left_target}")
        print(f"  RIGHT: {new_right_target}")

        try:
            # Solve bimanual IK simultaneously - both arms use SAME orientation
            new_joint_config_full = pks.solve_ik_with_multiple_targets(
                robot=robot,
                target_link_names=[left_end_effector, right_end_effector],
                target_positions=np.array([new_left_target, new_right_target]),
                target_wxyzs=np.array([shared_orientation, shared_orientation]),
            )

            # Extract arm joints (7-DOF each) from IK solution
            # IK solution: [urdf_left(0-6), urdf_right(7-13)] → [visual_right, visual_left]
            urdf_left_config = np.array([new_joint_config_full[i] for i in visual_right_ik_indices])
            urdf_right_config = np.array([new_joint_config_full[i] for i in visual_left_ik_indices])

            # Get starting position from ROS feedback
            if ROS_AVAILABLE and ros_node is not None:
                urdf_right_start = ros_node.get_right_current_position()
                if urdf_right_start is None:
                    urdf_right_start = right_start_config
            else:
                urdf_right_start = right_start_config

            # Generate trajectories: urdf_right → visual_left, urdf_left → visual_right
            current_visual_left_trajectory = generate_trajectory(urdf_right_start, urdf_right_config, timesteps)
            current_visual_right_trajectory = generate_trajectory(left_start_config, urdf_left_config, timesteps)

            # Update visualization for right target
            right_target_sphere.position = tuple(new_right_target)
            right_target_frame.position = tuple(new_right_target)
            right_target_frame.wxyz = tuple(shared_orientation)  # Ensure same orientation

            # Also update left frame to ensure consistent shared orientation
            left_target_frame.wxyz = tuple(shared_orientation)

            # Reset sliders
            left_slider.value = 0
            left_slider.max = len(current_visual_left_trajectory) - 1
            right_slider.value = 0
            right_slider.max = len(current_visual_right_trajectory) - 1

            # Save
            np.save("examples/nero_both_arms/left_arm_trajectory.npy", current_visual_left_trajectory)
            np.save("examples/nero_both_arms/right_arm_trajectory.npy", current_visual_right_trajectory)

            status_text.value = "✓ Trajectories updated (same orientation)!"
            print("✓ Bimanual trajectories regenerated with same orientation!")

        except Exception as e:
            status_text.value = f"Error: {str(e)}"
            print(f"Error: {e}")

    if ROS_AVAILABLE:
        @publish_left_button.on_click
        def _(_):
            """Publish trajectory to left arm"""
            left_ros_status.value = "Publishing LEFT ARM to ROS..."
            print("\n" + "="*50)
            print("Publishing LEFT ARM trajectory to ROS...")
            print("="*50)

            try:
                def publish_thread():
                    # Left arm gets visual left trajectory
                    ros_node.publish_left_trajectory(current_visual_left_trajectory)
                    left_ros_status.value = "✓ LEFT ARM published to ROS!"

                thread = threading.Thread(target=publish_thread, daemon=True)
                thread.start()

            except Exception as e:
                left_ros_status.value = f"ROS Error: {str(e)}"
                print(f"ROS Publishing Error: {e}")

        @publish_right_button.on_click
        def _(_):
            """Publish trajectory to right arm"""
            right_ros_status.value = "Publishing RIGHT ARM to ROS..."
            print("\n" + "="*50)
            print("Publishing RIGHT ARM trajectory to ROS...")
            print("="*50)

            try:
                def publish_thread():
                    # Right arm gets visual right trajectory
                    ros_node.publish_right_trajectory(current_visual_right_trajectory)
                    right_ros_status.value = "✓ RIGHT ARM published to ROS!"

                thread = threading.Thread(target=publish_thread, daemon=True)
                thread.start()

            except Exception as e:
                right_ros_status.value = f"ROS Error: {str(e)}"
                print(f"ROS Publishing Error: {e}")

        @publish_both_button.on_click
        def _(_):
            """Publish trajectories to both arms simultaneously"""
            both_ros_status.value = "Publishing BOTH ARMS to ROS..."
            left_ros_status.value = "Publishing..."
            right_ros_status.value = "Publishing..."
            print("\n" + "="*50)
            print("Publishing BOTH ARMS trajectories to ROS...")
            print("="*50)

            try:
                def publish_thread():
                    # Left arm gets visual left, right arm gets visual right
                    ros_node.publish_both_trajectories(current_visual_left_trajectory, current_visual_right_trajectory)
                    both_ros_status.value = "✓ BOTH ARMS published to ROS!"
                    left_ros_status.value = "✓ Complete"
                    right_ros_status.value = "✓ Complete"

                thread = threading.Thread(target=publish_thread, daemon=True)
                thread.start()

            except Exception as e:
                both_ros_status.value = f"ROS Error: {str(e)}"
                left_ros_status.value = "Error"
                right_ros_status.value = "Error"
                print(f"ROS Publishing Error: {e}")

    # =====================================================
    # MAIN VISUALIZATION LOOP
    # =====================================================

    # Get full robot configuration size
    full_config_size = len(robot.joints.names)

    while True:
        if playing.value:
            left_slider.value = left_slider.value + 1
            right_slider.value = right_slider.value + 1

            if left_slider.value >= len(current_visual_left_trajectory):
                if loop.value:
                    left_slider.value = 0
                else:
                    left_slider.value = len(current_visual_left_trajectory) - 1
                    playing.value = False

            if right_slider.value >= len(current_visual_right_trajectory):
                if loop.value:
                    right_slider.value = 0
                else:
                    right_slider.value = len(current_visual_right_trajectory) - 1

            time.sleep(dt / speed.value)
        else:
            time.sleep(0.01)

        # Create full robot configuration by combining left and right arm states
        # For SIMULATION robots (green/transparent)
        full_config_sim = np.zeros(full_config_size)

        # Set visual left arm (appears on left) using left trajectory and slider
        for i, idx in enumerate(visual_left_joint_indices):
            if i < len(current_visual_left_trajectory[left_slider.value]):
                full_config_sim[idx] = current_visual_left_trajectory[left_slider.value][i]

        # Set visual right arm (appears on right) using right trajectory and slider
        for i, idx in enumerate(visual_right_joint_indices):
            if i < len(current_visual_right_trajectory[right_slider.value]):
                full_config_sim[idx] = current_visual_right_trajectory[right_slider.value][i]

        # Update SIMULATION robot (green transparent) - shows planned trajectory for BOTH arms
        urdf_sim.update_cfg(full_config_sim)

        # Update REAL robot (original colors) - from ROS feedback for BOTH arms
        if ROS_AVAILABLE and ros_node is not None:
            urdf_left_real_pos = ros_node.get_left_current_position()
            urdf_right_real_pos = ros_node.get_right_current_position()

            full_config_real = np.zeros(full_config_size)

            # Left arm feedback → visual left arm position
            if urdf_left_real_pos is not None:
                for i, idx in enumerate(visual_left_joint_indices):
                    if i < len(urdf_left_real_pos):
                        full_config_real[idx] = urdf_left_real_pos[i]
                left_real_robot_status.value = f" Receiving LEFT ARM feedback"
            else:
                left_real_robot_status.value = " Waiting for LEFT ARM feedback..."

            # Right arm feedback → visual right arm position
            if urdf_right_real_pos is not None:
                for i, idx in enumerate(visual_right_joint_indices):
                    if i < len(urdf_right_real_pos):
                        full_config_real[idx] = urdf_right_real_pos[i]
                right_real_robot_status.value = f" Receiving RIGHT ARM feedback"
            else:
                right_real_robot_status.value = " Waiting for RIGHT ARM feedback..."

            urdf_real.update_cfg(full_config_real)
        else:
            # No ROS - keep real robot at zero position (not moving)
            urdf_real.update_cfg(np.zeros(full_config_size))


if __name__ == "__main__":
    main()
