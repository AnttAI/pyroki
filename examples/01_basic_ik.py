"""Basic IK for Unitree G1 Humanoid

Inverse Kinematics example using PyRoki with the Unitree G1 humanoid robot.
This example demonstrates IK control with 4 end-effectors: both hands and both feet.
"""

import time
from pathlib import Path

import numpy as np
import pyroki as pk
import viser
import yourdfpy
from viser.extras import ViserUrdf

import pyroki_snippets as pks


def main():
    """Main function for basic IK with Unitree G1."""

    # Load the custom G1 URDF
    urdf_path = Path(__file__).parent.parent / "robot_descriptions" / "unitree_g1" / "g1_custom_collision_29dof.urdf"

    if not urdf_path.exists():
        raise FileNotFoundError(
            f"G1 URDF not found at {urdf_path}. "
            "Please ensure the URDF and meshes are copied to robot_descriptions/unitree_g1/"
        )

    # Load URDF using yourdfpy
    urdf = yourdfpy.URDF.load(str(urdf_path))

    # Target links: both hands and both feet
    target_link_names = [
        "right_rubber_hand",  # Right hand
        "left_rubber_hand",   # Left hand
        "right_ankle_roll_link",  # Right foot
        "left_ankle_roll_link",   # Left foot
    ]

    # Create robot.
    robot = pk.Robot.from_urdf(urdf)

    print(f"Loaded G1 robot with {len(robot.joints.actuated_names)} actuated joints")
    print(f"Target links: {target_link_names}")

    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    # Create interactive controllers for standing pose
    # Position targets at reasonable standing pose locations
    ik_target_right_hand = server.scene.add_transform_controls(
        "/ik_target_right_hand", scale=0.15, position=(0.1, -0.25, 0.9), wxyz=(1, 0, 0, 0)
    )
    ik_target_left_hand = server.scene.add_transform_controls(
        "/ik_target_left_hand", scale=0.15, position=(0.1, 0.25, 0.9), wxyz=(1, 0, 0, 0)
    )
    ik_target_right_foot = server.scene.add_transform_controls(
        "/ik_target_right_foot", scale=0.15, position=(0.0, -0.088, 0.0), wxyz=(1, 0, 0, 0)
    )
    ik_target_left_foot = server.scene.add_transform_controls(
        "/ik_target_left_foot", scale=0.15, position=(0.0, 0.088, 0.0), wxyz=(1, 0, 0, 0)
    )

    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    print("\nVisualization server running. Open browser to view the IK control.")
    print("Move the transform controls to update the G1's hands and feet poses.")
    print("Press Ctrl+C to stop.")

    while True:
        # Solve IK for all 4 end-effectors.
        start_time = time.time()
        solution = pks.solve_ik_with_multiple_targets(
            robot=robot,
            target_link_names=target_link_names,
            target_positions=np.array([
                ik_target_right_hand.position,
                ik_target_left_hand.position,
                ik_target_right_foot.position,
                ik_target_left_foot.position,
            ]),
            target_wxyzs=np.array([
                ik_target_right_hand.wxyz,
                ik_target_left_hand.wxyz,
                ik_target_right_foot.wxyz,
                ik_target_left_foot.wxyz,
            ]),
        )

        # Update timing handle.
        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # Update visualizer.
        urdf_vis.update_cfg(solution)


if __name__ == "__main__":
    main()
