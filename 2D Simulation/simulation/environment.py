import pybullet as p
import pybullet_data
import time
import os
import math


class SoccerEnvironment:
    def __init__(self, render=True):
        # Connect to PyBullet
        self.physics_client = p.connect(p.GUI if render else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        
        # Load field and textures
        self._setup_field()
        
        # Load ball
        self.ball_id = self._setup_ball()
        
        # Add robots
        self.robots = []
        self._add_robots()
        
        # Setup camera
        self._setup_camera()

    def _setup_field(self):
        """Set up the RoboCup Kid Size field."""
        self.plane_id = p.loadURDF("plane.urdf")

        assets_path = os.path.join(os.path.dirname(__file__), "assets")
        texture_path = os.path.join(assets_path, "soccer_field_texture.jpg")
    
        # Check if the texture file exists
        if not os.path.exists(texture_path):
            raise FileNotFoundError(f"Texture file not found: {texture_path}")
    
        field_texture = p.loadTexture(texture_path)  # Replace with a valid field texture file
        p.changeVisualShape(self.plane_id, -1, textureUniqueId=field_texture)

        # Draw field lines
        self._draw_field_lines()

    def _draw_field_lines(self):
        """Draw field markings for the RoboCup Kid Size field."""
        # Field dimensions in meters
        field_length = 9  # Full length of the field
        field_width = 6   # Full width of the field
        line_width = 1  # Thickness of the lines

        def draw_line(start, end):
            """Helper function to draw a single line."""
            p.addUserDebugLine(start, end, [1, 1, 1], line_width)

        # Boundary lines (outer rectangle)
        top_left = [-field_length / 2, field_width / 2, 1]
        top_right = [field_length / 2, field_width / 2, 1]
        bottom_left = [-field_length / 2, -field_width / 2, 1]
        bottom_right = [field_length / 2, -field_width / 2, 1]

        draw_line(top_left, top_right)  # Top boundary
        draw_line(top_right, bottom_right)  # Right boundary
        draw_line(bottom_right, bottom_left)  # Bottom boundary
        draw_line(bottom_left, top_left)  # Left boundary

        # Center line
        draw_line([0, field_width / 2, 1], [0, -field_width / 2, 1])

        # Goal areas (rectangles)
        goal_area_length = 2  # Length of the goal area (along the field length)
        goal_area_width = 1   # Width of the goal area (perpendicular to the field length)

    # Left goal area
        draw_line(
            [-field_length / 2, goal_area_width / 2, 1],
            [-field_length / 2 + goal_area_length, goal_area_width / 2, 1],
        )  # Top of left goal area
        draw_line(
            [-field_length / 2 + goal_area_length, goal_area_width / 2, 1],
            [-field_length / 2 + goal_area_length, -goal_area_width / 2, 1],
        )  # Right of left goal area
        draw_line(
            [-field_length / 2 + goal_area_length, -goal_area_width / 2, 1],
            [-field_length / 2, -goal_area_width / 2, 1],
        )  # Bottom of left goal area

    # Right goal area
        draw_line(
            [field_length / 2, goal_area_width / 2, 1],
            [field_length / 2 - goal_area_length, goal_area_width / 2, 1],
        )  # Top of right goal area
        draw_line(
            [field_length / 2 - goal_area_length, goal_area_width / 2, 1],
            [field_length / 2 - goal_area_length, -goal_area_width / 2, 1],
        )  # Left of right goal area
        draw_line(
            [field_length / 2 - goal_area_length, -goal_area_width / 2, 1],
            [field_length / 2, -goal_area_width / 2, 1],
        )  # Bottom of right goal area

    def _setup_ball(self):
        """Add a soccer ball to the field."""
        ball_radius = 0.11
        ball_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        ball_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 0, 0, 1])  # Red ball
        return p.createMultiBody(
            baseMass=0.43,
            baseCollisionShapeIndex=ball_collision_shape,
            baseVisualShapeIndex=ball_visual_shape,
            basePosition=[0, 0, ball_radius + 0.1],  # Position adjusted to prevent clipping
        )

    def _add_robots(self):
        """Add robots to the field."""
        assets_path = os.path.join(os.path.dirname(__file__), "assets")
        print(f"Assets Path: {assets_path}")
        positions = [[-2, 1, 0.1], [-2, -1, 0.1], [2, 1, 0.1], [2, -1, 0.1]]
        for pos in positions:
            try:
                robot_id = p.loadURDF("r2d2.urdf", pos, p.getQuaternionFromEuler([0, 0, 0]))
                self.robots.append(robot_id)
            except Exception as e:
                print(f"Error loading robot: {e}")

    def _setup_camera(self):
        """Set up a dynamic camera for the simulation."""
        self.camera_distance = 10
        self.camera_yaw = 0
        self.camera_pitch = -40
        self.camera_target_position = [0, 0, 0]

    def _update_camera(self):
        """Update the camera position dynamically."""
        p.resetDebugVisualizerCamera(
            self.camera_distance,
            self.camera_yaw,
            self.camera_pitch,
            self.camera_target_position,
        )

    def step_simulation(self):
        """Step the simulation forward."""
        p.stepSimulation()
        self._update_camera()
        self._detect_collisions()
        time.sleep(1 / 240)

    def _detect_collisions(self):
        """Log collisions between the ball and other objects."""
        contact_points = p.getContactPoints()
        for contact in contact_points:
            if self.ball_id in [contact[1], contact[2]]:  # Check if the ball is involved
                print(f"Collision detected: {contact}")

    def reset(self):
        """Reset the environment."""
        p.resetSimulation()
        self.__init__()

    def close(self):
        """Disconnect the simulation."""
        p.disconnect()
