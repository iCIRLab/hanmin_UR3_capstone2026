"""Shared obstacle geometry for MPlib, MoveIt 2, and MuJoCo checks."""

from dataclasses import dataclass
from typing import Iterable

import mplib
import numpy as np


@dataclass(frozen=True)
class BoxObstacle:
    """Axis-aligned box expressed in the common world frame."""

    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    planning_padding: float = 0.0

    @property
    def collision_size(self) -> tuple[float, float, float]:
        """Return the side lengths enlarged on both sides for planning."""
        return tuple(
            dimension + 2.0 * self.planning_padding
            for dimension in self.size
        )


# Full side lengths are used here. MuJoCo's matching XML uses half sizes.
WORKCELL_BOXES: tuple[BoxObstacle, ...] = (
    BoxObstacle(
        name='pickup_table_top',
        center=(-0.015, -0.015, 0.635),
        size=(0.90, 0.90, 0.15),
    ),
)


def add_mplib_workcell(planner: mplib.Planner) -> None:
    """Add the common static boxes to an MPlib planning world."""
    fcl = mplib.collision_detection.fcl
    world = planner.planning_world
    for obstacle in WORKCELL_BOXES:
        if world.has_object(obstacle.name):
            world.remove_object(obstacle.name)
        geometry = fcl.Box(*obstacle.collision_size)
        pose = mplib.Pose(p=np.asarray(obstacle.center, dtype=float))
        world.add_object(
            obstacle.name,
            fcl.CollisionObject(geometry, pose),
        )


def moveit_collision_objects(
    obstacles: Iterable[BoxObstacle] = WORKCELL_BOXES,
):
    """Build MoveIt CollisionObject messages for the common workcell."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    messages = []
    for obstacle in obstacles:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(obstacle.collision_size)

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = obstacle.center
        pose.orientation.w = 1.0

        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = obstacle.name
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        messages.append(collision_object)
    return messages


def moveit_planning_scene():
    """Build an atomic PlanningScene diff containing the workcell boxes."""
    from moveit_msgs.msg import PlanningScene

    scene = PlanningScene()
    scene.name = 'capstone_ur3_workcell'
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = moveit_collision_objects()
    return scene
