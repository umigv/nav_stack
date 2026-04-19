from __future__ import annotations

import math
from dataclasses import dataclass, field

from geometry_msgs.msg import Point, Pose, Quaternion


@dataclass
class Rotation2d:
    """A 2D rotation represented as a yaw angle in radians.

    The angle is automatically wrapped to [-π, π] on construction. cos and sin
    are cached to avoid redundant trigonometry when rotating multiple points by
    the same rotation.

    Attributes:
        angle: Yaw angle in radians, wrapped to [-π, π].
    """

    angle: float
    _cos: float = field(init=False, repr=False, compare=False)
    _sin: float = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        self.angle = (self.angle + math.pi) % (2 * math.pi) - math.pi
        self._cos = math.cos(self.angle)
        self._sin = math.sin(self.angle)

    @property
    def cos(self) -> float:
        """Cached cosine of the angle."""
        return self._cos

    @property
    def sin(self) -> float:
        """Cached sine of the angle."""
        return self._sin

    def __neg__(self) -> Rotation2d:
        """Return the inverse rotation."""
        return Rotation2d(-self.angle)

    def __add__(self, other: Rotation2d) -> Rotation2d:
        """Compose two rotations by adding their angles."""
        return Rotation2d(self.angle + other.angle)

    def __sub__(self, other: Rotation2d) -> Rotation2d:
        """Return the relative rotation from other to self."""
        return Rotation2d(self.angle - other.angle)

    def __mul__(self, scalar: float) -> Rotation2d:
        """Scale this rotation's angle by a scalar."""
        return Rotation2d(self.angle * scalar)

    def __rmul__(self, scalar: float) -> Rotation2d:
        """Scale this rotation's angle by a scalar (scalar * rotation)."""
        return Rotation2d(self.angle * scalar)

    def __truediv__(self, scalar: float) -> Rotation2d:
        """Divide this rotation's angle by a scalar."""
        return Rotation2d(self.angle / scalar)

    def to_ros(self) -> Quaternion:
        """Convert to a pure-yaw ROS quaternion.

        Returns:
            Quaternion with x=0, y=0, z=sin(angle/2), w=cos(angle/2).
        """
        half = self.angle / 2.0
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    @classmethod
    def from_ros(cls, q: Quaternion) -> Rotation2d:
        """Construct from a ROS quaternion by extracting the yaw component.

        Args:
            q: ROS quaternion message.

        Returns:
            Rotation2d with angle equal to the yaw of q.
        """
        return cls(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


@dataclass
class Point2d:
    """A 2D point with arithmetic operators and ROS interop.

    Attributes:
        x: X component in meters.
        y: Y component in meters.
    """

    x: float
    y: float

    def __neg__(self) -> Point2d:
        """Negate both components."""
        return Point2d(x=-self.x, y=-self.y)

    def __add__(self, other: Point2d) -> Point2d:
        """Add two points component-wise."""
        return Point2d(x=self.x + other.x, y=self.y + other.y)

    def __sub__(self, other: Point2d) -> Point2d:
        """Subtract two points component-wise."""
        return Point2d(x=self.x - other.x, y=self.y - other.y)

    def __mul__(self, scalar: float) -> Point2d:
        """Scale both components by a scalar."""
        return Point2d(x=self.x * scalar, y=self.y * scalar)

    def __rmul__(self, scalar: float) -> Point2d:
        """Scale both components by a scalar (scalar * point)."""
        return Point2d(x=self.x * scalar, y=self.y * scalar)

    def __truediv__(self, scalar: float) -> Point2d:
        """Divide both components by a scalar."""
        return Point2d(x=self.x / scalar, y=self.y / scalar)

    def rotate_by(self, rotation: Rotation2d) -> Point2d:
        """Rotate this point about the origin by a rotation.

        Args:
            rotation: Rotation to apply.

        Returns:
            Rotated point.
        """
        return Point2d(
            x=rotation.cos * self.x - rotation.sin * self.y,
            y=rotation.sin * self.x + rotation.cos * self.y,
        )

    def mag(self) -> float:
        """Return the Euclidean distance from the origin.

        Returns:
            Magnitude of this point as a vector.
        """
        return math.hypot(self.x, self.y)

    def distance(self, other: Point2d) -> float:
        """Return the Euclidean distance to another point.

        Args:
            other: The other point.

        Returns:
            Distance in meters.
        """
        return math.hypot(self.x - other.x, self.y - other.y)

    def dot(self, other: Point2d) -> float:
        """Return the dot product of this point with another, treating both as vectors.

        Args:
            other: The other vector.

        Returns:
            Scalar dot product.
        """
        return self.x * other.x + self.y * other.y

    def lerp(self, other: Point2d, t: float) -> Point2d:
        """Linearly interpolate toward another point.

        Args:
            other: Target point.
            t: Interpolation parameter. t=0 returns self, t=1 returns other. Not clamped.

        Returns:
            self + (other - self) * t.
        """
        return self + (other - self) * t

    def to_ros(self) -> Point:
        """Convert to a ROS Point with z=0.

        Returns:
            geometry_msgs/Point with x, y copied and z=0.
        """
        return Point(x=self.x, y=self.y, z=0.0)

    @classmethod
    def from_ros(cls, point: Point) -> Point2d:
        """Construct from a ROS Point, discarding z.

        Args:
            point: ROS Point message.

        Returns:
            Point2d with x and y copied from point.
        """
        return cls(x=point.x, y=point.y)


@dataclass
class Pose2d:
    """A 2D pose consisting of a position and a rotation.

    Attributes:
        point: Position in meters.
        rotation: Heading as a Rotation2d.
    """

    point: Point2d
    rotation: Rotation2d

    def world_to_local(self, world_point: Point2d) -> Point2d:
        """Transform a world-frame point into this pose's local frame.

        Args:
            world_point: Point in the world frame.

        Returns:
            Point expressed in this pose's local frame.
        """
        return (world_point - self.point).rotate_by(-self.rotation)

    def local_to_world(self, local_point: Point2d) -> Point2d:
        """Transform a local-frame point back into the world frame.

        Args:
            local_point: Point in this pose's local frame.

        Returns:
            Point expressed in the world frame.
        """
        return local_point.rotate_by(self.rotation) + self.point

    def to_ros(self) -> Pose:
        """Convert to a ROS Pose.

        Returns:
            geometry_msgs/Pose with position from point and orientation from rotation.
        """
        return Pose(
            position=self.point.to_ros(),
            orientation=self.rotation.to_ros(),
        )

    @classmethod
    def from_ros(cls, pose: Pose) -> Pose2d:
        """Construct from a ROS Pose.

        Args:
            pose: ROS Pose message.

        Returns:
            Pose2d with position and rotation extracted from pose.
        """
        return cls(
            point=Point2d.from_ros(pose.position),
            rotation=Rotation2d.from_ros(pose.orientation),
        )
