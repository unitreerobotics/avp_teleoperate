# Code from yourdfpy with small modification for deprecated warning
# Source: https://github.com/clemense/yourdfpy/blob/main/src/yourdfpy/urdf.py

import copy
import logging
import os
from dataclasses import dataclass, field, is_dataclass
from functools import partial
from typing import Dict, List, Optional, Union

import anytree
import numpy as np
import six
import trimesh
import trimesh.transformations as tra
from anytree import Node, LevelOrderIter
from lxml import etree

_logger = logging.getLogger(__name__)


def _array_eq(arr1, arr2):
    if arr1 is None and arr2 is None:
        return True
    return (
        isinstance(arr1, np.ndarray)
        and isinstance(arr2, np.ndarray)
        and arr1.shape == arr2.shape
        and (arr1 == arr2).all()
    )


@dataclass(eq=False)
class TransmissionJoint:
    name: str
    hardware_interfaces: List[str] = field(default_factory=list)

    def __eq__(self, other):
        if not isinstance(other, TransmissionJoint):
            return NotImplemented
        return (
            self.name == other.name
            and all(self_hi in other.hardware_interfaces for self_hi in self.hardware_interfaces)
            and all(other_hi in self.hardware_interfaces for other_hi in other.hardware_interfaces)
        )


@dataclass(eq=False)
class Actuator:
    name: str
    mechanical_reduction: Optional[float] = None
    # The follwing is only valid for ROS Indigo and prior versions
    hardware_interfaces: List[str] = field(default_factory=list)

    def __eq__(self, other):
        if not isinstance(other, Actuator):
            return NotImplemented
        return (
            self.name == other.name
            and self.mechanical_reduction == other.mechanical_reduction
            and all(self_hi in other.hardware_interfaces for self_hi in self.hardware_interfaces)
            and all(other_hi in self.hardware_interfaces for other_hi in other.hardware_interfaces)
        )


@dataclass(eq=False)
class Transmission:
    name: str
    type: Optional[str] = None
    joints: List[TransmissionJoint] = field(default_factory=list)
    actuators: List[Actuator] = field(default_factory=list)

    def __eq__(self, other):
        if not isinstance(other, Transmission):
            return NotImplemented
        return (
            self.name == other.name
            and self.type == other.type
            and all(self_joint in other.joints for self_joint in self.joints)
            and all(other_joint in self.joints for other_joint in other.joints)
            and all(self_actuator in other.actuators for self_actuator in self.actuators)
            and all(other_actuator in self.actuators for other_actuator in other.actuators)
        )


@dataclass
class Calibration:
    rising: Optional[float] = None
    falling: Optional[float] = None


@dataclass
class Mimic:
    joint: str
    multiplier: Optional[float] = None
    offset: Optional[float] = None


@dataclass
class SafetyController:
    soft_lower_limit: Optional[float] = None
    soft_upper_limit: Optional[float] = None
    k_position: Optional[float] = None
    k_velocity: Optional[float] = None


@dataclass
class Sphere:
    radius: float


@dataclass
class Cylinder:
    radius: float
    length: float


@dataclass(eq=False)
class Box:
    size: np.ndarray

    def __eq__(self, other):
        if not isinstance(other, Box):
            return NotImplemented
        return _array_eq(self.size, other.size)


@dataclass(eq=False)
class Mesh:
    filename: str
    scale: Optional[Union[float, np.ndarray]] = None

    def __eq__(self, other):
        if not isinstance(other, Mesh):
            return NotImplemented

        if self.filename != other.filename:
            return False

        if isinstance(self.scale, float) and isinstance(other.scale, float):
            return self.scale == other.scale

        return _array_eq(self.scale, other.scale)


@dataclass
class Geometry:
    box: Optional[Box] = None
    cylinder: Optional[Cylinder] = None
    sphere: Optional[Sphere] = None
    mesh: Optional[Mesh] = None


@dataclass(eq=False)
class Color:
    rgba: np.ndarray

    def __eq__(self, other):
        if not isinstance(other, Color):
            return NotImplemented
        return _array_eq(self.rgba, other.rgba)


@dataclass
class Texture:
    filename: str


@dataclass
class Material:
    name: Optional[str] = None
    color: Optional[Color] = None
    texture: Optional[Texture] = None


@dataclass(eq=False)
class Visual:
    name: Optional[str] = None
    origin: Optional[np.ndarray] = None
    geometry: Optional[Geometry] = None  # That's not really optional according to ROS
    material: Optional[Material] = None

    def __eq__(self, other):
        if not isinstance(other, Visual):
            return NotImplemented
        return (
            self.name == other.name
            and _array_eq(self.origin, other.origin)
            and self.geometry == other.geometry
            and self.material == other.material
        )


@dataclass(eq=False)
class Collision:
    name: str
    origin: Optional[np.ndarray] = None
    geometry: Geometry = None

    def __eq__(self, other):
        if not isinstance(other, Collision):
            return NotImplemented
        return self.name == other.name and _array_eq(self.origin, other.origin) and self.geometry == other.geometry


@dataclass(eq=False)
class Inertial:
    origin: Optional[np.ndarray] = None
    mass: Optional[float] = None
    inertia: Optional[np.ndarray] = None

    def __eq__(self, other):
        if not isinstance(other, Inertial):
            return NotImplemented
        return (
            _array_eq(self.origin, other.origin) and self.mass == other.mass and _array_eq(self.inertia, other.inertia)
        )


@dataclass(eq=False)
class Link:
    name: str
    inertial: Optional[Inertial] = None
    visuals: List[Visual] = field(default_factory=list)
    collisions: List[Collision] = field(default_factory=list)

    def __eq__(self, other):
        if not isinstance(other, Link):
            return NotImplemented
        return (
            self.name == other.name
            and self.inertial == other.inertial
            and all(self_visual in other.visuals for self_visual in self.visuals)
            and all(other_visual in self.visuals for other_visual in other.visuals)
            and all(self_collision in other.collisions for self_collision in self.collisions)
            and all(other_collision in self.collisions for other_collision in other.collisions)
        )


@dataclass
class Dynamics:
    damping: Optional[float] = None
    friction: Optional[float] = None


@dataclass
class Limit:
    effort: Optional[float] = None
    velocity: Optional[float] = None
    lower: Optional[float] = None
    upper: Optional[float] = None


@dataclass(eq=False)
class Joint:
    name: str
    type: str = None
    parent: str = None
    child: str = None
    origin: np.ndarray = None
    axis: np.ndarray = None
    dynamics: Optional[Dynamics] = None
    limit: Optional[Limit] = None
    mimic: Optional[Mimic] = None
    calibration: Optional[Calibration] = None
    safety_controller: Optional[SafetyController] = None

    def __eq__(self, other):
        if not isinstance(other, Joint):
            return NotImplemented
        return (
            self.name == other.name
            and self.type == other.type
            and self.parent == other.parent
            and self.child == other.child
            and _array_eq(self.origin, other.origin)
            and _array_eq(self.axis, other.axis)
            and self.dynamics == other.dynamics
            and self.limit == other.limit
            and self.mimic == other.mimic
            and self.calibration == other.calibration
            and self.safety_controller == other.safety_controller
        )


@dataclass(eq=False)
class Robot:
    name: str
    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)
    materials: List[Material] = field(default_factory=list)
    transmission: List[str] = field(default_factory=list)
    gazebo: List[str] = field(default_factory=list)

    def __eq__(self, other):
        if not isinstance(other, Robot):
            return NotImplemented
        return (
            self.name == other.name
            and all(self_link in other.links for self_link in self.links)
            and all(other_link in self.links for other_link in other.links)
            and all(self_joint in other.joints for self_joint in self.joints)
            and all(other_joint in self.joints for other_joint in other.joints)
            and all(self_material in other.materials for self_material in self.materials)
            and all(other_material in self.materials for other_material in other.materials)
            and all(self_transmission in other.transmission for self_transmission in self.transmission)
            and all(other_transmission in self.transmission for other_transmission in other.transmission)
            and all(self_gazebo in other.gazebo for self_gazebo in self.gazebo)
            and all(other_gazebo in self.gazebo for other_gazebo in other.gazebo)
        )


class URDFError(Exception):
    """General URDF exception."""

    def __init__(self, msg):
        super(URDFError, self).__init__()
        self.msg = msg

    def __str__(self):
        return type(self).__name__ + ": " + self.msg

    def __repr__(self):
        return type(self).__name__ + '("' + self.msg + '")'


class URDFIncompleteError(URDFError):
    """Raised when needed data for an object isn't there."""

    pass


class URDFAttributeValueError(URDFError):
    """Raised when attribute value is not contained in the set of allowed values."""

    pass


class URDFBrokenRefError(URDFError):
    """Raised when a referenced object is not found in the scope."""

    pass


class URDFMalformedError(URDFError):
    """Raised when data is found to be corrupted in some way."""

    pass


class URDFUnsupportedError(URDFError):
    """Raised when some unexpectedly unsupported feature is found."""

    pass


class URDFSaveValidationError(URDFError):
    """Raised when XML validation fails when saving."""

    pass


def _str2float(s):
    """Cast string to float if it is not None. Otherwise return None.

    Args:
        s (str): String to convert or None.

    Returns:
        str or NoneType: The converted string or None.
    """
    return float(s) if s is not None else None


def apply_visual_color(
    geom: trimesh.Trimesh,
    visual: Visual,
    material_map: Dict[str, Material],
) -> None:
    """Apply the color of the visual material to the mesh.

    Args:
        geom: Trimesh to color.
        visual: Visual description from XML.
        material_map: Dictionary mapping material names to their definitions.
    """
    if visual.material is None:
        return

    if visual.material.color is not None:
        color = visual.material.color
    elif visual.material.name is not None and visual.material.name in material_map:
        color = material_map[visual.material.name].color
    else:
        return

    if color is None:
        return
    if isinstance(geom.visual, trimesh.visual.ColorVisuals):
        geom.visual.face_colors[:] = [int(255 * channel) for channel in color.rgba]


def filename_handler_null(fname):
    """A lazy filename handler that simply returns its input.

    Args:
        fname (str): A file name.

    Returns:
        str: Same file name.
    """
    return fname


def filename_handler_ignore_directive(fname):
    """A filename handler that removes anything before (and including) '://'.

    Args:
        fname (str): A file name.

    Returns:
        str: The file name without the prefix.
    """
    if "://" in fname or ":\\\\" in fname:
        return ":".join(fname.split(":")[1:])[2:]
    return fname


def filename_handler_ignore_directive_package(fname):
    """A filename handler that removes the 'package://' directive and the package it refers to.
    It subsequently calls filename_handler_ignore_directive, i.e., it removes any other directive.

    Args:
        fname (str): A file name.

    Returns:
        str: The file name without 'package://' and the package name.
    """
    if fname.startswith("package://"):
        string_length = len("package://")
        return os.path.join(*os.path.normpath(fname[string_length:]).split(os.path.sep)[1:])
    return filename_handler_ignore_directive(fname)


def filename_handler_add_prefix(fname, prefix):
    """A filename handler that adds a prefix.

    Args:
        fname (str): A file name.
        prefix (str): A prefix.

    Returns:
        str: Prefix plus file name.
    """
    return prefix + fname


def filename_handler_absolute2relative(fname, dir):
    """A filename handler that turns an absolute file name into a relative one.

    Args:
        fname (str): A file name.
        dir (str): A directory.

    Returns:
        str: The file name relative to the directory.
    """
    # TODO: that's not right
    if fname.startswith(dir):
        return fname[len(dir) :]
    return fname


def filename_handler_relative(fname, dir):
    """A filename handler that joins a file name with a directory.

    Args:
        fname (str): A file name.
        dir (str): A directory.

    Returns:
        str: The directory joined with the file name.
    """
    return os.path.join(dir, filename_handler_ignore_directive_package(fname))


def filename_handler_relative_to_urdf_file(fname, urdf_fname):
    return filename_handler_relative(fname, os.path.dirname(urdf_fname))


def filename_handler_relative_to_urdf_file_recursive(fname, urdf_fname, level=0):
    if level == 0:
        return filename_handler_relative_to_urdf_file(fname, urdf_fname)
    return filename_handler_relative_to_urdf_file_recursive(fname, os.path.split(urdf_fname)[0], level=level - 1)


def _create_filename_handlers_to_urdf_file_recursive(urdf_fname):
    return [
        partial(
            filename_handler_relative_to_urdf_file_recursive,
            urdf_fname=urdf_fname,
            level=i,
        )
        for i in range(len(os.path.normpath(urdf_fname).split(os.path.sep)))
    ]


def filename_handler_meta(fname, filename_handlers):
    """A filename handler that calls other filename handlers until the resulting file name points to an existing file.

    Args:
        fname (str): A file name.
        filename_handlers (list(fn)): A list of function pointers to filename handlers.

    Returns:
        str: The resolved file name that points to an existing file or the input if none of the files exists.
    """
    for fn in filename_handlers:
        candidate_fname = fn(fname=fname)
        _logger.debug(f"Checking filename: {candidate_fname}")
        if os.path.isfile(candidate_fname):
            return candidate_fname
    _logger.warning(f"Unable to resolve filename: {fname}")
    return fname


def filename_handler_magic(fname, dir):
    """A magic filename handler.

    Args:
        fname (str): A file name.
        dir (str): A directory.

    Returns:
        str: The file name that exists or the input if nothing is found.
    """
    return filename_handler_meta(
        fname=fname,
        filename_handlers=[
            partial(filename_handler_relative, dir=dir),
            filename_handler_ignore_directive,
        ]
        + _create_filename_handlers_to_urdf_file_recursive(urdf_fname=dir),
    )


def validation_handler_strict(errors):
    """A validation handler that does not allow any errors.

    Args:
        errors (list[yourdfpy.URDFError]): List of errors.

    Returns:
        bool: Whether any errors were found.
    """
    return len(errors) == 0


class URDF:
    def __init__(
        self,
        robot: Robot = None,
        build_scene_graph: bool = True,
        build_collision_scene_graph: bool = False,
        load_meshes: bool = True,
        load_collision_meshes: bool = False,
        filename_handler=None,
        mesh_dir: str = "",
        force_mesh: bool = False,
        force_collision_mesh: bool = True,
        build_tree: bool = False,
    ):
        """A URDF model.

        Args:
            robot (Robot): The robot model. Defaults to None.
            build_scene_graph (bool, optional): Wheter to build a scene graph to enable transformation queries and forward kinematics. Defaults to True.
            build_collision_scene_graph (bool, optional): Wheter to build a scene graph for <collision> elements. Defaults to False.
            load_meshes (bool, optional): Whether to load the meshes referenced in the <mesh> elements. Defaults to True.
            load_collision_meshes (bool, optional): Whether to load the collision meshes referenced in the <mesh> elements. Defaults to False.
            filename_handler ([type], optional): Any function f(in: str) -> str, that maps filenames in the URDF to actual resources. Can be used to customize treatment of `package://` directives or relative/absolute filenames. Defaults to None.
            mesh_dir (str, optional): A root directory used for loading meshes. Defaults to "".
            force_mesh (bool, optional): Each loaded geometry will be concatenated into a single one (instead of being turned into a graph; in case the underlying file contains multiple geometries). This might loose texture information but the resulting scene graph will be smaller. Defaults to False.
            force_collision_mesh (bool, optional): Same as force_mesh, but for collision scene. Defaults to True.
            build_tree (bool, optional): Build the tree structure for global kinematics computation
        """
        if filename_handler is None:
            self._filename_handler = partial(filename_handler_magic, dir=mesh_dir)
        else:
            self._filename_handler = filename_handler

        self.robot = robot
        self._create_maps()
        self._update_actuated_joints()

        self._cfg = self.zero_cfg

        if build_scene_graph or build_collision_scene_graph:
            self._base_link = self._determine_base_link()
        else:
            self._base_link = None

        self._errors = []

        if build_scene_graph:
            self._scene = self._create_scene(
                use_collision_geometry=False,
                load_geometry=load_meshes,
                force_mesh=force_mesh,
                force_single_geometry_per_link=force_mesh,
            )
        else:
            self._scene = None

        if build_collision_scene_graph:
            self._scene_collision = self._create_scene(
                use_collision_geometry=True,
                load_geometry=load_collision_meshes,
                force_mesh=force_collision_mesh,
                force_single_geometry_per_link=force_collision_mesh,
            )
        else:
            self._scene_collision = None

        if build_tree:
            self.tree_root = self.build_tree()
        else:
            self.tree_root = None

    @property
    def scene(self) -> trimesh.Scene:
        """A scene object representing the URDF model.

        Returns:
            trimesh.Scene: A trimesh scene object.
        """
        return self._scene

    @property
    def collision_scene(self) -> trimesh.Scene:
        """A scene object representing the <collision> elements of the URDF model

        Returns:
            trimesh.Scene: A trimesh scene object.
        """
        return self._scene_collision

    @property
    def link_map(self) -> dict:
        """A dictionary mapping link names to link objects.

        Returns:
            dict: Mapping from link name (str) to Link.
        """
        return self._link_map

    @property
    def joint_map(self) -> dict:
        """A dictionary mapping joint names to joint objects.

        Returns:
            dict: Mapping from joint name (str) to Joint.
        """
        return self._joint_map

    @property
    def joint_names(self):
        """List of joint names.

        Returns:
            list[str]: List of joint names of the URDF model.
        """
        return [j.name for j in self.robot.joints]

    @property
    def actuated_joints(self):
        """List of actuated joints. This excludes mimic and fixed joints.

        Returns:
            list[Joint]: List of actuated joints of the URDF model.
        """
        return self._actuated_joints

    @property
    def actuated_dof_indices(self):
        """List of DOF indices per actuated joint. Can be used to reference configuration.

        Returns:
            list[list[int]]: List of DOF indices per actuated joint.
        """
        return self._actuated_dof_indices

    @property
    def actuated_joint_indices(self):
        """List of indices of all joints that are actuated, i.e., not of type mimic or fixed.

        Returns:
            list[int]: List of indices of actuated joints.
        """
        return self._actuated_joint_indices

    @property
    def actuated_joint_names(self):
        """List of names of actuated joints. This excludes mimic and fixed joints.

        Returns:
            list[str]: List of names of actuated joints of the URDF model.
        """
        return [j.name for j in self._actuated_joints]

    @property
    def num_actuated_joints(self):
        """Number of actuated joints.

        Returns:
            int: Number of actuated joints.
        """
        return len(self.actuated_joints)

    @property
    def num_dofs(self):
        """Number of degrees of freedom of actuated joints. Depending on the type of the joint, the number of DOFs might vary.

        Returns:
            int: Degrees of freedom.
        """
        total_num_dofs = 0
        for j in self._actuated_joints:
            if j.type in ["revolute", "prismatic", "continuous"]:
                total_num_dofs += 1
            elif j.type == "floating":
                total_num_dofs += 6
            elif j.type == "planar":
                total_num_dofs += 2
        return total_num_dofs

    @property
    def zero_cfg(self):
        """Return the zero configuration.

        Returns:
            np.ndarray: The zero configuration.
        """
        return np.zeros(self.num_dofs)

    @property
    def center_cfg(self):
        """Return center configuration of URDF model by using the average of each joint's limits if present, otherwise zero.

        Returns:
            (n), float: Default configuration of URDF model.
        """
        config = []
        config_names = []
        for j in self._actuated_joints:
            if j.type == "revolute" or j.type == "prismatic":
                if j.limit is not None:
                    cfg = [j.limit.lower + 0.5 * (j.limit.upper - j.limit.lower)]
                else:
                    cfg = [0.0]
            elif j.type == "continuous":
                cfg = [0.0]
            elif j.type == "floating":
                cfg = [0.0] * 6
            elif j.type == "planar":
                cfg = [0.0] * 2

            config.append(cfg)
            config_names.append(j.name)

        for i, j in enumerate(self.robot.joints):
            if j.mimic is not None:
                index = config_names.index(j.mimic.joint)
                config[i][0] = config[index][0] * j.mimic.multiplier + j.mimic.offset

        if len(config) == 0:
            return np.array([], dtype=np.float64)
        return np.concatenate(config)

    @property
    def cfg(self):
        """Current configuration.

        Returns:
            np.ndarray: Current configuration of URDF model.
        """
        return self._cfg

    @property
    def base_link(self):
        """Name of URDF base/root link.

        Returns:
            str: Name of base link of URDF model.
        """
        return self._base_link

    @property
    def errors(self) -> list:
        """A list with validation errors.

        Returns:
            list: A list of validation errors.
        """
        return self._errors

    def clear_errors(self):
        """Clear the validation error log."""
        self._errors = []

    def show(self, collision_geometry=False, callback=None):
        """Open a simpler viewer displaying the URDF model.

        Args:
            collision_geometry (bool, optional): Whether to display the <collision> or <visual> elements. Defaults to False.
        """
        if collision_geometry:
            if self._scene_collision is None:
                raise ValueError(
                    "No collision scene available. Use build_collision_scene_graph=True and load_collision_meshes=True during loading."
                )
            else:
                self._scene_collision.show(callback=callback)
        else:
            if self._scene is None:
                raise ValueError("No scene available. Use build_scene_graph=True and load_meshes=True during loading.")
            elif len(self._scene.bounds_corners) < 1:
                raise ValueError(
                    "Scene is empty, maybe meshes failed to load? Use build_scene_graph=True and load_meshes=True during loading."
                )
            else:
                self._scene.show(callback=callback)

    def validate(self, validation_fn=None) -> bool:
        """Validate URDF model.

        Args:
            validation_fn (function, optional): A function f(list[yourdfpy.URDFError]) -> bool. None uses the strict handler (any error leads to False). Defaults to None.

        Returns:
            bool: Whether the model is valid.
        """
        self._errors = []
        self._validate_robot(self.robot)

        if validation_fn is None:
            validation_fn = validation_handler_strict

        return validation_fn(self._errors)

    def _create_maps(self):
        self._material_map = {}
        for m in self.robot.materials:
            self._material_map[m.name] = m

        self._joint_map = {}
        for j in self.robot.joints:
            self._joint_map[j.name] = j

        self._link_map = {}
        for l in self.robot.links:
            self._link_map[l.name] = l

    def _update_actuated_joints(self):
        self._actuated_joints = []
        self._actuated_joint_indices = []
        self._actuated_dof_indices = []

        dof_indices_cnt = 0
        for i, j in enumerate(self.robot.joints):
            if j.mimic is None and j.type != "fixed":
                self._actuated_joints.append(j)
                self._actuated_joint_indices.append(i)

                if j.type in ["prismatic", "revolute", "continuous"]:
                    self._actuated_dof_indices.append([dof_indices_cnt])
                    dof_indices_cnt += 1
                elif j.type == "floating":
                    self._actuated_dof_indices.append([dof_indices_cnt, dof_indices_cnt + 1, dof_indices_cnt + 2])
                    dof_indices_cnt += 3
                elif j.type == "planar":
                    self._actuated_dof_indices.append([dof_indices_cnt, dof_indices_cnt + 1])
                    dof_indices_cnt += 2

    def _validate_required_attribute(self, attribute, error_msg, allowed_values=None):
        if attribute is None:
            self._errors.append(URDFIncompleteError(error_msg))
        elif isinstance(attribute, str) and len(attribute) == 0:
            self._errors.append(URDFIncompleteError(error_msg))

        if allowed_values is not None and attribute is not None:
            if attribute not in allowed_values:
                self._errors.append(URDFAttributeValueError(error_msg))

    @staticmethod
    def load(fname_or_file, add_dummy_free_joints=False, **kwargs):
        """Load URDF file from filename or file object.

        Args:
            fname_or_file (str or file object): A filename or file object, file-like object, stream representing the URDF file.
            **build_scene_graph (bool, optional): Wheter to build a scene graph to enable transformation queries and forward kinematics. Defaults to True.
            **build_collision_scene_graph (bool, optional): Wheter to build a scene graph for <collision> elements. Defaults to False.
            **load_meshes (bool, optional): Whether to load the meshes referenced in the <mesh> elements. Defaults to True.
            **load_collision_meshes (bool, optional): Whether to load the collision meshes referenced in the <mesh> elements. Defaults to False.
            **filename_handler ([type], optional): Any function f(in: str) -> str, that maps filenames in the URDF to actual resources. Can be used to customize treatment of `package://` directives or relative/absolute filenames. Defaults to None.
            **mesh_dir (str, optional): A root directory used for loading meshes. Defaults to "".
            **force_mesh (bool, optional): Each loaded geometry will be concatenated into a single one (instead of being turned into a graph; in case the underlying file contains multiple geometries). This might loose texture information but the resulting scene graph will be smaller. Defaults to False.
            **force_collision_mesh (bool, optional): Same as force_mesh, but for collision scene. Defaults to True.

        Raises:
            ValueError: If filename does not exist.

        Returns:
            yourdfpy.URDF: URDF model.
        """
        if isinstance(fname_or_file, six.string_types):
            if not os.path.isfile(fname_or_file):
                raise ValueError("{} is not a file".format(fname_or_file))

            if not "mesh_dir" in kwargs:
                kwargs["mesh_dir"] = os.path.dirname(fname_or_file)

        try:
            parser = etree.XMLParser(remove_blank_text=True)
            tree = etree.parse(fname_or_file, parser=parser)
            xml_root = tree.getroot()
        except Exception as e:
            _logger.error(e)
            _logger.error("Using different parsing approach.")

            events = ("start", "end", "start-ns", "end-ns")
            xml = etree.iterparse(fname_or_file, recover=True, events=events)

            # Iterate through all XML elements
            for action, elem in xml:
                # Skip comments and processing instructions,
                # because they do not have names
                if not (isinstance(elem, etree._Comment) or isinstance(elem, etree._ProcessingInstruction)):
                    # Remove a namespace URI in the element's name
                    # elem.tag = etree.QName(elem).localname
                    if action == "end" and ":" in elem.tag:
                        elem.getparent().remove(elem)

            xml_root = xml.root

        # Remove comments
        etree.strip_tags(xml_root, etree.Comment)
        etree.cleanup_namespaces(xml_root)

        return URDF(
            robot=URDF._parse_robot(xml_element=xml_root, add_dummy_free_joints=add_dummy_free_joints), **kwargs
        )

    def contains(self, key, value, element=None) -> bool:
        """Checks recursively whether the URDF tree contains the provided key-value pair.

        Args:
            key (str): A key.
            value (str): A value.
            element (etree.Element, optional): The XML element from which to start the recursive search. None means URDF root. Defaults to None.

        Returns:
            bool: Whether the key-value pair was found.
        """
        if element is None:
            element = self.robot

        result = False
        for field in element.__dataclass_fields__:
            field_value = getattr(element, field)
            if is_dataclass(field_value):
                result = result or self.contains(key=key, value=value, element=field_value)
            elif isinstance(field_value, list) and len(field_value) > 0 and is_dataclass(field_value[0]):
                for field_value_element in field_value:
                    result = result or self.contains(key=key, value=value, element=field_value_element)
            else:
                if key == field and value == field_value:
                    result = True
        return result

    def _determine_base_link(self):
        """Get the base link of the URDF tree by extracting all links without parents.
        In case multiple links could be root chose the first.

        Returns:
            str: Name of the base link.
        """
        link_names = [l.name for l in self.robot.links]

        for j in self.robot.joints:
            link_names.remove(j.child)

        if len(link_names) == 0:
            # raise Error?
            return None

        return link_names[0]

    def _forward_kinematics_joint(self, joint, q=None):
        origin = np.eye(4) if joint.origin is None else joint.origin

        if joint.mimic is not None:
            if joint.mimic.joint in self.actuated_joint_names:
                mimic_joint_index = self.actuated_joint_names.index(joint.mimic.joint)
                q = self._cfg[mimic_joint_index] * joint.mimic.multiplier + joint.mimic.offset
            else:
                _logger.warning(
                    f"Joint '{joint.name}' is supposed to mimic '{joint.mimic.joint}'. But this joint is not actuated - will assume (0.0 + offset)."
                )
                q = 0.0 + joint.mimic.offset

        if joint.type in ["revolute", "prismatic", "continuous"]:
            if q is None:
                # Use internal cfg vector for forward kinematics
                q = float(self.cfg[self.actuated_dof_indices[self.actuated_joint_names.index(joint.name)]])

            if joint.type == "prismatic":
                matrix = origin @ tra.translation_matrix(q * joint.axis)
            else:
                matrix = origin @ tra.rotation_matrix(q, joint.axis)
        else:
            # this includes: floating, planar, fixed
            matrix = origin

        return matrix, q

    def update_cfg(self, configuration):
        """Update joint configuration of URDF; does forward kinematics.

        Args:
            configuration (dict, list[float], tuple[float] or np.ndarray): A mapping from joints or joint names to configuration values, or a list containing a value for each actuated joint.

        Raises:
            ValueError: Raised if dimensionality of configuration does not match number of actuated joints of URDF model.
            TypeError: Raised if configuration is neither a dict, list, tuple or np.ndarray.
        """
        joint_cfg = []

        if isinstance(configuration, dict):
            for joint in configuration:
                if isinstance(joint, six.string_types):
                    joint_cfg.append((self._joint_map[joint], configuration[joint]))
                elif isinstance(joint, Joint):
                    # TODO: Joint is not hashable; so this branch will not succeed
                    joint_cfg.append((joint, configuration[joint]))
        elif isinstance(configuration, (list, tuple, np.ndarray)):
            if len(configuration) == len(self.robot.joints):
                for joint, value in zip(self.robot.joints, configuration):
                    joint_cfg.append((joint, value))
            elif len(configuration) == self.num_actuated_joints:
                for joint, value in zip(self._actuated_joints, configuration):
                    joint_cfg.append((joint, value))
            else:
                raise ValueError(
                    f"Dimensionality of configuration ({len(configuration)}) doesn't match number of all ({len(self.robot.joints)}) or actuated joints ({self.num_actuated_joints})."
                )
        else:
            raise TypeError("Invalid type for configuration")

        # append all mimic joints in the update
        for j, q in joint_cfg + [(j, 0.0) for j in self.robot.joints if j.mimic is not None]:
            matrix, joint_q = self._forward_kinematics_joint(j, q=q)

            # update internal configuration vector - only consider actuated joints
            if j.name in self.actuated_joint_names:
                self._cfg[self.actuated_dof_indices[self.actuated_joint_names.index(j.name)]] = joint_q

            if self._scene is not None:
                self._scene.graph.update(frame_from=j.parent, frame_to=j.child, matrix=matrix)
            if self._scene_collision is not None:
                self._scene_collision.graph.update(frame_from=j.parent, frame_to=j.child, matrix=matrix)

    def get_transform(self, frame_to, frame_from=None, collision_geometry=False):
        """Get the transform from one frame to another.

        Args:
            frame_to (str): Node name.
            frame_from (str, optional): Node name. If None it will be set to self.base_frame. Defaults to None.
            collision_geometry (bool, optional): Whether to use the collision geometry scene graph (instead of the visual geometry). Defaults to False.

        Raises:
            ValueError: Raised if scene graph wasn't constructed during intialization.

        Returns:
            (4, 4) float: Homogeneous transformation matrix
        """
        if collision_geometry:
            if self._scene_collision is None:
                raise ValueError("No collision scene available. Use build_collision_scene_graph=True during loading.")
            else:
                return self._scene_collision.graph.get(frame_to=frame_to, frame_from=frame_from)[0]
        else:
            if self._scene is None:
                raise ValueError("No scene available. Use build_scene_graph=True during loading.")
            else:
                return self._scene.graph.get(frame_to=frame_to, frame_from=frame_from)[0]

    def _link_mesh(self, link, collision_geometry=True):
        geometries = link.collisions if collision_geometry else link.visuals

        if len(geometries) == 0:
            return None

        meshes = []
        for g in geometries:
            for m in g.geometry.meshes:
                m = m.copy()
                pose = g.origin
                if g.geometry.mesh is not None:
                    if g.geometry.mesh.scale is not None:
                        S = np.eye(4)
                        S[:3, :3] = np.diag(g.geometry.mesh.scale)
                        pose = pose.dot(S)
                m.apply_transform(pose)
                meshes.append(m)
        if len(meshes) == 0:
            return None
        self._collision_mesh = meshes[0] + meshes[1:]
        return self._collision_mesh

    def _geometry2trimeshscene(self, geometry, load_file, force_mesh, skip_materials):
        new_s = None
        if geometry.box is not None:
            new_s = trimesh.primitives.Box(extents=geometry.box.size).scene()
        elif geometry.sphere is not None:
            new_s = trimesh.primitives.Sphere(radius=geometry.sphere.radius).scene()
        elif geometry.cylinder is not None:
            new_s = trimesh.primitives.Cylinder(
                radius=geometry.cylinder.radius, height=geometry.cylinder.length
            ).scene()
        elif geometry.mesh is not None and load_file:
            new_filename = self._filename_handler(fname=geometry.mesh.filename)

            if os.path.isfile(new_filename):
                _logger.debug(f"Loading {geometry.mesh.filename} as {new_filename}")

                if force_mesh:
                    new_g = trimesh.load(
                        new_filename,
                        ignore_broken=True,
                        force="mesh",
                        skip_materials=skip_materials,
                    )

                    # add original filename
                    if "file_path" not in new_g.metadata:
                        new_g.metadata["file_path"] = os.path.abspath(new_filename)
                        new_g.metadata["file_name"] = os.path.basename(new_filename)

                    new_s = trimesh.Scene()
                    new_s.add_geometry(new_g)
                else:
                    new_s = trimesh.load(
                        new_filename,
                        ignore_broken=True,
                        force="scene",
                        skip_materials=skip_materials,
                    )

                    if "file_path" in new_s.metadata:
                        for i, (_, geom) in enumerate(new_s.geometry.items()):
                            if "file_path" not in geom.metadata:
                                geom.metadata["file_path"] = new_s.metadata["file_path"]
                                geom.metadata["file_name"] = new_s.metadata["file_name"]
                                geom.metadata["file_element"] = i

                # scale mesh appropriately
                if geometry.mesh.scale is not None:
                    if isinstance(geometry.mesh.scale, float):
                        new_s = new_s.scaled(geometry.mesh.scale)
                    elif isinstance(geometry.mesh.scale, np.ndarray):
                        new_s = new_s.scaled(geometry.mesh.scale)
                    else:
                        _logger.warning(f"Warning: Can't interpret scale '{geometry.mesh.scale}'")
            else:
                _logger.warning(f"Can't find {new_filename}")
        return new_s

    def _add_geometries_to_scene(
        self,
        s,
        geometries,
        link_name,
        load_geometry,
        force_mesh,
        force_single_geometry,
        skip_materials,
    ):
        if force_single_geometry:
            tmp_scene = trimesh.Scene(base_frame=link_name)

        first_geom_name = None

        for v in geometries:
            if v.geometry is not None:
                if first_geom_name is None:
                    first_geom_name = v.name

                new_s = self._geometry2trimeshscene(
                    geometry=v.geometry,
                    load_file=load_geometry,
                    force_mesh=force_mesh,
                    skip_materials=skip_materials,
                )
                if new_s is not None:
                    origin = v.origin if v.origin is not None else np.eye(4)

                    if force_single_geometry:
                        for name, geom in new_s.geometry.items():
                            if isinstance(v, Visual):
                                apply_visual_color(geom, v, self._material_map)
                            tmp_scene.add_geometry(
                                geometry=geom,
                                geom_name=v.name,
                                parent_node_name=link_name,
                                transform=origin @ new_s.graph.get(name)[0],
                            )
                    else:
                        # The following map is used to deal with glb format
                        # when the graph node and geometry have different names
                        geom_name_map = {new_s.graph[node_name][1]: node_name for node_name in new_s.graph.nodes}
                        for name, geom in new_s.geometry.items():
                            if isinstance(v, Visual):
                                apply_visual_color(geom, v, self._material_map)
                            s.add_geometry(
                                geometry=geom,
                                geom_name=v.name,
                                parent_node_name=link_name,
                                transform=origin @ new_s.graph.get(geom_name_map[name])[0],
                            )

        if force_single_geometry and len(tmp_scene.geometry) > 0:
            s.add_geometry(
                geometry=tmp_scene.dump(concatenate=True),
                geom_name=first_geom_name,
                parent_node_name=link_name,
                transform=np.eye(4),
            )

    def _create_scene(
        self,
        use_collision_geometry=False,
        load_geometry=True,
        force_mesh=False,
        force_single_geometry_per_link=False,
    ):
        s = trimesh.scene.Scene(base_frame=self._base_link)

        for j in self.robot.joints:
            matrix, _ = self._forward_kinematics_joint(j)

            s.graph.update(frame_from=j.parent, frame_to=j.child, matrix=matrix)

        for l in self.robot.links:
            if l.name not in s.graph.nodes and l.name != s.graph.base_frame:
                _logger.warning(f"{l.name} not connected via joints. Will add link to base frame.")
                s.graph.update(frame_from=s.graph.base_frame, frame_to=l.name)

            meshes = l.collisions if use_collision_geometry else l.visuals
            self._add_geometries_to_scene(
                s,
                geometries=meshes,
                link_name=l.name,
                load_geometry=load_geometry,
                force_mesh=force_mesh,
                force_single_geometry=force_single_geometry_per_link,
                skip_materials=use_collision_geometry,
            )

        return s

    def _successors(self, node):
        """
        Get all nodes of the scene that succeeds a specified node.

        Parameters
        ------------
        node : any
          Hashable key in `scene.graph`

        Returns
        -----------
        subnodes : set[str]
          Set of nodes.
        """
        # get every node that is a successor to specified node
        # this includes `node`
        return self._scene.graph.transforms.successors(node)

    def _create_subrobot(self, robot_name, root_link_name):
        subrobot = Robot(name=robot_name)
        subnodes = self._successors(node=root_link_name)

        if len(subnodes) > 0:
            for node in subnodes:
                if node in self.link_map:
                    subrobot.links.append(copy.deepcopy(self.link_map[node]))
            for joint_name, joint in self.joint_map.items():
                if joint.parent in subnodes and joint.child in subnodes:
                    subrobot.joints.append(copy.deepcopy(self.joint_map[joint_name]))

        return subrobot

    def split_along_joints(self, joint_type="floating", **kwargs):
        """Split URDF model along a particular joint type.
        The result is a set of URDF models which together compose the original URDF.

        Args:
            joint_type (str, or list[str], optional): Type of joint to use for splitting. Defaults to "floating".
            **kwargs: Arguments delegated to URDF constructor of new URDF models.

        Returns:
            list[(np.ndarray, yourdfpy.URDF)]: A list of tuples (np.ndarray, yourdfpy.URDF) whereas each homogeneous 4x4 matrix describes the root transformation of the respective URDF model w.r.t. the original URDF.
        """
        root_urdf = URDF(robot=copy.deepcopy(self.robot), build_scene_graph=False, load_meshes=False)
        result = []

        joint_types = joint_type if isinstance(joint_type, list) else [joint_type]

        # find all relevant joints
        joint_names = [j.name for j in self.robot.joints if j.type in joint_types]
        for joint_name in joint_names:
            root_link = self.link_map[self.joint_map[joint_name].child]
            new_robot = self._create_subrobot(
                robot_name=root_link.name,
                root_link_name=root_link.name,
            )

            result.append(
                (
                    self._scene.graph.get(root_link.name)[0],
                    URDF(robot=new_robot, **kwargs),
                )
            )

            # remove links and joints from root robot
            for j in new_robot.joints:
                root_urdf.robot.joints.remove(root_urdf.joint_map[j.name])
            for l in new_robot.links:
                root_urdf.robot.links.remove(root_urdf.link_map[l.name])

            # remove joint that connects root urdf to root_link
            if root_link.name in [j.child for j in root_urdf.robot.joints]:
                root_urdf.robot.joints.remove(
                    root_urdf.robot.joints[[j.child for j in root_urdf.robot.joints].index(root_link.name)]
                )

        result.insert(0, (np.eye(4), URDF(robot=root_urdf.robot, **kwargs)))

        return result

    def validate_filenames(self):
        for l in self.robot.links:
            meshes = [m.geometry.mesh for m in l.collisions + l.visuals if m.geometry.mesh is not None]
            for m in meshes:
                _logger.debug(m.filename, "-->", self._filename_handler(m.filename))
                if not os.path.isfile(self._filename_handler(m.filename)):
                    return False
        return True

    def write_xml(self):
        """Write URDF model to an XML element hierarchy.

        Returns:
            etree.ElementTree: XML data.
        """
        xml_element = self._write_robot(self.robot)
        return etree.ElementTree(xml_element)

    def write_xml_string(self, **kwargs):
        """Write URDF model to a string.

        Returns:
            str: String of the xml representation of the URDF model.
        """
        xml_element = self.write_xml()
        return etree.tostring(xml_element, xml_declaration=True, *kwargs)

    def write_xml_file(self, fname):
        """Write URDF model to an xml file.

        Args:
            fname (str): Filename of the file to be written. Usually ends in `.urdf`.
        """
        xml_element = self.write_xml()
        xml_element.write(fname, xml_declaration=True, pretty_print=True)

    def _parse_mimic(xml_element):
        if xml_element is None:
            return None

        return Mimic(
            joint=xml_element.get("joint"),
            multiplier=_str2float(xml_element.get("multiplier", 1.0)),
            offset=_str2float(xml_element.get("offset", 0.0)),
        )

    def _write_mimic(self, xml_parent, mimic):
        etree.SubElement(
            xml_parent,
            "mimic",
            attrib={
                "joint": mimic.joint,
                "multiplier": str(mimic.multiplier),
                "offset": str(mimic.offset),
            },
        )

    def _parse_safety_controller(xml_element):
        if xml_element is None:
            return None

        return SafetyController(
            soft_lower_limit=_str2float(xml_element.get("soft_lower_limit")),
            soft_upper_limit=_str2float(xml_element.get("soft_upper_limit")),
            k_position=_str2float(xml_element.get("k_position")),
            k_velocity=_str2float(xml_element.get("k_velocity")),
        )

    def _write_safety_controller(self, xml_parent, safety_controller):
        etree.SubElement(
            xml_parent,
            "safety_controller",
            attrib={
                "soft_lower_limit": str(safety_controller.soft_lower_limit),
                "soft_upper_limit": str(safety_controller.soft_upper_limit),
                "k_position": str(safety_controller.k_position),
                "k_velocity": str(safety_controller.k_velocity),
            },
        )

    def _parse_transmission_joint(xml_element):
        if xml_element is None:
            return None

        transmission_joint = TransmissionJoint(name=xml_element.get("name"))

        for h in xml_element.findall("hardware_interface"):
            transmission_joint.hardware_interfaces.append(h.text)

        return transmission_joint

    def _write_transmission_joint(self, xml_parent, transmission_joint):
        xml_element = etree.SubElement(
            xml_parent,
            "joint",
            attrib={
                "name": str(transmission_joint.name),
            },
        )
        for h in transmission_joint.hardware_interfaces:
            tmp = etree.SubElement(
                xml_element,
                "hardwareInterface",
            )
            tmp.text = h

    def _parse_actuator(xml_element):
        if xml_element is None:
            return None

        actuator = Actuator(name=xml_element.get("name"))
        if xml_element.find("mechanicalReduction"):
            actuator.mechanical_reduction = float(xml_element.find("mechanicalReduction").text)

        for h in xml_element.findall("hardwareInterface"):
            actuator.hardware_interfaces.append(h.text)

        return actuator

    def _write_actuator(self, xml_parent, actuator):
        xml_element = etree.SubElement(
            xml_parent,
            "actuator",
            attrib={
                "name": str(actuator.name),
            },
        )
        if actuator.mechanical_reduction is not None:
            tmp = etree.SubElement("mechanicalReduction")
            tmp.text = str(actuator.mechanical_reduction)

        for h in actuator.hardware_interfaces:
            tmp = etree.SubElement(
                xml_element,
                "hardwareInterface",
            )
            tmp.text = h

    def _parse_transmission(xml_element):
        if xml_element is None:
            return None

        transmission = Transmission(name=xml_element.get("name"))

        for j in xml_element.findall("joint"):
            transmission.joints.append(URDF._parse_transmission_joint(j))
        for a in xml_element.findall("actuator"):
            transmission.actuators.append(URDF._parse_actuator(a))

        return transmission

    def _write_transmission(self, xml_parent, transmission):
        xml_element = etree.SubElement(
            xml_parent,
            "transmission",
            attrib={
                "name": str(transmission.name),
            },
        )

        for j in transmission.joints:
            self._write_transmission_joint(xml_element, j)

        for a in transmission.actuators:
            self._write_actuator(xml_element, a)

    def _parse_calibration(xml_element):
        if xml_element is None:
            return None

        return Calibration(
            rising=_str2float(xml_element.get("rising")),
            falling=_str2float(xml_element.get("falling")),
        )

    def _write_calibration(self, xml_parent, calibration):
        etree.SubElement(
            xml_parent,
            "calibration",
            attrib={
                "rising": str(calibration.rising),
                "falling": str(calibration.falling),
            },
        )

    def _parse_box(xml_element):
        return Box(size=np.array(xml_element.attrib["size"].split(), dtype=float))

    def _write_box(self, xml_parent, box):
        etree.SubElement(xml_parent, "box", attrib={"size": " ".join(map(str, box.size))})

    def _parse_cylinder(xml_element):
        return Cylinder(
            radius=float(xml_element.attrib["radius"]),
            length=float(xml_element.attrib["length"]),
        )

    def _write_cylinder(self, xml_parent, cylinder):
        etree.SubElement(
            xml_parent,
            "cylinder",
            attrib={"radius": str(cylinder.radius), "length": str(cylinder.length)},
        )

    def _parse_sphere(xml_element):
        return Sphere(radius=float(xml_element.attrib["radius"]))

    def _write_sphere(self, xml_parent, sphere):
        etree.SubElement(xml_parent, "sphere", attrib={"radius": str(sphere.radius)})

    def _parse_scale(xml_element):
        if "scale" in xml_element.attrib:
            s = xml_element.get("scale").split()
            if len(s) == 0:
                return None
            elif len(s) == 1:
                return float(s[0])
            else:
                return np.array(list(map(float, s)))
        return None

    def _write_scale(self, xml_parent, scale):
        if scale is not None:
            if isinstance(scale, float) or isinstance(scale, int):
                xml_parent.set("scale", " ".join([str(scale)] * 3))
            else:
                xml_parent.set("scale", " ".join(map(str, scale)))

    def _parse_mesh(xml_element):
        return Mesh(filename=xml_element.get("filename"), scale=URDF._parse_scale(xml_element))

    def _write_mesh(self, xml_parent, mesh):
        # TODO: turn into different filename handler
        xml_element = etree.SubElement(
            xml_parent,
            "mesh",
            attrib={"filename": self._filename_handler(mesh.filename)},
        )

        self._write_scale(xml_element, mesh.scale)

    def _parse_geometry(xml_element):
        geometry = Geometry()
        if xml_element[0].tag == "box":
            geometry.box = URDF._parse_box(xml_element[0])
        elif xml_element[0].tag == "cylinder":
            geometry.cylinder = URDF._parse_cylinder(xml_element[0])
        elif xml_element[0].tag == "sphere":
            geometry.sphere = URDF._parse_sphere(xml_element[0])
        elif xml_element[0].tag == "mesh":
            geometry.mesh = URDF._parse_mesh(xml_element[0])
        else:
            raise ValueError(f"Unknown tag: {xml_element[0].tag}")

        return geometry

    def _validate_geometry(self, geometry):
        if geometry is None:
            self._errors.append(URDFIncompleteError("<geometry> is missing."))

        num_nones = sum(
            [
                x is not None
                for x in [
                    geometry.box,
                    geometry.cylinder,
                    geometry.sphere,
                    geometry.mesh,
                ]
            ]
        )
        if num_nones < 1:
            self._errors.append(
                URDFIncompleteError(
                    "One of <sphere>, <cylinder>, <box>, <mesh> needs to be defined as a child of <geometry>."
                )
            )
        elif num_nones > 1:
            self._errors.append(
                URDFError(
                    "Too many of <sphere>, <cylinder>, <box>, <mesh> defined as a child of <geometry>. Only one allowed."
                )
            )

    def _write_geometry(self, xml_parent, geometry):
        if geometry is None:
            return

        xml_element = etree.SubElement(xml_parent, "geometry")
        if geometry.box is not None:
            self._write_box(xml_element, geometry.box)
        elif geometry.cylinder is not None:
            self._write_cylinder(xml_element, geometry.cylinder)
        elif geometry.sphere is not None:
            self._write_sphere(xml_element, geometry.sphere)
        elif geometry.mesh is not None:
            self._write_mesh(xml_element, geometry.mesh)

    def _parse_origin(xml_element):
        if xml_element is None:
            return None

        xyz = xml_element.get("xyz", default="0 0 0")
        rpy = xml_element.get("rpy", default="0 0 0")

        return tra.compose_matrix(
            translate=np.array(list(map(float, xyz.split()))),
            angles=np.array(list(map(float, rpy.split()))),
        )

    def _write_origin(self, xml_parent, origin):
        if origin is None:
            return

        etree.SubElement(
            xml_parent,
            "origin",
            attrib={
                "xyz": " ".join(map(str, tra.translation_from_matrix(origin))),
                "rpy": " ".join(map(str, tra.euler_from_matrix(origin))),
            },
        )

    def _parse_color(xml_element):
        if xml_element is None:
            return None

        rgba = xml_element.get("rgba", default="1 1 1 1")

        return Color(rgba=np.array(list(map(float, rgba.split()))))

    def _write_color(self, xml_parent, color):
        if color is None:
            return

        etree.SubElement(xml_parent, "color", attrib={"rgba": " ".join(map(str, color.rgba))})

    def _parse_texture(xml_element):
        if xml_element is None:
            return None

        # TODO: use texture filename handler
        return Texture(filename=xml_element.get("filename", default=None))

    def _write_texture(self, xml_parent, texture):
        if texture is None:
            return

        # TODO: use texture filename handler
        etree.SubElement(xml_parent, "texture", attrib={"filename": texture.filename})

    def _parse_material(xml_element):
        if xml_element is None:
            return None

        material = Material(name=xml_element.get("name"))
        material.color = URDF._parse_color(xml_element.find("color"))
        material.texture = URDF._parse_texture(xml_element.find("texture"))

        return material

    def _write_material(self, xml_parent, material):
        if material is None:
            return

        attrib = {"name": material.name} if material.name is not None else {}
        xml_element = etree.SubElement(
            xml_parent,
            "material",
            attrib=attrib,
        )

        self._write_color(xml_element, material.color)
        self._write_texture(xml_element, material.texture)

    def _parse_visual(xml_element):
        visual = Visual(name=xml_element.get("name"))

        visual.geometry = URDF._parse_geometry(xml_element.find("geometry"))
        visual.origin = URDF._parse_origin(xml_element.find("origin"))
        visual.material = URDF._parse_material(xml_element.find("material"))

        return visual

    def _validate_visual(self, visual):
        self._validate_geometry(visual.geometry)

    def _write_visual(self, xml_parent, visual):
        attrib = {"name": visual.name} if visual.name is not None else {}
        xml_element = etree.SubElement(
            xml_parent,
            "visual",
            attrib=attrib,
        )

        self._write_geometry(xml_element, visual.geometry)
        self._write_origin(xml_element, visual.origin)
        self._write_material(xml_element, visual.material)

    def _parse_collision(xml_element):
        collision = Collision(name=xml_element.get("name"))

        collision.geometry = URDF._parse_geometry(xml_element.find("geometry"))
        collision.origin = URDF._parse_origin(xml_element.find("origin"))

        return collision

    def _validate_collision(self, collision):
        self._validate_geometry(collision.geometry)

    def _write_collision(self, xml_parent, collision):
        attrib = {"name": collision.name} if collision.name is not None else {}
        xml_element = etree.SubElement(
            xml_parent,
            "collision",
            attrib=attrib,
        )

        self._write_geometry(xml_element, collision.geometry)
        self._write_origin(xml_element, collision.origin)

    def _parse_inertia(xml_element):
        if xml_element is None:
            return None

        x = xml_element

        return np.array(
            [
                [
                    x.get("ixx", default=1.0),
                    x.get("ixy", default=0.0),
                    x.get("ixz", default=0.0),
                ],
                [
                    x.get("ixy", default=0.0),
                    x.get("iyy", default=1.0),
                    x.get("iyz", default=0.0),
                ],
                [
                    x.get("ixz", default=0.0),
                    x.get("iyz", default=0.0),
                    x.get("izz", default=1.0),
                ],
            ],
            dtype=np.float64,
        )

    def _write_inertia(self, xml_parent, inertia):
        if inertia is None:
            return None

        etree.SubElement(
            xml_parent,
            "inertia",
            attrib={
                "ixx": str(inertia[0, 0]),
                "ixy": str(inertia[0, 1]),
                "ixz": str(inertia[0, 2]),
                "iyy": str(inertia[1, 1]),
                "iyz": str(inertia[1, 2]),
                "izz": str(inertia[2, 2]),
            },
        )

    def _parse_mass(xml_element):
        if xml_element is None:
            return None

        return _str2float(xml_element.get("value", default=0.0))

    def _write_mass(self, xml_parent, mass):
        if mass is None:
            return

        etree.SubElement(
            xml_parent,
            "mass",
            attrib={
                "value": str(mass),
            },
        )

    def _parse_inertial(xml_element):
        if xml_element is None:
            return None

        inertial = Inertial()
        inertial.origin = URDF._parse_origin(xml_element.find("origin"))
        inertial.inertia = URDF._parse_inertia(xml_element.find("inertia"))
        inertial.mass = URDF._parse_mass(xml_element.find("mass"))

        return inertial

    def _write_inertial(self, xml_parent, inertial):
        if inertial is None:
            return

        xml_element = etree.SubElement(xml_parent, "inertial")

        self._write_origin(xml_element, inertial.origin)
        self._write_mass(xml_element, inertial.mass)
        self._write_inertia(xml_element, inertial.inertia)

    def _parse_link(xml_element):
        link = Link(name=xml_element.attrib["name"])

        link.inertial = URDF._parse_inertial(xml_element.find("inertial"))

        for v in xml_element.findall("visual"):
            link.visuals.append(URDF._parse_visual(v))

        for c in xml_element.findall("collision"):
            link.collisions.append(URDF._parse_collision(c))

        return link

    def _validate_link(self, link):
        self._validate_required_attribute(attribute=link.name, error_msg="The <link> tag misses a 'name' attribute.")

        for v in link.visuals:
            self._validate_visual(v)

        for c in link.collisions:
            self._validate_collision(c)

    def _write_link(self, xml_parent, link):
        xml_element = etree.SubElement(
            xml_parent,
            "link",
            attrib={
                "name": link.name,
            },
        )

        self._write_inertial(xml_element, link.inertial)
        for visual in link.visuals:
            self._write_visual(xml_element, visual)
        for collision in link.collisions:
            self._write_collision(xml_element, collision)

    def _parse_axis(xml_element):
        if xml_element is None:
            return np.array([1.0, 0, 0])

        xyz = xml_element.get("xyz", "1 0 0")
        results = []
        for x in xyz.split():
            try:
                x = float(x)
            except ValueError:
                x = 0
            results.append(x)
        return np.array(results)
        # return np.array(list(map(float, xyz.split())))

    def _write_axis(self, xml_parent, axis):
        if axis is None:
            return

        etree.SubElement(xml_parent, "axis", attrib={"xyz": " ".join(map(str, axis))})

    def _parse_limit(xml_element):
        if xml_element is None:
            return None

        return Limit(
            effort=_str2float(xml_element.get("effort", default=None)),
            velocity=_str2float(xml_element.get("velocity", default=None)),
            lower=_str2float(xml_element.get("lower", default=None)),
            upper=_str2float(xml_element.get("upper", default=None)),
        )

    def _validate_limit(self, limit, type):
        if type in ["revolute", "prismatic"]:
            self._validate_required_attribute(
                limit,
                error_msg="The <limit> of a (prismatic, revolute) joint is missing.",
            )

            if limit is not None:
                self._validate_required_attribute(
                    limit.upper,
                    error_msg="Tag <limit> of joint is missing attribute 'upper'.",
                )
                self._validate_required_attribute(
                    limit.lower,
                    error_msg="Tag <limit> of joint is missing attribute 'lower'.",
                )

        if limit is not None:
            self._validate_required_attribute(
                limit.effort,
                error_msg="Tag <limit> of joint is missing attribute 'effort'.",
            )

            self._validate_required_attribute(
                limit.velocity,
                error_msg="Tag <limit> of joint is missing attribute 'velocity'.",
            )

    def _write_limit(self, xml_parent, limit):
        if limit is None:
            return

        attrib = {}
        if limit.effort is not None:
            attrib["effort"] = str(limit.effort)
        if limit.velocity is not None:
            attrib["velocity"] = str(limit.velocity)
        if limit.lower is not None:
            attrib["lower"] = str(limit.lower)
        if limit.upper is not None:
            attrib["upper"] = str(limit.upper)

        etree.SubElement(
            xml_parent,
            "limit",
            attrib=attrib,
        )

    def _parse_dynamics(xml_element):
        if xml_element is None:
            return None

        dynamics = Dynamics()
        dynamics.damping = xml_element.get("damping", default=None)
        dynamics.friction = xml_element.get("friction", default=None)

        return dynamics

    def _write_dynamics(self, xml_parent, dynamics):
        if dynamics is None:
            return

        attrib = {}
        if dynamics.damping is not None:
            attrib["damping"] = str(dynamics.damping)
        if dynamics.friction is not None:
            attrib["friction"] = str(dynamics.friction)

        etree.SubElement(
            xml_parent,
            "dynamics",
            attrib=attrib,
        )

    def _parse_joint(xml_element):
        joint = Joint(name=xml_element.attrib["name"])

        joint.type = xml_element.get("type", default=None)
        joint.parent = xml_element.find("parent").get("link")
        joint.child = xml_element.find("child").get("link")
        joint.origin = URDF._parse_origin(xml_element.find("origin"))
        joint.axis = URDF._parse_axis(xml_element.find("axis"))
        joint.limit = URDF._parse_limit(xml_element.find("limit"))
        joint.dynamics = URDF._parse_dynamics(xml_element.find("dynamics"))
        joint.mimic = URDF._parse_mimic(xml_element.find("mimic"))
        joint.calibration = URDF._parse_calibration(xml_element.find("calibration"))
        joint.safety_controller = URDF._parse_safety_controller(xml_element.find("safety_controller"))

        return joint

    def _validate_joint(self, joint):
        self._validate_required_attribute(
            attribute=joint.name,
            error_msg="The <joint> tag misses a 'name' attribute.",
        )

        allowed_types = [
            "revolute",
            "continuous",
            "prismatic",
            "fixed",
            "floating",
            "planar",
        ]
        self._validate_required_attribute(
            attribute=joint.type,
            error_msg=f"The <joint> tag misses a 'type' attribute or value is not part of allowed values [{', '.join(allowed_types)}].",
            allowed_values=allowed_types,
        )

        self._validate_required_attribute(
            joint.parent,
            error_msg=f"The <parent> of a <joint> is missing.",
        )

        self._validate_required_attribute(
            joint.child,
            error_msg=f"The <child> of a <joint> is missing.",
        )

        self._validate_limit(joint.limit, type=joint.type)

    def _write_joint(self, xml_parent, joint):
        xml_element = etree.SubElement(
            xml_parent,
            "joint",
            attrib={
                "name": joint.name,
                "type": joint.type,
            },
        )

        etree.SubElement(xml_element, "parent", attrib={"link": joint.parent})
        etree.SubElement(xml_element, "child", attrib={"link": joint.child})
        self._write_origin(xml_element, joint.origin)
        self._write_axis(xml_element, joint.axis)
        self._write_limit(xml_element, joint.limit)
        self._write_dynamics(xml_element, joint.dynamics)

    @staticmethod
    def _parse_robot(xml_element, add_dummy_free_joints=False):
        robot = Robot(name=xml_element.attrib["name"])

        for l in xml_element.findall("link"):
            robot.links.append(URDF._parse_link(l))
        for j in xml_element.findall("joint"):
            robot.joints.append(URDF._parse_joint(j))
        for m in xml_element.findall("material"):
            robot.materials.append(URDF._parse_material(m))

        if add_dummy_free_joints:
            # Determine root link
            link_names = [l.name for l in robot.links]
            for j in robot.joints:
                link_names.remove(j.child)

            if len(link_names) == 0:
                raise RuntimeError(f"No root link found for robot.")

            root_link_name = link_names[0]
            _add_dummy_joints(robot, root_link_name)

        return robot

    def _validate_robot(self, robot):
        if robot is not None:
            self._validate_required_attribute(
                attribute=robot.name,
                error_msg="The <robot> tag misses a 'name' attribute.",
            )

            for l in robot.links:
                self._validate_link(l)

            for j in robot.joints:
                self._validate_joint(j)

    def _write_robot(self, robot):
        xml_element = etree.Element("robot", attrib={"name": robot.name})
        for link in robot.links:
            self._write_link(xml_element, link)
        for joint in robot.joints:
            self._write_joint(xml_element, joint)
        for material in robot.materials:
            self._write_material(xml_element, material)

        return xml_element

    def __eq__(self, other):
        if not isinstance(other, URDF):
            raise NotImplemented
        return self.robot == other.robot

    @property
    def filename_handler(self):
        return self._filename_handler

    def build_tree(self):
        parent_child_map: Dict[str, List[str]] = {}
        for joint in self.robot.joints:
            if joint.parent in parent_child_map:
                parent_child_map[joint.parent].append(joint.child)
            else:
                parent_child_map[joint.parent] = [joint.child]

        # Sort link with bfs order
        bfs_link_list = [self.base_link]
        to_be_handle_list = [self.base_link]
        while len(to_be_handle_list) > 0:
            parent = to_be_handle_list.pop(0)
            if parent not in parent_child_map:
                continue

            children = parent_child_map[parent]
            to_be_handle_list.extend(children)
            bfs_link_list.extend(children)
        bfs_joint_list = []
        for link_name in bfs_link_list[1:]:
            joint_index = [i for i in range(len(self.robot.joints)) if self.robot.joints[i].child == link_name][0]
            bfs_joint_list.append(self.robot.joints[joint_index])

        # Build tree
        root = Node(self.base_link, matrix=np.eye(4))
        for joint in bfs_joint_list:
            matrix, _ = self._forward_kinematics_joint(joint, 0)
            parent_node = anytree.search.findall_by_attr(root, value=joint.parent)[0]
            node = Node(joint.child, parent=parent_node, matrix=matrix)
        return root

    def update_kinematics(self, configuration):
        joint_cfg = []

        if isinstance(configuration, dict):
            for joint in configuration:
                if isinstance(joint, six.string_types):
                    joint_cfg.append((self._joint_map[joint], configuration[joint]))
                elif isinstance(joint, Joint):
                    # TODO: Joint is not hashable; so this branch will not succeed
                    joint_cfg.append((joint, configuration[joint]))
        elif isinstance(configuration, (list, tuple, np.ndarray)):
            if len(configuration) == len(self.robot.joints):
                for joint, value in zip(self.robot.joints, configuration):
                    joint_cfg.append((joint, value))
            elif len(configuration) == self.num_actuated_joints:
                for joint, value in zip(self._actuated_joints, configuration):
                    joint_cfg.append((joint, value))
            else:
                raise ValueError(
                    f"Dimensionality of configuration ({len(configuration)}) doesn't match number of all ({len(self.robot.joints)}) or actuated joints ({self.num_actuated_joints})."
                )
        else:
            raise TypeError("Invalid type for configuration")

        # append all mimic joints in the update
        for j, q in joint_cfg + [(j, 0.0) for j in self.robot.joints if j.mimic is not None]:
            matrix, _ = self._forward_kinematics_joint(j, q=q)
            node = anytree.search.findall_by_attr(self.tree_root, j.child)[0]
            node.matrix = matrix

        for node in LevelOrderIter(self.tree_root):
            if node.name == self.base_link:
                node.global_pose = np.eye(4)
            else:
                node.global_pose = node.parent.global_pose @ node.matrix

    def get_link_global_transform(self, link_name):
        node = anytree.search.findall_by_attr(self.tree_root, link_name)[0]

        return node.global_pose


def _add_dummy_joints(robot: Robot, root_link_name: str):
    # Prepare link and joint properties
    translation_range = (-5, 5)
    rotation_range = (-2 * np.pi, 2 * np.pi)
    joint_types = ["prismatic"] * 3 + ["revolute"] * 3
    joint_limit = [translation_range] * 3 + [rotation_range] * 3
    joint_name = DUMMY_JOINT_NAMES.copy()
    link_name = [f"dummy_{name}_translation_link" for name in "xyz"] + [f"dummy_{name}_rotation_link" for name in "xyz"]

    links = []
    joints = []

    for i in range(6):
        inertial = Inertial(
            mass=0.01, inertia=np.array([[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]]), origin=np.identity(4)
        )
        link = Link(name=link_name[i], inertial=inertial)
        links.append(link)

        joint_axis = np.zeros(3, dtype=int)
        joint_axis[i % 3] = 1
        limit = Limit(lower=joint_limit[i][0], upper=joint_limit[i][1], velocity=3.14, effort=10)

        child_name = link_name[i + 1] if i < 5 else root_link_name
        joint = Joint(
            name=joint_name[i],
            type=joint_types[i],
            parent=link_name[i],
            child=child_name,
            origin=np.identity(4),
            axis=joint_axis,
            limit=limit,
        )
        joints.append(joint)

    robot.joints = joints + robot.joints
    robot.links = links + robot.links


DUMMY_JOINT_NAMES = [f"dummy_{name}_translation_joint" for name in "xyz"] + [
    f"dummy_{name}_rotation_joint" for name in "xyz"
]
