from build123d import *
from build123d.topology import Shape
from ocp_vscode import (
    show_all,
    show,
    show_object,
    reset_show,
    set_port,
    set_defaults,
    get_defaults,
)
from bd_warehouse import fastener
import copy
from dataclasses import dataclass
from typing import Self, Iterator
from anytree import Node

UNIT_VEC = Vector(1, 1, 1)

# servo_mount_screw = fastener.ButtonHeadScrew(size='M1.6-0.35', length=3 * MM)
m3 = fastener.ButtonHeadScrew(size="M3-0.5", length=3)

importer = Mesher()
servo_wheel_hole_radius = m3.thread_diameter * 1.05 / 2
servo_mount_hole_radius = 1.3 * 1.05 / 2
guide_axle_mount_radius = 3.02
motor_axle_mount_radius = 1.6
wheel_radius = 9.0
carriage_mount_hole_rad = 0.8


@dataclass
class Path:
    path: list[str]

    def __str__(self):
        return ".".join(self.path)

    def matches(self, sub: list[str]) -> bool:
        idx = 0
        for s in sub:
            try:
                idx = self.path.index(s, idx)
            except:
                return False
        return True

    def __add__(self, other: Self | str) -> Self:
        if isinstance(other, Path):
            return Path(self.path + other.path)
        if isinstance(other, str):
            return Path(self.path + [other])
        raise Exception("invalid type")


@dataclass
class TreeNode[T]:
    path: Path
    value: T


def ancestor_joints(compound: Compound) -> dict[str, Joint]:
    result = {}
    for anc in compound.ancestors:
        if isinstance(anc, Solid) or isinstance(anc, Compound):
            for j in anc.joints:
                result[j] = anc.joints[j]
    return result


def walk_joints(node: Compound | Shape | Part) -> Iterator[TreeNode[Joint]]:
    def rec(path: Path, node: Compound | Shape | Part):
        for c in node.joints:
            yield TreeNode(path, node.joints[c])
        for c in node.children:
            if isinstance(c, Shape) or isinstance(c, Compound) or isinstance(c, Part):
                yield from rec(path + c.label, c)

    yield from rec(Path([]), node)


def joint_dict(node: Compound | Shape | Part) -> dict[str, Joint]:
    return {str(n.path + n.value.label): n.value for n in walk_joints(node)}


def get_descendant(compound: Compound, label: str):
    for desc in compound.descendants:
        if isinstance(desc, Shape) and desc.label == label:
            return desc
    print(f"ERROR: no ancestor {label} in {compound.label}")
    print(compound.show_topology())


def build_servo():
    st3215 = import_step("resources/ST3215-simplified.step")
    st3215.label = "st3215"
    drive_wheel: Solid = None
    guide_wheel: Solid = None
    housing: Solid = None
    for descendant in st3215.children:
        if isinstance(descendant, Shape):
            match descendant.label:
                case "guide_wheel":
                    guide_wheel = descendant
                case "drive_wheel":
                    drive_wheel = descendant
                case "housing":
                    housing = descendant

    RevoluteJoint(
        "drive_axle",
        to_part=housing,
        axis=Location(Plane.XY, (-25.5, 0, -4.8)).to_axis().reverse(),
    )
    RevoluteJoint(
        "guide_axle",
        to_part=housing,
        axis=Location(Plane.YX, (-25.5, 0, 24.3)).to_axis().reverse(),
    )

    # wheel joints
    with Locations(Location(Plane.XY.reverse(), (-25.5, 0, -5.1))) as locs:
        RigidJoint(f"axle", to_part=drive_wheel, joint_location=locs.locations[0])
    with Locations(Location(Plane.XY.reverse(), (-25.5, 0, -9.6))) as locs:
        with PolarLocations(7, 4) as locs:
            for i, l in enumerate(locs):
                RigidJoint(f"mount_{i}", to_part=drive_wheel, joint_location=l)

    with Locations(Location(Plane.XY, (-25.5, 0, 24.3))) as locs:
        RigidJoint(f"axle", to_part=guide_wheel, joint_location=locs.locations[0])
    with Locations(Location(Plane.XY, (-25.5, 0, 27.65))) as locs:
        with PolarLocations(7, 4) as locs:
            for i, l in enumerate(locs):
                RigidJoint(f"mount_{i}", to_part=guide_wheel, joint_location=l)

    jp = Plane.XY.rotated(Vector(Z=180))
    # mount joints
    RigidJoint(
        label="guide_ne_mount",
        to_part=housing,
        joint_location=Location(jp, (-18.2 + 1, 10.25, 25.6)),
    )
    RigidJoint(
        label="guide_nw_mount",
        to_part=housing,
        joint_location=Location(jp, (-18.2 + 1, -10.25, 25.6)),
    )
    RigidJoint(
        label="guide_se_mount",
        to_part=housing,
        joint_location=Location(jp, (6.25 + 1, 10.25, 25.6)),
    )
    RigidJoint(
        label="guide_sw_mount",
        to_part=housing,
        joint_location=Location(jp, (6.25 + 1, -10.25, 25.6)),
    )

    RigidJoint(
        label="drive_nw_mount",
        to_part=housing,
        joint_location=Location(jp, (-18.2 + 1, 10.25, -6.4)),
    )
    RigidJoint(
        label="drive_ne_mount",
        to_part=housing,
        joint_location=Location(jp, (-18.2 + 1, -10.25, -6.4)),
    )
    RigidJoint(
        label="drive_sw_mount",
        to_part=housing,
        joint_location=Location(jp, (3.5, 10.25, -6.4)),
    )
    RigidJoint(
        label="drive_se_mount",
        to_part=housing,
        joint_location=Location(jp, (3.5, -10.25, -6.4)),
    )

    return st3215


def build_carriage(housing: Shape, tol: float, thickness: float):
    guide_bump_sy = 18.2
    servo_size = Vector(32, 24.62, 45.12)
    height_cut = Vector(0, 0, 23)
    carriage_size = servo_size - height_cut + Vector(1, 1, 1) * thickness * 2
    motor_cut_top_z = 1.55
    motor_cut_bot_z = -17.2
    motor_cut_sz = motor_cut_top_z - motor_cut_bot_z
    motor_bump_sy = 13.92
    guide_bump_start_dist_from_bot = 5.27

    motor_side_z = housing.faces().filter_by(Axis.Z).sort_by(Axis.Z)[3]
    guide_side_z = housing.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-5]
    motor_left_y = housing.faces().filter_by(Axis.Y).sort_by(Axis.Y)[0]


    #   carriage = (
    #     Workplane(origin=-height_cut / 2)
    #     .box(*carriage_size)
    #     .faces("+Z")
    #     .shell(-wall_thickness, "intersection")
    #     .faces('+X').faces('>X')
    #     .workplane(origin=(0,0,motor_cut_bot_z + carriage_size.z / 2))
    #     .rect(guide_bump_sy * tol, carriage_size.z)
    #     .extrude(-wall_thickness, combine='s')
    #     .faces('-X').faces('<X')
    #     .workplane(origin=(0, 0, 0))
    #     .rect(motor_bump_sy * tol, servo_size.z)
    #     .extrude(-wall_thickness, combine='s')
    # )

    with BuildPart(Plane.XY) as part:
        b = Box(*carriage_size)
        topf = faces().sort_by(Axis.Z)[-1]
        offset(amount=-thickness, openings=topf)
        motorf = faces(Select.LAST).sort_by(Axis.X)[-1]
        with BuildSketch(motorf) as s:
            Rectangle(motorf.width, guide_bump_sy * tol)
        extrude(s.sketch, amount=-thickness, mode=Mode.SUBTRACT)
        guidef = faces(Select.ALL).sort_by(Axis.X)[0]
        with BuildSketch(guidef) as s:
            Rectangle(guide_bump_sy, carriage_size.Z - guide_bump_start_dist_from_bot - thickness)
        extrude(s.sketch, amount=-thickness, mode=Mode.SUBTRACT)

    
    
    # return part
    # return housing.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-5]
    return housing.faces().filter_by(Plane.ZY)


def build_plate(length: float, thickness: float, wheel_dist: float):
    with BuildPart() as arm_plate:
        Box(length, 25, thickness)
        with Locations(
            Vector(X=-(length / 2 - 7 - servo_wheel_hole_radius - 2.5))
        ) as locs:
            with BuildSketch(locs.locations[0]) as sketch:
                Circle(9.6)
            extrude(sketch.sketch, -wheel_dist - thickness / 2)
            with Locations(Rot((180, 0, 0))):
                Hole(3.25, thickness + 5)
                with PolarLocations(7, 4) as locs:
                    Hole(servo_wheel_hole_radius, thickness + 5)
        with BuildPart() as cutout:
            Box(25, 25, thickness)
            Cylinder(12.5, thickness, mode=Mode.SUBTRACT)
            with Locations(Location((12.5, 0, 0))):
                Box(25, 25, thickness, mode=Mode.SUBTRACT)

        with Locations(
            Location(Vector(X=-(length / 2 - 7 - servo_mount_hole_radius - 5 + 0.19)))
        ):
            add(cutout, mode=Mode.SUBTRACT)
    return arm_plate


def build_limb(
    length: float = 150,
    thickness: float = 2.5,
    tol: float = 0.125,
    wheel_dist: float = 0,
    carriage_offset: float = 0,
    mirrored: bool = False,
):
    with BuildPart() as part:
        carriage = build_carriage(tol=tol, thickness=thickness).part
        carriage_bb = carriage.bounding_box().size
        if mirrored:
            carriage = carriage.rotate(Axis.X, 180)
            carriage = carriage.translate(Vector(Z=37.25/2 + .57))
        carriage = carriage.rotate(Axis.Y, 90)
        r = build_plate(length, thickness + .005, wheel_dist).part
        r.locate(
            Location(
                (
                    -9.03 - thickness / 2,
                    -length / 2 + carriage_bb.Y / 2,
                    -carriage_offset + .31,
                ),
                Vector(X=-90, Y=-90),
            )
        )
        l = copy.copy(r)
        l.locate(
            Location(
                (
                    28.23 + thickness / 2,
                    -length / 2 + carriage_bb.Y / 2,
                    -carriage_offset + .31,
                ),
                Vector(X=90, Y=90),
            )
        )
        add(carriage)
        add(r)
        add(l)

        if mirrored:
            drive_mount_hole_x = -11.53 + thickness / 2
            guide_mount_hole_x = 25.73 + thickness + thickness / 2
        else:
            guide_mount_hole_x = -11.53 + thickness / 2
            drive_mount_hole_x = 25.73 + thickness + thickness / 2
        with Locations(Location((guide_mount_hole_x, 10.25, -4.7 + 1.2))):
            with Locations(Plane.ZY):
                Cylinder(1.2, thickness, mode=Mode.SUBTRACT)
        with Locations(Location((guide_mount_hole_x, -10.25, -4.7 + 1.2))):
            with Locations(Plane.ZY):
                Cylinder(1.2, thickness, mode=Mode.SUBTRACT)
        with Locations(Location((drive_mount_hole_x, 10.25, -8.45 + 1.2))):
            with Locations(Plane.ZY):
                Cylinder(1.2, thickness, mode=Mode.SUBTRACT)
        with Locations(Location((drive_mount_hole_x, -10.25, -8.45 + 1.2))):
            with Locations(Plane.ZY):
                Cylinder(1.2, thickness, mode=Mode.SUBTRACT)

        # create mount joints
        # left wheel mount
        with Locations(
            Location(Plane.YZ.reverse(), (-9.03 + wheel_dist, -124, -carriage_offset))
        ):
            with PolarLocations(7, 4) as locs:
                for i, l in enumerate(locs):
                    RigidJoint(label=f"l_arm_mount_{i}", joint_location=l)
        with Locations(
            Location(Plane.ZY.reverse(), (28.23 - wheel_dist, -124, -carriage_offset))
        ):
            with PolarLocations(7, 4) as locs:
                for i, l in enumerate(locs):
                    RigidJoint(label=f"r_arm_mount_{i}", joint_location=l)

        # create carriage joint
        if mirrored:
            RigidJoint(
                label="carriage_guide_west",
                joint_location=Location((-6.53, 10.25, -8.05 + .8), Vector(Y=90, Z=180)),
            )
            RigidJoint(
                label="carriage_drive_west",
                joint_location=Location((25.72, -10.25, -4.3 + .8), Vector(Y=-90)),
            )  
        else:
            RigidJoint(
                label="carriage_drive_west",
                joint_location=Location((-6.53, 10.25, -4.3 + .8), Vector(Y=90, Z=180)),
            )
            RigidJoint(
                label="carriage_guide_west",
                joint_location=Location((25.73, -10.25, -8.05 + .8), Vector(Y=-90)),
            )

        return part


def connect_arm_to_servo_wheel(arm: Compound, servo: Compound):
    print("connecting", arm.label, "wheel to", servo.label)
    aj = joint_dict(arm)
    sj = joint_dict(servo)
    for i in range(4):
        sj[f"guide_wheel.mount_{i}"].connect_to(aj[f"l_arm_mount_{i}"])
        sj[f"drive_wheel.mount_{i}"].connect_to(aj[f"r_arm_mount_{i}"])


def connect_servo_to_arm_mount(arm: Compound, servo: Compound):
    print("connecting", arm.label, "mount to", servo.label)
    aj = joint_dict(arm)
    sj = joint_dict(servo)
    aj['carriage_guide_west'].connect_to(sj['housing.guide_sw_mount'])
    aj['carriage_drive_west'].connect_to(sj['housing.drive_sw_mount'])


def connect_wheels_to_housing(servo: Compound):
    j = joint_dict(servo)
    j["housing.guide_axle"].connect_to(j["guide_wheel.axle"], angle=0)
    j["housing.drive_axle"].connect_to(j["drive_wheel.axle"], angle=0)


def build_full_arm():
    servos = [build_servo()]
    servos[0].label = "servo_0"
    for i in range(1, 3):
        servos.append(copy.copy(servos[0]))
        servos[i].label = f"servo_{i}"
    bicep = build_limb(carriage_offset=6).part
    bicep.label = "bicep"
    forearm = build_limb(carriage_offset=6, mirrored=True).part
    forearm.label = "forearm"
    connect_wheels_to_housing(servos[0])
    connect_arm_to_servo_wheel(bicep, servos[0])
    connect_servo_to_arm_mount(bicep, servos[1])
    connect_wheels_to_housing(servos[1])
    connect_arm_to_servo_wheel(forearm, servos[1])
    connect_servo_to_arm_mount(forearm, servos[2])
    connect_wheels_to_housing(servos[2])
    export_step(bicep, "bicep.step")
    export_step(forearm, "forearm.step")
    return Compound(label="arm", children=[bicep, forearm] + servos)

def build_test_arm():
    servo = build_servo()
    servo.label = 'servo'
    servo2 = copy.copy(servo)
    servo2.label = 'servo2'
    limb = build_limb(mirrored=True).part
    limb.label = 'limb'
    connect_arm_to_servo_wheel(limb, servo)
    connect_wheels_to_housing(servo)
    connect_servo_to_arm_mount(limb, servo2)
    # connect_wheels_to_housing(servo2)
    return Compound(label='bicep', children=[servo, servo2, limb])




# arm = build_full_arm()
# show(build_servo(), render_joints=True)
# show(build_full_arm(), render_joints=True)
servo = build_servo()
carriage = build_carriage(get_descendant(servo, 'housing'), 1.01, 2.5)
show(carriage, servo, render_joints=True)
# export_step(build_limb().part, "limb.step")
# show(build_test_arm(), render_joints=True)
