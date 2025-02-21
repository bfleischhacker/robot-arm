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
from enum import Enum, auto

UNIT_VEC = Vector(1, 1, 1)

# servo_mount_screw = fastener.ButtonHeadScrew(size='M1.6-0.35', length=3 * MM)
m3 = fastener.ButtonHeadScrew(size="M3-0.5", length=3)
print(m3.thread_diameter, m3.thread_pitch)

importer = Mesher()
servo_wheel_hole_radius = m3.thread_diameter / 2 * 1.05
servo_mount_hole_radius = .8
guide_axle_mount_radius = 3.02 * 1.05
motor_axle_mount_radius = 1.6 * 1.05
wheel_radius = 9.0
carriage_mount_hole_rad = .8
carriage_mount_hole_head_rad = 1.4
wheel_mount_hole_rad_from_center = 7
arm_width = wheel_radius * 1.1


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
    st3215 = import_step("resources/ST3215-simplified.step").locate(Location(Plane.XY, (0, 0, 0)))
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
    with Locations(Location(Plane.XY, (-25.5, 0, -9.6))) as locs:
        with PolarLocations(7, 4) as locs:
            for i, l in enumerate(locs):
                RigidJoint(f"mount_{i}", to_part=drive_wheel, joint_location=l)

    with Locations(Location(Plane.XY, (-25.5, 0, 24.3))) as locs:
        RigidJoint(f"axle", to_part=guide_wheel, joint_location=locs.locations[0])
    with Locations(Location(Plane.XY.reverse(), (-25.5, 0, 27.65))) as locs:
        with PolarLocations(7, 4) as locs:
            for i, l in enumerate(locs):
                RigidJoint(f"mount_{i}", to_part=guide_wheel, joint_location=l)

    guide_jp = Plane.XY.rotated((180, 0, 0))
    motor_jp = Plane.XY.rotated((0, 0, 0))

    # mount joints
    RigidJoint(
        label="guide_mount_ne",
        to_part=housing,
        joint_location=Location(guide_jp, (-18.2 + 1, 10.25, 25.6)),
    )
    RigidJoint(
        label="guide_mount_nw",
        to_part=housing,
        joint_location=Location(guide_jp, (-18.2 + 1, -10.25, 25.6)),
    )
    RigidJoint(
        label="guide_mount_se",
        to_part=housing,
        joint_location=Location(guide_jp, (6.25 + 1, 10.25, 25.6)),
    )
    RigidJoint(
        label="guide_mount_sw",
        to_part=housing,
        joint_location=Location(guide_jp, (6.25 + 1, -10.25, 25.6)),
    )

    RigidJoint(
        label="motor_mount_nw",
        to_part=housing,
        joint_location=Location(motor_jp, (-18.2 + 1, 10.25, -6.4)),
    )
    RigidJoint(
        label="motor_mount_ne",
        to_part=housing,
        joint_location=Location(motor_jp, (-18.2 + 1, -10.25, -6.4)),
    )
    RigidJoint(
        label="motor_mount_sw",
        to_part=housing,
        joint_location=Location(motor_jp, (3.5, 10.25, -6.4)),
    )
    RigidJoint(
        label="motor_mount_se",
        to_part=housing,
        joint_location=Location(motor_jp, (3.5, -10.25, -6.4)),
    )

    return st3215


def build_carriage(tol: float, thickness: float):
    thickness 
    guide_bump_sy = 18.2
    motor_cut_top_z = 1.55
    motor_cut_bot_z = -17.2
    motor_cut_sz = motor_cut_top_z - motor_cut_bot_z
    motor_bump_sy = 13.92
    guide_bump_bot_sz = 5.49

    bot_guide_mount_sz_from_bot = 2.31
    bot_motor_mount_sz_from_bot = 6.06
    top_mount_sz_from_bot = 26.76
    mount_sy = 20.5
    mount_sx_from_middle = 31.6

    servo_size = Vector(32, 24.62, 45.12)
    carriage_size = servo_size + Vector(2, 2, 2) * thickness
    motor_mount_sz = top_mount_sz_from_bot - bot_motor_mount_sz_from_bot
    guide_mount_sz = top_mount_sz_from_bot - bot_guide_mount_sz_from_bot

    height_cut_z = 23

    hole_names = ['se', 'sw']

    with BuildPart() as part:
        Box(*carriage_size)
        topf = faces().sort_by(Axis.Z)[-1]
        offset(amount=-thickness, openings=topf)
        motorf = faces(Select.ALL).sort_by(Axis.X)[-1].moved(
            Location((-thickness / 2, 0, 0))
        )
        # motor side holes
        with BuildPart(motorf, mode=Mode.SUBTRACT):
            with Locations(Location(Vector(X=-carriage_size.Z / 2 + thickness + bot_motor_mount_sz_from_bot))):
                with GridLocations(0, mount_sy, 1, 2):
                    Cylinder(servo_mount_hole_radius, thickness)
                    hole = faces(Select.LAST).filter_by(GeomType.CYLINDER)
                    with Locations(Location((0, 0, -thickness/2), (180, 0, -180))) as locs:
                        for i, l in enumerate(locs):
                            RigidJoint(f'motor_mount_{hole_names[i]}', part, l)
            # motor side cutout
            with Locations(Location((thickness / 2, 0, 0))):
                Box(carriage_size.Z - thickness, motor_bump_sy * tol, thickness)
        
        guidef = faces(Select.ALL).sort_by(Axis.X)[0].moved(
            Location((thickness / 2, 0, 0))
        )
        with BuildPart(guidef, mode=Mode.SUBTRACT):
            with Locations(Location(Vector(X=-carriage_size.Z / 2 + thickness + bot_guide_mount_sz_from_bot))):
                with GridLocations(0, mount_sy, 1, 2):
                    Cylinder(servo_mount_hole_radius, thickness)
                    hole = faces(Select.LAST).filter_by(GeomType.CYLINDER)
                    with Locations(Location((0, 0, -thickness/2), (180, 0, -180))) as locs:
                        for i, l in enumerate(locs):
                            RigidJoint(f'guide_mount_{hole_names[i]}', part, l)
            with Locations(Location((thickness / 2 + guide_bump_bot_sz / 2, 0, 0))):
                Box(carriage_size.Z - thickness - guide_bump_bot_sz * tol, guide_bump_sy * tol, thickness)
                    
        
        # chop off top
        with Locations(Location((0, 0, carriage_size.Z / 2 - height_cut_z / 2))):
            Box(carriage_size.X, carriage_size.Y, height_cut_z, mode=Mode.SUBTRACT)

    
    return part

def build_wheel_arm(length: float, thickness: float, axle_radius: float):
    with BuildPart() as arm:
        b = Box(length, arm_width * 2, thickness)
        f = [edges(Select.ALL).sort_by(Axis.X)[-2], edges(Select.ALL).sort_by(Axis.X)[-4]]
        fillet(f, arm_width - .001)
        with Locations(Location((length/2-wheel_radius, 0, 0))):
            Hole(axle_radius, thickness)
            with PolarLocations(wheel_mount_hole_rad_from_center, 4) as locs:
                Hole(servo_wheel_hole_radius, thickness)
                for i,l in enumerate(locs):
                    RigidJoint(label=f'left_wheel_mount_{i}', to_part=arm, joint_location=l)

    return arm

def build_bicep(
    length: float = 150,
    thickness: float = 2.5,
    tol: float = 1.01,
):
    with BuildPart() as bicep:
        carriage = build_carriage(tol, thickness)
        add(carriage)
        for l,j in carriage.joints.items():
            bicep.joints[l] = j
        guidef = faces(Select.LAST).sort_by(Axis.X)[0]
        motorf = faces(Select.LAST).sort_by(Axis.X)[-1]
        botz = faces(Select.LAST).sort_by(Axis.Z)[0].location.position.Z
        carriage_sy = guidef.faces()[0].width / 2
        motor_arm = build_wheel_arm(length, thickness, motor_axle_mount_radius)
        guide_arm = build_wheel_arm(length, thickness, guide_axle_mount_radius)
        with BuildPart(motorf):
            x = -(motorf.center().Z - botz)+arm_width
            y = -length / 2 + motorf.bounding_box().size.Y / 2
            with Locations(Location((x, y, thickness / 2), Vector(Z=-90))) as loc:
                add(motor_arm)
        with Locations([bicep.joints['motor_mount_sw'].location, bicep.joints['motor_mount_se'].location]):
            with Locations(Location((0, 0, -thickness * 1.5))):
                Hole(carriage_mount_hole_head_rad, thickness / 2)
        with BuildPart(guidef):
            x = -(guidef.center().Z - botz)+arm_width
            y = length / 2 - guidef.bounding_box().size.Y / 2
            with Locations(Location((x, y, thickness / 2), Vector(Z=90))):
                add(guide_arm)
        with Locations([bicep.joints['guide_mount_sw'].location, bicep.joints['guide_mount_se'].location]):
            with Locations(Location((0, 0, -thickness * 1.5))):
                Hole(carriage_mount_hole_head_rad, thickness / 2)
        cross_barf = faces(Select.ALL).filter_by(Axis.Y).sort_by(Axis.Y)[-3]
        with BuildPart(cross_barf):
            with Locations(Location((-cross_barf.length / 2 + arm_width, 0, (length - motorf.width)/2))):
                r = Box(arm_width * 2, cross_barf.width, thickness)
                # with BuildPart(motor_armf):
            # return 
        motor_arm_mount_holesf = faces(Select.ALL).filter_by(GeomType.CYLINDER).sort_by(Axis.X).sort_by(Axis.Y)[8:22]
        motor_arm_mount_holesf = motor_arm_mount_holesf.sort_by(Axis.X)[7:]
        motor_arm_mount_holesf = motor_arm_mount_holesf.sort_by(SortBy.RADIUS)[:4]
        for i, f in enumerate(motor_arm_mount_holesf.faces()):
            RigidJoint(f'motor_wheel_mount_{i}', bicep, Location(f.bounding_box().center() + (-thickness / 2, 0, 0), (0, -90, 0)))
        guide_arm_mount_holesf = faces(Select.ALL).filter_by(GeomType.CYLINDER).sort_by(Axis.X).sort_by(Axis.Y)[8:22]
        guide_arm_mount_holesf = guide_arm_mount_holesf.sort_by(Axis.X)[:7]
        guide_arm_mount_holesf = guide_arm_mount_holesf.sort_by(SortBy.RADIUS)[:4]
        for i, f in enumerate(guide_arm_mount_holesf.faces()):
            RigidJoint(f'guide_wheel_mount_{i}', bicep, Location(f.bounding_box().center() + (thickness / 2, 0, 0), (0, 90, 0)))
    return bicep
    
def build_forearm(
    length: float = 150,
    thickness: float = 2.5,
    tol: float = 1.01,
):
    carriage = build_carriage(tol, thickness)
    carriage_motorf = carriage.faces(Select.ALL).filter_by(GeomType.PLANE).filter_by(Axis.X).sort_by(Axis.X)[-1] 
    carriage_guidef = carriage.faces(Select.ALL).filter_by(GeomType.PLANE).filter_by(Axis.X).sort_by(Axis.X)[0]
    carriage_botz = carriage.faces(Select.ALL).filter_by(GeomType.PLANE).filter_by(Axis.Z).sort_by(Axis.Z)[0].center().Z
    carriage_topz = carriage.faces(Select.ALL).filter_by(GeomType.PLANE).filter_by(Axis.Z).sort_by(Axis.Z)[-1].center().Z
    cj_mot_bot = RigidJoint('carriage_motor_side_to_arm_mount_bot', carriage.part, Location((carriage_motorf.center().X, carriage_motorf.center().Y, carriage_botz), (0, 90, 0)))
    cj_mot_top = RigidJoint('carriage_motor_side_to_arm_mount_top', carriage.part, Location((carriage_motorf.center().X, carriage_motorf.center().Y, carriage_topz), (0, 90, 0)))
    cj_guide_bot = RigidJoint('carriage_guide_side_to_arm_mount_bot', carriage.part, Location((carriage_guidef.center().X, carriage_motorf.center().Y, carriage_botz), (0, 90, 0)))
    cj_guide_top = RigidJoint('carriage_guide_side_to_arm_mount_top', carriage.part, Location((carriage_guidef.center().X, carriage_motorf.center().Y, carriage_topz), (0, 90, 0)))
    carriage_sz = carriage_topz - carriage_botz
    carriage_sx = carriage_motorf.center().X - carriage_guidef.center().X
    carriage_sy = carriage_motorf.length
    
    with BuildPart() as motor_arm:
        with Locations(Location((-length / 4, 0, 0), (0, 0, 180))):
            front = build_wheel_arm(length / 2, thickness, motor_axle_mount_radius)
            add(front)
        with Locations(Location((length / 4, 0, 0))):
            Box(length/2, arm_width * 2, thickness)
    mj_bot = RigidJoint('motor_arm_carriage_side_bot_mount', motor_arm.part, Location((length / 2, 0, -thickness / 2), (0, 0, 180)))
    mj_top = RigidJoint('motor_arm_carriage_side_top_mount', motor_arm.part, Location((length / 2 - carriage_sz, 0, -thickness / 2), (0, 0, 180)))

    with BuildPart() as guide_arm:
        with Locations(Location((-length / 4, 0, 0), (0, 0, 180))):
            front = build_wheel_arm(length / 2, thickness, guide_axle_mount_radius)
            add(front)
        with Locations(Location((length / 4, 0, 0))):
            Box(length/2, arm_width * 2, thickness)
    gj_bot = RigidJoint('guide_arm_carriage_side_bot_mount', guide_arm.part, Location((length / 2, 0, thickness / 2), (0, 0, 180)))
    gj_top = RigidJoint('guide_arm_carriage_side_top_mount', guide_arm.part, Location((length / 2 - carriage_sz, 0, thickness / 2), (0, 0, 180)))
    
    cj_mot_top.connect_to(mj_bot)
    cj_mot_bot.connect_to(mj_top)
    cj_guide_bot.connect_to(gj_top)
    cj_guide_top.connect_to(gj_bot)
    part = carriage.part + motor_arm.part + guide_arm.part
    with BuildPart(carriage_motorf) as p:
        with Locations(Location((0, 0, thickness/2))):
            extrude(carriage_motorf, amount=thickness)
    part += p.part
    with BuildPart(carriage_guidef) as p:
        with Locations(Location((0, 0, thickness/2))):
            extrude(carriage_guidef, amount=thickness)
    part += p.part
    
    jls = [part.joints[k].location for k in part.joints if k.startswith('motor_mount_') or k.startswith('guide_mount_')]
    with BuildPart() as p:
        with Locations(jls):
            with Locations(Location((0,0,-thickness - thickness))):
                Cylinder(carriage_mount_hole_head_rad, thickness * 1.5)
                Cylinder(carriage_mount_hole_rad, thickness * 2)
    part -= p.part

    arm_mount_holesf = part.faces().filter_by(GeomType.CYLINDER).sort_by(Axis.X).sort_by(Axis.Y)[5:19]
    motor_arm_mount_holesf = arm_mount_holesf.sort_by(Axis.X)[7:]
    motor_arm_mount_holesf = motor_arm_mount_holesf.sort_by(SortBy.RADIUS)[:4]
    for i, f in enumerate(motor_arm_mount_holesf.faces()):
        RigidJoint(f'motor_wheel_mount_{i}', part, Location(f.bounding_box().center() + (-thickness / 2, 0, 0), (0, -90, 0)))
    guide_arm_mount_holesf = arm_mount_holesf.sort_by(Axis.X)[:7]
    guide_arm_mount_holesf = guide_arm_mount_holesf.sort_by(SortBy.RADIUS)[:4]
    for i, f in enumerate(guide_arm_mount_holesf.faces()):
        RigidJoint(f'guide_wheel_mount_{i}', part, Location(f.bounding_box().center() + (thickness / 2, 0, 0), (0, 90, 0)))

    return part


def connect_arm_to_servo_wheel(arm: Compound, servo: Compound):
    print("connecting", arm.label, "wheel to", servo.label)
    aj = joint_dict(arm)
    sj = joint_dict(servo)
    for i in range(0, 4, 2):
        sj[f"guide_wheel.mount_{i}"].connect_to(aj[f"guide_wheel_mount_{i}"])
        sj[f"drive_wheel.mount_{i}"].connect_to(aj[f"motor_wheel_mount_{i}"])


def connect_servo_to_arm_mount(arm: Compound, servo: Compound):
    print("connecting", arm.label, "mount to", servo.label)
    aj = joint_dict(arm)
    sj = joint_dict(servo)
    print(aj.keys())
    aj['guide_mount_sw'].connect_to(sj['housing.guide_mount_sw'])
    aj['motor_mount_sw'].connect_to(sj['housing.motor_mount_sw'])


def connect_wheels_to_housing(servo: Compound, angle: float):
    j = joint_dict(servo)
    j["housing.guide_axle"].connect_to(j["guide_wheel.axle"], angle=angle)
    j["housing.drive_axle"].connect_to(j["drive_wheel.axle"], angle=angle)


def build_full_arm():
    servos = [build_servo()]
    servos[0].label = "servo_0"
    for i in range(1, 3):
        servos.append(copy.copy(servos[0]))
        servos[i].label = f"servo_{i}"
    bicep = build_bicep(150, 2.5, 1.01).part
    bicep.label = "bicep"
    forearm = build_forearm(150, 2.5, 1.01)
    forearm.label = "forearm"
    servos[0].rotate(Axis.Y, -90)
    connect_wheels_to_housing(servos[0], 180)
    connect_arm_to_servo_wheel(bicep, servos[0])
    connect_servo_to_arm_mount(bicep, servos[1])
    connect_wheels_to_housing(servos[1], angle=90)
    connect_arm_to_servo_wheel(forearm, servos[1])
    connect_servo_to_arm_mount(forearm, servos[2])
    connect_wheels_to_housing(servos[2], 0)
    # export_step(forearm, "forearm.step")
    return Compound(label="arm", children=[bicep, forearm] + servos)


# arm = build_full_arm()
# show(build_servo(), render_joints=True)
# show(build_full_arm(), render_joints=True)
# servo = build_servo()
# carriage = build_carriage(1.01, 2.0)
# limb = build_bicep().part
# limb.joints['motor_mount_se'].connect_to(get_descendant(servo, 'housing').joints['motor_mount_se'])
# limb.joints['motor_mount_sw'].connect_to(get_descendant(servo, 'housing').joints['motor_mount_sw'])
# connect_servo_to_arm_mount(limb, servo)
# connect_wheels_to_housing(servo)
arm = build_full_arm()
show(arm, render_joints=True)
# export_step(build_limb().part, "limb.step")
# show(build_test_arm(), render_joints=True)
# export_step(build_carriage(1.01, 2).part, 'carriage_test.step')
# export_step(limb, 'limb.step')