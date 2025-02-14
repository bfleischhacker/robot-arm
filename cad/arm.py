from build123d import *
from build123d.topology import Shape
from ocp_vscode import show_all, show, show_object, reset_show, set_port, set_defaults, get_defaults
from bd_warehouse import fastener
import copy
from dataclasses import dataclass
from typing import Self, Iterator
from anytree import Node


# servo_mount_screw = fastener.ButtonHeadScrew(size='M1.6-0.35', length=3 * MM)
m3 = fastener.ButtonHeadScrew(size='M3-0.5', length=3)
# print(m3.joints)

# set_port(3939)

importer = Mesher()
# importer.read()
# servo_mount = import_stl('servo_holder.stl')
# servo_mount.label = 'case'
servo_wheel_hole_radius = m3.thread_diameter * 1.05 / 2
servo_mount_hole_radius = 1.3 * 1.05 / 2

@dataclass
class Path:
    path: list[str] 

    def __str__(self):
        return '.'.join(self.path)

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
        raise Exception('invalid type')
        

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
    return { str(n.path + n.value.label): n.value for n in walk_joints(node) }

def get_descendant(compound: Compound, label: str):
    for desc in compound.descendants:
        if isinstance(desc, Shape) and desc.label == label:
            return desc
    print(f'ERROR: no ancestor {label} in {compound.label}')
    print(compound.show_topology())
        
def build_servo():
    st3215 = import_step('resources/ST3215-simplified.step')
    st3215.label = 'st3215'
    drive_wheel: Solid = None
    guide_wheel: Solid = None
    housing: Solid = None
    for descendant in st3215.children:
        if isinstance(descendant, Shape):
            match descendant.label:
                case 'guide_wheel':
                    guide_wheel = descendant
                case 'drive_wheel':
                    drive_wheel = descendant
                case 'housing':
                    housing = descendant

    def prepare_wheel(wheel: Solid, axel: Location, pin_face: Face):
        RigidJoint(f'{wheel.label}_axel',to_part=wheel, joint_location=axel)
        with Locations(pin_face.center_location) as loc: 
            with PolarLocations(7, 4) as locs:
                with Locations(Rot((180, 0, 0))) as locs:
                  for i, l in enumerate(locs):
                        RigidJoint(f'{wheel.label}_mount_{i}',to_part=wheel, joint_location=l)
    prepare_wheel(drive_wheel, drive_wheel.faces().sort_by(Axis.Y)[0].center_location, drive_wheel.faces().sort_by(Axis.Y)[-1])
    prepare_wheel(guide_wheel, guide_wheel.faces().sort_by(Axis.Y)[-1].center_location, guide_wheel.faces().sort_by(Axis.Y)[0])
    RevoluteJoint('motor_gear', to_part=housing, axis=Location(Plane.XZ, (-25.5, 5.1, 0)).to_axis())
    RevoluteJoint('guide_axel' , to_part=housing, axis=Location(Plane.XZ.reverse(), (-25.5, -24.2, 0)).to_axis())
    return st3215

def build_carriage(tol: float = 0.5):
    with BuildPart() as part:
        # with Locations(Location(Vector(Y=(44.92 + tol) / 2))):
            # Box(44.92 + tol, 24.62 + tol, 32 + tol)
            # with Locations(Rot(Axis.Z, 180)):
        housing = get_descendant(build_servo(), 'housing')
        housing.rotate(Axis.Z, 90)
        add(housing)
        Sphere(7)
        # Box(10, 10, 10)
        # with Locations(Location(Vector(-1, -27.35, 22.71))):
            # Box(10, 10, 10, mode=Mode.ADD)
            # add(housing)
            
    return part

def build_plate(length: float = 150 * MM, thickness: float = 3 * MM):
    with BuildPart() as arm_plate:
        Box(length, 25, thickness)
        with Locations(Vector(X=-(length / 2 - 7 - servo_wheel_hole_radius - 5))) as locs:
            with BuildSketch(locs.locations[0]) as sketch:
                Circle(9.6)
            extrude(sketch.sketch, -3)
            with BuildSketch(locs.locations[0]) as sketch:
                Circle(9.8)
                Circle(9.6, mode=Mode.SUBTRACT)
            extrude(sketch.sketch, -3.4)
            with Locations(Rot((180, 0, 0))):
                Hole(servo_wheel_hole_radius, thickness + 5)
                with PolarLocations(7, 4) as locs:
                    Hole(servo_wheel_hole_radius, thickness + 5)
        with Locations(Rot(Plane.XZ)):
            with Locations(Location(((length / 2 - 25.5 / 2), -4.45 / 2, 0))):
                Box(25.5, 29.45, thickness)
                with GridLocations(21.5, 25.45, 2, 2):
                    Hole(servo_mount_hole_radius, thickness)
                
                # Box(5, thickness, 5, mode=Mode.SUBTRACT)
        # 25.45 Y 21.5 X
    return arm_plate
    
def build_arm(length: float = 150, thickness: float = 3):
    with BuildPart() as arm:
        r = build_plate(length, thickness).part
        add(r.moved(Rot((-90, 0, 0))))
        l = copy.copy(r)
        mirror(l.moved(Rot((-90, 0, 0))), about=Plane.XZ.offset((37.25 + 5) / 2))
        with Locations(Location(Plane.XZ.reverse(), (-61.42, -39.25, 0))):
            with PolarLocations(7, 4) as locs: 
                for i, l in enumerate(locs):
                    RigidJoint(f'left_wheel_mount_{i}', arm, joint_location=l)
        with Locations(Location(Plane.XZ, (-61.42, -3, 0))):
            with PolarLocations(7, 4) as locs: 
                for i, l in enumerate(locs):
                    RigidJoint(f'right_wheel_mount_{i}', arm, joint_location=l)
        # with Locations(Location(Plane.XZ, ()))
        #     with GridLocations(21.5, 25.45, 2, 2) as locs:
        #         for i, l in enumerate(locs):
        #             RigidJoint(f'right_house_mount_{i}', arm, joint_location=l)
         
    return arm

def connect_arm_to_servo_wheel(arm: Compound, servo: Compound):
    print('connecting', arm.label, 'wheel to', servo.label)
    aj = joint_dict(arm)
    sj = joint_dict(servo)
    for i in range(4):
        aj[f'left_wheel_mount_{i}'].connect_to(sj[f'guide_wheel.guide_wheel_mount_{i}'])
        aj[f'right_wheel_mount_{i}'].connect_to(sj[f'drive_wheel.drive_wheel_mount_{i}'])
    sj[f'guide_wheel.guide_wheel_axel'].connect_to(sj[f'housing.guide_axel'])
    sj[f'drive_wheel.drive_wheel_axel'].connect_to(sj[f'housing.motor_gear'])

def connect_arm_to_servo_mount(arm: Compound, servo: Compound):
    print('connecting', arm.label, 'mount to', servo.label)
    pass

def build_full_arm():
    servos = [build_servo()]
    servos[0].label = 'servo_0'
    for i in range(1, 3):
        servos.append(copy.copy(servos[0]))
        servos[i].label = f'servo_{i}'
    backarm = build_arm().part
    backarm.label = 'backarm'
    forearm = copy.copy(backarm)
    forearm.label = 'forearm'
    connect_arm_to_servo_wheel(backarm, servos[0])
    connect_arm_to_servo_mount(backarm, servos[1])
    connect_arm_to_servo_wheel(forearm, servos[1])
    connect_arm_to_servo_mount(forearm, servos[2])
    return Compound(label='arm', children=[backarm, forearm, *servos])

# show(arm, render_joints=True)
carriage = build_carriage()
servo = build_servo()
show(carriage, servo)
# export_stl(backarm, 'arms.stl')
    

