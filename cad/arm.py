from cadquery import *
from ocp_vscode import show

guide_axle_mount_radius = 3.02
motor_axle_mount_radius = 1.6
wheel_radius = 9.0
carriage_mount_hole_rad = 0.8

def build_servo() -> Assembly:
    imp = importers.importStep("resources/ST3215-simplified.step")
    imp = imp.translate((-9.56 + 45.12 / 2, 12.31 - 24.62 / 2, 6.4 - 32 / 2))
    imp = imp.rotate((0, 0, 0), (0, 1, 0), 90)
    [housing, guide_wheel, motor_wheel] = map(
        Workplane, sorted(imp.objects, key=lambda o: o.CenterOfBoundBox().z)
    )

    # motor_wheel = (
    #     motor_wheel.tag("base")
    #     .edges("%CIRCLE and <Z")
    #     .edges(selectors.RadiusNthSelector(0))
    #     .tag("motor_wheel.mount")
    #     .edges(">X")
    #     .tag("mount_top")
    #     .end()
    #     .edges("<X")
    #     .tag("mount_bot")
    #     .workplaneFromTagged("base")
    #     .edges(selectors.RadiusNthSelector(1))
    #     .edges(">Z")
    #     .tag("axle")
    #     .workplaneFromTagged("base")
    # )

    # guide_wheel = (
    #     guide_wheel.tag("base")
    #     .edges("%CIRCLE and <Z")
    #     .edges(selectors.RadiusNthSelector(0))
    #     .tag("motor_wheel.axle")
    #     .workplaneFromTagged('base')
    #     .edges("%CIRCLE and >Z")
    #     .edges(selectors.RadiusNthSelector(0))
    #     .edges('<X')
    #     .tag("mount_top")
    #     .end()
    #     .edges('>X')
    #     .tag('mount_bot')
    #     .end()
    #     .workplaneFromTagged('base')
    # )

    servo: Assembly = Assembly(name="ST3215")
    servo = servo.add(motor_wheel, name="motor_wheel", color="silver")
    servo = servo.add(housing, name="housing", color="black")
    servo = servo.add(guide_wheel, name="guide_wheel", color="silver")

    # servo.constrain('motor_wheel?axle', 'FixedPoint', (0, 0, 0))
    # servo.constrain('guide_wheel?axle', 'FixedPoint', (0, 0, 0))
    # servo.constrain('motor_wheel?', '', '')
    return servo

def build_carriage_up_arm_side(wall_thickness: float, length: float):
    return (
        Workplane(origin=(0, 0, 0))
            .box(wheel_radius, length, wall_thickness)
            .edges('|Z').edges('<Y')
            .faces('>Z')
            .workplane(origin=(0, -length / 2 + wheel_radius))
        )
    
def build_carriage_forward_arm_side(wall_thickness: float, length: float, carriage_size: float):
    return (
        Workplane(origin=(0, 0, 0))
            .box(wheel_radius / 2, length, wall_thickness)
            .faces('>Z')
            .workplane(origin=(0, -length / 2 + carriage_size))
            .hole(carriage_mount_hole_rad * 2)
            .mirror('YZ', basePointVector=(wheel_radius /4, 0, 0))

            # .workplane(origin=(wheel_radius / 4, 0))
            # .mirror('ZY', union=True)
    )
    

def build_wheel_arm_side(axle_radius: float, wall_thickness: float, length: float):
    mount_hole_rad_from_center = 7.0
    hole_radius = 1.25
    return (
        Workplane(origin=(0, 0, 0))
            .box(wheel_radius * 2, length, wall_thickness)
            .edges('|Z').edges('<Y')
            .fillet(wheel_radius - .01)
            .faces('>Z')
            .workplane(origin=(0, -length / 2 + wheel_radius))
            .hole(axle_radius * 2)
            .polarArray(mount_hole_rad_from_center, 0, 360, count=4)
            .hole(hole_radius * 2)
        )
    
def build_carriage(housing: Shape, wall_thickness: float, tol: float):
    guide_bump_sy = 18.2
    servo_size = Vector(32, 24.62, 45.12)
    height_cut = Vector(0, 0, 23)
    carriage_size = servo_size - height_cut + Vector(1, 1, 1) * wall_thickness * 2
    motor_cut_top_z = 1.55
    motor_cut_bot_z = -17.2
    motor_cut_sz = motor_cut_top_z - motor_cut_bot_z
    motor_bump_sy = 13.92

    carriage = (
        Workplane(origin=-height_cut / 2)
        .box(*carriage_size)
        .faces("+Z")
        .shell(-wall_thickness, "intersection")
        .faces('+X').faces('>X')
        .workplane(origin=(0,0,motor_cut_bot_z + carriage_size.z / 2))
        .rect(guide_bump_sy * tol, carriage_size.z)
        .extrude(-wall_thickness, combine='s')
        .faces('-X').faces('<X')
        .workplane(origin=(0, 0, 0))
        .rect(motor_bump_sy * tol, servo_size.z)
        .extrude(-wall_thickness, combine='s')
    )

    return carriage

def build_bicep(length: float, thickness: float):
    wheel_mount_to_servo_frame_sx = 3.2

    guide_side = build_wheel_arm_side(guide_axle_mount_radius, thickness, length / 2)
    motor_side = build_wheel_arm_side(motor_axle_mount_radius, thickness, length / 2)





def build(tol: float):
    servo = build_servo()
    # show(servo)
    carriage = build_carriage(servo, 2.5, tol)
    # show(carriage)
    arm = Assembly().add(carriage, name="carriage").add(servo, name='servo')
    # for a in servo.children:
        # arm.add(a, name=f'servo_{a.name}')

    # housing = housing.pushPoints([bottom_center]).tag("bot_mount").end()
    # arm = arm.constrain('servo_housing?')
    # arm = arm.constrain('servo_housing', Vertex.makeVertex(0,0,0), 'carriage', Vertex.makeVertex(0, 0, 0), 'Point')
    # arm = arm.solve()
    return arm


# show(build(1.01))
servo = build_servo()

show(build_carriage_forward_arm_side(2.5, 75, 10))
# motor_arm = build_wheel_arm(guide_axle_mount_radius, 2.5, 100)
# show(motor_arm, servo)
# exporters.export(motor_arm, 'arm.step', 'STEP')
# show(arm)
# show(build_carriage(build_servo(), 3))
# show(build_servo())

# show(build_servo())