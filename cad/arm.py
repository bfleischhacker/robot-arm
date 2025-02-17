
from cadquery import *
from ocp_vscode import show


def build_servo() -> Assembly:
    imp = importers.importStep("resources/ST3215-simplified.step")
    [motor_wheel, housing, guide_wheel] = map(Workplane, sorted(
        imp.objects, key=lambda o: o.CenterOfBoundBox().z))
    motor_wheel = (
        motor_wheel
        .tag('base')
        .edges('%CIRCLE and <Z')
        .edges(selectors.RadiusNthSelector(0))
        .tag('motor_wheel.mount')
        .edges('>X').tag('mount_top')
        .end()
        .edges('<X').tag('mount_bot') 
        .workplaneFromTagged('base')
        .edges(selectors.RadiusNthSelector(1))
        .edges(">Z")
        .tag('axle')
        .workplaneFromTagged('base')
    )

    # guide_wheel = tag_mounts('guide_wheel', guide_wheel)
    # motor_wheel = tag_mounts('motor_wheel', motor_wheel)
    servo: Assembly = Assembly(name='ST3215')
    servo = servo.add(motor_wheel, name='motor_wheel', color='silver')
    servo = servo.add(housing, name='housing', color='black')
    servo = servo.add(guide_wheel, name='guide_wheel', color='silver')
    # servo.constrain('motor_wheel?axle', 'FixedPoint', (0, 0, 0))
    # servo.constrain('drive_wheel?axle', 'FixedPoint', (0, 0, 0))
    # servo.constrain('motor_wheel?', '', '')
    return servo

def build_carriage(servo: Assembly, wall_thickness: float):
    housing = Workplane().add(servo.objects['housing'].shapes[0])
    housing.shells()
    guide_bump = housing.faces('<Z[1]').objects[0].CenterOfBoundBox().z
    guide_edge = housing.faces('<Z[4]').objects[0].CenterOfBoundBox().z
    motor_bump = housing.faces('>Z[1]').objects[0].CenterOfBoundBox().z
    motor_edge = housing.faces('>Z[3]').objects[0].CenterOfBoundBox().z
    servo_sz = guide_edge - motor_edge

    bottom = housing.faces('>X').objects[0].CenterOfBoundBox().x
    top = housing.faces('<X').objects[0].CenterOfBoundBox().x
    servo_sx = bottom - top

    left = housing.faces('>Y').objects[0].CenterOfBoundBox().y
    right = housing.faces('<Y').objects[0].CenterOfBoundBox().y
    servo_sy = left - right


    return (
        Workplane()
            # .add(housing)
            .box(servo_sx + wall_thickness * 2, servo_sy + wall_thickness * 2, servo_sz + wall_thickness * 2)
            .shell(wall_thickness)
        
            # .cut(housing)
            # .box(servo_sx / 2 + wall_thickness, servo_sy + wall_thickness, servo_sz + wall_thickness, combine='s')
            
            # .box(guide_edge.x - guide_bump.x, right.y)
    )
        


servo = build_servo()
carriage = build_carriage(servo, 3)
show(servo, carriage)

