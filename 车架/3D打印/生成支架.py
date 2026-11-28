"""Generate the V18 printable mounts in millimetres with build123d 0.11."""

from math import cos, hypot, radians, sin
from pathlib import Path
import re

from build123d import Align, Axis, Box, Cylinder, Face, Pos, Wire, export_step, export_stl, extrude


OUTPUT = Path(__file__).resolve().parent
DRIVER_CENTER = (51.5, -8.0)
DRIVER_HOLE_PITCH = 44.0
TRAY_FOOT_PITCH = 76.0
TRAY_DECK_THICKNESS = 4.5
TRAY_FOOT_EXTENSION = 4.0
TRAY_STANDOFF = 28.0

# PCB faces point towards -X. Positive yaw points out towards +Y on the car.
ULTRASONIC_ANCHORS = ((-44.0, 0.0, 0.0), (-35.0, -49.5, -30.0), (-35.0, 49.5, 30.0))
ULTRASONIC_FOOT_HOLES = ((15.5, -15.5), (15.5, 15.5))
ULTRASONIC_SENSOR_HEIGHT = 12.5  # Relative to the underside of the top deck.
V18_UPPER_OUTLINE = ((-34,-50), (-24,-60), (118,-60), (124,-54),
                     (124,54), (118,60), (-24,60), (-34,50))

CAMERA_ANCHOR = (-30.0, 0.0)
CAMERA_FOOT_SPACING = 17.0
CAMERA_PIVOT_HEIGHT = 59.0
CAMERA_PIVOT_BACKSET = 24.0
CAMERA_PITCH = 30.0
CAMERA_PCB_SPACER = 3.0


def block(x0, x1, y0, y1, z0, z1):
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN,) * 3)


def prism(points, amount, direction):
    return extrude(Face(Wire.make_polygon(points, close=True)), amount=amount, dir=direction)


def cylinder_z(radius, height, x, y, z):
    return Pos(x, y, z) * Cylinder(radius, height, align=(Align.CENTER, Align.CENTER, Align.MIN))


def cylinder_x(radius, length, x, y, z):
    return cylinder_z(radius, length, 0, 0, 0).rotate(Axis.Y, 90).translate((x, y, z))


def cylinder_y(radius, length, x, y, z):
    return cylinder_z(radius, length, 0, 0, 0).rotate(Axis.X, -90).translate((x, y, z))


def slot_x(start, end, width, x0=-1, x1=5):
    """Cut a capsule with two centre points in the YZ mounting plane."""
    dy, dz = end[0] - start[0], end[1] - start[1]
    radius = width / 2
    distance = hypot(dy, dz)
    side_y, side_z = -dz * radius / distance, dy * radius / distance
    profile = [(x0, y + sign * side_y, z + sign * side_z)
               for (y, z), sign in ((start, -1), (end, -1), (end, 1), (start, 1))]
    return prism(profile, x1 - x0, (1, 0, 0)).fuse(
        cylinder_x(radius, x1 - x0, x0, *start), cylinder_x(radius, x1 - x0, x0, *end)
    )


def driver_tray():
    """Print with the surface that supports the driver down and the feet up."""
    half_driver = DRIVER_HOLE_PITCH / 2
    half_feet = TRAY_FOOT_PITCH / 2
    top = TRAY_DECK_THICKNESS
    rails = [block(-half_driver-5, half_driver+5, y-2.5, y+2.5, 0, top) for y in (-29, 29)]
    crossbars = [block(x-5, x+5, -half_feet-5, half_feet+5, 0, top)
                 for x in (-half_driver, half_driver)]
    feet = [cylinder_z(4.8, TRAY_FOOT_EXTENSION + 0.1, x, y, top - 0.1)
            for x in (-half_driver, half_driver) for y in (-half_feet, half_feet)]
    bores = [cylinder_z(1.75, top + TRAY_FOOT_EXTENSION + 2, x, y, -1)
             for x in (-half_driver, half_driver) for y in (-half_feet, -half_driver, half_driver, half_feet)]
    return rails[0].fuse(*rails[1:], *crossbars, *feet).cut(*bores)


def ultrasonic_bracket():
    """Bolt underneath the nose deck. The sensor sits on the front of its two uprights."""
    base = block(0, 5, -24.5, 24.5, -4, 0)
    attachments = []
    for side in (-1, 1):
        foot = [(0, 17), (0, 24.5), (5, 24.5), (11, 20.25),
                (20.5, 20.25), (20.5, 10.75), (9, 10.75)]
        attachments.append(prism([(x, side*y, -4) for x, y in foot], 4, (0, 0, 1)))
        low_y, high_y = sorted((side * 17, side * 24.5))
        attachments.append(block(0, 4, low_y, high_y, -1, 25.5))
        gusset_y = min(side * 18.4, side * 22.8)
        attachments.append(prism([(0, gusset_y, -0.6), (10, gusset_y, -0.6),
                                  (4, gusset_y, 11), (0, gusset_y, 11)], 4.4, (0, 1, 0)))
    base = base.fuse(*attachments)
    mounting = [cylinder_z(1.75, 6, x, y, -5) for x, y in ULTRASONIC_FOOT_HOLES]
    sensor = [slot_x((y-0.5, z), (y+0.5, z), 3.8, x0=-1, x1=11)
              for y in (-20.6, 20.6) for z in (ULTRASONIC_SENSOR_HEIGHT-8.4, ULTRASONIC_SENSOR_HEIGHT+8.4)]
    lower_z = ULTRASONIC_SENSOR_HEIGHT - 8.4
    # Let the lower washers sit flat even where the gussets cross the bolt line.
    nut_clearance = [slot_x((y-0.8, lower_z), (y+0.8, lower_z), 6.7, x0=4, x1=12)
                     for y in (-20.6, 20.6)]
    header_clearance = block(-1, 6, -9, 9, -1.6, 1)
    return base.cut(*mounting, *sensor, *nut_clearance, header_clearance)


def camera_mast():
    height = CAMERA_PIVOT_HEIGHT
    foot = block(-4, 4, -11.5, 11.5, 0, 4.5)
    stem = prism([(-3.6, -4.6, 4), (3.6, -4.6, 4), (3.2, -4.6, height),
                  (-3.2, -4.6, height)], 9.2, (0, 1, 0))
    joint = cylinder_y(6.1, 9.2, 0, -4.6, height)
    base_screws = [cylinder_z(1.75, 7, 0, side * CAMERA_FOOT_SPACING / 2, -1)
                   for side in (-1, 1)]
    hinge_screw = cylinder_y(1.75, 12, 0, -6, height)
    return foot.fuse(stem, joint).cut(*base_screws, hinge_screw)


def camera_carrier():
    """Support the camera on four M2 spacers. Keep the two connector exits clear."""
    pivot = CAMERA_PIVOT_BACKSET
    bridge = block(8, 13, -20, 20, -4, 4)
    features = []
    for side in (-1, 1):
        r0, r1 = sorted((side*10.5, side*22))
        b0, b1 = sorted((side*13, side*20.5))
        y0, y1 = sorted((side*5.05, side*9.6))
        features.extend((
            block(0, 4, r0, r1, -22, 22), block(3.7, 9, b0, b1, -4, 4),
            block(12.5, pivot, y0, y1, -4.3, 4.3), cylinder_y(7.2, y1-y0, pivot, y0, 0),
        ))
    screw_slots = [slot_x((side_y*13.7, side_z*13.7), (side_y*17.7, side_z*17.7), 2.6)
                   for side_y in (-1, 1) for side_z in (-1, 1)]
    hinge_screw = cylinder_y(1.75, 22, pivot, -11, 0)
    return bridge.fuse(*features).cut(*screw_slots, hinge_screw)


def upper_drill_pattern():
    result = [("CAM", CAMERA_ANCHOR[0], side * CAMERA_FOOT_SPACING / 2) for side in (-1, 1)]
    for name, (x, y, yaw) in zip(("U2", "U1", "U3"), ULTRASONIC_ANCHORS, strict=True):
        c, s = cos(radians(yaw)), sin(radians(yaw))
        result.extend((name, x + a*c + b*s, y - a*s + b*c) for a,b in ULTRASONIC_FOOT_HOLES)
    return result


def mounted_camera(carrier, pitch):
    return carrier.translate((-CAMERA_PIVOT_BACKSET, 0, 0)).rotate(Axis.Y, -pitch).translate(
        (*CAMERA_ANCHOR, CAMERA_PIVOT_HEIGHT)
    )


def export_drill_pattern():
    import ezdxf

    doc = ezdxf.new("R2013")
    doc.units = ezdxf.units.MM
    doc.layers.new("DRILL_3_5", dxfattribs={"color": 1})
    doc.layers.new("REFERENCE", dxfattribs={"color": 7})
    doc.layers.new("LABELS", dxfattribs={"color": 5})
    view = doc.modelspace()
    reference = {"layer": "REFERENCE"}
    view.add_lwpolyline(V18_UPPER_OUTLINE, close=True, dxfattribs=reference)
    view.add_line((-43, 0), (128, 0), dxfattribs=reference)
    view.add_line((0, -64), (0, 64), dxfattribs=reference)
    view.add_text("V18 upper deck bracket drilling template | mm | front to the left",
                  dxfattribs={"layer": "LABELS", "height": 3.2, "insert": (-34, 70)})
    view.add_text("DRILL_3_5: eight new holes only. REFERENCE: align to existing deck.",
                  dxfattribs={"layer": "LABELS", "height": 2.6, "insert": (-34, 64)})
    for name, x, y in upper_drill_pattern():
        view.add_circle((x, y), 1.75, dxfattribs={"layer": "DRILL_3_5"})
        view.add_line((x-2.4, y), (x+2.4, y), dxfattribs=reference)
        view.add_line((x, y-2.4), (x, y+2.4), dxfattribs=reference)
        view.add_text(name, dxfattribs={"layer": "LABELS", "height": 2.3, "insert": (x+3, y-1)})
    doc.set_modelspace_vport(160, center=(43, 6))
    doc.saveas(OUTPUT / "V18上层板支架孔位.dxf")


def main():
    for folder in ("STL", "STEP"):
        (OUTPUT / folder).mkdir(exist_ok=True)
    tray = driver_tray()
    sonar = ultrasonic_bracket()
    mast = camera_mast()
    carrier = camera_carrier()
    items = (
        ("D157B跨电池托板", tray),
        ("HC-SR04支架_同款打印3件", sonar.translate((0, 0, 4))),
        ("DCXGW10相机支架_底座", mast),
        ("DCXGW10相机支架_俯仰座", carrier.rotate(Axis.Y, -90)),
    )
    for name, solid in items:
        if len(solid.solids()) != 1 or not solid.is_valid or solid.volume <= 0:
            raise ValueError(f"{name}: expected one valid solid")
        solid.label = name
        step_path = OUTPUT / "STEP" / f"{name}.step"
        export_step(solid, step_path)
        step_path.write_bytes(re.sub(rb"[ \t]+(?=\r?\n)", b"", step_path.read_bytes()))
        export_stl(solid, OUTPUT / "STL" / f"{name}.stl", tolerance=0.045, angular_tolerance=0.10)
        print(name, tuple(round(value, 1) for value in solid.bounding_box().size))
    export_drill_pattern()
    for name, x, y in upper_drill_pattern():
        print(f"{name}: ({x:.2f}, {y:.2f})")


if __name__ == "__main__":
    main()
