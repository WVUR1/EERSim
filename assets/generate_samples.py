"""Regenerate EERsim sample robots and wrappers for the supplied official CAD.

These models demonstrate configuration, not WVU's measured mass distribution
or certified competition dimensions. Units are metres, kilograms, radians.
"""
from __future__ import annotations

from copy import deepcopy
import importlib.util
import json
import math
import struct
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def node(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {key: str(value) for key, value in attributes.items()})


def link(robot, name, mass, shape, dimensions, color, rpy="0 0 0", origin="0 0 0"):
    part = node(robot, "link", name=name)
    inertial = node(part, "inertial")
    node(inertial, "mass", value=mass)
    node(inertial, "origin", xyz=origin, rpy="0 0 0")
    if shape == "box":
        x, y, z = [float(v) for v in dimensions.split()]
        inertias = [mass * (y*y + z*z) / 12, mass * (x*x + z*z) / 12, mass * (x*x + y*y) / 12]
    else:
        radius, length = dimensions
        # Isotropic inertia conservatively includes the shaft/gear cartridge.
        moment = mass * (3*radius*radius + length*length) / 12
        inertias = [max(moment, .5*mass*radius*radius)] * 3
    node(inertial, "inertia", ixx=inertias[0], iyy=inertias[1], izz=inertias[2], ixy=0, ixz=0, iyz=0)
    for tag in ("visual", "collision"):
        view = node(part, tag)
        node(view, "origin", xyz=origin, rpy=rpy)
        geo = node(view, "geometry")
        if shape == "box":
            node(geo, "box", size=dimensions)
        else:
            node(geo, "cylinder", radius=dimensions[0], length=dimensions[1])
        if tag == "visual":
            material = node(view, "material", name=name + "_material")
            node(material, "color", rgba=color)
    return part


def joint(robot, name, parent, child, position, axis, kind="continuous", limits=None):
    element = node(robot, "joint", name=name, type=kind)
    node(element, "parent", link=parent)
    node(element, "child", link=child)
    node(element, "origin", xyz=position, rpy="0 0 0")
    node(element, "axis", xyz=axis)
    if kind != "fixed":
        node(element, "dynamics", damping=.0001, friction=0)
        bound = {"effort": 60, "velocity": 200}
        if limits is not None:
            bound.update(lower=limits[0], upper=limits[1])
        node(element, "limit", **bound)
    return element


def wheel_stripe(part, radius, width, rpy, axis):
    """Visual spokes make shaft motion visible without changing mass/contact."""
    for sign in (-1, 1):
        visual = node(part, "visual")
        offset = [sign * coordinate * (width / 2 + .0007) for coordinate in axis]
        node(visual, "origin", xyz=" ".join(map(str, offset)), rpy=rpy)
        node(node(visual, "geometry"), "box", size=f"{radius*1.7} .005 .001")
        node(node(visual, "material", name="motion_spoke"), "color", rgba="1 .72 .04 1")


def save_urdf(robot, path):
    ET.indent(robot)
    # Serialize bytes so Windows cannot translate canonical XML LF to CRLF.
    path.write_bytes(ET.tostring(robot, encoding="utf-8", xml_declaration=True))


def save_json(data, path):
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n")


def make_robot(omni=True):
    name = "xdrive" if omni else "tank"
    robot = ET.Element("robot", name="eerkit_" + name)
    link(robot, "base_link", 8, "box", ".32 .32 .06", ".045 .10 .20 1")
    # Gold top rail and blue electronics cover give forward direction a clear cue.
    link(robot, "electronics", .3, "box", ".19 .20 .035", ".025 .24 .42 1")
    joint(robot, "electronics_fixed", "base_link", "electronics", "0 0 .0475", "0 0 1", "fixed")
    link(robot, "nose", .08, "box", ".015 .24 .035", "1 .72 .04 1")
    joint(robot, "nose_fixed", "base_link", "nose", ".169 0 .015", "0 0 1", "fixed")
    root2 = math.sqrt(.5)
    wheel_defs = [
        ("front_left", .15, .15, [root2, -root2, 0], [15, 17, -16]),
        ("front_right", .15, -.15, [root2, root2, 0], [-8, -6, 21]),
        ("back_left", -.15, .15, [root2, root2, 0], [19, 18, -20]),
        ("back_right", -.15, -.15, [root2, -root2, 0], [-9, -1, 10]),
    ]
    bindings, wheels = [], []
    for ordinal, (wheel, x, y, direction, ports) in enumerate(wheel_defs):
        if not omni:
            direction = [1, 0, 0]
            ports = [[1], [3], [2], [4]][ordinal]
        axis = [-direction[1], direction[0], 0]
        # Cylinder local +Z maps to the axle. +axis spin rolls along direction.
        yaw = math.atan2(direction[1], direction[0])
        radius = .0254 if omni else .0508
        wheel_z = -.04 if omni else -.015
        part = link(robot, wheel + "_wheel", .35, "cylinder", (radius, .022), ".07 .085 .10 1", f"{-math.pi/2} 0 {yaw}")
        wheel_stripe(part, radius, .022, f"{-math.pi/2} 0 {yaw}", axis)
        joint(robot, wheel, "base_link", wheel + "_wheel", f"{x} {y} {wheel_z}", " ".join(map(str, axis)))
        for port in ports:
            bindings.append({"port": port, "joint": wheel, "cartridge": "blue" if omni else "green", "rpm": 600 if omni else 200, "gear_ratio": 1})
        wheels.append({"joint": wheel, "radius": radius, "direction": direction, "roller_friction": .025, "slip_speed": .12})
    link(robot, "intake_roller", .25, "cylinder", (.026, .25), "1 .67 .025 1", f"{-math.pi/2} 0 0")
    joint(robot, "intake", "base_link", "intake_roller", ".195 0 -.014", "0 1 0")
    for port in ([11, -7, 3, -14] if omni else [5]):
        bindings.append({"port": port, "joint": "intake", "cartridge": "blue", "rpm": 600, "gear_ratio": 1})
    link(robot, "flap_link", .15, "box", ".12 .20 .012", ".15 .45 .6 1", origin=".06 0 0")
    joint(robot, "flap", "base_link", "flap_link", ".15 0 .055", "0 -1 0", "revolute", [0, 1.3])
    link(robot, "lift_link", .3, "box", ".13 .16 .012", "1 .72 .04 1")
    joint(robot, "lift", "base_link", "lift_link", "-.07 0 .075", "0 0 1", "prismatic", [0, .12])
    link(robot, "loader_link", .15, "box", ".14 .025 .025", ".15 .45 .6 1", origin="-.07 0 0")
    joint(robot, "loader", "base_link", "loader_link", "-.16 0 .04", "0 -1 0", "revolute", [0, 1.3])
    config = {
        "schema_version": 1, "name": "WVU EERKit X-drive example" if omni else "Four-wheel tank example",
        "description": "Illustrative 10.73 kg robot; measure and replace mass, inertia, friction and geometry for your robot.",
        "urdf": name + ".urdf", "motors": bindings,
        "drive": {"type": "omni" if omni else "tank", "wheels": wheels},
        "pneumatics": [
            {"port": "A", "joint": "flap", "extended": 1.1, "retracted": 0, "force": 3, "max_velocity": 2},
            {"port": "B", "joint": "lift", "extended": .11, "retracted": 0, "force": 35, "max_velocity": .15},
            {"port": "C", "joint": "loader", "extended": 1.2, "retracted": 0, "force": 3, "max_velocity": 2},
        ],
        "distance_sensors": [{"port": 2 if omni else 6, "position": [.23, 0, .06], "direction": [1, 0, 0], "max_range": 2}],
        "sensors": {"imu_heading_sign": -1, "otos": {"port": 5, "x_sign": 1, "y_sign": -1, "heading_sign": 1}},
        "sources": ["https://kb.vex.com/hc/en-us/articles/360044325872-Understanding-V5-Smart-Motor-11W-Performance"],
    }
    save_urdf(robot, ROOT / "robots" / (name + ".urdf"))
    save_json(config, ROOT / "robots" / (name + ".json"))


def make_cup():
    robot = ET.Element("robot", name="approximate_hollow_cup")
    base = link(robot, "cup", .10, "cylinder", (.055, .006), ".05 .35 .8 1")
    # Convex boxes around a hollow wall preserve the opening for physical tests.
    for i in range(12):
        angle = i * math.tau / 12
        for tag in ("visual", "collision"):
            piece = node(base, tag)
            node(piece, "origin", xyz=f"{.057*math.cos(angle)} {.057*math.sin(angle)} .048", rpy=f"0 0 {angle}")
            node(node(piece, "geometry"), "box", size=".007 .030 .09")
            if tag == "visual":
                node(node(piece, "material", name="blue"), "color", rgba=".05 .35 .8 1")
    save_urdf(robot, ROOT / "objects" / "cup.urdf")


def make_claw_robot():
    robot = ET.Element("robot", name="eerkit_simple_xdrive_claw")
    link(robot, "base_link", 4, "box", ".30 .30 .055", ".045 .10 .20 1")
    link(robot, "electronics", .2, "box", ".16 .19 .035", ".025 .24 .42 1")
    joint(robot, "electronics_fixed", "base_link", "electronics", "-.035 0 .045", "0 0 1", "fixed")
    directions = [("front_left", .14, .14, -1), ("front_right", .14, -.14, 1),
                  ("back_left", -.14, .14, 1), ("back_right", -.14, -.14, -1)]
    motors, wheels = [], []
    for port, (name, x, y, diagonal) in enumerate(directions, 1):
        direction = [math.sqrt(.5), diagonal * math.sqrt(.5), 0]
        axis = [-direction[1], direction[0], 0]
        yaw = math.atan2(direction[1], direction[0])
        part = link(robot, name + "_wheel", .22, "cylinder", (.034925, .026), ".055 .065 .075 1", f"{-math.pi/2} 0 {yaw}")
        wheel_stripe(part, .034925, .026, f"{-math.pi/2} 0 {yaw}", axis)
        joint(robot, name, "base_link", name + "_wheel", f"{x} {y} -.04", " ".join(map(str, axis)))
        motors.append({"port": port, "joint": name, "cartridge": "blue", "rpm": 600, "gear_ratio": 2})
        wheels.append({"joint": name, "radius": .034925, "direction": direction, "roller_friction": .025, "slip_speed": .12})
    link(robot, "mast", .4, "box", ".025 .08 .52", ".55 .58 .62 1")
    joint(robot, "mast_fixed", "base_link", "mast", ".115 0 .24", "0 0 1", "fixed")
    link(robot, "carriage", .3, "box", ".16 .12 .04", "1 .72 .04 1", origin=".20 0 0")
    joint(robot, "lift", "base_link", "carriage", "0 0 .06", "0 0 1", "prismatic", [0, .40])
    link(robot, "fixed_finger", .075, "box", ".20 .012 .025", ".62 .68 .74 1", rpy="-.208 0 0")
    joint(robot, "fixed_finger_mount", "carriage", "fixed_finger", ".32 -.026 0", "0 1 0", "fixed")
    link(robot, "moving_finger", .075, "box", ".20 .012 .025", ".62 .68 .74 1", rpy=".208 0 0")
    joint(robot, "claw", "carriage", "moving_finger", ".32 .026 0", "0 1 0", "prismatic", [-.015, .045])
    motors.extend([
        {"port": 5, "joint": "lift", "cartridge": "red", "rpm": 100, "lead_m": .04, "current_limit": .25},
        {"port": 6, "joint": "claw", "cartridge": "red", "rpm": 100, "lead_m": .02, "current_limit": .15},
    ])
    config = {"schema_version": 1, "name": "Simple X-drive + pin claw", "urdf": "xdrive_claw.urdf",
              "description": "Four-wheel X-drive and motorized lift with slim friction jaws that grip the upper taper of an Override pin. No grasp attachments or teleports.",
              "motors": motors, "drive": {"type": "omni", "wheels": wheels},
              "initial_joints": {"claw": .04, "lift": 0},
              "link_friction": {"fixed_finger": 1.8, "moving_finger": 1.8},
              "sensors": {"imu_heading_sign": -1, "otos": {"port": 7, "x_sign": 1, "y_sign": -1, "heading_sign": 1}},
              "claw": {"pin_target_offset": [.34, 0, .06], "open_motor_degrees": 720, "close_motor_degrees": -144,
                       "lift_metres_per_revolution": .04, "gap_metres": ".040 + joint position"}}
    save_urdf(robot, ROOT / "robots" / "xdrive_claw.urdf")
    save_json(config, ROOT / "robots" / "xdrive_claw.json")


def convex_mesh(filename, vertices):
    """Write vertices and a hull triangulation; Bullet rebuilds convex hulls.

    Small shape files contain just rings; the input vertex cloud is the actual
    collision contract. The fan faces keep ordinary OBJ readers compatible.
    """
    path = ROOT / "objects" / filename
    lines = ["# EERsim CAD-profile collision hull; units metres"]
    lines.extend("v " + " ".join(f"{v:.9f}" for v in point) for point in vertices)
    lines.extend(f"f 1 {i} {i+1}" for i in range(2, len(vertices)))
    # OBJ files are binary-tagged because provenance stores their byte hashes.
    # Retain an existing file's format; new profiles use the shipped CRLF form
    # even when regenerated on Linux. Geometry and binary STL are unchanged.
    newline = "\r\n" if not path.exists() or b"\r\n" in path.read_bytes() else "\n"
    path.write_bytes((newline.join(lines) + newline).encode("utf-8"))
    # Bullet's OBJ loader inserts an extra compound around each convex object,
    # preventing changeDynamics from reaching leaf margins. STL imports the
    # same vertex hull directly. Keep the small OBJ alongside it for inspection.
    faces = [(0, i, i+1) for i in range(1, len(vertices)-1)]
    data = b"EERsim CAD-profile convex collision vertices".ljust(80, b"\0") + struct.pack("<I", len(faces))
    for face in faces:
        data += struct.pack("<12fH", *([0., 0., 0.] + [value for i in face for value in vertices[i]]), 0)
    path.with_suffix(".stl").write_bytes(data)


def mesh_geometry(part, filename, tag="visual", color=None, concave=False):
    attributes = {"concave": "yes"} if concave else {}
    element = node(part, tag, **attributes)
    node(node(element, "geometry"), "mesh", filename=filename)
    if color is not None:
        node(node(element, "material", name=filename.replace("/", "_")), "color", rgba=color)
    return element


def cad_visual(filename):
    lod = ROOT / "meshes" / "override" / "lod" / filename
    return "../meshes/override/" + ("lod/" if lod.exists() else "") + filename


def collision_generator():
    """Load the shared offline generator from this checkout, without sys.path edits."""
    path = ROOT.parent / "tools" / "build_goal_pin_colliders.py"
    spec = importlib.util.spec_from_file_location("eersim_goal_pin_generator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Cannot load the goal/pin collision generator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_goal_socket_collision():
    """Regenerate the CAD lip, retaining its shoulder and correct face winding."""
    collision_generator().build_goal(ROOT.parent, ROOT)


def make_pin_and_goals():
    colors = {"red": ".82 .08 .13 1", "blue": ".02 .48 .83 1", "yellow": ".98 .83 .05 1"}
    # Official 276-9250-80x CAD: two tapered ends with near-cylindrical tips,
    # a 16 mm central D-flange (80.293 by 76.793 mm), height 165 mm.
    # 48-sided convex hulls have <0.09 mm circular chord error. Small molded
    # details are omitted from dynamic collision only. Tip rings and the inset
    # D-flat below follow sections of the source CAD, including the rounded cap.
    make_goal_socket_collision()
    for half in (-1, 1):
        root_profile = [(.008, .0301318, -.0266318),
                        (.0081, .031132475235, -.027500872514),
                        (.0085, .030430664234, -.026799267368),
                        (.009, .030022385255, -.026389292022),
                        (.010, .029708910092, -.026080415567)]
        root_vertices = [(radius*math.cos(i*math.tau/48),
                          max(flat_y, radius*math.sin(i*math.tau/48)), half*z)
                         for z, radius, flat_y in root_profile for i in range(48)]
        convex_mesh(f"pin_root_{half}.obj", root_vertices)
        # The root fillet must be a separate hull. Bridging its larger radius
        # directly to the tip falsely thickens the whole cone by up to 0.8 mm.
        taper_profile = [(.010, .029708910092, -.026080415567),
                         (.020, .02759770, -.02406175),
                         (.040, .02337479, -.02002900),
                         (.060, .01915172, -.01621179),
                         (.065, .01809600, -.01559048),
                         (.0665, .017797516993, -.015529281039)]
        vertices = [(radius*math.cos(i*math.tau/48),
                     max(flat_y, radius*math.sin(i*math.tau/48)), half*z)
                    for z, radius, flat_y in taper_profile for i in range(48)]
        convex_mesh(f"pin_taper_{half}.obj", vertices)
        tip_profile = [(.0665, .017797517), (.070, .017718188), (.075, .017630913),
                       (.080, .017543638), (.0805349, .017534301), (.081, .017470258),
                       (.0815, .017265831), (.082, .016855986), (.08225, .016501199),
                       (.0825, .015534632)]
        vertices = [(radius*math.cos(i*math.tau/48),
                     max(-(radius-.00225), radius*math.sin(i*math.tau/48)), half*z)
                    for z, radius in tip_profile for i in range(48)]
        convex_mesh(f"pin_tip_{half}.obj", vertices)
    # The flange has beveled faces: a solid 16 mm cylinder would incorrectly
    # intersect the cup rim in the official authored starting stacks.
    flange_profile = [(-.008, .0345), (-.0075, .0362961), (-.001, .0401465),
                      (.001, .0401465), (.0075, .0362961), (.008, .0345)]
    flange = [(radius*math.cos(i*math.tau/48), max(-(radius-.0035), radius*math.sin(i*math.tau/48)), z)
              for z, radius in flange_profile for i in range(48)]
    convex_mesh("pin_flange.obj", flange)
    for top, bottom in [("red", "yellow"), ("blue", "yellow"), ("yellow", "yellow"), ("red", "blue"), ("blue", "red")]:
        robot = ET.Element("robot", name=f"override_pin_{top}_{bottom}")
        part = link(robot, "pin", .075, "cylinder", (.027, .165), colors[top])
        for element in list(part.findall("visual")) + list(part.findall("collision")):
            part.remove(element)
        mesh_geometry(part, cad_visual("pin_upper.obj"), color=colors[top])
        mesh_geometry(part, cad_visual("pin_lower.obj"), color=colors[bottom])
        for mesh in ("pin_root_-1.obj", "pin_root_1.obj", "pin_taper_-1.obj", "pin_taper_1.obj",
                     "pin_tip_-1.obj", "pin_tip_1.obj", "pin_flange.obj"):
            mesh_geometry(part, str(Path(mesh).with_suffix(".stl")), "collision")
        save_urdf(robot, ROOT / "objects" / f"pin_{top}_{bottom}.urdf")
    # Keep the original root profiles as reproducible inputs, then replace each
    # whole-root hull with the CAD-faithful axial bands. Otherwise regeneration
    # would silently restore the convex bridge that intersects the goal lip.
    collision_generator().build_root_bands(ROOT.parent, ROOT)
    for name, kind, color in [("red", "alliance", colors["red"]), ("blue", "alliance", colors["blue"]),
                              ("neutral", "neutral", ".065 .075 .085 1"), ("center", "center", ".065 .075 .085 1")]:
        robot = ET.Element("robot", name=f"override_goal_{name}")
        # Static concave CAD-profile collision preserves the tapered opening.
        # Mounting plates and raised supports use structural box approximations;
        # the full displayed assembly still comes from the original CAD.
        part = node(robot, "link", name="goal", concave="yes")
        inertial = node(part, "inertial")
        node(inertial, "mass", value=0)
        node(inertial, "inertia", ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)
        filename = f"goal_{kind}.obj"
        mesh_geometry(part, cad_visual(filename), color=color)
        socket_path = ROOT / "objects" / "goal_socket_collision.obj"
        if socket_path.exists():
            raise_by = .064 if kind == "neutral" else .1402 if kind == "center" else 0
            socket = mesh_geometry(part, "goal_socket_collision.obj", "collision", concave=True)
            node(socket, "origin", xyz=f"0 0 {raise_by}")
            plate = node(part, "collision")
            node(plate, "origin", xyz="0 0 -.002")
            node(node(plate, "geometry"), "box", size=".300 .300 .004")
            if raise_by:
                support = node(part, "collision")
                node(support, "origin", xyz=f"0 0 {raise_by/2}")
                node(node(support, "geometry"), "box", size=f".14262532 .14262532 {raise_by}")
        else:
            mesh_geometry(part, "../meshes/override/"+filename, "collision", concave=True)
        save_urdf(robot, ROOT / "objects" / f"goal_{name}.urdf")


def make_override_elements():
    # A hollow dynamic body needs convex components. Each cup half uses sixteen
    # angular sectors and two axial bands from the measured CAD radial profile;
    # openings stay open. Profile/chord approximation is about 1 mm, rather than
    # filling the cup with the single convex hull Bullet would otherwise use.
    cup_hulls = []
    transition = (.073789140953, .036298133453, .034277407511)
    profile_bands = [[(0, .029155850274, .016106666994), (.020, .031206, .020210),
                      (.040, .032955, .025373), (.060, .034705, .030491), transition],
                     [transition, (.08225, .040098507558, .03916228657)]]
    for half in (-1, 1):
        for band in range(2):
            for sector in range(16):
                angles = [(sector + offset) * math.tau / 16 for offset in (0, .5, 1)]
                # Circumscribe the inner circle: an inscribed chord would
                # narrow the opening by up to 0.75 mm and start authored stacks
                # in penetration. The opening's angular approximation is <.77mm.
                vertices = [(r*math.cos(angle), r*math.sin(angle), .08225+half*z)
                            for z, outer, inner in profile_bands[band]
                            for r in (inner/math.cos(math.pi/16), outer) for angle in angles]
                filename = f"cup_hull_{half}_{band}_{sector}.obj"
                convex_mesh(filename, vertices)
                cup_hulls.append(filename)
    for clear_up in (False, True):
        robot = ET.Element("robot", name="override_dual_cup")
        part = node(robot, "link", name="cup")
        inertial = node(part, "inertial")
        node(inertial, "origin", xyz="0 0 .08225")
        node(inertial, "mass", value=.075)
        node(inertial, "inertia", ixx=.00019, iyy=.00019, izz=.00009, ixy=0, ixz=0, iyz=0)
        for upper, filename in [(True, "cup_upper.obj"), (False, "cup_lower.obj")]:
            color = ".63 .68 .72 .42" if upper == clear_up else ".27 .29 .31 1"
            mesh_geometry(part, cad_visual(filename), color=color)
        for filename in cup_hulls:
            mesh_geometry(part, str(Path(filename).with_suffix(".stl")), "collision")
        save_urdf(robot, ROOT / "objects" / ("cup_clear_up.urdf" if clear_up else "cup_gray_up.urdf"))
    robot = ET.Element("robot", name="override_toggle")
    mount = node(robot, "link", name="mount", concave="yes")
    inertial = node(mount, "inertial")
    node(inertial, "mass", value=0)
    node(inertial, "inertia", ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)
    mesh_geometry(mount, cad_visual("toggle_fixed.obj"), color=".75 .77 .80 1")
    mesh_geometry(mount, "../meshes/override/toggle_fixed.obj", "collision", concave=True)
    # The CAD mechanism gives a true axle/pivot. Its broadphase rotor contact
    # box omits individual screws and small molded ribs, preserving free motion.
    pivot = [-.00351410538862319, 0, .0292747046375939]
    rotor_centre = [.000851519561376779, 0, .030715432530110566]
    relative = [a-b for a,b in zip(rotor_centre, pivot)]
    paddle = link(robot, "paddle", .12, "box", ".0515937501 .6980200002 .0585273930", ".4 .42 .44 1", origin=" ".join(map(str, relative)))
    for visual in list(paddle.findall("visual")):
        paddle.remove(visual)
    visual = mesh_geometry(paddle, cad_visual("toggle_rotor.obj"), color=".4 .42 .44 1")
    node(visual, "origin", xyz=" ".join(str(-v) for v in pivot))
    joint(robot, "toggle", "mount", "paddle", " ".join(map(str, pivot)), "0 -1 0", "continuous")
    save_urdf(robot, ROOT / "objects" / "toggle.urdf")
    for color_name, color in [("red", ".8 .08 .12 1"), ("blue", ".03 .43 .8 1")]:
        robot = ET.Element("robot", name="override_loader_" + color_name)
        base = node(robot, "link", name="loader", concave="yes")
        inertial = node(base, "inertial")
        node(inertial, "mass", value=0)
        node(inertial, "inertia", ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)
        loader_mesh = "loader_blue.obj" if color_name == "blue" else "loader.obj"
        mesh_geometry(base, cad_visual(loader_mesh), color=color)
        mesh_geometry(base, "../meshes/override/"+loader_mesh, "collision", concave=True)
        save_urdf(robot, ROOT / "objects" / f"loader_{color_name}.urdf")
    for filename, width in [("white_panel", 1.16205), ("white_panel_short", 1.13665)]:
        robot = ET.Element("robot", name=filename)
        part = link(robot, "panel", 0, "box", f"{width} .0025 .18796", ".96 .965 .975 1", origin="0 -.00125 0")
        for visual in list(part.findall("visual")):
            part.remove(visual)
        mesh_geometry(part, cad_visual(filename+".obj"), color=".96 .965 .975 1")
        save_urdf(robot, ROOT / "objects" / (filename+".urdf"))


def make_override_game(practice):
    field = {**practice["field"], "wall_color": [.94, .95, .96, 1],
             "tile_size": .598058534, "size": [6*.598058534, 6*.598058534]}
    game = {
        "schema_version": 1, "name": "Override 2026–27", "season": "2026-2027",
        "manual_version": "2.0", "verified_date": "2026-09-10",
        "description": "Override starting layout: white perimeter, 9 goals, 4 loaders, 4 hinged toggles, 36 cups and 37 on-field pins. Four preloads and remaining match loads are inventoried separately.",
        "geometry_accuracy": "Official STEP-derived visuals and CAD placements. Static goal sockets and moving pieces use measured collision profile approximations; rails, supports and toggle rotor contacts are simplified. Released-pin contact detection is a prototype, not official adjudication.",
        "sources": ["https://www.vexrobotics.com/override-manual", "https://www.vexrobotics.com/26-27-manuals", "https://link.vex.com/docs/26-27/v5rc/field-cad"],
        "field": field, "mode_durations": practice["mode_durations"],
        "inventory": {"cups": {"total": 56, "on_field": 36, "gray_up": 24, "clear_up": 12, "match_loads": 20},
                      "pins": {"total": 63, "on_field": 37, "preloads": 4, "match_loads": 22,
                               "on_field_colors": {"red_blue": 4, "red_yellow": 8, "blue_yellow": 8, "yellow_yellow": 17}}},
        "objects": [],
    }
    objects = game["objects"]
    def urdf(name, filename, position, fixed=False, rotation=None, **extra):
        entry = {"name": name, "urdf": "../objects/" + filename + ".urdf", "fixed": fixed, "position": position, **extra}
        if rotation is not None:
            entry["rotation"] = rotation
        objects.append(entry)
    def pin(name, x, y, top="yellow", bottom="yellow", z=.083, rotation=None):
        urdf(name, f"pin_{top}_{bottom}", [x, y, z], rotation=rotation,
             kind="pin", colors=[top, bottom], scoring={"type": "pin", "length": .165, "radius": .0178})
    def cup(name, x, y, clear=False):
        urdf(name, "cup_clear_up" if clear else "cup_gray_up", [x, y, .0015], kind="cup", clear_up=clear)
    # Actual official STEP grid pitch; it differs from the nominal 24-inch tile.
    tile = .598058534
    goals = [("center", 0, 0), ("neutral", -tile, 2*tile), ("neutral", -2*tile, tile),
             ("neutral", 2*tile, -tile), ("neutral", tile, -2*tile),
             ("red", -2*tile, -tile), ("red", -tile, -2*tile),
             ("blue", tile, 2*tile), ("blue", 2*tile, tile)]
    for i, (color, x, y) in enumerate(goals):
        height = .22273265 if color == "center" else .14653265 if color == "neutral" else .08253265
        urdf(f"goal_{color}_{i}", f"goal_{color}", [x, y, 0], fixed=True, kind="goal",
             scoring={"type": "goal", "height": height, "socket_depth": .08253265, "socket_radius": .0384,
                      "alliance": color if color in ("red", "blue") else "neutral"})
        if color in ("neutral", "center"):
            pin(f"pin_goal_{i}", x, y, z=.2247 if color == "center" else .1485)
    # Midfield square and autonomous diagonals are visual tape, never obstacles.
    for sign in (-1, 1):
        objects.append({"name": f"autonomous_tape_{sign}", "shape": "box", "size": [4.8, .018, .001],
                        "position": [0, 0, .002], "rotation": [0, 0, sign*math.pi/4], "color": [.92, .93, .94, 1], "collision": False})
    for x, y, angle in [(tile/2, tile/2, -math.pi/4), (tile/2, -tile/2, math.pi/4), (-tile/2, -tile/2, -math.pi/4), (-tile/2, tile/2, math.pi/4)]:
        objects.append({"name": "midfield_tape", "shape": "box", "size": [tile*math.sqrt(2), .018, .001],
                        "position": [x, y, .003], "rotation": [0, 0, angle], "color": [1, 1, 1, 1], "collision": False})
    # Four diagonally arranged clusters: clear cup plus four horizontal pins.
    for i, (x, y) in enumerate([(-2*tile, 2*tile), (-tile, tile), (tile, -tile), (2*tile, -2*tile)]):
        cup(f"cup_cluster_{i}", x, y, True)
        for side, dx, dy, rotation, top in [
            ("north", 0, .127, [-math.pi/2, 0, 0], "blue"),
            ("east", .127, 0, [0, math.pi/2, 0], "blue"),
            ("south", 0, -.127, [math.pi/2, 0, 0], "red"),
            ("west", -.127, 0, [0, -math.pi/2, 0], "red"),
        ]:
            pin(f"pin_cluster_{i}_{side}", x+dx, y+dy, top, z=.04015, rotation=rotation)
    # Four other-diagonal cup+yellow stacks and four midfield red/blue stacks.
    for i, coordinate in enumerate((-2*tile, -tile, tile, 2*tile)):
        cup(f"cup_diagonal_{i}", coordinate, coordinate, True)
        pin(f"pin_diagonal_{i}", coordinate, coordinate, z=.16728265)
    for i, (x, y, top, bottom) in enumerate([(0, tile, "blue", "red"), (tile, 0, "blue", "red"),
                                           (0, -tile, "red", "blue"), (-tile, 0, "red", "blue")]):
        cup(f"cup_midfield_{i}", x, y, True)
        pin(f"pin_midfield_{i}", x, y, top, bottom, z=.16728265)
    # Each perimeter side: two groups of three gray-up cups, with a YY pin in
    # the middle cup. Supplies off the field remain inventory, not loose objects.
    edge = 3*tile - .055
    for side in range(4):
        for group, centre in enumerate((-tile, tile)):
            for offset in (-1, 0, 1):
                along = centre + offset*.105
                x, y = ((along, edge) if side == 0 else (edge, along) if side == 1 else
                        (along, -edge) if side == 2 else (-edge, along))
                cup(f"cup_wall_{side}_{group}_{offset}", x, y)
                if offset == 0:
                    pin(f"pin_wall_{side}_{group}", x, y, z=.16728265)
        x, y = [(0, 3*tile), (3*tile, 0), (0, -3*tile), (-3*tile, 0)][side]
        urdf(f"toggle_{side}", "toggle", [x, y, .27], fixed=True, rotation=[0, 0, math.pi/2 if side % 2 else 0], kind="toggle")
    for i, (x, y, color) in enumerate([(-3*tile, 2.55*tile, "red"), (-3*tile, -2.55*tile, "red"),
                                      (3*tile, 2.55*tile, "blue"), (3*tile, -2.55*tile, "blue")]):
        urdf(f"loader_{i}", "loader_"+color, [x, y, .18], fixed=True, rotation=[0, 0, math.pi/2], kind="loader")
    manifest_path = ROOT / "cad_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        placements = manifest.get("field_placements", {}).get("instances", [])
        if placements:
            # The authored assembly supplies rotations as well as position:
            # upside-down cups have their base-frame origin above the floor.
            # Canonical colors are applied before those source rotations.
            candidates = [entry for entry in objects if entry.get("kind")]
            authored = []
            for row in placements:
                if not row.get("in_field"):
                    continue
                kind = "goal" if row["kind"].startswith("goal") else row["kind"]
                if kind.startswith("white_panel"):
                    authored.append({"name": f"white_panel_{row['source_instance_index']}",
                                     "urdf": "../objects/"+Path(row["mesh"]).stem+".urdf",
                                     "fixed": True, "kind": "white_panel", "position": row["position"],
                                     "rotation": row["rpy"], "source_instance_index": row["source_instance_index"]})
                    continue
                same_kind = [entry for entry in candidates if entry.get("kind") == kind]
                old = min(same_kind, key=lambda entry: sum((a-b)**2 for a,b in zip(entry["position"], row["position"])))
                candidates.remove(old)
                entry = {**old, "position": [round(v, 12) for v in row["position"]],
                         "rotation": row["rpy"], "source_instance_index": row["source_instance_index"]}
                if kind == "pin":
                    top, bottom = row["colors"]["upper"], row["colors"]["lower"]
                    entry.update(urdf=f"../objects/pin_{top}_{bottom}.urdf", colors=[top, bottom])
                elif kind == "cup":
                    entry.update(urdf="../objects/cup_gray_up.urdf", clear_up=row["rotation_matrix"][2][2] < 0)
                elif kind == "loader":
                    entry["urdf"] = "../objects/loader_"+row["colors"]["body"]+".urdf"
                authored.append(entry)
            objects[:] = [entry for entry in objects if not entry.get("kind")] + authored
            game["cad_source_sha256"] = manifest["source"]["step_sha256"]
            if any(entry.get("kind") == "white_panel" for entry in authored):
                field.update(walls=False, wall_height=.3090484, inner_panel_span=3.570357986)
                # Rails and posts use rectangular structural approximations;
                # the twelve visible white panels themselves are official CAD.
                boundary = 1.797878993
                for side in range(4):
                    for z in (.0381, .25146):
                        along_x = side % 2 == 0
                        objects.append({"name": f"perimeter_rail_{side}_{z}", "shape": "box",
                                        "size": [3.62, .0254, .0254] if along_x else [.0254, 3.62, .0254],
                                        "position": ([0, boundary if side == 0 else -boundary, z] if along_x else
                                                     [boundary if side == 1 else -boundary, 0, z]),
                                        "color": [.65, .68, .72, 1]})
                    for along in (-boundary, -tile, tile, boundary):
                        objects.append({"name": f"perimeter_post_{side}_{along}", "shape": "box", "size": [.022, .022, .26],
                                        "position": ([along, boundary if side == 0 else -boundary, .13] if side % 2 == 0 else
                                                     [boundary if side == 1 else -boundary, along, .13]),
                                        "color": [.70, .73, .77, 1]})
    save_json(game, ROOT / "games" / "override-2026-27.json")
    make_route_planning_pack(game)


def make_route_planning_pack(game):
    """An explicit fixed-obstacle experiment for fast route iteration."""
    route = deepcopy(game)
    route["name"] = "Override 2026–27 · route planning"
    route["description"] = (
        "Route-planning experiment: the 36 cups and 37 field pins are fixed, immovable obstacles. "
        "They cannot be pushed or grasped. Robot dynamics and the hinged toggles still simulate. "
        "Use the full Override preset to test interactions with the field's game pieces."
    )
    route["geometry_accuracy"] = (
        "Official CAD layout and the same collision approximations as the full field. "
        "Fixed game pieces intentionally change contact behavior for route planning; "
        "this preset is not a full-game dynamics or scoring test."
    )
    route["scenario"] = {"fixed_game_pieces": True, "source_pack": "override-2026-27.json"}
    for item in route["objects"]:
        if item.get("kind", item.get("scoring", {}).get("type")) in ("pin", "cup"):
            item["fixed"] = True
            item["mass"] = 0
    save_json(route, ROOT / "games" / "override-route-planning.json")


def make_games():
    practice = {
        "schema_version": 1, "name": "Autonomy practice field", "season": "Practice",
        "description": "12 ft square practice arena with movable objects and calibration obstacles.",
        "geometry_accuracy": "Practice geometry; no competition scoring or rules adjudication.",
        "field": {"size": [3.6576, 3.6576], "wall_height": .30, "tile_size": .6096, "material": "vex_foam_tile"},
        "mode_durations": {"skills_driver": 60, "skills_auto": 60, "match_auto": 30, "driver_auto": 90},
        "objects": [
            {"name": "red_start_zone", "size": [.55, .55, .002], "position": [-1.2, -1.2, .002], "color": [.55, .12, .17, 1], "collision": False},
            {"name": "blue_start_zone", "size": [.55, .55, .002], "position": [1.2, 1.2, .002], "color": [.08, .30, .60, 1], "collision": False},
            {"name": "calibration_barrier", "size": [.08, .70, .16], "position": [.65, .35, .08], "color": [.9, .68, .08, 1]},
            {"name": "pickup_cube", "size": [.12, .12, .12], "position": [-.55, -.6, .062], "mass": .18, "color": [.95, .24, .14, 1]},
            {"name": "movable_ball", "shape": "sphere", "radius": .065, "position": [.1, -.25, .066], "mass": .12, "color": [.08, .46, .92, 1], "rolling_friction": .002},
            {"name": "hollow_cup", "urdf": "../objects/cup.urdf", "fixed": False, "position": [-.4, .4, .007]},
        ],
    }
    save_json(practice, ROOT / "games" / "practice.json")
    make_override_game(practice)


if __name__ == "__main__":
    for folder in ("robots", "games", "objects"):
        (ROOT / folder).mkdir(parents=True, exist_ok=True)
    make_robot(True)
    make_robot(False)
    make_cup()
    make_claw_robot()
    make_pin_and_goals()
    make_override_elements()
    make_games()
