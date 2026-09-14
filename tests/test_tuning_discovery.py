"""Source discovery rejects guessed hardware and preserves actual drive groups."""
import pytest

from eersim.tuning_discovery import discover_targets, make_target


def workspace(tmp_path, source, headers=None):
    (tmp_path / "src").mkdir(exist_ok=True)
    (tmp_path / "src/main.cpp").write_text(source, encoding="utf-8")
    for name, text in (headers or {}).items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return tmp_path


def targets(result):
    return {target["name"]: target for target in result["targets"]}


def test_eerkit_xdrive_expands_real_motor_groups_and_signs(tmp_path):
    root = workspace(tmp_path, """
pros::MotorGroup front_left({15,17,-16}, pros::MotorGearset::blue);
pros::MotorGroup front_right({-8,-6,21}, pros::MotorGearset::blue);
pros::MotorGroup back_left({19,18,-20}, pros::MotorGearset::blue);
pros::MotorGroup back_right({-9,-1,10}, pros::MotorGearset::blue);
eerkit::XDrivetrain drivetrain(front_left, front_right, back_left, back_right, 2., 600., 12., 0.);
""")
    result = discover_targets(root)
    drive = targets(result)["drivetrain"]
    assert drive["ports"] == [15,17,-16,-8,-6,21,19,18,-20,-9,-1,10]
    assert drive["axes"]["turn"] == [1,1,1,-1,-1,-1,1,1,1,-1,-1,-1]
    assert drive["axes"]["strafe"] == [1,1,1,-1,-1,-1,-1,-1,-1,1,1,1]
    assert drive["verified"] and drive["source"] == "src/main.cpp:6"
    assert not result["warnings"]


def test_constants_vectors_namespaces_comments_and_reverse_arguments(tmp_path):
    root = workspace(tmp_path, r'''
#define LIFT_PORT 0x05
namespace robot {
constexpr int RIGHT = 2;
constexpr bool BACKWARDS = true;
std::vector<std::int8_t> ports = {1, -RIGHT};
pros::MotorGroup drive(ports, pros::MotorGears::blue);
pros::Motor lift(LIFT_PORT, pros::E_MOTOR_GEARSET_36, BACKWARDS, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor twice(-6, true);
pros::Motor direct{7};
}
// pros::Motor fake(8);
const char *message = "pros::Motor imaginary(9);";
const char *raw = R"tag(pros::Motor ghost(10);)tag";
''')
    result = discover_targets(root)
    found = targets(result)
    assert set(found) == {"robot::drive", "robot::lift", "robot::twice", "robot::direct"}
    assert found["robot::drive"]["ports"] == [1, -2]
    assert found["robot::lift"]["ports"] == [-5]
    assert found["robot::twice"]["ports"] == [6]
    assert found["robot::direct"]["ports"] == [7]
    assert not result["warnings"]


def test_scope_resolution_never_merges_same_names(tmp_path):
    root = workspace(tmp_path, """
namespace one { pros::MotorGroup left({1}); pros::MotorGroup right({-2});
eerkit::TankDrive drive(left, right, 4, 200, 12); }
namespace two { pros::MotorGroup left({3}); pros::MotorGroup right({-4});
eerkit::TankDrive drive(left, right, 4, 200, 12); }
eerkit::TankDrive explicit_drive(one::left, two::right, 4, 200, 12);
eerkit::TankDrive ambiguous(left, right, 4, 200, 12);
void local_a() { pros::Motor motor(5); }
void local_b() { pros::Motor motor(6); }
""")
    result = discover_targets(root)
    found = targets(result)
    assert found["one::drive"]["ports"] == [1, -2]
    assert found["two::drive"]["ports"] == [3, -4]
    assert found["explicit_drive"]["ports"] == [1, -4]
    assert "ambiguous" not in found
    local = [t for t in result["targets"] if t["name"].startswith("motor (")]
    assert len(local) == 2 and len({t["id"] for t in local}) == 2
    assert any("unresolved reference left" in warning for warning in result["warnings"])


def test_header_guard_and_cross_file_constants_but_not_conditional_configuration(tmp_path):
    root = workspace(tmp_path, """
#include "ports.hpp"
pros::MotorGroup motors({ports::LEFT, -RIGHT_PORT});
#ifdef USE_ALTERNATE
pros::Motor alternate(9);
#else
pros::Motor alternate(10);
#endif
""", {"include/ports.hpp": """
#pragma once
#ifndef ROBOT_PORTS_HPP
#define ROBOT_PORTS_HPP
#define RIGHT_PORT 12
namespace ports { constexpr int LEFT = 11; }
#endif
"""})
    result = discover_targets(root)
    assert targets(result)["motors"]["ports"] == [11, -12]
    assert "alternate" not in targets(result)
    assert any("conditional" in message for message in result["warnings"])


def test_duplicate_ports_unresolved_factories_and_cycles_do_not_guess(tmp_path):
    root = workspace(tmp_path, """
constexpr int A = B; constexpr int B = A;
pros::Motor unresolved(A);
pros::Motor dynamic(pick_port());
pros::MotorGroup duplicate({1, -1});
pros::MotorGroup good({2, -3});
eerkit::TankDrive overlap(good, good, 4, 200, 12);
""")
    result = discover_targets(root)
    assert set(targets(result)) == {"good"}
    assert len(result["warnings"]) == 4
    assert any("cyclic" in message for message in result["warnings"])
    assert any("same physical" in message for message in result["warnings"])


def test_templates_using_and_braced_constructor_forms(tmp_path):
    root = workspace(tmp_path, """
using pros::MotorGroup;
namespace robot { using namespace pros;
const std::array<int8_t, 2> ports{{-1, 2}};
MotorGroup a{ports};
Motor motor = pros::Motor{3};
}
MotorGroup b{4, -5};
pros::MotorGroup c(std::vector<std::int8_t>{6,-7});
Motor unrelated(8);
""")
    result = discover_targets(root)
    found = targets(result)
    assert set(found) == {"robot::a", "robot::motor", "b", "c"}
    assert found["robot::a"]["ports"] == [-1, 2]
    assert found["b"]["ports"] == [4, -5]
    assert found["c"]["ports"] == [6, -7]
    assert not result["warnings"]


def test_simulation_fallback_does_not_claim_code_discovery(tmp_path):
    root = workspace(tmp_path, "pros::Motor known(1);")
    result = discover_targets(root, {"motors": [{"port": 1, "joint": "wheel"},
                                                  {"port": 2, "joint": "claw"}]})
    assert len(result["targets"]) == 2
    fallback = result["targets"][1]
    assert fallback["ports"] == [2] and not fallback["verified"]
    assert fallback["source"] == "simulation mapping"
    assert "not discovered" in fallback["notes"][0]


def test_hdrive_detection_is_explicit_about_unverified_order(tmp_path):
    root = workspace(tmp_path, """
pros::MotorGroup left({1,2}); pros::MotorGroup right({-3,-4}); pros::Motor strafe(5);
eerkit::HDrivetrain drive(left, right, strafe);
""")
    drive = targets(discover_targets(root))["drive"]
    assert drive["kind"] == "h_drive" and not drive["verified"]
    assert drive["axes"]["forward"] == [1,1,1,1,0]
    assert drive["axes"]["strafe"] == [0,0,0,0,1]
    assert "order assumed" in drive["notes"][0]


def test_static_objects_in_other_translation_units_are_not_visible(tmp_path):
    root = workspace(tmp_path, "eerkit::TankDrive drive(left, right, 4, 200, 12);", {
        "src/other.cpp": "static pros::Motor left(1); static pros::Motor right(2);"})
    result = discover_targets(root)
    assert "drive" not in targets(result)
    assert any("unresolved reference left" in message for message in result["warnings"])


@pytest.mark.parametrize("ports", [[], [0], [22], [True], [1, -1], [1.0], ["1"]])
def test_manual_invalid_ports_rejected(ports):
    with pytest.raises(ValueError):
        make_target("group", "motor_group", ports)


@pytest.mark.parametrize("axes", [{}, {"shaft": [0]}, {"shaft": [float("nan")]},
                                  {"shaft": [2]}, {"shaft": [1, 1]}])
def test_manual_invalid_axes_rejected(axes):
    with pytest.raises(ValueError):
        make_target("motor", "motor", [1], axes)


def test_manual_drivetrain_conventions_are_stable_and_validated():
    target = make_target("X", "x_drive", [1,-2,3,-4])
    assert target["axes"] == {"forward": [1,1,1,1], "turn": [1,-1,1,-1], "strafe": [1,-1,-1,1]}
    assert target == make_target("X", "x_drive", [1,-2,3,-4])
    assert not target["verified"]
    with pytest.raises(ValueError, match="explicit"):
        make_target("six-motor tank", "tank", [1,2,3,4,5,6])


def test_workspace_is_read_only_and_vendor_examples_are_excluded(tmp_path):
    root = workspace(tmp_path, "pros::Motor real(1);", {
        "include/pros/motors.hpp": "pros::Motor documentation_example(2);"})
    before = {p: p.read_bytes() for p in root.rglob("*") if p.is_file()}
    assert set(targets(discover_targets(root))) == {"real"}
    assert {p: p.read_bytes() for p in root.rglob("*") if p.is_file()} == before


def test_no_workspace_can_still_show_simulation_bindings():
    result = discover_targets("", {"motors": [{"port": 3, "joint": "lift"}]})
    assert result["targets"][0]["ports"] == [3]
    assert result["warnings"]


def test_directives_in_comments_and_raw_strings_cannot_supply_ports(tmp_path):
    root = workspace(tmp_path, '''
/*
#define GHOST 1
*/
const char *raw = R"tag(
#define IMAGINARY 2
)tag";
pros::Motor a(GHOST);
pros::Motor b(IMAGINARY);
''')
    result = discover_targets(root)
    assert not result["targets"]
    assert len(result["warnings"]) == 2


def test_qualified_reverse_constant_resolves_but_unknown_reverse_is_not_guessed(tmp_path):
    root = workspace(tmp_path, '''
namespace options { constexpr bool reversed = true; }
pros::Motor known(1, pros::E_MOTOR_GEARSET_18, options::reversed);
pros::Motor modern(2, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor unknown(3, runtime_reverse);
pros::Motor legacy(4, E_MOTOR_GEARSET_18, runtime_reverse, E_MOTOR_ENCODER_DEGREES);
''')
    result = discover_targets(root)
    found = targets(result)
    assert found["known"]["ports"] == [-1]
    assert found["modern"]["ports"] == [2]
    assert "unknown" not in found and "legacy" not in found
    assert len(result["warnings"]) == 2
