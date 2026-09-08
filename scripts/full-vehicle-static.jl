using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

const DEG = 180 / π

@named model = VehicleComponents.FullVehicleTestStatic()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "MR25.json")
data = JSON3.read(read(data_path, String))

car = ssys.vehicle
body = data.body
tires = data.tires
wheels = data.wheels
drivetrain = data.drivetrain
ctrl = data.control
front, rear = data.suspension.front, data.suspension.rear

corners = [car.corner_fl, car.corner_fr, car.corner_rl, car.corner_rr]

# TireMF61 takes the data as describing a LEFT-side tyre and mirrors the right.
# Which side each corner is on is structural and set in FullVehicle, so it
# cannot be overridden here.
@assert uppercase(String(tires.TYRESIDE)) == "LEFT" "tires.TYRESIDE is $(tires.TYRESIDE), but TireMF61 assumes a LEFT-side fit; mirror the data first"

tire_parameters(tire) = [
    # [DIMENSION] / [VERTICAL]
    tire.width => tires.WIDTH,
    tire.unloaded_radius => tires.UNLOADED_RADIUS,
    tire.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    tire.vertical_damping => tires.VERTICAL_DAMPING,
    tire.longitudinal_stiffness => tires.LONGITUDINAL_STIFFNESS,
    tire.lateral_stiffness => tires.LATERAL_STIFFNESS,
    tire.FNOMIN => tires.FNOMIN,
    tire.BREFF => tires.BREFF,
    tire.DREFF => tires.DREFF,
    tire.FREFF => tires.FREFF,

    # [OPERATING_CONDITIONS] / [MODEL]
    tire.INFLPRES => tires.INFLPRES,
    tire.NOMPRES => tires.NOMPRES,
    tire.LONGVL => tires.LONGVL,

    # [LONGITUDINAL_COEFFICIENTS]
    tire.PCX1 => tires.PCX1,
    tire.PDX1 => tires.PDX1,
    tire.PDX2 => tires.PDX2,
    tire.PDX3 => tires.PDX3,
    tire.PEX1 => tires.PEX1,
    tire.PEX2 => tires.PEX2,
    tire.PEX3 => tires.PEX3,
    tire.PEX4 => tires.PEX4,
    tire.PKX1 => tires.PKX1,
    tire.PKX2 => tires.PKX2,
    tire.PKX3 => tires.PKX3,
    tire.PHX1 => tires.PHX1,
    tire.PHX2 => tires.PHX2,
    tire.PVX1 => tires.PVX1,
    tire.PVX2 => tires.PVX2,
    tire.PPX1 => tires.PPX1,
    tire.PPX2 => tires.PPX2,
    tire.PPX3 => tires.PPX3,
    tire.PPX4 => tires.PPX4,
    tire.RBX1 => tires.RBX1,
    tire.RBX2 => tires.RBX2,
    tire.RBX3 => tires.RBX3,
    tire.RCX1 => tires.RCX1,
    tire.REX1 => tires.REX1,
    tire.REX2 => tires.REX2,
    tire.RHX1 => tires.RHX1,

    # [LATERAL_COEFFICIENTS]
    tire.PCY1 => tires.PCY1,
    tire.PDY1 => tires.PDY1,
    tire.PDY2 => tires.PDY2,
    tire.PDY3 => tires.PDY3,
    tire.PEY1 => tires.PEY1,
    tire.PEY2 => tires.PEY2,
    tire.PEY3 => tires.PEY3,
    tire.PEY4 => tires.PEY4,
    tire.PEY5 => tires.PEY5,
    tire.PKY1 => tires.PKY1,
    tire.PKY2 => tires.PKY2,
    tire.PKY3 => tires.PKY3,
    tire.PKY4 => tires.PKY4,
    tire.PKY5 => tires.PKY5,
    tire.PKY6 => tires.PKY6,
    tire.PKY7 => tires.PKY7,
    tire.PHY1 => tires.PHY1,
    tire.PHY2 => tires.PHY2,
    tire.PVY1 => tires.PVY1,
    tire.PVY2 => tires.PVY2,
    tire.PVY3 => tires.PVY3,
    tire.PVY4 => tires.PVY4,
    tire.PPY1 => tires.PPY1,
    tire.PPY2 => tires.PPY2,
    tire.PPY3 => tires.PPY3,
    tire.PPY4 => tires.PPY4,
    tire.PPY5 => tires.PPY5,
    tire.RBY1 => tires.RBY1,
    tire.RBY2 => tires.RBY2,
    tire.RBY3 => tires.RBY3,
    tire.RBY4 => tires.RBY4,
    tire.RCY1 => tires.RCY1,
    tire.REY1 => tires.REY1,
    tire.REY2 => tires.REY2,
    tire.RHY1 => tires.RHY1,
    tire.RHY2 => tires.RHY2,
    tire.RVY1 => tires.RVY1,
    tire.RVY2 => tires.RVY2,
    tire.RVY3 => tires.RVY3,
    tire.RVY4 => tires.RVY4,
    tire.RVY5 => tires.RVY5,
    tire.RVY6 => tires.RVY6,

    # [ALIGNING_COEFFICIENTS]
    tire.QBZ1 => tires.QBZ1,
    tire.QBZ2 => tires.QBZ2,
    tire.QBZ3 => tires.QBZ3,
    tire.QBZ4 => tires.QBZ4,
    tire.QBZ5 => tires.QBZ5,
    tire.QBZ6 => tires.QBZ6,
    tire.QBZ9 => tires.QBZ9,
    tire.QBZ10 => tires.QBZ10,
    tire.QCZ1 => tires.QCZ1,
    tire.QDZ1 => tires.QDZ1,
    tire.QDZ2 => tires.QDZ2,
    tire.QDZ3 => tires.QDZ3,
    tire.QDZ4 => tires.QDZ4,
    tire.QDZ6 => tires.QDZ6,
    tire.QDZ7 => tires.QDZ7,
    tire.QDZ8 => tires.QDZ8,
    tire.QDZ9 => tires.QDZ9,
    tire.QDZ10 => tires.QDZ10,
    tire.QDZ11 => tires.QDZ11,
    tire.QEZ1 => tires.QEZ1,
    tire.QEZ2 => tires.QEZ2,
    tire.QEZ3 => tires.QEZ3,
    tire.QEZ4 => tires.QEZ4,
    tire.QEZ5 => tires.QEZ5,
    tire.QHZ1 => tires.QHZ1,
    tire.QHZ2 => tires.QHZ2,
    tire.QHZ3 => tires.QHZ3,
    tire.QHZ4 => tires.QHZ4,
    tire.PPZ1 => tires.PPZ1,
    tire.PPZ2 => tires.PPZ2,
    tire.SSZ1 => tires.SSZ1,
    tire.SSZ2 => tires.SSZ2,
    tire.SSZ3 => tires.SSZ3,
    tire.SSZ4 => tires.SSZ4,

    # [SCALING_COEFFICIENTS]
    tire.LFZO => tires.LFZO,
    tire.LCX => tires.LCX,
    tire.LMUX => tires.LMUX,
    tire.LEX => tires.LEX,
    tire.LKX => tires.LKX,
    tire.LHX => tires.LHX,
    tire.LVX => tires.LVX,
    tire.LCY => tires.LCY,
    tire.LMUY => tires.LMUY,
    tire.LEY => tires.LEY,
    tire.LKY => tires.LKY,
    tire.LHY => tires.LHY,
    tire.LVY => tires.LVY,
    tire.LKYC => tires.LKYC,
    tire.LXAL => tires.LXAL,
    tire.LYKA => tires.LYKA,
    tire.LVYKA => tires.LVYKA,
    tire.LTR => tires.LTR,
    tire.LRES => tires.LRES,
    tire.LKZC => tires.LKZC,
    tire.LS => tires.LS,
    tire.LMUV => tires.LMUV,
]

corner_parameters(corner, upright) = [
    corner.upright_mass => upright.mass,
    corner.upright_I_11 => upright.i_11,
    corner.upright_I_22 => upright.i_22,
    corner.upright_I_33 => upright.i_33,
    corner.wheel_assembly.rim_mass => wheels.rim_mass,
    corner.wheel_assembly.rim_inertia => wheels.rim_inertia,
    corner.wheel_assembly.tire_mass => tires.MASS,
    corner.wheel_assembly.tire_inertia => tires.IYY,
    corner.motor.gear_ratio => drivetrain.gear_ratio,
]

parameter_map = Dict([
    # torque control
    car.control.slip_target_front => ctrl.slip_target_front,
    car.control.slip_target_rear => ctrl.slip_target_rear,
    car.control.launch_torque => ctrl.launch_torque,
    car.control.motor_torque_max => ctrl.motor_torque_max,
    car.control.k => ctrl.slip_loop_gain,
    car.control.Ti => ctrl.slip_loop_Ti,
    car.control.Ni => ctrl.slip_loop_Ni,

    # body
    car.sprung_mass => body.mass,
    car.sprung_cg => body.cg,
    car.sprung_I_11 => body.i_11,
    car.sprung_I_22 => body.i_22,
    car.sprung_I_33 => body.i_33,

    # tires
    tire_parameters(car.corner_fl.wheel_assembly.tire)...,
    tire_parameters(car.corner_fr.wheel_assembly.tire)...,
    tire_parameters(car.corner_rl.wheel_assembly.tire)...,
    tire_parameters(car.corner_rr.wheel_assembly.tire)...,

    # corners: unsprung body and rim
    corner_parameters(car.corner_fl, front.geometry.linkages.left.upright)...,
    corner_parameters(car.corner_fr, front.geometry.linkages.right.upright)...,
    corner_parameters(car.corner_rl, rear.geometry.linkages.left.upright)...,
    corner_parameters(car.corner_rr, rear.geometry.linkages.right.upright)...,

    # front axle, shared references
    car.front_suspension.wheel_center_left => front.geometry.linkages.left.wheel_center,
    car.front_suspension.wheel_center_right => front.geometry.linkages.right.wheel_center,
    car.front_suspension.pushrod_outer_left => front.geometry.linkages.left.pushrod_outer,
    car.front_suspension.pushrod_outer_right => front.geometry.linkages.right.pushrod_outer,

    # front left corner
    car.front_suspension.linkage_left.lca_front => front.geometry.linkages.left.lca_front,
    car.front_suspension.linkage_left.lca_rear => front.geometry.linkages.left.lca_rear,
    car.front_suspension.linkage_left.lca_outer => front.geometry.linkages.left.lca_outer,
    car.front_suspension.linkage_left.uca_front => front.geometry.linkages.left.uca_front,
    car.front_suspension.linkage_left.uca_rear => front.geometry.linkages.left.uca_rear,
    car.front_suspension.linkage_left.uca_outer => front.geometry.linkages.left.uca_outer,
    car.front_suspension.linkage_left.tierod_inner => front.geometry.linkages.left.tierod_inner,
    car.front_suspension.linkage_left.tierod_outer => front.geometry.linkages.left.tierod_outer,
    car.front_suspension.linkage_left.static_camber => front.setup.alignment.left.static_camber,
    car.front_suspension.linkage_left.static_toe => front.setup.alignment.left.static_toe,

    # front right corner
    car.front_suspension.linkage_right.lca_front => front.geometry.linkages.right.lca_front,
    car.front_suspension.linkage_right.lca_rear => front.geometry.linkages.right.lca_rear,
    car.front_suspension.linkage_right.lca_outer => front.geometry.linkages.right.lca_outer,
    car.front_suspension.linkage_right.uca_front => front.geometry.linkages.right.uca_front,
    car.front_suspension.linkage_right.uca_rear => front.geometry.linkages.right.uca_rear,
    car.front_suspension.linkage_right.uca_outer => front.geometry.linkages.right.uca_outer,
    car.front_suspension.linkage_right.tierod_inner => front.geometry.linkages.right.tierod_inner,
    car.front_suspension.linkage_right.tierod_outer => front.geometry.linkages.right.tierod_outer,
    car.front_suspension.linkage_right.static_camber => front.setup.alignment.right.static_camber,
    car.front_suspension.linkage_right.static_toe => front.setup.alignment.right.static_toe,

    # front inboard
    car.front_suspension.inboard.rocker_pivot_left => front.geometry.inboard.rocker_pivot.left,
    car.front_suspension.inboard.rocker_pivot_right => front.geometry.inboard.rocker_pivot.right,
    car.front_suspension.inboard.pushrod_inner_left => front.geometry.inboard.pushrod_inner.left,
    car.front_suspension.inboard.pushrod_inner_right => front.geometry.inboard.pushrod_inner.right,
    car.front_suspension.inboard.heave_pickup_left => front.geometry.inboard.heave_pickup.left,
    car.front_suspension.inboard.heave_pickup_right => front.geometry.inboard.heave_pickup.right,
    car.front_suspension.inboard.roll_pickup_left => front.geometry.inboard.roll_pickup.left,
    car.front_suspension.inboard.roll_pickup_right => front.geometry.inboard.roll_pickup.right,
    car.front_suspension.inboard.heave_stiffness => front.setup.heave.stiffness,
    car.front_suspension.inboard.heave_damping => front.setup.heave.damping,
    car.front_suspension.inboard.heave_preload => front.setup.heave.preload,
    car.front_suspension.inboard.roll_stiffness => front.setup.roll.stiffness,
    car.front_suspension.inboard.roll_damping => front.setup.roll.damping,
    car.front_suspension.inboard.roll_preload => front.setup.roll.preload,
    car.front_suspension.inboard.pushrod_adjust_left => front.setup.pushrod_adjust.left,
    car.front_suspension.inboard.pushrod_adjust_right => front.setup.pushrod_adjust.right,

    # rear axle, shared references
    car.rear_suspension.wheel_center_left => rear.geometry.linkages.left.wheel_center,
    car.rear_suspension.wheel_center_right => rear.geometry.linkages.right.wheel_center,
    car.rear_suspension.pushrod_outer_left => rear.geometry.linkages.left.pushrod_outer,
    car.rear_suspension.pushrod_outer_right => rear.geometry.linkages.right.pushrod_outer,

    # rear left corner
    car.rear_suspension.linkage_left.lca_front => rear.geometry.linkages.left.lca_front,
    car.rear_suspension.linkage_left.lca_rear => rear.geometry.linkages.left.lca_rear,
    car.rear_suspension.linkage_left.lca_outer => rear.geometry.linkages.left.lca_outer,
    car.rear_suspension.linkage_left.uca_front => rear.geometry.linkages.left.uca_front,
    car.rear_suspension.linkage_left.uca_rear => rear.geometry.linkages.left.uca_rear,
    car.rear_suspension.linkage_left.uca_outer => rear.geometry.linkages.left.uca_outer,
    car.rear_suspension.linkage_left.tierod_inner => rear.geometry.linkages.left.tierod_inner,
    car.rear_suspension.linkage_left.tierod_outer => rear.geometry.linkages.left.tierod_outer,
    car.rear_suspension.linkage_left.static_camber => rear.setup.alignment.left.static_camber,
    car.rear_suspension.linkage_left.static_toe => rear.setup.alignment.left.static_toe,

    # rear right corner
    car.rear_suspension.linkage_right.lca_front => rear.geometry.linkages.right.lca_front,
    car.rear_suspension.linkage_right.lca_rear => rear.geometry.linkages.right.lca_rear,
    car.rear_suspension.linkage_right.lca_outer => rear.geometry.linkages.right.lca_outer,
    car.rear_suspension.linkage_right.uca_front => rear.geometry.linkages.right.uca_front,
    car.rear_suspension.linkage_right.uca_rear => rear.geometry.linkages.right.uca_rear,
    car.rear_suspension.linkage_right.uca_outer => rear.geometry.linkages.right.uca_outer,
    car.rear_suspension.linkage_right.tierod_inner => rear.geometry.linkages.right.tierod_inner,
    car.rear_suspension.linkage_right.tierod_outer => rear.geometry.linkages.right.tierod_outer,
    car.rear_suspension.linkage_right.static_camber => rear.setup.alignment.right.static_camber,
    car.rear_suspension.linkage_right.static_toe => rear.setup.alignment.right.static_toe,

    # rear inboard
    car.rear_suspension.inboard.rocker_pivot_left => rear.geometry.inboard.rocker_pivot.left,
    car.rear_suspension.inboard.rocker_pivot_right => rear.geometry.inboard.rocker_pivot.right,
    car.rear_suspension.inboard.pushrod_inner_left => rear.geometry.inboard.pushrod_inner.left,
    car.rear_suspension.inboard.pushrod_inner_right => rear.geometry.inboard.pushrod_inner.right,
    car.rear_suspension.inboard.heave_pickup_left => rear.geometry.inboard.heave_pickup.left,
    car.rear_suspension.inboard.heave_pickup_right => rear.geometry.inboard.heave_pickup.right,
    car.rear_suspension.inboard.roll_pickup_left => rear.geometry.inboard.roll_pickup.left,
    car.rear_suspension.inboard.roll_pickup_right => rear.geometry.inboard.roll_pickup.right,
    car.rear_suspension.inboard.heave_stiffness => rear.setup.heave.stiffness,
    car.rear_suspension.inboard.heave_damping => rear.setup.heave.damping,
    car.rear_suspension.inboard.heave_preload => rear.setup.heave.preload,
    car.rear_suspension.inboard.roll_stiffness => rear.setup.roll.stiffness,
    car.rear_suspension.inboard.roll_damping => rear.setup.roll.damping,
    car.rear_suspension.inboard.roll_preload => rear.setup.roll.preload,
    car.rear_suspension.inboard.pushrod_adjust_left => rear.setup.pushrod_adjust.left,
    car.rear_suspension.inboard.pushrod_adjust_right => rear.setup.pushrod_adjust.right,
])

prob = ODEProblem(ssys, parameter_map, (0.0, 2.0))
sol = solve(prob)

plot_tire_forces = Plots.plot(
    sol;
    idxs=[c.wheel_assembly.tire.Fz for c in corners],
    labels=["FL" "FR" "RL" "RR"],
    linestyle=[:dot :dot :dash :dash],
    linewidth=2,
    xlabel="t [s]",
    ylabel="Vertical load [N]",
    title="Tire normal loads",
);
plot_spring_forces = Plots.plot(
    sol;
    idxs=[car.front_suspension.inboard.heave_spring.f, car.front_suspension.inboard.roll_spring.f],
    labels=["heave" "roll"],
    linewidth=2,
    xlabel="t [s]",
    ylabel="Spring force [N]",
    title="Front inboard spring forces",
);
plot_body_angles = Plots.plot(
    sol;
    idxs=[ssys.roll_joint.phi * DEG, ssys.pitch_joint.phi * DEG],
    labels=["roll" "pitch"],
    linewidth=2,
    xlabel="t [s]",
    ylabel="angle [deg]",
    title="Body attitude",
);
Plots.plot(
    plot_tire_forces,
    plot_spring_forces,
    plot_body_angles;
    layout=(2, 2),
    size=(1400, 800),
    left_margin=5Plots.PlotMeasures.mm,
    bottom_margin=5Plots.PlotMeasures.mm
)

render(model, sol; filename="output/full_vehicle_static.gif", up=[0, 0, 1], x=1.5, y=0.4, z=0.7, lookat=[0, 0.05, 0.3])
