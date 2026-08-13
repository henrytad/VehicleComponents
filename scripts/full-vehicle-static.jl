using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

const DEG = 180 / π

@named model = VehicleComponents.FullVehicleTestStatic()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "Test.json")
data = JSON3.read(read(data_path, String))

car = ssys.vehicle
body = data.body
tires = data.tires
front, rear = data.suspension.front, data.suspension.rear

parameter_map = Dict([
    # body
    car.sprung_mass => body.mass,
    car.cg_offset => body.cg,
    car.sprung_I_11 => body.i_11,
    car.sprung_I_22 => body.i_22,
    car.sprung_I_33 => body.i_33,

    # tires
    car.tire_fl.width => tires.WIDTH,
    car.tire_fl.unloaded_radius => tires.UNLOADED_RADIUS,
    car.tire_fl.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    car.tire_fl.vertical_damping => tires.VERTICAL_DAMPING,
    car.tire_fr.width => tires.WIDTH,
    car.tire_fr.unloaded_radius => tires.UNLOADED_RADIUS,
    car.tire_fr.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    car.tire_fr.vertical_damping => tires.VERTICAL_DAMPING,
    car.tire_rl.width => tires.WIDTH,
    car.tire_rl.unloaded_radius => tires.UNLOADED_RADIUS,
    car.tire_rl.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    car.tire_rl.vertical_damping => tires.VERTICAL_DAMPING,
    car.tire_rr.width => tires.WIDTH,
    car.tire_rr.unloaded_radius => tires.UNLOADED_RADIUS,
    car.tire_rr.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    car.tire_rr.vertical_damping => tires.VERTICAL_DAMPING,

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
    car.front_suspension.linkage_left.upright_mass => front.geometry.linkages.left.upright.mass,
    car.front_suspension.linkage_left.upright_I_11 => front.geometry.linkages.left.upright.i_11,
    car.front_suspension.linkage_left.upright_I_22 => front.geometry.linkages.left.upright.i_22,
    car.front_suspension.linkage_left.upright_I_33 => front.geometry.linkages.left.upright.i_33,
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
    car.front_suspension.linkage_right.upright_mass => front.geometry.linkages.right.upright.mass,
    car.front_suspension.linkage_right.upright_I_11 => front.geometry.linkages.right.upright.i_11,
    car.front_suspension.linkage_right.upright_I_22 => front.geometry.linkages.right.upright.i_22,
    car.front_suspension.linkage_right.upright_I_33 => front.geometry.linkages.right.upright.i_33,
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
    car.rear_suspension.linkage_left.upright_mass => rear.geometry.linkages.left.upright.mass,
    car.rear_suspension.linkage_left.upright_I_11 => rear.geometry.linkages.left.upright.i_11,
    car.rear_suspension.linkage_left.upright_I_22 => rear.geometry.linkages.left.upright.i_22,
    car.rear_suspension.linkage_left.upright_I_33 => rear.geometry.linkages.left.upright.i_33,
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
    car.rear_suspension.linkage_right.upright_mass => rear.geometry.linkages.right.upright.mass,
    car.rear_suspension.linkage_right.upright_I_11 => rear.geometry.linkages.right.upright.i_11,
    car.rear_suspension.linkage_right.upright_I_22 => rear.geometry.linkages.right.upright.i_22,
    car.rear_suspension.linkage_right.upright_I_33 => rear.geometry.linkages.right.upright.i_33,
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
    idxs=[car.tire_fl.Fz, car.tire_fr.Fz, car.tire_rl.Fz, car.tire_rr.Fz],
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
