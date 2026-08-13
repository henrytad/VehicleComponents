using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

const MM = 1000

@named model = VehicleComponents.InboardTestRig()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "Test.json")
data = JSON3.read(read(data_path, String))

front = data.suspension.front
geom, setup = front.geometry, front.setup
parameter_map = Dict([
    # Test rig
    ssys.pushrod_outer_left => geom.linkages.left.pushrod_outer,
    ssys.pushrod_outer_right => geom.linkages.right.pushrod_outer,

    # Geometry
    ssys.inboard.rocker_pivot_left => geom.inboard.rocker_pivot.left,
    ssys.inboard.rocker_pivot_right => geom.inboard.rocker_pivot.right,
    ssys.inboard.pushrod_inner_left => geom.inboard.pushrod_inner.left,
    ssys.inboard.pushrod_inner_right => geom.inboard.pushrod_inner.right,
    ssys.inboard.heave_pickup_left => geom.inboard.heave_pickup.left,
    ssys.inboard.heave_pickup_right => geom.inboard.heave_pickup.right,
    ssys.inboard.roll_pickup_left => geom.inboard.roll_pickup.left,
    ssys.inboard.roll_pickup_right => geom.inboard.roll_pickup.right,

    # Setup
    ssys.inboard.heave_stiffness => setup.heave.stiffness,
    ssys.inboard.heave_damping => setup.heave.damping,
    ssys.inboard.heave_preload => setup.heave.preload,
    ssys.inboard.roll_stiffness => setup.roll.stiffness,
    ssys.inboard.roll_damping => setup.roll.damping,
    ssys.inboard.roll_preload => setup.roll.preload,
    ssys.inboard.pushrod_adjust_left => setup.pushrod_adjust.left,
    ssys.inboard.pushrod_adjust_right => setup.pushrod_adjust.right,
])

prob = ODEProblem(ssys, parameter_map, (0.0, 2.0))
sol = solve(prob)

plot_deflection = Plots.plot(
    sol,
    idxs=[MM * model.inboard.heave_deflection, MM * model.inboard.roll_deflection],
    ylabel="deflection [mm]",
    label=["heave" "roll"]
);
plot_spring = Plots.plot(
    sol,
    idxs=[model.inboard.heave_spring_force, model.inboard.roll_spring_force],
    ylabel="spring force [N]",
    label=["heave" "roll"]
);
plot_damper = Plots.plot(
    sol,
    idxs=[model.inboard.heave_damper_force, model.inboard.roll_damper_force],
    ylabel="damper force [N]",
    xlabel="time [s]",
    label=["heave" "roll"]
);
Plots.plot(plot_deflection, plot_spring, plot_damper; layout=(3, 1), link=:x, lw=2, size=(800, 900))

render(model, sol; filename="output/inboard.gif", up=[0, 0, 1], x=1, y=0, z=1, lookat=[0, 0, 0.4])
