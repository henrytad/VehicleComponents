using JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

@named model = VehicleComponents.DamperTestRig()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "MR25.json")
data = JSON3.read(read(data_path, String))

# Damper to sweep. The setup entry picks the valve code and both clicks.
axle, mode = :front, :heave
selection = data.suspension[axle].setup[mode].damper
curve = VehicleComponents.damper_map(data.dampers, selection)

parameter_map = Dict([
    ssys.damper.force_map.independent_var => curve.velocity,
    ssys.damper.force_map.data => curve.force,
])

prob = ODEProblem(ssys, parameter_map, (0.0, 0.4))
sol = solve(prob)

name = "$axle $mode: $(selection.valve_code), compression $(selection.compression_position), extension $(selection.extension_position)"

t = range(0.0, 0.4, length=2000)
plot_curve = Plots.plot(
    sol(t, idxs=model.damper.v).u,
    sol(t, idxs=model.damper.f).u,
    label="simulated",
    title=name,
    xlabel="velocity [m/s] (negative = compression)",
    ylabel="force [N] (positive = tension)"
);
plot_velocity = Plots.plot(
    sol,
    idxs=[model.damper.v],
    ylabel="velocity [m/s]",
    label="velocity"
);
plot_force = Plots.plot(
    sol,
    idxs=[model.damper.f],
    ylabel="force [N]",
    xlabel="time [s]",
    label="force"
);
Plots.plot(plot_curve, plot_velocity, plot_force; layout=(3, 1), lw=2, size=(800, 900))
