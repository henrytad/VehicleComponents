using Revise
using VehicleComponents
using ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

@named model = VehicleComponents.DamperTestRig()
ssys = multibody(model)
prob = ODEProblem(ssys, [], (0.0, 0.4))
sol = solve(prob)

selection = VehicleComponents.params.suspension.front.setup.heave.damper
name = "front heave: $(selection.valve_code), compression $(selection.compression_position), extension $(selection.extension_position)"

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
