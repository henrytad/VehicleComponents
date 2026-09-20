# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

const MM = 1000

@named model = VehicleComponents.InboardTestRig()
ssys = multibody(model)
prob = ODEProblem(ssys, [], (0.0, 2.0))
sol = solve(prob)

plot_deflection = Plots.plot(
    sol,
    idxs=[MM * model.inboard.heave_strut.s, MM * model.inboard.roll_strut.s],
    ylabel="deflection [mm]",
    label=["heave" "roll"]
);
plot_spring = Plots.plot(
    sol,
    idxs=[model.inboard.heave_strut.spring.f, model.inboard.roll_strut.spring.f],
    ylabel="spring force [N]",
    label=["heave" "roll"]
);
plot_damper = Plots.plot(
    sol,
    idxs=[model.inboard.heave_strut.damper.f, model.inboard.roll_strut.damper.f],
    ylabel="damper force [N]",
    xlabel="time [s]",
    label=["heave" "roll"]
);
Plots.plot(plot_deflection, plot_spring, plot_damper; layout=(3, 1), link=:x, lw=2, size=(800, 900))

render(model, sol; filename="output/inboard.gif", up=[0, 0, 1], x=1, y=0, z=1, lookat=[0, 0, 0.4])
