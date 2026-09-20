using Revise
using VehicleComponents
using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

const DEG = 180 / π
const MM = 1000

@named model = VehicleComponents.LinkageTestRig()
ssys = multibody(model)
prob = ODEProblem(ssys, [], (0.0, 2.0))
sol = solve(prob)

plot_camber = Plots.plot(
    sol,
    idxs=(MM * model.linkage.wc_height, DEG * model.linkage.camber),
    ylabel="camber [deg]"
);
plot_toe = Plots.plot(
    sol,
    idxs=(MM * model.linkage.wc_height, DEG * model.linkage.toe),
    ylabel="toe [deg]",
    xlabel="wheel travel [mm]"
);
Plots.plot(
    plot_camber, plot_toe;
    layout=(2, 1),
    link=:x,
    legend=false,
    lw=2
)

render(model, sol; filename="output/linkage.gif", up=[0, 0, 1], x=1, y=0.1, z=1, lookat=[0, 0, 0.3])
