using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

const MM = 1000

@named model = VehicleComponents.TireTestRig()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "Test.json")
data = JSON3.read(read(data_path, String))

tires = data.tires

parameter_map = Dict([
    ssys.unloaded_radius => tires.UNLOADED_RADIUS
    ssys.tire.width => tires.WIDTH
    ssys.tire.vertical_stiffness => tires.VERTICAL_STIFFNESS
    ssys.tire.vertical_damping => tires.VERTICAL_DAMPING
])

prob = ODEProblem(ssys, parameter_map, (0.0, 2.0))
sol = solve(prob)

plot_height = Plots.plot(
    sol,
    idxs=MM * model.tire.z_w,
    ylabel="wheel centre [mm]"
);
plot_deflection = Plots.plot(
    sol,
    idxs=MM * model.tire.rho,
    ylabel="deflection [mm]"
);
plot_force = Plots.plot(
    sol,
    idxs=model.tire.Fz,
    ylabel="contact force [N]",
    xlabel="time [s]"
);
Plots.plot(plot_height, plot_deflection, plot_force; layout=(3, 1), link=:x, lw=2, legend=false, size=(800, 900))

render(model, sol; filename="output/tire.gif", up=[0, 0, 1], x=1, y=0.1, z=1, lookat=[0, 0, 0.3])
