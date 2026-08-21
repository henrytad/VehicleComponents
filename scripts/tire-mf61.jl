# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

const MM = 1000

@named model = VehicleComponents.TireTestRig()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "Test.json")
data = JSON3.read(read(data_path, String))

tires = data.tires

# Slip sweep. The carriage rolls the wheel forward at `speed` while the spin
# drive ramps wheel speed either side of free rolling, so the tyre walks across
# its slip curve at a held vertical load.
speed = 10.0
static_deflection = 0.005
kappa_start = -0.2
kappa_end = 0.2
stop = 2.0

parameter_map = Dict([
    ssys.unloaded_radius => tires.UNLOADED_RADIUS
    ssys.tire.width => tires.WIDTH
    ssys.tire.vertical_stiffness => tires.VERTICAL_STIFFNESS
    ssys.tire.vertical_damping => tires.VERTICAL_DAMPING
    ssys.tire.longitudinal_stiffness => tires.LONGITUDINAL_STIFFNESS

    # Hold the vertical load steady and sweep slip instead
    ssys.speed => speed
    ssys.static_deflection => static_deflection
    ssys.stroke => 0.0
    ssys.kappa_start => kappa_start
    ssys.kappa_end => kappa_end
    ssys.slip_ramp_time => stop
])

prob = ODEProblem(ssys, parameter_map, (0.0, stop))
sol = solve(prob)

# The slip curve itself: Fx against the slip the model actually reports, not
# the commanded value. Peak Fx and the slip it occurs at are what to read here.
plot_slip_curve = Plots.plot(
    sol;
    idxs=(model.tire.kappa, model.tire.Fx),
    xlabel="slip ratio [-]",
    ylabel="Fx [N]",
    title="Slip curve",
    legend=false,
);
plot_kappa = Plots.plot(
    sol;
    idxs=model.tire.kappa,
    xlabel="t [s]",
    ylabel="kappa [-]",
    title="Slip ratio",
    legend=false,
);
plot_fx = Plots.plot(
    sol;
    idxs=model.tire.Fx,
    xlabel="t [s]",
    ylabel="Fx [N]",
    title="Longitudinal force",
    legend=false,
);
plot_fz = Plots.plot(
    sol;
    idxs=model.tire.Fz,
    xlabel="t [s]",
    ylabel="Fz [N]",
    title="Vertical load",
    legend=false,
);
Plots.plot(
    plot_slip_curve, plot_kappa, plot_fx, plot_fz;
    layout=(2, 2), lw=2, size=(1200, 800),
    left_margin=5Plots.PlotMeasures.mm,
    bottom_margin=5Plots.PlotMeasures.mm,
)

# Vertical sweep. Same rig with the carriage and spin drive parked, which is
# what the `sweep` test exercises.
vertical_map = Dict([
    ssys.unloaded_radius => tires.UNLOADED_RADIUS
    ssys.tire.width => tires.WIDTH
    ssys.tire.vertical_stiffness => tires.VERTICAL_STIFFNESS
    ssys.tire.vertical_damping => tires.VERTICAL_DAMPING
    ssys.tire.longitudinal_stiffness => tires.LONGITUDINAL_STIFFNESS
])

prob_vertical = ODEProblem(ssys, vertical_map, (0.0, 2.0))
sol_vertical = solve(prob_vertical)

plot_height = Plots.plot(
    sol_vertical,
    idxs=MM * model.tire.z_w,
    ylabel="wheel centre [mm]"
);
plot_deflection = Plots.plot(
    sol_vertical,
    idxs=MM * model.tire.rho,
    ylabel="deflection [mm]"
);
plot_force = Plots.plot(
    sol_vertical,
    idxs=model.tire.Fz,
    ylabel="contact force [N]",
    xlabel="time [s]"
);
Plots.plot(plot_height, plot_deflection, plot_force; layout=(3, 1), link=:x, lw=2, legend=false, size=(800, 900))

render(model, sol; filename="output/tire.gif", up=[0, 0, 1], x=1, y=0.1, z=1, lookat=[0, 0, 0.3])
