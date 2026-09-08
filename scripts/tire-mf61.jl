# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

const MM = 1000
const DEG = 180 / π

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
    ssys.unloaded_radius => tires.UNLOADED_RADIUS,
    ssys.tire.width => tires.WIDTH,
    ssys.tire.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    ssys.tire.vertical_damping => tires.VERTICAL_DAMPING,
    ssys.tire.longitudinal_stiffness => tires.LONGITUDINAL_STIFFNESS,
    ssys.tire.lateral_stiffness => tires.LATERAL_STIFFNESS,

    # Hold the vertical load steady and sweep slip instead
    ssys.speed => speed,
    ssys.static_deflection => static_deflection,
    ssys.stroke => 0.0,
    ssys.kappa_start => kappa_start,
    ssys.kappa_end => kappa_end,
    ssys.slip_ramp_time => stop,
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

# Slip angle sweep. The carriage still rolls the wheel forward along world x,
# but now the steer drive rotates the wheel plane, so the contact patch picks up
# lateral velocity. The spin drive holds a fixed wheel speed, so steering cuts
# the forward velocity to speed*cos(alpha) while omega*Re stays put and the tyre
# picks up a little drive slip: kappa runs from -0.003 at centre to +0.018 at
# the ends of the sweep. Small, and with no combined slip yet it does not touch
# Fy, but it is why Fx is not zero here.
alpha_start = -0.3
alpha_end = 0.3

# Swept at a family of vertical loads. Every load term in the fit is linear in
# dfz = (Fz - FNOMIN) / FNOMIN, so a single sweep tells you nothing about whether
# the load sensitivity holds up. The first three are where this car actually
# runs; the last is the fit's own 4850 N nominal, for reference. The mount
# preload sets the load: Fz = vertical_stiffness * static_deflection.
sweep_loads = [400.0, 700.0, 1000.0, 4850.0]

lateral_map(deflection) = Dict([
    ssys.unloaded_radius => tires.UNLOADED_RADIUS,
    ssys.tire.width => tires.WIDTH,
    ssys.tire.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    ssys.tire.vertical_damping => tires.VERTICAL_DAMPING,
    ssys.tire.longitudinal_stiffness => tires.LONGITUDINAL_STIFFNESS,
    ssys.tire.lateral_stiffness => tires.LATERAL_STIFFNESS,
    ssys.speed => speed,
    ssys.static_deflection => deflection,
    ssys.stroke => 0.0,
    ssys.alpha_start => alpha_start,
    ssys.alpha_end => alpha_end,
    ssys.steer_ramp_time => stop,
])

lateral_sols = map(sweep_loads) do Fz
    solve(ODEProblem(ssys, lateral_map(Fz / tires.VERTICAL_STIFFNESS), (0.0, stop)))
end

# The lateral carcass deflection starts at zero, so Fy needs a few relaxation
# lengths to catch up with the commanded slip angle. At 10 m/s that is about
# 6 ms; everything before 50 ms is startup, not tyre behaviour.
settled(sol) = findfirst(>(0.05), sol.t):length(sol.t)

# The cornering curve. Read the slope through the origin (cornering stiffness),
# the peak, and the slip angle it happens at. Plotted against the slip angle the
# model reports, not the commanded steer angle.
plot_cornering_curve = Plots.plot(
    xlabel="slip angle [deg]", ylabel="Fy [N]",
    title="Cornering curve", legend=:topright,
);
# Normalised. A curve that walks up the page with load, or asks for more than
# about 1.8, is telling you the lateral fit is wrong rather than that the tyre
# grips that well. Divided on the solution rather than through `idxs`, since a
# ratio of two observed variables is not something the indexer can build a
# getter for.
plot_mu_y = Plots.plot(
    xlabel="slip angle [deg]", ylabel="Fy / Fz [-]",
    title="Lateral friction used", legend=false,
);
plot_alpha = Plots.plot(
    xlabel="t [s]", ylabel="alpha [deg]",
    title="Slip angle", legend=false,
);
plot_fz_lat = Plots.plot(
    xlabel="t [s]", ylabel="Fz [N]",
    title="Vertical load", legend=false,
);
for (Fz, sol) in zip(sweep_loads, lateral_sols)
    k = settled(sol)
    alpha_deg = sol[model.tire.alpha] * DEG
    Plots.plot!(plot_cornering_curve, alpha_deg[k], sol[model.tire.Fy][k];
        label="Fz = $(round(Int, Fz)) N")
    Plots.plot!(plot_mu_y, alpha_deg[k], sol[model.tire.Fy][k] ./ sol[model.tire.Fz][k])
    Plots.plot!(plot_alpha, sol.t, alpha_deg)
    Plots.plot!(plot_fz_lat, sol.t, sol[model.tire.Fz])
end
Plots.plot(
    plot_cornering_curve, plot_mu_y, plot_alpha, plot_fz_lat;
    layout=(2, 2), lw=2, size=(1200, 800),
    left_margin=5Plots.PlotMeasures.mm,
    bottom_margin=5Plots.PlotMeasures.mm,
)

# Vertical sweep. Same rig with the carriage and spin drive parked, which is
# what the `sweep` test exercises.
vertical_map = Dict([
    ssys.unloaded_radius => tires.UNLOADED_RADIUS,
    ssys.tire.width => tires.WIDTH,
    ssys.tire.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    ssys.tire.vertical_damping => tires.VERTICAL_DAMPING,
    ssys.tire.longitudinal_stiffness => tires.LONGITUDINAL_STIFFNESS,
    ssys.tire.lateral_stiffness => tires.LATERAL_STIFFNESS,
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
