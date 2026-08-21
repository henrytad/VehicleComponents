using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents


const DEG = 180 / π

@named model = VehicleComponents.FullVehicleTestStraightLine()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "Test.json")
data = JSON3.read(read(data_path, String))

car = ssys.vehicle
body = data.body
tires = data.tires
wheels = data.wheels
drivetrain = data.drivetrain
front, rear = data.suspension.front, data.suspension.rear

corners = [car.corner_fl, car.corner_fr, car.corner_rl, car.corner_rr]

tire_parameters(tire) = [
    # [DIMENSION] / [VERTICAL]
    tire.width => tires.WIDTH,
    tire.unloaded_radius => tires.UNLOADED_RADIUS,
    tire.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    tire.vertical_damping => tires.VERTICAL_DAMPING,
    tire.longitudinal_stiffness => tires.LONGITUDINAL_STIFFNESS,
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

    # [SCALING_COEFFICIENTS]
    tire.LFZO => tires.LFZO,
    tire.LCX => tires.LCX,
    tire.LMUX => tires.LMUX,
    tire.LEX => tires.LEX,
    tire.LKX => tires.LKX,
    tire.LHX => tires.LHX,
    tire.LVX => tires.LVX,
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
    corner.drive_gear_ratio => drivetrain.gear_ratio,
]

parameter_map = Dict([
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

render(model, sol; filename="output/full_vehicle_straight_line.gif", up=[0, 0, 1], x=1.5, y=0.4, z=0.7, lookat=[0, 0.05, 0.3])

corner_labels = ["FL" "FR" "RL" "RR"]
corner_styles = [:solid :dash :solid :dash]
corner_colors = [1 1 2 2]

t_drive = 1.5     # drive_start_time
eps_sigma = 0.001   # relaxation-length floor [m]

mark!(p) = Plots.vline!(p, [t_drive]; color=:black, linestyle=:dot, linewidth=1, label="")

tpre = filter(t -> t < t_drive, sol.t)
println("\n--- before drive torque (t < $t_drive s) ---")
for (lbl, c) in zip(corner_labels, corners)
    fx = sol(tpre, idxs=c.wheel_assembly.tire.Fx).u
    sv = sol(tpre, idxs=c.wheel_assembly.tire.SVx).u
    println("  $lbl  max|Fx| = ", round(maximum(abs, fx), sigdigits=4),
        " N   max|SVx| = ", round(maximum(abs, sv), sigdigits=4), " N")
end
vpre = sol(tpre, idxs=ssys.longitudinal_joint.v).u
println("  max|vehicle speed| = ", round(maximum(abs, vpre), sigdigits=4), " m/s")
println("--- relaxation length over the whole run ---")
for (lbl, c) in zip(corner_labels, corners)
    sk = sol(sol.t, idxs=c.wheel_assembly.tire.sigma_kappa).u
    println("  $lbl  sigma_kappa: ", round(1000*minimum(sk), sigdigits=3), " .. ",
        round(1000*maximum(sk), sigdigits=3), " mm   (floor = ", 1000*eps_sigma, " mm)")
end
println("  final speed = ", round(sol(sol.t[end], idxs=ssys.longitudinal_joint.v), sigdigits=4), " m/s\n")

p_fx = Plots.plot(sol; idxs=[c.wheel_assembly.tire.Fx for c in corners],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="Fx [N]", title="Fx — must be flat 0 left of dotted line");
mark!(p_fx)

p_v = Plots.plot(sol; idxs=[ssys.longitudinal_joint.v], label="vehicle", color=3, linewidth=2,
    xlabel="t [s]", ylabel="v [m/s]", title="Speed — must be 0 before drive");
mark!(p_v)

p_slip = Plots.plot(sol;
    idxs=[corners[1].wheel_assembly.tire.kappa, corners[3].wheel_assembly.tire.kappa],
    labels=["FL kappa (4.E5)" "RL kappa (4.E5)"], color=[1 2], linewidth=2,
    xlabel="t [s]", ylabel="slip [-]", title="Instantaneous vs transient slip")
Plots.plot!(p_slip, sol;
    idxs=[corners[1].wheel_assembly.tire.kappa_prime, corners[3].wheel_assembly.tire.kappa_prime],
    labels=["FL kappa' (7.26)" "RL kappa' (7.26)"], color=[1 2], linestyle=:dash, linewidth=2)

p_sig = Plots.plot(sol; idxs=[1000*c.wheel_assembly.tire.sigma_kappa for c in corners],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="sigma_kappa [mm]", title="Relaxation length (7.8)")
Plots.hline!(p_sig, [1000*eps_sigma]; color=:red, linestyle=:dot, label="eps_sigma floor")

p_lim = Plots.plot(sol;
    idxs=[corners[3].wheel_assembly.tire.kappa_prime,
        corners[3].wheel_assembly.tire.kappa_sl,
        -corners[3].wheel_assembly.tire.kappa_sl],
    labels=["RL kappa'" "+kappa_sl" "-kappa_sl"], color=[2 :red :red],
    linestyle=[:solid :dot :dot], linewidth=2,
    xlabel="t [s]", ylabel="slip [-]", title="(7.25) limiter arms if kappa' leaves the band")

p_lam = Plots.plot(sol; idxs=[c.wheel_assembly.tire.lam_low for c in corners],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="lam_low [-]", title="Shift fade — 0 at rest, 1 once rolling")

p_svx = Plots.plot(sol; idxs=[c.wheel_assembly.tire.SVx for c in corners],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="SVx [N]", title="SVx — the artefact, 0 at rest");
mark!(p_svx)

p_fz = Plots.plot(sol; idxs=[c.wheel_assembly.tire.Fz for c in corners],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="Fz [N]", title="Normal load");
mark!(p_fz)

p_tau = Plots.plot(sol; idxs=[c.motor.tau for c in corners],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="tau [N.m]", title="Motor torque")

Plots.plot(p_fx, p_v, p_slip, p_sig, p_lim, p_lam, p_svx, p_fz, p_tau;
    layout=(3, 3), size=(1800, 1100), legendfontsize=6,
    left_margin=5Plots.PlotMeasures.mm, bottom_margin=5Plots.PlotMeasures.mm)