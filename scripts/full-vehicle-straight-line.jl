# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

const DEG = 180 / π

@named model = VehicleComponents.FullVehicleTestStraightLine()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "MR25.json")
data = JSON3.read(read(data_path, String))

car = ssys.vehicle
body = data.body
tires = data.tires
wheels = data.wheels
drivetrain = data.drivetrain
ctrl = data.control
front, rear = data.suspension.front, data.suspension.rear

t_drive = 1.5

corners = [car.corner_fl, car.corner_fr, car.corner_rl, car.corner_rr]

# TireMF61 takes the data as describing a LEFT-side tyre and mirrors the right.
# Which side each corner is on is structural and set in FullVehicle, so it
# cannot be overridden here.
@assert uppercase(String(tires.TYRESIDE)) == "LEFT" "tires.TYRESIDE is $(tires.TYRESIDE), but TireMF61 assumes a LEFT-side fit; mirror the data first"

tire_parameters(tire) = [
    # [DIMENSION] / [VERTICAL]
    tire.width => tires.WIDTH,
    tire.unloaded_radius => tires.UNLOADED_RADIUS,
    tire.vertical_stiffness => tires.VERTICAL_STIFFNESS,
    tire.vertical_damping => tires.VERTICAL_DAMPING,
    tire.longitudinal_stiffness => tires.LONGITUDINAL_STIFFNESS,
    tire.lateral_stiffness => tires.LATERAL_STIFFNESS,
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
    tire.RBX1 => tires.RBX1,
    tire.RBX2 => tires.RBX2,
    tire.RBX3 => tires.RBX3,
    tire.RCX1 => tires.RCX1,
    tire.REX1 => tires.REX1,
    tire.REX2 => tires.REX2,
    tire.RHX1 => tires.RHX1,

    # [LATERAL_COEFFICIENTS]
    tire.PCY1 => tires.PCY1,
    tire.PDY1 => tires.PDY1,
    tire.PDY2 => tires.PDY2,
    tire.PDY3 => tires.PDY3,
    tire.PEY1 => tires.PEY1,
    tire.PEY2 => tires.PEY2,
    tire.PEY3 => tires.PEY3,
    tire.PEY4 => tires.PEY4,
    tire.PEY5 => tires.PEY5,
    tire.PKY1 => tires.PKY1,
    tire.PKY2 => tires.PKY2,
    tire.PKY3 => tires.PKY3,
    tire.PKY4 => tires.PKY4,
    tire.PKY5 => tires.PKY5,
    tire.PKY6 => tires.PKY6,
    tire.PKY7 => tires.PKY7,
    tire.PHY1 => tires.PHY1,
    tire.PHY2 => tires.PHY2,
    tire.PVY1 => tires.PVY1,
    tire.PVY2 => tires.PVY2,
    tire.PVY3 => tires.PVY3,
    tire.PVY4 => tires.PVY4,
    tire.PPY1 => tires.PPY1,
    tire.PPY2 => tires.PPY2,
    tire.PPY3 => tires.PPY3,
    tire.PPY4 => tires.PPY4,
    tire.PPY5 => tires.PPY5,
    tire.RBY1 => tires.RBY1,
    tire.RBY2 => tires.RBY2,
    tire.RBY3 => tires.RBY3,
    tire.RBY4 => tires.RBY4,
    tire.RCY1 => tires.RCY1,
    tire.REY1 => tires.REY1,
    tire.REY2 => tires.REY2,
    tire.RHY1 => tires.RHY1,
    tire.RHY2 => tires.RHY2,
    tire.RVY1 => tires.RVY1,
    tire.RVY2 => tires.RVY2,
    tire.RVY3 => tires.RVY3,
    tire.RVY4 => tires.RVY4,
    tire.RVY5 => tires.RVY5,
    tire.RVY6 => tires.RVY6,

    # [ALIGNING_COEFFICIENTS]
    tire.QBZ1 => tires.QBZ1,
    tire.QBZ2 => tires.QBZ2,
    tire.QBZ3 => tires.QBZ3,
    tire.QBZ4 => tires.QBZ4,
    tire.QBZ5 => tires.QBZ5,
    tire.QBZ6 => tires.QBZ6,
    tire.QBZ9 => tires.QBZ9,
    tire.QBZ10 => tires.QBZ10,
    tire.QCZ1 => tires.QCZ1,
    tire.QDZ1 => tires.QDZ1,
    tire.QDZ2 => tires.QDZ2,
    tire.QDZ3 => tires.QDZ3,
    tire.QDZ4 => tires.QDZ4,
    tire.QDZ6 => tires.QDZ6,
    tire.QDZ7 => tires.QDZ7,
    tire.QDZ8 => tires.QDZ8,
    tire.QDZ9 => tires.QDZ9,
    tire.QDZ10 => tires.QDZ10,
    tire.QDZ11 => tires.QDZ11,
    tire.QEZ1 => tires.QEZ1,
    tire.QEZ2 => tires.QEZ2,
    tire.QEZ3 => tires.QEZ3,
    tire.QEZ4 => tires.QEZ4,
    tire.QEZ5 => tires.QEZ5,
    tire.QHZ1 => tires.QHZ1,
    tire.QHZ2 => tires.QHZ2,
    tire.QHZ3 => tires.QHZ3,
    tire.QHZ4 => tires.QHZ4,
    tire.PPZ1 => tires.PPZ1,
    tire.PPZ2 => tires.PPZ2,
    tire.SSZ1 => tires.SSZ1,
    tire.SSZ2 => tires.SSZ2,
    tire.SSZ3 => tires.SSZ3,
    tire.SSZ4 => tires.SSZ4,

    # [SCALING_COEFFICIENTS]
    tire.LFZO => tires.LFZO,
    tire.LCX => tires.LCX,
    tire.LMUX => tires.LMUX,
    tire.LEX => tires.LEX,
    tire.LKX => tires.LKX,
    tire.LHX => tires.LHX,
    tire.LVX => tires.LVX,
    tire.LCY => tires.LCY,
    tire.LMUY => tires.LMUY,
    tire.LEY => tires.LEY,
    tire.LKY => tires.LKY,
    tire.LHY => tires.LHY,
    tire.LVY => tires.LVY,
    tire.LKYC => tires.LKYC,
    tire.LXAL => tires.LXAL,
    tire.LYKA => tires.LYKA,
    tire.LVYKA => tires.LVYKA,
    tire.LTR => tires.LTR,
    tire.LRES => tires.LRES,
    tire.LKZC => tires.LKZC,
    tire.LS => tires.LS,
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
    corner.motor.gear_ratio => drivetrain.gear_ratio,
]

parameter_map = Dict([
    # torque control
    car.control.slip_target_front => ctrl.slip_target_front,
    car.control.slip_target_rear => ctrl.slip_target_rear,
    car.control.launch_torque => ctrl.launch_torque,
    car.control.motor_torque_max => ctrl.motor_torque_max,
    car.control.k => ctrl.slip_loop_gain,
    car.control.Ti => ctrl.slip_loop_Ti,
    car.control.Ni => ctrl.slip_loop_Ni,

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

    # Setup
    ssys.drive_start_time => t_drive,
])

prob = ODEProblem(ssys, parameter_map, (0.0, 6.0))
sol = solve(prob; tstops=[t_drive])

# Chase camera
cam_offset = GLMakie.Vec3f(-2.5, -0.35, 0.4)
framerate = 25
timevec = range(sol.t[1], sol.t[end], step=1 / framerate)

cg = [car.body.frame_a.r_0[i] + body.cg[i] for i in 1:3]

fig, tobs, scene = render(model, sol, sol.t[1];
    x=cam_offset[1], y=cam_offset[2], z=cam_offset[3],
    up=[0, 0, 1], slider=false, size=(800, 600))

GLMakie.record(fig, "output/full_vehicle_straight_line.gif", timevec; framerate) do time
    tobs[] = time
    target = GLMakie.Vec3f(sol(time, idxs=cg)...)
    GLMakie.update_cam!(scene.scene, GLMakie.cameracontrols(scene.scene),
        target + cam_offset, target, GLMakie.Vec3f(0, 0, 1))
end

corner_labels = ["FL" "FR" "RL" "RR"]
corner_styles = [:solid :dash :solid :dash]
corner_colors = [1 1 2 2]

eps_sigma = 0.001

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

# (7.25) gates both carcass deflections on one condition: the equivalent slip
# angle of (4.E78) leaving +/- alpha_sl = 3*Dy/CFalpha, AND |Vx| below Vlow.
# Leaving the band is necessary but not sufficient, so the speed gate is drawn
# alongside: the limiter can only bite where both hold.
p_lim = Plots.plot(sol;
    idxs=[corners[3].wheel_assembly.tire.alpha_r_eq,
        corners[3].wheel_assembly.tire.alpha_sl,
        -corners[3].wheel_assembly.tire.alpha_sl],
    labels=["RL alpha_r,eq" "+alpha_sl" "-alpha_sl"], color=[2 :red :red],
    linestyle=[:solid :dot :dot], linewidth=2,
    xlabel="t [s]", ylabel="equivalent slip angle [rad]",
    title="(7.25) limiter: arms only outside the band AND below Vlow")
Plots.hline!(p_lim, [0.0]; color=:black, linewidth=0.5, label="")
mark!(p_lim)

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

linkages = [car.front_suspension.linkage_left, car.front_suspension.linkage_right,
    car.rear_suspension.linkage_left, car.rear_suspension.linkage_right]

p_toe = Plots.plot(sol; idxs=[l.toe * DEG for l in linkages],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="toe [deg]", title="Toe — positive is toe-in");
mark!(p_toe)

p_camber = Plots.plot(sol; idxs=[l.camber * DEG for l in linkages],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="camber [deg]", title="Camber — positive leans outboard");
mark!(p_camber)

p_attitude = Plots.plot(sol;
    idxs=[ssys.roll_joint.phi * DEG, ssys.pitch_joint.phi * DEG],
    labels=["roll" "pitch"], linewidth=2,
    xlabel="t [s]", ylabel="angle [deg]", title="Body attitude — roll and pitch");
mark!(p_attitude)

Plots.plot(p_fx, p_v, p_slip, p_sig, p_lim, p_lam, p_svx, p_fz, p_tau,
    p_toe, p_camber, p_attitude;
    layout=(4, 3), size=(1800, 1450), legendfontsize=6,
    left_margin=5Plots.PlotMeasures.mm, bottom_margin=5Plots.PlotMeasures.mm)

# ---------------------------------------------------------------------------
# Lateral balance. All four tyres carry one unmirrored coefficient set, so ply
# steer, conicity and camber thrust push the same way on both sides rather than
# cancelling. Two independent measurements of that:
#   lateral_joint is free, so any net side force integrates into visible drift;
#   yaw is locked, so the net yaw moment appears as the torque the lock holds.
# If the sides genuinely cancelled, net Fy, the drift and the lock torque would
# all sit at zero.
# ---------------------------------------------------------------------------
# Fy_c and Mz_c, not Fy and Mz: the latter are in each tyre fit's own convention,
# which now reads mirrored between left and right. Only the contact-frame values
# are in a common frame and can meaningfully be summed.
fy = [sol(sol.t, idxs=c.wheel_assembly.tire.Fy_c).u for c in corners]
mz = [sol(sol.t, idxs=c.wheel_assembly.tire.Mz_c).u for c in corners]
fy_net = sum(fy)
mz_net = sum(mz)
y_drift = sol(sol.t, idxs=ssys.lateral_joint.s).u
yaw_hold = sol(sol.t, idxs=ssys.yaw_joint.tau).u

println("\n--- lateral balance (one unmirrored coefficient set on all four corners) ---")
for (lbl, f) in zip(corner_labels, fy)
    println("  $lbl  Fy final = ", lpad(round(f[end], digits=2), 9), " N")
end
println("  net Fy final       = ", round(fy_net[end], digits=2), " N  ",
    abs(fy_net[end]) < 1 ? "(cancels)" : "(DOES NOT CANCEL)")
println("  net Mz final       = ", round(mz_net[end], digits=3), " N.m")
println("  yaw lock reaction  = ", round(yaw_hold[end], digits=3), " N.m")
println("  lateral drift      = ", round(1000 * y_drift[end], digits=1), " mm  (peak ",
    round(1000 * maximum(abs, y_drift), digits=1), " mm)")
vehicle_mass = body.mass +
               front.geometry.linkages.left.upright.mass + front.geometry.linkages.right.upright.mass +
               rear.geometry.linkages.left.upright.mass + rear.geometry.linkages.right.upright.mass +
               4 * (wheels.rim_mass + tires.MASS)
println("  vehicle mass       = ", round(vehicle_mass, digits=1), " kg")
println("  lateral accel      = ", round(fy_net[end] / vehicle_mass, digits=3), " m/s^2")

p_fy = Plots.plot(sol; idxs=[c.wheel_assembly.tire.Fy_c for c in corners],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="Fy [N]", title="Fy per corner, contact frame — should be +/- paired L vs R");
mark!(p_fy)

p_net = Plots.plot(sol.t, fy_net; label="sum Fy", color=:red, linewidth=2,
    xlabel="t [s]", ylabel="net Fy [N]", title="Net side force (0 if the sides cancel)");
Plots.hline!(p_net, [0.0]; color=:black, linewidth=0.5, label="");
mark!(p_net)

p_drift = Plots.plot(sol.t, 1000 * y_drift; label="lateral drift", color=:black, linewidth=2,
    xlabel="t [s]", ylabel="lateral drift [mm]",
    title="Drift — free y joint, so this is the net side force integrated");
mark!(p_drift)

p_mz = Plots.plot(sol; idxs=[c.wheel_assembly.tire.Mz_c for c in corners],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="Mz [N.m]", title="Aligning moment per corner");
mark!(p_mz)

p_yaw = Plots.plot(sol.t, yaw_hold; label="yaw lock reaction", color=:purple, linewidth=2,
    xlabel="t [s]", ylabel="tau [N.m]", title="Torque the yaw lock absorbs (0 if balanced)");
mark!(p_yaw)

Plots.plot(p_fy, p_net, p_drift, p_mz, p_yaw; layout=(2, 3), size=(1800, 900), legendfontsize=7,
    left_margin=5Plots.PlotMeasures.mm, bottom_margin=5Plots.PlotMeasures.mm)

travel(t) = sol(t, idxs=ssys.longitudinal_joint.s)
s_launch = travel(t_drive)

function time_to(distance)
    f(t) = travel(t) - s_launch - distance
    f(sol.t[end]) < 0 && return NaN
    lo, hi = t_drive, sol.t[end]
    for _ in 1:200
        mid = (lo + hi) / 2
        f(mid) < 0 ? (lo = mid) : (hi = mid)
    end
    (lo + hi) / 2
end

println("\n--- split times from launch ---")
for d in (25.0, 50.0, 75.0)
    t = time_to(d)
    if isnan(t)
        println("  ", lpad(Int(d), 2), " m: not reached in ",
            round(sol.t[end] - t_drive, digits=2), " s")
    else
        v = sol(t, idxs=ssys.longitudinal_joint.v)
        println("  ", lpad(Int(d), 2), " m: ", round(t - t_drive, digits=3), " s   passing at ",
            round(v, digits=2), " m/s (", round(v * 3.6, digits=1), " km/h)")
    end
end
println("  total travel = ", round(travel(sol.t[end]) - s_launch, digits=1), " m in ",
    round(sol.t[end] - t_drive, digits=2), " s")
