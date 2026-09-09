# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

# ===========================================================================
# Setup
# ===========================================================================
const DEG = 180 / π
const G = 9.80665

@named model = VehicleComponents.FullVehicleTestStep()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "MR25.json")
data = JSON3.read(read(data_path, String))

car = ssys.vehicle
body = data.body
tires = data.tires
wheels = data.wheels
drivetrain = data.drivetrain
ctrl = data.control
aero = data.aero
front, rear = data.suspension.front, data.suspension.rear

t_drive = 1.5
t_steer = 3.0
steer_angle = 0.15
t_end = 6.0

corners = [car.corner_fl, car.corner_fr, car.corner_rl, car.corner_rr]
linkages = [car.front_suspension.linkage_left, car.front_suspension.linkage_right,
    car.rear_suspension.linkage_left, car.rear_suspension.linkage_right]
inboards = [car.front_suspension.inboard, car.rear_suspension.inboard]

# TireMF61 takes the data as describing a LEFT-side tyre and mirrors the right.
# Which side each corner is on is structural and set in FullVehicle, so it
# cannot be overridden here.
@assert uppercase(String(tires.TYRESIDE)) == "LEFT" "tires.TYRESIDE is $(tires.TYRESIDE), but TireMF61 assumes a LEFT-side fit; mirror the data first"

# AeroLoad places the centre of pressure at (1 - bal_f) * wheelbase aft of the
# front axle, and nothing in the model ties that wheelbase to the suspension
# geometry. Derive it from the same wheel centres the linkages are built from so
# the two cannot drift apart.
@assert front.geometry.linkages.left.wheel_center[1] == front.geometry.linkages.right.wheel_center[1] "front wheel centres disagree on x"
@assert rear.geometry.linkages.left.wheel_center[1] == rear.geometry.linkages.right.wheel_center[1] "rear wheel centres disagree on x"
wheelbase = front.geometry.linkages.left.wheel_center[1] -
            rear.geometry.linkages.left.wheel_center[1]

# ===========================================================================
# Parameters
# ===========================================================================
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

linkage_parameters(linkage, geom, align) = [
    linkage.lca_front => geom.lca_front,
    linkage.lca_rear => geom.lca_rear,
    linkage.lca_outer => geom.lca_outer,
    linkage.uca_front => geom.uca_front,
    linkage.uca_rear => geom.uca_rear,
    linkage.uca_outer => geom.uca_outer,
    linkage.tierod_inner => geom.tierod_inner,
    linkage.tierod_outer => geom.tierod_outer,
    linkage.static_camber => align.static_camber,
    linkage.static_toe => align.static_toe,
]

inboard_parameters(inboard, geom, setup) = [
    inboard.rocker_pivot_left => geom.rocker_pivot.left,
    inboard.rocker_pivot_right => geom.rocker_pivot.right,
    inboard.pushrod_inner_left => geom.pushrod_inner.left,
    inboard.pushrod_inner_right => geom.pushrod_inner.right,
    inboard.heave_pickup_left => geom.heave_pickup.left,
    inboard.heave_pickup_right => geom.heave_pickup.right,
    inboard.roll_pickup_left => geom.roll_pickup.left,
    inboard.roll_pickup_right => geom.roll_pickup.right,
    inboard.heave_stiffness => setup.heave.stiffness,
    inboard.heave_damping => setup.heave.damping,
    inboard.heave_preload => setup.heave.preload,
    inboard.roll_stiffness => setup.roll.stiffness,
    inboard.roll_damping => setup.roll.damping,
    inboard.roll_preload => setup.roll.preload,
    inboard.pushrod_adjust_left => setup.pushrod_adjust.left,
    inboard.pushrod_adjust_right => setup.pushrod_adjust.right,
]

suspension_parameters(suspension, ax) = [
    suspension.wheel_center_left => ax.geometry.linkages.left.wheel_center,
    suspension.wheel_center_right => ax.geometry.linkages.right.wheel_center,
    suspension.pushrod_outer_left => ax.geometry.linkages.left.pushrod_outer,
    suspension.pushrod_outer_right => ax.geometry.linkages.right.pushrod_outer,
    suspension.tierod_inner_left => ax.geometry.linkages.left.tierod_inner,
    suspension.tierod_inner_right => ax.geometry.linkages.right.tierod_inner,
    linkage_parameters(suspension.linkage_left,
        ax.geometry.linkages.left, ax.setup.alignment.left)...,
    linkage_parameters(suspension.linkage_right,
        ax.geometry.linkages.right, ax.setup.alignment.right)...,
    inboard_parameters(suspension.inboard, ax.geometry.inboard, ax.setup)...,
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

    # aero
    car.aero.rho => aero.rho,
    car.aero.CdA => aero.CdA,
    car.aero.ClA => aero.ClA,
    car.aero.bal_f => aero.balance_front,
    car.aero.wheelbase => wheelbase,

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

    # suspension: geometry and setup
    suspension_parameters(car.front_suspension, front)...,
    suspension_parameters(car.rear_suspension, rear)...,

    # Setup
    ssys.drive_start_time => t_drive,
    ssys.steer_start_time => t_steer,
    ssys.steer_angle => steer_angle,
])

# ===========================================================================
# Solve
# ===========================================================================
prob = ODEProblem(ssys, parameter_map, (0.0, t_end))
sol = solve(prob; tstops=[t_drive, t_steer])

# ===========================================================================
# Report
# ===========================================================================
corner_labels = ["FL" "FR" "RL" "RR"]
corner_styles = [:solid :dash :solid :dash]
corner_colors = [1 1 2 2]

eps_sigma = 0.001

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

println("\n--- relaxation length over the whole run ---")
for (lbl, c) in zip(corner_labels, corners)
    sk = sol(sol.t, idxs=c.wheel_assembly.tire.sigma_kappa).u
    println("  $lbl  sigma_kappa: ", round(1000 * minimum(sk), sigdigits=3), " .. ",
        round(1000 * maximum(sk), sigdigits=3), " mm   (floor = ", 1000 * eps_sigma, " mm)")
end
println("  final speed = ", round(sol(sol.t[end], idxs=ssys.longitudinal_joint.v), sigdigits=4), " m/s")

println("\n--- aero ---")
println("  wheelbase = ", round(1000 * wheelbase, digits=1), " mm, balance = ",
    round(100 * aero.balance_front, digits=1), "% front")
println("  x_cop     = ", round(1000 * (1 - aero.balance_front) * wheelbase, digits=1),
    " mm aft of the front axle, z = 0")
let tend = sol.t[end]
    println("  at t = ", round(tend, digits=3), " s, v = ",
        round(sol(tend, idxs=car.aero.v), digits=2), " m/s")
    println("    drag      = ", round(sol(tend, idxs=car.aero.drag), digits=1), " N")
    println("    downforce = ", round(sol(tend, idxs=car.aero.downforce), digits=1), " N  (",
        round(sol(tend, idxs=car.aero.downforce_front), digits=1), " front / ",
        round(sol(tend, idxs=car.aero.downforce_rear), digits=1), " rear)")
end

# The rocker has a hard kinematic ceiling: the heave arm swings on a circle
# about the pivot, and the spring stops shortening once that arm goes normal to
# the spring axis. There the motion ratio is zero and the loop closure is
# singular, so the margin to it is what matters, not the deflection alone.
heave_dead_point(ax) =
    let piv = ax.geometry.inboard.rocker_pivot.left, hp = ax.geometry.inboard.heave_pickup.left

        2 * (hp[2] - (piv[2] - hypot(hp[2] - piv[2], hp[3] - piv[3])))
    end

println("\n--- heave travel ---")
for (lbl, ib, ax) in (("front", car.front_suspension.inboard, front),
    ("rear", car.rear_suspension.inboard, rear))
    defl = sol(sol.t, idxs=ib.heave_deflection).u
    dp = heave_dead_point(ax)
    peak = maximum(defl)
    println("  $lbl  peak compression = ", round(1000 * peak, digits=2), " mm",
        "   (dead point ", round(1000 * dp, digits=1), " mm)")
    println("         ", round(100 * peak / dp, digits=1), "% of the way to the dead point",
        peak / dp > 0.8 ? "   *** TOO CLOSE ***" : "")
end

# Lateral balance. TireMF61 mirrors the fit per corner (`is_left` sets `side`,
# which flips gamma_star and V_y_mf going in and undoes the flip on Fy_c and
# Mz_c coming out), so ply steer, conicity and camber thrust should cancel left
# to right. Two independent measurements of whether they do: lateral_joint is
# free, so any net side force integrates into visible drift; yaw is locked, so
# the net yaw moment appears as the torque the lock holds.
#
# Fy_c and Mz_c, not Fy and Mz: the latter are in each tyre fit's own
# convention, which reads mirrored between left and right. Only the
# contact-frame values are in a common frame and can meaningfully be summed.
fy = [sol(sol.t, idxs=c.wheel_assembly.tire.Fy_c).u for c in corners]
mz = [sol(sol.t, idxs=c.wheel_assembly.tire.Mz_c).u for c in corners]
fy_net = sum(fy)
mz_net = sum(mz)
y_drift = sol(sol.t, idxs=ssys.lateral_joint.s).u
yaw_rate = sol(sol.t, idxs=ssys.yaw_joint.w).u
vehicle_mass = body.mass +
               front.geometry.linkages.left.upright.mass + front.geometry.linkages.right.upright.mass +
               rear.geometry.linkages.left.upright.mass + rear.geometry.linkages.right.upright.mass +
               4 * (wheels.rim_mass + tires.MASS)

println("\n--- lateral balance (mirrored per corner, so these should cancel) ---")
for (lbl, f) in zip(corner_labels, fy)
    println("  $lbl  Fy final = ", lpad(round(f[end], digits=2), 9), " N")
end
# Judge the residual against the per-corner magnitude, not an absolute newton.
# A fixed 1 N threshold calls a 1.5% residual on 84 N corners a failure.
fy_corner = maximum(abs, [f[end] for f in fy])
println("  net Fy final       = ", round(fy_net[end], digits=2), " N  (",
    round(100 * abs(fy_net[end]) / max(fy_corner, eps()), digits=2), "% of the per-corner Fy, ",
    abs(fy_net[end]) < 0.02 * fy_corner ? "cancels" : "DOES NOT CANCEL", ")")
println("  net Mz final       = ", round(mz_net[end], digits=3), " N.m")
println("  peak yaw rate      = ", round(DEG * maximum(abs, yaw_rate), digits=1), " deg/s")
println("  lateral drift      = ", round(1000 * y_drift[end], digits=1), " mm  (peak ",
    round(1000 * maximum(abs, y_drift), digits=1), " mm)")
println("  vehicle mass       = ", round(vehicle_mass, digits=1), " kg")
println("  lateral accel      = ", round(fy_net[end] / vehicle_mass, digits=3), " m/s^2")

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
    round(sol.t[end] - t_drive, digits=2), " s\n")

# ===========================================================================
# Plots
# ===========================================================================
mark!(p) = Plots.vline!(p, [t_drive]; color=:black, linestyle=:dot, linewidth=1, label="")

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

p_sig = Plots.plot(sol; idxs=[1000 * c.wheel_assembly.tire.sigma_kappa for c in corners],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="sigma_kappa [mm]", title="Relaxation length (7.8)")
Plots.hline!(p_sig, [1000 * eps_sigma]; color=:red, linestyle=:dot, label="eps_sigma floor")

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

p_toe = Plots.plot(sol; idxs=[l.toe * DEG for l in linkages],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="toe [deg]", title="Toe — positive is toe-in");
mark!(p_toe)

p_camber = Plots.plot(sol; idxs=[l.camber * DEG for l in linkages],
    labels=corner_labels, linestyle=corner_styles, color=corner_colors, linewidth=2,
    xlabel="t [s]", ylabel="camber [deg]", title="Camber — positive leans outboard");
mark!(p_camber)

p_attitude = Plots.plot(sol;
    idxs=[car.w_cg[3] * DEG, car.w_cg[2] * DEG],
    labels=["roll" "pitch"], linewidth=2,
    xlabel="t [s]", ylabel="angle [deg]", title="Body attitude vs world");
mark!(p_attitude)

Plots.plot(p_fx, p_v, p_slip, p_sig, p_lim, p_lam, p_svx, p_fz, p_tau,
    p_toe, p_camber, p_attitude;
    layout=(4, 3), size=(1800, 1450), legendfontsize=6,
    left_margin=5Plots.PlotMeasures.mm, bottom_margin=5Plots.PlotMeasures.mm)

p_aero = Plots.plot(sol; idxs=[car.aero.drag, car.aero.downforce,
        car.aero.downforce_front, car.aero.downforce_rear],
    labels=["drag" "downforce" "DF front" "DF rear"], linewidth=2,
    xlabel="t [s]", ylabel="force [N]", title="Aero");
mark!(p_aero)

p_heave = Plots.plot(sol; idxs=[1000 * ib.heave_deflection for ib in inboards],
    labels=["front" "rear"], linewidth=2,
    xlabel="t [s]", ylabel="compression [mm]", title="Heave spring compression");
Plots.hline!(p_heave, [1000 * heave_dead_point(front), 1000 * heave_dead_point(rear)];
    color=[:red :red], linestyle=:dot, labels=["front dead point" "rear dead point"]);
mark!(p_heave)

p_roll_spring = Plots.plot(sol; idxs=[1000 * ib.roll_deflection for ib in inboards],
    labels=["front" "rear"], linewidth=2,
    xlabel="t [s]", ylabel="deflection [mm]", title="Roll spring deflection");
mark!(p_roll_spring)

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

p_yaw = Plots.plot(sol.t, DEG * yaw_rate; label="yaw rate", color=:purple, linewidth=2,
    xlabel="t [s]", ylabel="r [deg/s]", title="Yaw rate");
mark!(p_yaw)

a_long = sol(sol.t, idxs=car.a_cg[1]).u
a_lat = sol(sol.t, idxs=car.a_cg[2]).u

println("\n--- acceleration ---\n")
println("  peak longitudinal  = ", round(maximum(abs, a_long) / G, digits=2), " g")
println("  peak lateral       = ", round(maximum(abs, a_lat) / G, digits=2), " g")

p_accel = Plots.plot(sol.t, [a_long / G a_lat / G];
    labels=["longitudinal" "lateral"], color=[:steelblue :orangered], linewidth=2,
    xlabel="t [s]", ylabel="a [g]", title="Body acceleration");
Plots.vline!(p_accel, [t_steer]; color=:black, linestyle=:dash, linewidth=1, label="");
mark!(p_accel)

Plots.plot(p_aero, p_heave, p_roll_spring, p_fy, p_net, p_drift, p_mz, p_yaw, p_accel;
    layout=(3, 3), size=(1800, 1350), legendfontsize=7,
    left_margin=5Plots.PlotMeasures.mm, bottom_margin=5Plots.PlotMeasures.mm)

# ===========================================================================
# Animation
# ===========================================================================
cam_offset = GLMakie.Vec3f(-2.5, -0.35, 0.4)
framerate = 25
timevec = range(sol.t[1], sol.t[end], step=1 / framerate)

cg = [car.body.frame_a.r_0[i] + body.cg[i] for i in 1:3]

fig, tobs, scene = render(model, sol, sol.t[1];
    x=cam_offset[1], y=cam_offset[2], z=cam_offset[3],
    up=[0, 0, 1], slider=false, size=(800, 600))

GLMakie.record(fig, "output/full_vehicle_step.gif", timevec; framerate) do time
    tobs[] = time
    target = GLMakie.Vec3f(sol(time, idxs=cg)...)
    GLMakie.update_cam!(scene.scene, GLMakie.cameracontrols(scene.scene),
        target + cam_offset, target, GLMakie.Vec3f(0, 0, 1))
end
