"""
    VehicleComponents.Telemetry

Maps a `FullVehicle` solution onto MoTeC i2 channels.

Naming follows a MoTeC export of Bosch channels: lower case, a quantity
prefix, then a position suffix, `_fl _fr _rl _rr` for a corner and `_f _r` for an
axle. `acc_` acceleration, `a_` angle, `f_` force, `h_` height, `n_` rotational
speed, `p_` pressure, `r_` ratio, `s_` displacement, `tq_` torque, `v_` velocity.
"""
module Telemetry

using ..Motec

const G = 9.80665
const DEG = 180 / pi
const KMH = 3.6
const MM = 1000

corners(car) = (
    ("fl", car.corner_fl, car.front_suspension.linkage_left),
    ("fr", car.corner_fr, car.front_suspension.linkage_right),
    ("rl", car.corner_rl, car.rear_suspension.linkage_left),
    ("rr", car.corner_rr, car.rear_suspension.linkage_right),
)

axles(car) = (("f", car.front_suspension), ("r", car.rear_suspension))

function vehicle_channels(model, sol; rate::Int=100)
    car = model.vehicle
    times = range(0.0, sol.t[end], step=1 / rate)
    channels = Motec.Channel[]
    skipped = String[]

    sample(signal) = sol(times, idxs=signal).u

    function add(name, signal, scale, unit)
        try
            values = signal()
            values isa AbstractVector{<:Real} || (values = sample(values))
            push!(channels, Motec.Channel(name, values .* scale; freq=rate, unit))
        catch
            push!(skipped, name)
        end
    end

    # Body
    add("acc_x", () -> car.a_cg[1], 1 / G, "G")
    add("acc_y", () -> car.a_cg[2], 1 / G, "G")
    add("acc_z", () -> car.a_cg[3], 1 / G, "G")
    add("a_yaw", () -> car.w_cg[1], DEG, "deg")
    add("a_pitch", () -> car.w_cg[2], DEG, "deg")
    add("a_roll", () -> car.w_cg[3], DEG, "deg")
    add("v_ground_speed", () -> car.aero.v, KMH, "km/h")

    # Driver inputs
    add("ath", () -> car.throttle, 100, "%")
    add("a_steer", () -> car.steer, DEG, "deg")

    # Aero
    add("f_aero_drag", () -> car.aero.drag, 1, "N")
    add("f_aero_downforce", () -> car.aero.downforce, 1, "N")
    add("f_aero_downforce_f", () -> car.aero.downforce_front, 1, "N")
    add("f_aero_downforce_r", () -> car.aero.downforce_rear, 1, "N")
    add("p_dynamic", () -> car.aero.q, 1 / 100, "mbar")

    # Corners
    for (tag, corner, linkage) in corners(car)
        tire = corner.wheel_assembly.tire
        add("v_wheel_$tag", () -> sample(tire.omega) .* sample(tire.Re), KMH, "km/h")
        add("n_wheel_$tag", () -> tire.omega, 60 / 2pi, "rpm")
        add("f_tire_x_$tag", () -> tire.Fx, 1, "N")
        add("f_tire_y_$tag", () -> tire.Fy, 1, "N")
        add("f_tire_z_$tag", () -> tire.Fz, 1, "N")
        add("r_tire_long_slip_$tag", () -> tire.kappa, 1, "ratio")
        add("a_tire_lateral_slip_$tag", () -> tire.alpha, DEG, "deg")
        add("a_tire_inclination_$tag", () -> linkage.camber, DEG, "deg")
        add("a_tire_toe_$tag", () -> linkage.toe, DEG, "deg")
        add("s_wheel_$tag", () -> linkage.wc_height, MM, "mm")
        add("tq_wheel_$tag", () -> corner.motor.tau_wheel, 1, "N.m")
    end

    # Axles
    for (tag, suspension) in axles(car)
        inboard = suspension.inboard
        add("s_heave_$tag", () -> suspension.wc_heave, MM, "mm")
        add("s_roll_$tag", () -> suspension.wc_roll, MM, "mm")
        add("s_damper_heave_$tag", () -> inboard.heave_strut.s, MM, "mm")
        add("s_damper_roll_$tag", () -> inboard.roll_strut.s, MM, "mm")
        add("f_damper_heave_$tag", () -> inboard.heave_strut.f, 1, "N")
        add("f_damper_roll_$tag", () -> inboard.roll_strut.f, 1, "N")
    end

    isempty(skipped) ||
        @warn "telemetry: $(length(skipped)) channels are not in the solution" skipped
    return channels
end

function write_telemetry(model, sol; run::AbstractString, dir="output/telemetry",
    rate::Int=100, driver="Sim", vehicle_id="",
    vehicle_desc="Full vehicle", venue="", event="", session="S1",
    comment="")
    stamp = Base.time()
    parts = filter(!isempty, [run, String(vehicle_id), Base.Libc.strftime("%Y%m%d_%H%M%S", stamp)])
    meta = Motec.LogMeta(; driver, vehicle_id, vehicle_desc, venue, event, session, comment,
        date=Base.Libc.strftime("%d/%m/%Y", stamp),
        time=Base.Libc.strftime("%H:%M:%S", stamp))
    return Motec.write_ld(joinpath(dir, join(parts, "_") * ".ld"),
        vehicle_channels(model, sol; rate), meta)
end

end
