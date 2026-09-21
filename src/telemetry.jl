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
using ..Data: VEHICLE

const G = 9.80665
const DEG = 180 / pi
const KMH = 3.6
const MM = 1000
const METERS_TO_DEGREES = 9e-6
const GPS_LATITUDE_ORIGIN = 1.0
const GPS_LONGITUDE_ORIGIN = 1.0
const GPS_RATE = 20

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

    specs = Tuple{String,Any,Float64,String}[]
    add(name, signal, scale, unit) = push!(specs, (name, signal, scale, unit))

    # Body
    add("acc_x", () -> car.a_cg[1], 1 / G, "G")
    add("acc_y", () -> car.a_cg[2], 1 / G, "G")
    add("acc_z", () -> car.a_cg[3], 1 / G, "G")
    add("a_yaw", () -> model.yaw_joint.phi, DEG, "deg")
    add("a_pitch", () -> model.pitch_joint.phi, DEG, "deg")
    add("a_roll", () -> model.roll_joint.phi, DEG, "deg")
    add("a_yaw_rate", () -> model.yaw_joint.w, DEG, "deg/s")
    add("a_pitch_rate", () -> model.pitch_joint.w, DEG, "deg/s")
    add("a_roll_rate", () -> model.roll_joint.w, DEG, "deg/s")
    add("v_vx", () -> car.v_cg[1], KMH, "km/h")
    add("v_vy", () -> car.v_cg[2], KMH, "km/h")
    add("a_beta", () -> atan.(sample(car.v_cg[2]), sample(car.v_cg[1])), DEG, "deg")

    # Driver inputs
    add("r_throttle", () -> car.throttle, 100, "%")
    add("r_brake", () -> car.brake, 100, "%")
    add("a_steer", () -> car.steer, DEG, "deg")

    # Brake control
    add("r_brake_abs", () -> car.control.brake_demand, 100, "%")
    add("r_abs_intervention", () -> car.control.abs.intervention, 100, "%")

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
        add("tq_motor_$tag", () -> corner.motor.tau_motor, 1, "N.m")
        add("pwr_motor_$tag", () -> corner.motor.power, 1 / 1000, "kW")
        add("tq_drive_wheel_$tag", () -> corner.motor.tau_wheel, 1, "N.m")
        add("tq_brake_wheel_$tag", () -> corner.brake.tau_brake, 1, "N.m")

        # Joint loads
        for (axis, i) in (("x", 1), ("y", 2), ("z", 3))
            add("f_lca_$(axis)_$tag", () -> linkage.f_lca[i], 1, "N")
            add("f_uca_ball_$(axis)_$tag", () -> linkage.f_uca_ball[i], 1, "N")
            add("f_uca_mount_$(axis)_$tag", () -> linkage.f_uca[i], 1, "N")
            add("f_pushrod_$(axis)_$tag", () -> linkage.f_push[i], 1, "N")
        end
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

    signals = Any[(try spec[2]() catch; nothing end) for spec in specs]
    direct = [i for (i, s) in enumerate(signals) if !(s === nothing || s isa AbstractVector{<:Real})]
    series = Dict{Int,Vector{Float64}}()

    if !isempty(direct)
        try
            batch = sol(times, idxs=[signals[i] for i in direct])
            for (k, i) in enumerate(direct)
                series[i] = Float64[u[k] for u in batch.u]
            end
        catch
            for i in direct
                try
                    series[i] = sample(signals[i])
                catch
                end
            end
        end
    end

    for (i, (name, _, scale, unit)) in enumerate(specs)
        values = signals[i] isa AbstractVector{<:Real} ? signals[i] : get(series, i, nothing)
        if values === nothing
            push!(skipped, name)
        else
            push!(channels, Motec.Channel(name, values .* scale; freq=rate, unit))
        end
    end

    isempty(skipped) ||
        @warn "telemetry: $(length(skipped)) channels are not in the solution" skipped

    # GPS coordinates in degrees, with a nonzero "pretend" origin.
    body = sol.prob.f.sys.vehicle.body
    function cg_world_component(i, sample_times)
        sample_frame(signal) = Float64[sol(time, idxs=signal) for time in sample_times]
        position = sample_frame(body.frame_a.r_0[i])
        for j in 1:3
            position .+= sample_frame(body.frame_a.R[i, j]) .* VEHICLE.body.cg[j]
        end
        return position
    end

    gps_times = range(0.0, sol.t[end], step=1 / GPS_RATE)
    gps_x = cg_world_component(1, gps_times)
    gps_y = cg_world_component(2, gps_times)
    gps_step = hypot.(diff(gps_x), diff(gps_y))
    gps_speed = vcat(0.0, gps_step .* GPS_RATE) .* KMH
    lap_distance = vcat(0.0, cumsum(gps_step))
    gps_latitude = GPS_LATITUDE_ORIGIN .+ gps_x .* METERS_TO_DEGREES
    gps_longitude = GPS_LONGITUDE_ORIGIN .+ gps_y .* (METERS_TO_DEGREES / cosd(GPS_LATITUDE_ORIGIN))
    for (name, values, unit) in (
        ("GPS Latitude", gps_latitude, "deg"),
        ("GPS Longitude", gps_longitude, "deg"),
        ("GPS Speed", gps_speed, "km/h"),
        ("Lap Distance", lap_distance, "m"),
    )
        push!(channels, Motec.Channel(name, values; freq=GPS_RATE, unit=unit))
    end

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
