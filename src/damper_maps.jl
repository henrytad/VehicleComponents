using DyadData: DyadTimeseries

"""
Velocity breakpoints of the Multimatic DSSV valve code tables, in m/s: the
published 0, 10, 25, 50, 100 ... 500 mm/s.
"""
const DAMPER_VELOCITIES = [0.0, 0.01, 0.025, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5]

"""
    signed_damper_curve(velocity, compression, extension)

Build the single curve `Damper` interpolates from a table in the form damper data
is published: peak force against peak velocity from zero, with compression
forces positive and extension forces negative.

`Damper` looks the curve up at strut velocity `der(s)`, which is negative while
the strut shortens, and its force is tension-positive. A compressing damper
pushes the frames apart, so compression lands at `(-v, -F_compression)`. An
extending damper pulls them together, so extension lands at `(+v, -F_extension)`,
which is positive because the published extension force is negative. The zero
point is shared, giving `2n - 1` points in ascending velocity.
"""
function signed_damper_curve(velocity, compression, extension)
    v, fc, fe = Float64.(velocity), Float64.(compression), Float64.(extension)
    length(v) == length(fc) == length(fe) ||
        throw(ArgumentError("velocity, compression and extension must be the same length"))
    v[1] == 0 && fc[1] == 0 && fe[1] == 0 ||
        throw(ArgumentError("damper tables must start at zero velocity and zero force"))
    issorted(v; lt = <=) ||
        throw(ArgumentError("velocity breakpoints must be strictly increasing"))
    all(>=(0), fc) || throw(ArgumentError("compression forces must be positive, as published"))
    all(<=(0), fe) || throw(ArgumentError("extension forces must be negative, as published"))
    return (velocity = vcat(-reverse(v[2:end]), v), force = vcat(-reverse(fc[2:end]), -fe))
end

"""
    damper_map(dampers, selection)

Curve for one damper from the vehicle JSON. `dampers` is the valve code catalog
(`data.dampers`) and `selection` is a setup entry such as `setup.heave.damper`,
naming a `valve_code` and the `compression_position` and `extension_position`
clicks. The two adjusters are independent, so the compression side comes from
one position's column and the extension side from another's.
"""
function damper_map(dampers, selection)
    table = dampers[Symbol(selection.valve_code)]
    compression = table.positions[Symbol(selection.compression_position)].compression
    extension = table.positions[Symbol(selection.extension_position)].extension
    return signed_damper_curve(table.velocity, compression, extension)
end

"""
    linear_damper_map(d; velocity = DAMPER_VELOCITIES)

A linear damper, `f = d * der(s)`, on the valve code breakpoints so it fits
`Damper`'s map. Linear extrapolation keeps it exact beyond them. For setups that
give a single damping coefficient rather than a valve code.
"""
linear_damper_map(d; velocity = DAMPER_VELOCITIES) =
    signed_damper_curve(velocity, d .* velocity, -d .* velocity)

"""
    default_damper_dataset()

The map `Damper` is built with: linear 1500 N·s/m. Its length fixes how many
points every run-time map must have, `2 * 13 - 1 = 25`.
"""
function default_damper_dataset()
    curve = linear_damper_map(1500.0)
    return DyadTimeseries(hcat(curve.velocity, curve.force);
        independent_var = "velocity", dependent_vars = ["force"])
end
