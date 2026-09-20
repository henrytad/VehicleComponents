using DyadData: DyadTimeseries

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

Curve for one damper. `dampers` is the valve code catalog (`data.dampers`) and
`selection` is a setup entry such as `setup.heave.damper`, naming a `valve_code`
and the `compression_position` and `extension_position` clicks. The two adjusters
are independent, so the compression side comes from one position's column and the
extension side from another's.
"""
function damper_map(dampers, selection)
    table = dampers[selection.valve_code]
    compression = table.positions[selection.compression_position].compression
    extension = table.positions[selection.extension_position].extension
    return signed_damper_curve(table.velocity, compression, extension)
end

"""
    damper_dataset(selection; dampers = params.dampers)

The map a `Damper` interpolates, for the valve code and clicks named by a setup
entry such as `setup.heave.damper`. This is what a strut's `dataset` takes, so a
damper carries its own curve instead of being patched at run time.
"""
function damper_dataset(selection; dampers = params.dampers)
    curve = damper_map(dampers, selection)
    return DyadTimeseries(hcat(curve.velocity, curve.force);
        independent_var = "velocity", dependent_vars = ["force"])
end
