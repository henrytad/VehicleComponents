using DyadData: DyadTimeseries, build_table

"""
Combine independently selected compression and extension CSV assets.

Files use speed in m/s and the published force signs in N: compression positive,
extension negative. The model uses negative velocity/force for compression and
positive velocity/force for extension, with a single zero point.
"""
function damper_dataset(compression_file::AbstractString, extension_file::AbstractString)
    function read_curve(file, force_sign)
        curve = build_table(DyadTimeseries("dyad://VehicleComponents/$file";
            independent_var="velocity", dependent_vars=["force"]))
        velocity, force = curve.velocity, curve.force
        if length(velocity) < 2 || !all(isfinite, velocity) ||
           first(velocity) != 0 || !all(>(0), diff(velocity))
            throw(ArgumentError("$file: velocity must start at zero and strictly increase"))
        end
        if !all(isfinite, force) || first(force) != 0 || !all(>=(0), force_sign .* force)
            throw(ArgumentError("$file: force must start at zero, with compression positive and extension negative"))
        end
        return curve
    end

    compression = read_curve(compression_file, 1)
    extension = read_curve(extension_file, -1)
    velocity = vcat(-reverse(compression.velocity[2:end]), extension.velocity)
    force = vcat(-reverse(compression.force[2:end]), -extension.force)
    return DyadTimeseries(hcat(velocity, force);
        independent_var="velocity", dependent_vars=["force"])
end
