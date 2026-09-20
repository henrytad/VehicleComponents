"""
    VehicleComponents.Motec

Writes a MoTeC i2 `.ld` log so a solution can be opened in MoTeC i2 Pro.

The format is not encrypted, just an undocumented little-endian binary
container. The layout here follows the community `ldparser` description and a
known-good i2 export; every offset is fixed:

    0x0000  header          1762 bytes, holds the three block pointers
    0x06E2  event block     3156 bytes, -> venue pointer at 0x0B62
    0x1336  venue block     3102 bytes, -> vehicle pointer at 0x1780
    0x1F54  vehicle block   3316 bytes
    0x2C48  channel table   124 bytes per channel, a doubly linked list
    ....    channel data    one block per channel, in table order

Each channel carries its own sample count and rate, so channels of different
frequencies can share a file. i2 decodes a sample as
`value = raw * scale / 10^dec_places / multiplier + shift`; this writer stores
Float32 and leaves that transform at identity.

Channels must be sampled on a uniform time grid: the file stores a rate, not
timestamps, so an adaptive solver's own steps cannot be written directly.
"""
module Motec

const HEADER_SIZE = 0x06E2
const EVENT_PTR = 0x06E2
const EVENT_SIZE = 0x1336 - 0x06E2
const VENUE_PTR = 0x1336
const VENUE_SIZE = 0x1F54 - 0x1336
const VEHICLE_PTR = 0x1F54
const VEHICLE_SIZE = 0x2C48 - 0x1F54
const CHANNEL_TABLE_PTR = 0x2C48
const CHANNEL_SIZE = 124

# Pointer to the next block, stored inside the event and venue blocks
const EVENT_VENUE_PTR_OFF = 0x0B62 - EVENT_PTR
const VENUE_VEHICLE_PTR_OFF = 0x1780 - VENUE_PTR

# Copied from a working i2 export. The meaning is unknown; i2 wants them present
const MAGIC_SOF = 0x00000040
const MAGIC_HEADER = (0x0002, 0x4240, 0x000F)
const MAGIC_UNKNOWN = 0x0080
const MAGIC_CHANNEL = 0x0303

const DTYPE_FLOAT = 0x0007
const DTYPE_SIZE = 0x0004

"""
One logged channel, sampled at a fixed rate. `unit` is the text i2 shows on the
axis, and must be one i2 knows: a MoTeC export of 539 channels uses only
`%  C  G  L  MJ  N  N.m  bar  cc  deg  deg/s  kJ  kW  kg/m3  km/h  l/h  m  m/s
m/s/s  mbar  mm  mm/s  rad/s  ratio  rpm  s`.
"""
Base.@kwdef struct Channel
    name::String
    data::Vector{Float32}
    freq::Int = 100
    unit::String = ""
end

Channel(name, data; freq=100, unit="") =
    Channel(; name, data=Float32.(collect(data)), freq, unit)

"""
What i2 shows in its outing and details panes. `date` and `time` are written as
text, so they carry the format the file stores them in and default to now.
"""
Base.@kwdef struct LogMeta
    driver::String = ""
    vehicle_id::String = ""
    vehicle_desc::String = ""
    venue::String = ""
    event::String = ""
    session::String = ""
    comment::String = ""
    date::String = Base.Libc.strftime("%d/%m/%Y", Base.time())
    time::String = Base.Libc.strftime("%H:%M:%S", Base.time())
end

# Header string fields, as (offset, width, field of LogMeta)
const HEADER_STRINGS = (
    (0x09E, 64, :driver),
    (0x0DE, 64, :vehicle_id),
    (0x15E, 64, :venue),
    (0x5E4, 64, :session),
    (0x624, 64, :comment),
)

"NUL-padded fixed-width field, truncated to fit. A string that fills it exactly gets no terminator, as MoTeC writes it."
function fixed(text::AbstractString, width::Integer)
    out = zeros(UInt8, width)
    i = 0
    for c in text
        i >= width && break
        out[i+=1] = UInt32(c) <= 0xff ? UInt8(c) : UInt8('?')
    end
    return out
end

# Offsets are 0-based, as in the layout above
putle!(buf, offset, value) = copyto!(buf, offset + 1, reinterpret(UInt8, [htol(value)]))
putstr!(buf, offset, text, width) = copyto!(buf, offset + 1, fixed(text, width))

function header(channels, meta::LogMeta, data_ptr)
    buf = zeros(UInt8, HEADER_SIZE)
    putle!(buf, 0x00, UInt32(MAGIC_SOF))
    putle!(buf, 0x08, UInt32(CHANNEL_TABLE_PTR))
    putle!(buf, 0x0C, UInt32(data_ptr))
    putle!(buf, 0x24, UInt32(EVENT_PTR))
    putle!(buf, 0x40, UInt16(MAGIC_HEADER[1]))
    putle!(buf, 0x42, UInt16(MAGIC_HEADER[2]))
    putle!(buf, 0x44, UInt16(MAGIC_HEADER[3]))
    putle!(buf, 0x54, UInt16(MAGIC_UNKNOWN))

    # The reference file stores the channel count twice, then the highest rate twice
    count = UInt16(length(channels))
    rate = UInt16(maximum(channel.freq for channel in channels))
    putle!(buf, 0x56, count)
    putle!(buf, 0x58, count)
    putle!(buf, 0x5A, rate)
    putle!(buf, 0x5C, rate)

    putstr!(buf, 0x5E, meta.date, 16)
    putstr!(buf, 0x7E, meta.time, 16)
    for (offset, width, name) in HEADER_STRINGS
        putstr!(buf, offset, getfield(meta, name), width)
    end
    return buf
end

"A fixed-size block of strings, optionally ending in a pointer to the next block."
function block(size, strings, link=nothing)
    buf = zeros(UInt8, size)
    for (offset, text, width) in strings
        putstr!(buf, offset, text, width)
    end
    isnothing(link) && return buf
    putle!(buf, link[1], UInt16(link[2]))
    return buf
end

meta_blocks(meta::LogMeta) = vcat(
    block(EVENT_SIZE,
        ((0x00, meta.event, 64), (0x40, meta.session, 64), (0x80, meta.comment, 1024)),
        (EVENT_VENUE_PTR_OFF, VENUE_PTR)),
    block(VENUE_SIZE,
        ((0x00, meta.venue, 64),),
        (VENUE_VEHICLE_PTR_OFF, VEHICLE_PTR)),
    block(VEHICLE_SIZE,
        ((0x00, meta.vehicle_id, 64), (0x40, meta.vehicle_desc, 64))),
)

"One 124-byte entry in the doubly linked channel table."
function channel_record(channel::Channel, index, count, data_ptr)
    buf = zeros(UInt8, CHANNEL_SIZE)
    putle!(buf, 0, UInt32(index > 1 ? CHANNEL_TABLE_PTR + (index - 2) * CHANNEL_SIZE : 0))
    putle!(buf, 4, UInt32(index < count ? CHANNEL_TABLE_PTR + index * CHANNEL_SIZE : 0))
    putle!(buf, 8, UInt32(data_ptr))
    putle!(buf, 12, UInt32(length(channel.data)))
    putle!(buf, 16, UInt16(index))
    putle!(buf, 18, UInt16(DTYPE_FLOAT))
    putle!(buf, 20, UInt16(DTYPE_SIZE))
    putle!(buf, 22, UInt16(channel.freq))
    putle!(buf, 24, Int16(0))
    putle!(buf, 26, Int16(1))
    putle!(buf, 28, Int16(1))
    putle!(buf, 30, Int16(0))
    putstr!(buf, 32, channel.name, 32)
    # The unit sits at 64, not at 72 as ldparser's layout has it: a_steer_ecu in
    # a MoTeC export carries "deg" here and nothing at 72. Writing anything else
    # here, a short name say, shows up in i2 as the unit
    putstr!(buf, 64, channel.unit, 8)
    putle!(buf, 100, UInt16(MAGIC_CHANNEL))
    return buf
end

"""
    write_ld(path, channels, meta = LogMeta())

Write `channels` to a MoTeC i2 `.ld` file and return the path. The file is about
four bytes per sample per channel.
"""
function write_ld(path, channels::AbstractVector{Channel}, meta::LogMeta=LogMeta())
    isempty(channels) && throw(ArgumentError("need at least one channel"))
    any(channel -> channel.freq <= 0, channels) &&
        throw(ArgumentError("sample rates must be positive"))

    mkpath(dirname(abspath(path)))
    data_ptr = CHANNEL_TABLE_PTR + length(channels) * CHANNEL_SIZE
    open(path, "w") do io
        write(io, header(channels, meta, data_ptr))
        write(io, meta_blocks(meta))
        for (index, channel) in enumerate(channels)
            write(io, channel_record(channel, index, length(channels), data_ptr))
            data_ptr += length(channel.data) * DTYPE_SIZE
        end
        for channel in channels
            write(io, htol.(channel.data))
        end
    end
    return path
end

end
