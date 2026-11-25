-- QYAW_* parameters are native ArduPlane parameters.
-- Read them by name so GCS changes take effect without restarting the script.
local parameter_names = {
    "ENABLE", "SYSID", "COMPID", "MIN_HGT", "DIST_MAX", "MAX_AGE",
    "STABLE", "TOL", "RATE_MAX", "GAIN", "FORWARD"
}
for _, name in ipairs(parameter_names) do
    assert(param:get("QYAW_" .. name) ~= nil, "QYAW: missing native parameter QYAW_" .. name)
end

local function qyaw(name)
    return param:get("QYAW_" .. name)
end

local supported, api = pcall(function()
    return quadplane.set_landing_yaw_rate and quadplane.clear_landing_yaw_rate
end)
assert(supported and api, "QYAW: firmware lacks landing yaw API")
mavlink:init(10, 1)
mavlink:register_rx_msgid(149)

local target, stable_since, last_capture, last_received
local transport_delay_ms = 0
local correcting = false
local state

local function finite(x)
    return x == x and math.abs(x) < math.huge
end

local function set_state(value)
    if state ~= value then
        state = value
        gcs:send_text(6, "QYAW: " .. value)
    end
end

local function release(reason)
    quadplane:clear_landing_yaw_rate()
    stable_since = nil
    correcting = false
    set_state(reason)
end

local function valid_parameters()
    return qyaw("ENABLE") == 1 and qyaw("SYSID") >= 1 and qyaw("SYSID") <= 255
        and qyaw("COMPID") >= 1 and qyaw("COMPID") <= 255
        and qyaw("MIN_HGT") >= 1 and finite(qyaw("DIST_MAX")) and qyaw("DIST_MAX") > 0
        and qyaw("MAX_AGE") >= 0.05 and qyaw("MAX_AGE") <= 0.5
        and qyaw("STABLE") >= 0.1 and qyaw("STABLE") <= 5
        and qyaw("TOL") >= 1 and qyaw("TOL") <= 20
        and qyaw("RATE_MAX") > 0 and qyaw("RATE_MAX") <= 45
        and qyaw("GAIN") > 0 and qyaw("GAIN") <= 5 and finite(qyaw("FORWARD"))
end

local function payload_offset(raw)
    -- receive_chan returns the packed C structure, not the wire packet.
    -- Standard MAVLink buffers are 291 bytes (8-bit sysid) or 298 (32-bit sysid).
    if #raw == 291 then return 13 end
    if #raw == 298 then return 16 end
    return nil
end

local function decode(raw, offset)
    -- ArduPilot has already checked the MAVLink checksum/signature on reception.
    if string.byte(raw, 3) ~= 0xFD or string.byte(raw, 4) < 60 then
        return nil
    end
    -- ArduPilot Lua has 32-bit integers/floats: keep the uint64 capture time in four exact words.
    local a, b, c, d, _, _, distance, _, _, number, frame, x, y, z, w, qx, qy, qz, kind, valid =
        string.unpack("<I2I2I2I2fffffBBfffffffBB", raw, offset)
    local norm = w*w + qx*qx + qy*qy + qz*qz
    if (a == 0 and b == 0 and c == 0 and d == 0) or number ~= 0 or frame ~= 12 or kind ~= 2 or valid ~= 1
        or not finite(x) or not finite(y) or not finite(z) or z <= 0
        or not finite(distance) or distance <= 0 or not finite(norm)
        or norm < 0.9 or norm > 1.1 then
        return nil
    end
    norm = math.sqrt(norm)
    return {stamp = {a,b,c,d}, x = x, y = y, z = z, w = w/norm, qx = qx/norm, qy = qy/norm, qz = qz/norm}
end

local function newer(a, b)
    if not b then return true end
    for i = 4, 1, -1 do
        if a[i] ~= b[i] then return a[i] > b[i] end
    end
    return false
end

local function capture_delta_ms(a, b)
    local delta, borrow = {}, 0
    for i = 1, 4 do
        local value = a[i] - b[i] - borrow
        borrow = value < 0 and 1 or 0
        delta[i] = value + borrow*65536
    end
    if delta[3] ~= 0 or delta[4] ~= 0 then return math.huge end
    return (delta[2]*65536.0 + delta[1])/1000.0
end

local function receive(now)
    for _ = 1, 10 do
        local raw, _, received_ms = mavlink:receive_chan()
        if not raw then break end
        local offset = payload_offset(raw)
        if offset and string.byte(raw, 8) == qyaw("SYSID")
            -- QYAW_SYSID accepts 1..255; reject extended IDs sharing the low byte.
            and (offset == 13 or string.unpack("<I3", raw, 9) == 0)
            and string.byte(raw, offset - 4) == qyaw("COMPID")
            and string.unpack("<I3", raw, offset - 3) == 149 then
            local sample = decode(raw, offset)
            local age = (now - received_ms):toint()
            if not sample or age < 0 or age > qyaw("MAX_AGE")*1000 then
                target, stable_since = nil, nil
            elseif newer(sample.stamp, last_capture) then
                local gap = last_capture and capture_delta_ms(sample.stamp, last_capture) or math.huge
                if last_received then
                    transport_delay_ms = math.max(0, transport_delay_ms + (received_ms-last_received):toint() - gap)
                end
                -- Bound added transport delay without converting either clock to floating point.
                if transport_delay_ms > qyaw("MAX_AGE")*1000 then
                    target, stable_since = nil, nil
                else
                    if not target or gap > qyaw("MAX_AGE")*1000 then
                        stable_since = nil
                    end
                    sample.received = received_ms
                    target = sample
                end
                last_capture = sample.stamp
                last_received = received_ms
            end
        end
    end
end

local function heading_error(sample)
    local roll, pitch = ahrs:get_roll_rad(), ahrs:get_pitch_rad()
    if not finite(roll) or not finite(pitch) then return nil end
    local cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    local north = cp*sample.x + sp*sr*sample.y + sp*cr*sample.z
    local east = cr*sample.y - sr*sample.z
    local down = -sp*sample.x + cp*sr*sample.y + cp*cr*sample.z
    if down < qyaw("MIN_HGT") or north*north + east*east > qyaw("DIST_MAX")^2 then return nil end

    local w, x, y, z = sample.w, sample.qx, sample.qy, sample.qz
    local offset = math.rad(qyaw("FORWARD"))
    local c, s = math.cos(offset), math.sin(offset)
    local vx = c*(1 - 2*(y*y + z*z)) + s*2*(x*y - w*z)
    local vy = c*2*(x*y + w*z) + s*(1 - 2*(x*x + z*z))
    local vz = c*2*(x*z - w*y) + s*2*(y*z + w*x)
    local hx, hy = cp*vx + sp*sr*vy + sp*cr*vz, cr*vy - sr*vz
    if hx*hx + hy*hy < 0.04 then return nil end
    return math.deg(math.atan(hy, hx))
end

local function update()
    local now = millis()
    receive(now)

    if not valid_parameters() or vehicle:get_mode() ~= 20 or not arming:is_armed() then
        target, last_capture, last_received = nil, nil, nil
        transport_delay_ms = 0
        release("inactive")
        return
    end
    if not ahrs:healthy() or vehicle:has_ekf_failsafed() or battery:has_failsafed()
        or not rc:has_valid_input() or param:get("Q_WVANE_ENABLE") ~= 0 then
        release("vehicle unavailable")
        return
    end
    if not target or (now - target.received):toint() > qyaw("MAX_AGE")*1000 then
        target = nil
        release("waiting for target")
        return
    end
    local error_deg = heading_error(target)
    if not error_deg then
        release("outside alignment envelope")
        return
    end
    stable_since = stable_since or now
    if (now - stable_since):toint() < qyaw("STABLE")*1000 then
        quadplane:clear_landing_yaw_rate()
        set_state("acquiring target")
        return
    end
    if math.abs(error_deg) <= qyaw("TOL") then
        correcting = false
    elseif math.abs(error_deg) > qyaw("TOL") + 2 then
        correcting = true
    end
    local rate = correcting and math.max(-qyaw("RATE_MAX"), math.min(qyaw("RATE_MAX"), qyaw("GAIN")*error_deg)) or 0
    if not quadplane:set_landing_yaw_rate(rate) then
        release("pilot or flight controller priority")
        return
    end
    set_state(correcting and "aligning during descent" or "aligned")
end

local function run()
    local ok, err = pcall(update)
    if not ok then
        quadplane:clear_landing_yaw_rate()
        target, stable_since = nil, nil
        correcting = false
        gcs:send_text(3, "QYAW: " .. tostring(err))
    end
    return run, ok and 50 or 1000
end

gcs:send_text(6, "QYAW: loaded; QLAND yaw controller")
return run, 50
