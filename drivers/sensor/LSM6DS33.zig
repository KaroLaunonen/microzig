//!
//! I²C driver for ST LSM6DS33 motion sensor.
//!
//! Datasheet:
//!     https://www.mouser.com/datasheet/2/389/lsm6ds33-1849769.pdf
//!

const std = @import("std");
const mdf = @import("../framework.zig");

const sensor_log = std.log.scoped(.lsm6ds33);

pub const LSM6DS33 = struct {
    dev: mdf.base.I2C_Device,
    address: mdf.base.I2C_Device.Address,
    debug: bool,

    const Self = @This();
    const CHIP_ID = 0x69;
    const MILLI_G_TO_ACCEL = 0.00980665;

    // The chip defaults to 2g
    var configured_full_scale = accelerator_full_scale.fs_2g;

    pub const Error = mdf.base.I2C_Device.Error;
    pub const InterfaceError = mdf.base.I2C_Device.InterfaceError;
    pub const RawError = InterfaceError || error { ReadError, };
    pub const InitError = InterfaceError || error { UnexpectedDDeviceId, };

    const register = enum(u8) {
        int1_ctrl   = 0x0D,
        whoami      = 0x0F,
        ctrl1_xl    = 0x10,
        ctrl2_g     = 0x11,
        ctrl3_c     = 0x12,
        ctrl4_c     = 0x13,
        ctrl6_c     = 0x15,
        ctrl8_xl    = 0x17,
        ctrl9_xl    = 0x18,
        ctrl10_c    = 0x19,
        all_int_src = 0x1A,
        out_temp    = 0x20,
        outx_l_xl   = 0x28,
        outy_l_xl   = 0x2A,
        outz_l_xl   = 0x2C,
    };

    /// Operating mode
    /// Datasheet 5.1
    pub const operating_mode = enum(u2) {
        accelerator,
        gyroscope,
        combo,
    };

    /// Output data rate
    /// Datasheet 9.1
    pub const output_data_rate = enum(u4) {
        power_down  = 0b0000,
        hz_12_5     = 0b0001,
        hz_26       = 0b0010,
        hz_52       = 0b0011,
        hz_104      = 0b0100,
        hz_208      = 0b0101,
        hz_416      = 0b0110,
        hz_833      = 0b0111,
        khz_1_66    = 0b1000,
        khz_3_33    = 0b1001,
        khz_6_66    = 0b1010,
    };

    /// Accelerator full scale
    /// Datasheet 9.1
    pub const accelerator_full_scale = enum(u2) {
        fs_2g   = 0b00,
        fs_16g  = 0b01,
        fs_4g   = 0b10,
        fs_8g   = 0b11,

        /// Returns milli Gs per LSB for given full scale
        inline fn mg_per_lsb(fs: accelerator_full_scale) f16 {
            return switch (fs) {
                .fs_2g => 0.061,
                .fs_4g => 0.122,
                .fs_8g => 0.244,
                .fs_16g => 0.488,
            };
        }
    };

    /// Anti-aliasing filter bandwidth
    /// Datasheet 9.1
    pub const anti_aliasing_filter_bandwidth = enum(u2) {
        fb_400hz    = 0b00,
        fb_200hz    = 0b01,
        fb_100hz    = 0b10,
        fb_50hz     = 0b11,
    };

    /// High-performance mode disable
    /// Datasheet 9.16
    pub const high_performance_mode = enum(u1) {
        enabled     = 0,
        disabled    = 1,
    };

    /// Gyroscope level-sensitive latched enable.
    /// Datasheet 9.16
    pub const gyro_level_sensitive_latch = enum(u1) {
        disabled    = 0,
        enabled     = 1,
    };

    /// Gyroscope data level-sensitive trigger enable.
    /// Datasheet 9.16
    pub const gyro_level_sensitive_trigger = enum(u1) {
        disabled    = 0,
        enabled     = 1,
    };

    /// Gyroscope data edge-sensitive trigger enable.
    /// Datasheet 9.16
    pub const gyro_data_edge_sensitive_trigger = enum(u1) {
        disabled    = 0,
        enabled     = 1,
    };

    /// Accelerometer bandwidth selection.
    pub const accl_bandwidth_selection = enum(u1) {
        odr_setting     = 0,
        bw_xl_setting   = 1,
    };

    /// Linear acceleration sensor control register 1 (r/w)
    /// Datasheet 9.1
    const ctrl1_xl = packed struct {
        bw_xl: anti_aliasing_filter_bandwidth,
        fs_xl: accelerator_full_scale,
        odr_xl: output_data_rate,
    };

    /// Control register 3 (r/w)
    /// Datasheet 9.13
    const ctrl3_c = packed struct {
        sw_reset: bool,
        _the_rest: u7,
    };


    /// Control register 4 (r/w)
    /// Datasheet 9.14
    const ctrl4_c = packed struct {
        _the_rest: u7,
        bw_scal_odr: accl_bandwidth_selection,
    };

    /// Angular rate sensor control register 6 (r/w)
    /// Datasheet 9.16
    const ctrl6_c = packed struct {
        reserved: u4,
        hm_mode: high_performance_mode,
        lvl2_en: gyro_level_sensitive_latch,
        lvl_en: gyro_level_sensitive_trigger,
        trig_en: gyro_data_edge_sensitive_trigger,
    };

    pub const acceleration = struct {
        x: f16,
        y: f16,
        z: f16,
    };

    pub fn init(dev: mdf.base.I2C_Device, address: mdf.base.I2C_Device.Address, debug: bool) InitError!Self {
        var self = Self{ .dev = dev, .address = address, .debug = debug, };

        const chip_id = self.read_byte(.whoami) catch |err| {
            sensor_log.err("failed to read whoami register: {}", .{ err });
            return err;
        };

        if (chip_id != CHIP_ID) {
            if (self.debug) sensor_log.debug("Chip id not valid: {x}", .{ chip_id });
            return InitError.UnexpectedDDeviceId;
        }

        return self;
    }

    pub fn reset(self: *const Self) RawError!void {
        try self.modify_reg(register.ctrl3_c, ctrl3_c, .{
            .sw_reset = true,
        });

        if (self.debug) sensor_log.debug("resetting lsm6ds33\r\n", .{});
        var value: ctrl3_c = try self.read_raw(register.ctrl3_c, ctrl3_c);
        while (value.sw_reset) {
            value = try self.read_raw(register.ctrl3_c, ctrl3_c);
        }
        if (self.debug) sensor_log.debug("lsm6ds33 reset done\r\n", .{});
    }

    pub fn set_output_data_rate(self: *const Self, dr: output_data_rate) RawError!void {
        try self.modify_reg(register.ctrl1_xl, ctrl1_xl, .{
            .odr_xl = dr,
        });
    }

    pub fn set_accelerator_full_scale(self: *const Self, fs: accelerator_full_scale) RawError!void {
        try self.modify_reg(register.ctrl1_xl, ctrl1_xl, .{
            .fs_xl = fs,
        });
        configured_full_scale = fs;
    }

    pub fn set_anti_aliasing_filter_bandwidth(self: *const Self, bw: anti_aliasing_filter_bandwidth) RawError!void {
        // Enable bw_xl selection
        try self.modify_reg(register.ctrl4_c, ctrl4_c, .{
            .bw_scal_odr = .bw_xl_setting,
        });

        try self.modify_reg(register.ctrl1_xl, ctrl1_xl, .{
            .bw_xl = bw,
        });
    }

    pub fn set_high_performance_mode(self: *const Self, hpm: high_performance_mode) RawError!void {
        const raw_value = try self.read_raw(register.ctrl6_c, u8);

        var value: ctrl6_c = @bitCast(raw_value);
        const temp = value;

        value.hm_mode = hpm;
        sensor_log.debug("set_high_performance_mode orig: {b:08} new: {b:08}", .{ @as(u8, @bitCast(temp)), @as(u8, @bitCast(value)) });
        try self.dev.write(self.address, &([2]u8 { @intFromEnum(register.ctrl6_c), @as(u8, @bitCast(value)) }));
    }

    pub fn read_temperature(self: *const Self) Error!f16 {
        var buf: [2]u8 = undefined;
        try self.dev.write_then_read(self.address, &[_]u8 { @intFromEnum(register.out_temp) }, &buf);

        const temp_raw = std.mem.readVarInt(i16, buf[0..2], .little);

        if (self.debug) sensor_log.debug("temp raw: {x:4}", .{ temp_raw });

        // Datasheet says 16 LSB/℃, but we need 256?!
        return @as(f16, @floatFromInt(temp_raw)) / 256.0 + 25.0;
    }

    pub fn read_acceleration(self: *const Self) Error!acceleration {
        var buf: [6]u8 = undefined;
        try self.dev.write_then_read(self.address, &[_]u8 { @intFromEnum(register.outx_l_xl) }, &buf);

        const acc: acceleration = .{
            .x = @as(f16, @floatFromInt(std.mem.readVarInt(i16, buf[0..2], .little))) * configured_full_scale.mg_per_lsb() / 1000.0,
            .y = @as(f16, @floatFromInt(std.mem.readVarInt(i16, buf[2..4], .little))) * configured_full_scale.mg_per_lsb() / 1000.0,
            .z = @as(f16, @floatFromInt(std.mem.readVarInt(i16, buf[4..6], .little))) * configured_full_scale.mg_per_lsb() / 1000.0,
        };

        return acc;
    }

    pub fn read_raw_acceleration(self: *const Self) RawError![3]i16 {
        // Read consecutive x, y, z acceleration values
        var buf: [6]u8 = undefined;
        try self.dev.write_then_read(self.address, &[_]u8 { @intFromEnum(register.outx_l_xl) }, &buf);

        var acc: [3]i16 = undefined;

        acc[0] = std.mem.readVarInt(i16, buf[0..2], .little);
        acc[1] = std.mem.readVarInt(i16, buf[2..4], .little);
        acc[2] = std.mem.readVarInt(i16, buf[4..6], .little);

        return acc;
    }

    fn read_raw(self: *const Self, reg: Self.register, comptime T: type) RawError!T {
        return @bitCast(try self.read_byte(reg));
    }

    inline fn read_byte(self: *const Self, reg: Self.register) InterfaceError!u8 {
        var buf: u8 = 0;

        try self.dev.write_then_read(self.address, &[_]u8{ @intFromEnum(reg) }, (&buf)[0..1]);

        return buf;
    }

    fn write_byte(self: *const Self, reg: Self.register, value: u8) RawError!void {
        try self.dev.write(self.address, &.{ @intFromEnum(reg), value });
    }

    inline fn read_reg(self: *const Self, reg: Self.register, T: type) RawError!T {
        return @bitCast(try self.read_byte(reg));
    }

    inline fn modify_reg(self: *const Self, reg: Self.register, T: type, fields: anytype) RawError!void {
        const current_val = try self.read_reg(reg, T);

        var val: T = @bitCast(current_val);
        inline for (@typeInfo(@TypeOf(fields)).@"struct".fields) |field| {
            @field(val, field.name) = @field(fields, field.name);
        }

        try self.write_byte(reg, @bitCast(val));
    }
};
