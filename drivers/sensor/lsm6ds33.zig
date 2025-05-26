//!
//! I²C driver for ST LSM6DS33 motion sensor.
//!
//! Driver adapted from adafruit-lsm6ds
//!
//! Datasheet:
//!     https://www.mouser.com/datasheet/2/389/lsm6ds33-1849769.pdf
//!

const std = @import("std");
const mdf = @import("../framework.zig");

pub const LSM6DS33 = struct {
    dev: mdf.base.Datagram_Device,
    debug: bool,

    const Self = @This();
    const CHIP_ID = 0x69;
    const MILLI_G_TO_ACCEL = 0.00980665;

    // The chip default to 2g
    var configured_full_scale = accelerator_full_scale.fs_2g;

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
        outz_l_xl   = 0x2C
    };

    /// Operating mode
    /// Datasheet 5.1
    pub const operating_mode = enum(u2) {
        accelerator,
        gyroscope,
        combo
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

    const ctrl3_c = packed struct {
        sw_reset: bool,
        the_rest: u7
    };

    const ctrl4_c = packed struct {
        the_rest: u7,
        bw_scal_odr: accl_bandwidth_selection,
    };

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
        z: f16
    };

    pub fn init(dev: mdf.base.Datagram_Device, debug: bool) !Self {
        var self = Self{ .dev = dev, .debug = debug };

        self.dev.write(&[_]u8{ @intFromEnum(register.whoami) }) catch |err| return err;
        var chip_id: [1]u8 = undefined;
        const size = try self.dev.read(&chip_id);
        if (size != 1) return error.ReadError;

        if (chip_id[0] != CHIP_ID) {
            if (debug) std.log.debug("Chip id not valid: {}", .{ chip_id[0] });
        }

        return self;
    }

    pub fn reset(self: *const Self) !void {
        var value: ctrl3_c = @bitCast(try self.read_raw(register.ctrl3_c, u8));

        value.sw_reset = true;
        try self.dev.write(&([2]u8 { @intFromEnum(register.ctrl3_c), @as(u8, @bitCast(value)) }));

        if (self.debug) std.log.debug("resetting lsm6ds33\r\n", .{});
        while (value.sw_reset) {
            value = @bitCast(try self.read_raw(register.ctrl3_c, u8));
        }
        if (self.debug) std.log.debug("lsm6ds33 reset done\r\n", .{});
    }

    pub fn set_output_data_rate(self: *const Self, dr: output_data_rate) !void {
        var value: ctrl1_xl = @bitCast(try self.read_raw(register.ctrl1_xl, u8));
        const temp = value;

        value.odr_xl = dr;
        std.log.debug("set_output_data_rate orig: {b:08} new: {b:08}", .{ @as(u8, @bitCast(temp)), @as(u8, @bitCast(value)) });
        try self.dev.write(&([2]u8 { @intFromEnum(register.ctrl1_xl), @as(u8, @bitCast(value)) }));
    }

    pub fn set_accelerator_full_scale(self: *const Self, fs: accelerator_full_scale) !void {
        var value: ctrl1_xl = @bitCast(try self.read_raw(register.ctrl1_xl, u8));
        const temp = value;

        value.fs_xl = fs;
        std.log.debug("set_accl_full_scale orig: {b:08} new: {b:08}", .{ @as(u8, @bitCast(temp)), @as(u8, @bitCast(value)) });
        try self.dev.write(&([2]u8 { @intFromEnum(register.ctrl1_xl), @as(u8, @bitCast(value)) }));
        configured_full_scale = fs;
    }

    pub fn set_anti_aliasing_filter_bandwidth(self: *const Self, bw: anti_aliasing_filter_bandwidth) !void {
        // Enable bw_xl selection
        var bw_sel: ctrl4_c = @bitCast(try self.read_raw(register.ctrl4_c, u8));
        bw_sel.bw_scal_odr = .bw_xl_setting;
        try self.dev.write(&([2]u8 { @intFromEnum(register.ctrl4_c), @as(u8, @bitCast(bw_sel)) }));

        var value: ctrl1_xl = @bitCast(try self.read_raw(register.ctrl1_xl, u8));
        const temp = value;

        value.bw_xl = bw;
        std.log.debug("set_anti_aliasing_filter_bw orig: {b:08} new: {b:08}", .{ @as(u8, @bitCast(temp)), @as(u8, @bitCast(value)) });
        try self.dev.write(&([2]u8 { @intFromEnum(register.ctrl1_xl), @as(u8, @bitCast(value)) }));
    }

    pub fn set_high_performance_mode(self: *const Self, hpm: high_performance_mode) !void {
        var value: ctrl6_c = @bitCast(try self.read_raw(register.ctrl6_c, u8));
        const temp = value;

        value.hm_mode = hpm;
        std.log.debug("set_high_performance_mode orig: {b:08} new: {b:08}", .{ @as(u8, @bitCast(temp)), @as(u8, @bitCast(value)) });
        try self.dev.write(&([2]u8 { @intFromEnum(register.ctrl6_c), @as(u8, @bitCast(value)) }));
    }

    pub fn read_temperature(self: *const Self) !f16 {
        try self.dev.write(&[_]u8 { @intFromEnum(register.out_temp) });
        var buf: [2]u8 = undefined;

        const size = try self.dev.read(&buf);
        if (size != buf.len) {
            return error.ReadError;
        }

        // For some reason the value is in little endian format, although it should be big endian
        const temp_raw = std.mem.readVarInt(i16, buf[0..2], .little);

        if (self.debug) std.log.debug("temp raw: {x:4}", .{ temp_raw });

        // Datasheet says 16 LSB/℃, but we need 256?!
        return @as(f16, @floatFromInt(temp_raw)) / 256.0 + 25.0;
    }

    pub fn read_acceleration(self: *const Self) !acceleration {
        try self.dev.write(&[_]u8 { @intFromEnum(register.outx_l_xl) });
        var buf: [6]u8 = undefined;

        const size = try self.dev.read(&buf);
        if (size != buf.len) {
            return error.ReadError;
        }

        const acc: acceleration = .{
            .x = @as(f16, @floatFromInt(std.mem.readVarInt(i16, buf[0..2], .little))) * mg_per_lsb(configured_full_scale) / 1000.0,
            .y = @as(f16, @floatFromInt(std.mem.readVarInt(i16, buf[2..4], .little))) * mg_per_lsb(configured_full_scale) / 1000.0,
            .z = @as(f16, @floatFromInt(std.mem.readVarInt(i16, buf[4..6], .little))) * mg_per_lsb(configured_full_scale) / 1000.0,
        };

        return acc;
    }

    pub fn read_raw_acceleration(self: *const Self) ![3]i16 {
        // Read consequtive x, y, z acceleration values
        try self.dev.write(&[_]u8 { @intFromEnum(register.outx_l_xl) });
        var buf: [6]u8 = undefined;

        const size = try self.dev.read(&buf);
        if (size != 6) {
            std.log.err("size error", .{});
            return error.ReadError;
        }

        var acc: [3]i16 = undefined;

        acc[0] = std.mem.readVarInt(i16, buf[0..2], .little);
        acc[1] = std.mem.readVarInt(i16, buf[2..4], .little);
        acc[2] = std.mem.readVarInt(i16, buf[4..6], .little);

        return acc;
    }

    /// Returns milli Gs per LSB for given full scale
    inline fn mg_per_lsb(fs: accelerator_full_scale) f16 {
        return switch (fs) {
            .fs_2g => 0.061,
            .fs_4g => 0.122,
            .fs_8g => 0.244,
            .fs_16g => 0.488,
        };
    }

    inline fn read_raw(self: *const Self, reg: Self.register, T: type) !T {
        try self.dev.write(&[_]u8{ @intFromEnum(reg) });
        var buf: [@sizeOf(T)]u8 = undefined;
        const size = try self.dev.read(&buf);

        if (size != @sizeOf(T))
            return error.ReadError;

        return std.mem.readInt(T, &buf, .little);
    }
};
