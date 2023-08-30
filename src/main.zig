const std = @import("std");

// Ziegler–Nichols method for auto calibrating pid parameters / auto adjusting pid controller.
// https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
// https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
// https://en.wikipedia.org/wiki/Riemann_sum
// https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control

pub const ZnGains = struct {
    Kp: f32,
    Ti: f32,
    Td: f32,
};

pub const ZnConstants = [4]ZnGains{
    .{ .Kp = 0.6, .Ti = 0.5, .Td = 0.125 }, // ZnBasic
    .{ .Kp = 0.7, .Ti = 0.4, .Td = 0.15 }, // ZnPessenIntegralRule
    .{ .Kp = 0.33, .Ti = 0.5, .Td = 0.33 }, // ZnLessOvershoot
    .{ .Kp = 0.2, .Ti = 0.5, .Td = 0.33 }, // ZnSafe
};

pub const ZnMode = enum(u8) {
    ZnBasic = 0,
    ZnPessenIntegralRule = 1,
    ZnLessOvershoot = 2,
    ZnSafe = 3,
};

pub const ZnEdge = enum(u8) { ZnRisingEdge = 0, ZnFallingEdge = 1, ZnBothEdges = 2 };

const PI = 3.14159265358979323846;

pub const ZnPidCtrl = struct {
    g: ZnGains, // Default gains
    iterations: u32 = 0, // Calibration iterations
    edge: ZnEdge = .ZnRisingEdge,

    ts: u64 = 0, // Timestamp of last interval
    tsHigh: u64 = 0, // Timestamp of rising edge
    tsLow: u64 = 0, // Timestamp of falling edge
    diffHigh: f32 = 0, // Differential time at rising edge
    diffLow: f32 = 0, // Differential time at falling edge

    iMin: f32 = 0, // Minimum instantaneous input value
    iMax: f32 = 0, // Maximum instantaneous input value

    pError: f32 = 0, // Kp error
    iError: f32 = 0, // Ti error
    dError: f32 = 0, // Td error

    Kp: f32, // Kp (Proportional gain)
    Ki: f32, // Ti (Integral gain)
    Kd: f32, // Td (Derivative gain)

    ku: f32 = 0, // Ultimate gain
    tu: f32 = 0, // Period of output oscillations

    const Self = @This();

    pub fn init(
        gains: ZnGains, // Default Gain values.
        timeStampUs: u64, // Timestamp in microseconds.
    ) Self {
        return Self{
            .g = gains,
            .Kp = gains.Kp,
            .Ki = gains.Ti,
            .Kd = gains.Td,
            .ts = timeStampUs,
            .tsHigh = timeStampUs,
            .tsLow = timeStampUs,
        };
    }

    pub fn initMode(
        comptime mode: ZnMode, // Default Gain values.
        timeStampUs: u64, // Timestamp in microseconds.
    ) Self {
        const constant = ZnConstants[@intFromEnum(mode)];

        return Self{
            .g = constant,
            .Kp = constant.Kp,
            .Ki = constant.Ti,
            .Kd = constant.Td,
            .ts = timeStampUs,
            .tsHigh = timeStampUs,
            .tsLow = timeStampUs,
        };
    }

    pub fn isCalibrating(self: *Self) bool {
        return self.iterations > 0;
    }

    pub fn setIterations(self: *Self, iterations: u32) void {
        self.iterations = iterations;

        self.pError = 0;
        self.iError = 0;
        self.dError = 0;

        self.edge = .ZnRisingEdge;
    }

    pub fn setGains(self: *Self, gains: ZnGains) void {
        self.Kp = gains.Kp;
        self.Ki = gains.Ti;
        self.Kd = gains.Td;
    }

    pub fn getGains(self: *Self) ZnGains {
        return ZnGains{
            .Kp = self.Kp,
            .Ti = self.Ki,
            .Td = self.Kd,
        };
    }

    pub fn getPid(
        self: *Self,
        timeStampUs: u64, // Timestamp in microseconds
        input: f32, // Raw or filtered value. In during calibration the raw input value is preferred.
        setPoint: f32, // setpoint value.
        minOutput: f32, // Setpoint value of the mechanical static friction. If the input value is all the time much lower than setpoint value in during calibration (or there is much of slack) then increase the minOutput value, and if input value is all the time over the setpoint value then decrease the minOutput (Otherwise the mechanical static friction is higher than the actual setpoint value).
        maxOutput: f32, // Must be higher than minOutput value and preferably below the maximum limit value (ex: minOutput = 10% maxOutput = 50%)
        minBangBang: u32, // Min threshold value for bang-bang control (not used in during calibration) or min level for triggerin calibration (iterations must be greater than 0)
        maxBangBang: u32, // Max threshold value for bang-bang control (not used in during calibration) or max level for triggerin calibration (iterations must be greater than 0)
        autoCalibrate: f32, // Threshold value for +/-(input value). iterations must be defined
        iterations: u32, // Iterations to run auto calibrate ex: 1000
    ) f32 {
        if (self.iMax < input) {
            self.iMax = input;
        }

        if (self.iMin > input) {
            self.iMin = input;
        }

        switch (self.edge) {
            .ZnRisingEdge => {
                if (input > setPoint) {
                    self.edge = .ZnFallingEdge;
                    self.tsHigh = timeStampUs;
                    self.diffHigh = @floatFromInt(self.tsHigh - self.tsLow);

                    if (self.iterations > 0) {
                        self.iMax = setPoint;
                        self.ts = timeStampUs;

                        return minOutput;
                    }
                } else if (self.iterations > 0) {
                    self.ts = timeStampUs;
                    return maxOutput;
                }
            },
            .ZnFallingEdge => {
                if (input < setPoint) {
                    self.edge = .ZnRisingEdge;
                    self.tsLow = timeStampUs;
                    self.diffLow = @floatFromInt(self.tsLow - self.tsHigh);

                    self.ku = (4.0 * ((maxOutput - minOutput) / 2.0)) / (PI * (self.iMax - self.iMin) / 2.0);
                    self.tu = self.diffLow + self.diffHigh;

                    if (self.iterations > 0) {
                        const diff = @as(f32, @floatFromInt(timeStampUs - self.ts));
                        const p = self.g.Kp * self.ku;
                        const i = (p / (self.g.Ti * self.tu)) * diff;
                        const d = (self.g.Td * p * self.tu) / diff;

                        self.Kp = (self.Kp + p) / 2;
                        self.Ki = (self.Ki + i) / 2;
                        self.Kd = (self.Kd + d) / 2;

                        self.iMin = setPoint;
                        self.ts = timeStampUs;
                        self.iterations -= 1;

                        return maxOutput;
                    }
                } else if (self.iterations > 0) {
                    self.ts = timeStampUs;
                    return minOutput;
                }
            },
            .ZnBothEdges => {
                unreachable;
            },
        }

        if (maxBangBang > 0 and ((setPoint - input) > @as(f32, @floatFromInt(maxBangBang)))) {
            self.ts = timeStampUs;
            return maxOutput;
        }

        if (minBangBang > 0 and ((input - setPoint) > @as(f32, @floatFromInt(minBangBang)))) {
            self.ts = timeStampUs;
            return minOutput;
        }

        if (iterations > 0 and autoCalibrate > 0 and ((setPoint - input) > autoCalibrate or (input - setPoint) > autoCalibrate)) {
            self.setIterations(iterations);
            return maxOutput;
        }

        const pError = setPoint - input;
        const diff = @as(f32, @floatFromInt(timeStampUs - self.ts)) / 1000000;

        self.dError = (pError - self.pError) / diff / 1000.0;
        self.iError += (pError + self.pError) / 2 * diff / 1000.0;
        self.pError = pError;
        self.ts = timeStampUs;

        const pid = (self.Kp * self.pError) + (self.Ki * self.iError) + (self.Kd * self.dError);

        if (pid > maxOutput) {
            return maxOutput;
        }

        if (pid < minOutput) {
            return minOutput;
        }

        return pid;
    }
};

fn getInput() f32 {
    var reng = std.rand.DefaultPrng.init(@as(u64, @intCast(std.time.microTimestamp())));

    return reng.random().float(f32) * 100;
}

fn setOutput(input: f32, speed: f32, friction: f32) f32 {
    var reng = std.rand.DefaultPrng.init(@as(u64, @intCast(std.time.microTimestamp())));
    var ret: f32 = input;

    if (speed >= 50) {
        ret += (0.004337 + ((reng.random().float(f32) / 10000) * friction)) * speed;
    } else {
        ret -= ((0.007337 + ((reng.random().float(f32) / 10000) * friction)) * 5) * ((50 - speed) + 1);
    }

    return ret;
}

test "ZnSafe - With calibration" {
    var ctrl = ZnPidCtrl.initMode(.ZnSafe, 0);
    var input: f32 = getInput();
    var speed: f32 = 0;
    var calibrated: f32 = 0;
    const setpoint = 668.5;

    ctrl.setIterations(10000);

    std.debug.print("\nInput: {d}\n", .{input});

    for (0..10000000) |us| { // Every iteration is happening in 100ms cycles
        if (!ctrl.isCalibrating()) {
            if ((setpoint - input) > 0.5 or (input - setpoint) > 0.5) {
                ctrl.setIterations(1000);
                calibrated += 1;
            }
        }

        speed = ctrl.getPid((us * 100000), input, setpoint, 0, 100, 0, 0, 0, 0);
        input = setOutput(input, speed, 25);

        // if ((us % 1000) == 0) {
        //     if (ctrl.isCalibrating()) {
        //         std.debug.print("{d}\n\n", .{input});
        //     } else {
        //         std.debug.print("{d} {d}\n\n", .{ input, speed });
        //     }
        // }
    }

    std.debug.print("Input: {d} Setpoint: {d}\n", .{ input, setpoint });
    std.debug.print("{any}\n", .{ctrl.getGains()});
    std.debug.print("Calibrated {d} times.\n\n", .{calibrated});
}

test "Static ZnGains - Without calibration" {
    var ctrl = ZnPidCtrl.init(.{ .Kp = 4.86740913e+01, .Ti = 2.65768165e+01, .Td = 5.98150596e+01 }, 0);
    var input: f32 = getInput();
    var speed: f32 = 0;
    const setpoint = 668.5;

    std.debug.print("\nInput: {d}\n", .{input});

    for (0..10000000) |us| { // Every iteration is happening in 100ms cycles
        speed = ctrl.getPid((us * 100000), input, setpoint, 0, 100, 0, 0, 0, 0);
        input = setOutput(input, speed, 25);

        // if ((us % 1000) == 0) {
        //     if (ctrl.isCalibrating()) {
        //         std.debug.print("{d}\n\n", .{input});
        //     } else {
        //         std.debug.print("{d} {d}\n\n", .{ input, speed });
        //     }
        // }
    }

    std.debug.print("Input: {d} Setpoint: {d}\n", .{ input, setpoint });
    std.debug.print("{any}\n\n", .{ctrl.getGains()});
}

test "ZnSafe - With auto calibration" {
    var ctrl = ZnPidCtrl.initMode(.ZnSafe, 0);
    var input: f32 = getInput();
    var speed: f32 = 0;
    const setpoint = 668.5;

    std.debug.print("\nInput: {d}\n", .{input});

    for (0..10000000) |us| { // Every iteration is happening in 100ms cycles
        speed = ctrl.getPid((us * 100000), input, setpoint, 0, 100, 0, 0, 0.5, 1000);
        input = setOutput(input, speed, 25);

        // if ((us % 1000) == 0) {
        //     if (ctrl.isCalibrating()) {
        //         std.debug.print("{d}\n\n", .{input});
        //     } else {
        //         std.debug.print("{d} {d}\n\n", .{ input, speed });
        //     }
        // }
    }

    std.debug.print("Input: {d} Setpoint: {d}\n", .{ input, setpoint });
    std.debug.print("{any}\n\n", .{ctrl.getGains()});
}
