// 1_delay.rhai: delay (dead time).
//
// Sharp velocity steps 0 -> amp -> 0, `reps` times on vx, then the same on vy.
// The sign alternates (+amp, then -amp) so the robot goes back and forth
// instead of leaving the field; between steps it drives back to the start
// pose (closed loop, world frame) and waits `rest` s at zero.
// Gives: tau, the time from a step in cmd_vx / cmd_vy to the first motion
// in vx / vy. It should be the same for both axes.
//
// Before:
//   - The engine limits shape the step: `set max_accel 50` (or higher) so the
//     command is a real step, and max_speed >= amp. Set them back after.
//   - Nothing to record by hand: the script records itself (named after
//     the script) and the recording stops when it ends or is aborted.
// Segments: a step is a stretch where the command stays at a test value
// with cmd_omega = 0. The return to start (wvel, capped at 0.7 m/s) never
// holds one of those values, and the console prints each step's start time.

fn params() {
    [
        #{ name: "amp", type: "float", value: 1.0, min: 0.1, max: 3.0, help: "step size, m/s" },
        #{ name: "reps", type: "int", value: 10, min: 1, max: 50, help: "steps per axis" },
        #{ name: "hold", type: "float", value: 1.0, min: 0.2, max: 3.0, help: "time at amp, s" },
        #{ name: "rest", type: "float", value: 1.0, min: 0.3, max: 5.0, help: "time at zero, s" },
        #{ name: "axes", type: "choice", options: ["both", "vx", "vy"], value: "both" },
    ]
}

fn wrap(a) {
    while a > PI() { a -= 2.0 * PI(); }
    while a <= -PI() { a += 2.0 * PI(); }
    a
}

// Drive back to the start pose `s` (world frame), then hold zero.
fn home(s) {
    let t_end = time() + 6.0;
    while time() < t_end {
        let p = pose();
        if !p.seen { stop(); wait(0.1); continue; }
        let ex = s.x - p.x; let ey = s.y - p.y; let et = wrap(s.theta - p.theta);
        if hypot(ex, ey) < 0.03 && et.abs() < 0.05 { break; }
        let k = 2.0;
        let vx = k * ex; let vy = k * ey;
        let v = hypot(vx, vy);
        let cap = 0.7;  // m/s, not one of the test values (see Segments)
        if v > cap { vx *= cap / v; vy *= cap / v; }
        wvel(vx, vy, 3.0 * et);
        wait(1.0 / 60.0);
    }
    stop();
}

let start = pose();
for i in 0..20 { if start.seen { break; } wait(0.1); start = pose(); }  // up to 2 s for vision
if !start.seen { throw "robot not seen"; }
record();

let axes = if args.axes == "both" { ["vx", "vy"] } else { [args.axes] };
for axis in axes {
    print(`delay: ${axis}, ${args.reps} steps of +-${args.amp} m/s`);
    for i in 0..args.reps {
        home(start);
        wait(args.rest);
        let a = if i % 2 == 0 { args.amp } else { -args.amp };
        print(`delay: ${axis} step ${i + 1} = ${a} at t=${time()}`);
        if axis == "vx" { vel(a, 0, 0); } else { vel(0, a, 0); }
        wait(args.hold);
        stop();
        wait(args.rest);
    }
}
home(start);
print("delay: done");
