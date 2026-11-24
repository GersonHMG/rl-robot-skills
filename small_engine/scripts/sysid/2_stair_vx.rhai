// 2_stair_vx.rhai: staircase, forward.
//
// Holds vx at each level for `hold` s, +level then -level (so the robot
// comes back), stopping and returning to the start pose between levels.
// Levels: 0.1 0.2 0.3 0.5 0.8 1.0 1.5 m/s and `max` (skipped if not above 1.5).
// Gives: gain (measured / commanded vx) and saturation for vx.
// Sideways motion (vy) and rotation (omega) during these runs is the coupling.
//
// Before:
//   - The engine clips speed to max_speed and ramps at max_accel:
//     `set max_speed <max>` and `set max_accel 50` (or higher) so the
//     commanded levels reach the robot. Set them back after.
//   - Press Record (or type `rec`); stop recording afterwards.
// Segments: a step is a stretch where the command stays at a test value
// with cmd_omega = 0. The return to start (wvel, capped at 0.7 m/s) never
// holds one of those values, and the console prints each step's start time.

fn params() {
    [
        #{ name: "max", type: "float", value: 2.0, min: 0.1, max: 5.0, help: "last level, m/s (robot max)" },
        #{ name: "hold", type: "float", value: 1.0, min: 0.3, max: 3.0, help: "time at each level, s" },
        #{ name: "rest", type: "float", value: 0.5, min: 0.2, max: 5.0, help: "time at zero, s" },
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

let levels = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0, 1.5];
if args.max > 1.5 { levels.push(args.max); }

for level in levels {
    for v in [level, -level] {
        home(start);
        wait(args.rest);
        print(`stair vx: ${v} m/s at t=${time()}`);
        vel(v, 0, 0);
        wait(args.hold);
        stop();
        wait(args.rest);
    }
}
home(start);
print("stair vx: done");
