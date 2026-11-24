// lateral.rhai: strafe sideways back and forth, closed loop on vision.
//
//   rhai scripts/lateral.rhai -- distance=1.0 speed=0.5 legs=4 side=left
//
// The robot moves along its own lateral axis (robot y, "left") as it was
// when the script started, between the start point A and a point B that is
// `distance` to the `side`. It keeps the start heading and stays on the
// line A-B, so only the lateral motion is left.
//
// Math (world frame; start pose (x0, y0, theta0), robot p = (x, y), heading theta)
//
//   u_l = (-sin theta0, cos theta0)      lateral axis (robot left at start)
//   u_f = ( cos theta0, sin theta0)      forward axis (robot front at start)
//   B   = A + s * distance * u_l,        s = +1 left, -1 right
//
//   e_l = (goal - p) . u_l               lateral error, m (along the line)
//   e_f = (A - p)    . u_f               cross-track error, m (off the line)
//
//   Velocity:  v = v_l * u_l + v_f * u_f
//     v_l = clamp(K_L * e_l, +-speed)   go to the goal, slowing down near it
//     v_f = clamp(K_F * e_f, +-speed)   pull back onto the line
//   Heading:   omega = K_TH * wrap(theta0 - theta)
//
// A leg ends when |e_l| < TOL and the robot has slowed below SETTLED; then
// it waits `pause` and heads to the other end. legs = 0: until Stop.
// Commands are world frame (wvel): the engine rotates them into the robot
// frame every tick and applies the speed / acceleration / field limits.

fn params() {
    [
        #{ name: "distance", type: "float", value: 1.0, min: 0.05, max: 6.0, help: "length of the strafe, m" },
        #{ name: "side", type: "choice", options: ["left", "right"], value: "left", help: "first leg goes to the robot's left or right" },
        #{ name: "speed", type: "float", value: 0.5, min: 0.05, max: 2.0, help: "max lateral speed, m/s (engine limits still apply)" },
        #{ name: "legs", type: "int", value: 4, min: 0, max: 1000, help: "one-way trips; 0 = until Stop" },
        #{ name: "pause", type: "float", value: 0.5, min: 0.0, max: 10.0, help: "stop at each end, s" },
        #{ name: "record", type: "bool", value: false, help: "record the run to CSV" },
    ]
}

// Angle in (-pi, pi].
fn wrap(a) {
    while a > PI() { a -= 2.0 * PI(); }
    while a <= -PI() { a += 2.0 * PI(); }
    a
}

fn clamp(v, lim) { max(-lim, min(lim, v)) }

const K_L = 2.5;        // lateral gain, 1/s
const K_F = 2.0;        // cross-track gain, 1/s
const K_TH = 3.0;       // heading gain, 1/s
const TOL = 0.02;       // at the goal when closer than this, m
const SETTLED = 0.05;   // ... and slower than this, m/s
const DT = 0.02;        // loop period, s (the engine runs at 60 Hz)

let s = if args.side == "right" { -1.0 } else { 1.0 };
let speed = args.speed;

// Wait for vision to fix the start pose.
let p = pose();
while !p.seen {
    print("lateral: waiting for the robot");
    wait(0.2);
    p = pose();
}
let ax = p.x;
let ay = p.y;
let th0 = p.theta;
let lx = -sin(th0);
let ly = cos(th0);
let fx = cos(th0);
let fy = sin(th0);
let bx = ax + s * args.distance * lx;
let by = ay + s * args.distance * ly;

if args.record { record(); }
print(`lateral: A (${ax}, ${ay}) -> B (${bx}, ${by})  heading ${th0}  ${speed} m/s  legs ${args.legs}`);

let leg = 0;
let to_b = true;
loop {
    p = pose();
    if !p.seen {
        // The engine already stops the robot without vision; wait for it.
        stop();
        wait(0.1);
        continue;
    }
    let gx = if to_b { bx } else { ax };
    let gy = if to_b { by } else { ay };

    let e_l = (gx - p.x) * lx + (gy - p.y) * ly;
    let e_f = (ax - p.x) * fx + (ay - p.y) * fy;

    let v_l = clamp(K_L * e_l, speed);
    let v_f = clamp(K_F * e_f, speed);
    let w = K_TH * wrap(th0 - p.theta);
    wvel(v_l * lx + v_f * fx, v_l * ly + v_f * fy, w);

    if e_l.abs() < TOL && hypot(p.vx, p.vy) < SETTLED {
        stop();
        leg += 1;
        print(`lateral: leg ${leg} done, lateral error ${e_l} m, off the line ${e_f} m, heading error ${wrap(th0 - p.theta)} rad`);
        if args.legs > 0 && leg >= args.legs { break; }
        wait(args.pause);
        to_b = !to_b;
        continue;
    }
    wait(DT);
}

stop();
print(`lateral: done, ${leg} legs in ${time()} s`);
