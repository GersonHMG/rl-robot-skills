// orbit.rhai: circle a point while facing it.
//
//   rhai scripts/orbit.rhai -- center_x=0 center_y=0 radius=0.6 direction=cw turns=3
//
// Phases
//   1. approach: drive radially to the nearest point of the circle, turning
//      to face the centre on the way.
//   2. orbit: move along the circle at `speed`, still facing the centre,
//      until `turns` revolutions are done (turns = 0: until Stop).
//
// Math (world frame; centre c = (cx, cy), robot p = (x, y), heading theta)
//
//   d   = p - c,   rho = |d|             distance robot <-> centre
//   phi = atan2(dy, dx)                  robot's angle around the centre
//   u_r = ( cos phi, sin phi)            unit radial vector, centre -> robot
//   u_t = s * (-sin phi, cos phi)        unit tangent, s = +1 CCW, -1 CW
//
//   Velocity:  v = speed * u_t  +  K_R * (r - rho) * u_r
//     The tangential part moves the robot along the circle. The radial
//     part is a proportional correction that pulls it back onto the circle:
//     going straight along the tangent for one tick drifts it outwards by
//     about speed^2 * dt^2 / (2 r), and the acceleration limits add more.
//     During the approach only the radial part is used (capped at `speed`),
//     which drives the robot to the nearest point of the circle, c + r * u_r.
//
//   Heading:   facing the centre means pointing along -u_r, so
//     theta* = atan2(cy - y, cx - x) = phi + pi
//   On the circle, arc length = r * angle, so phi turns at
//     dphi/dt = s * speed / r
//   and theta* turns at the same rate. The command is that feed-forward
//   plus a proportional correction of the heading error:
//     omega = s * speed / r  +  K_TH * wrap(theta* - theta)
//
// Commands are world frame (wvel): the engine rotates them into the robot
// frame every tick and applies the speed / acceleration / field limits.
// A typed `stop` or the Stop button aborts the script; the robot is stopped
// when it ends.

fn params() {
    [
        #{ name: "center_x", type: "float", value: 0.0, min: -4.5, max: 4.5, help: "circle centre x, m (world)" },
        #{ name: "center_y", type: "float", value: 0.0, min: -3.0, max: 3.0, help: "circle centre y, m (world)" },
        #{ name: "radius", type: "float", value: 0.5, min: 0.15, max: 3.0, help: "circle radius, m" },
        #{ name: "direction", type: "choice", options: ["ccw", "cw"], value: "ccw", help: "ccw = counter-clockwise seen from above" },
        #{ name: "speed", type: "float", value: 0.5, min: 0.05, max: 2.0, help: "speed along the circle, m/s (engine limits still apply)" },
        #{ name: "turns", type: "float", value: 2.0, min: 0.0, max: 100.0, help: "revolutions; 0 = until Stop" },
        #{ name: "face_center", type: "bool", value: true, help: "keep facing the centre (off: keep the start heading)" },
    ]
}

// Angle in (-pi, pi].
fn wrap(a) {
    while a > PI() { a -= 2.0 * PI(); }
    while a <= -PI() { a += 2.0 * PI(); }
    a
}

const K_R = 2.0;         // radial correction gain, 1/s
const K_TH = 3.0;        // heading correction gain, 1/s
const ON_CIRCLE = 0.05;  // |r - rho| below this (m) ends the approach
const DT = 0.02;         // loop period, s (the engine runs at 60 Hz)

let cx = args.center_x;
let cy = args.center_y;
let r = args.radius;
let speed = args.speed;
let s = if args.direction == "cw" { -1.0 } else { 1.0 };
let goal = args.turns * 2.0 * PI();  // angle to sweep, rad; 0 = no limit

let orbiting = false;
let travelled = 0.0;  // angle swept around the centre while orbiting, rad
let last_phi = 0.0;
let hold = ();        // start heading, used when face_center is off

print(`orbit: centre (${cx}, ${cy})  r ${r}  ${args.direction}  ${speed} m/s  turns ${args.turns}`);

loop {
    let p = pose();
    if !p.seen {
        // The engine already stops the robot without vision; wait for it.
        stop();
        wait(0.1);
        continue;
    }
    if hold == () { hold = p.theta; }

    let dx = p.x - cx;
    let dy = p.y - cy;
    let rho = hypot(dx, dy);
    // At the centre the direction is undefined: leave backwards, so the
    // robot already faces the centre when it reaches the circle.
    let phi = if rho < 0.001 { p.theta + PI() } else { atan(dy, dx) };

    let ur_x = cos(phi);
    let ur_y = sin(phi);
    let ut_x = -s * ur_y;
    let ut_y = s * ur_x;
    let err = r - rho;  // > 0: inside the circle

    if !orbiting && err.abs() < ON_CIRCLE {
        orbiting = true;
        last_phi = phi;
        print("orbit: on the circle");
    }

    // Velocity: tangential (only while orbiting) + radial correction.
    let v_r = max(-speed, min(speed, K_R * err));
    let v_t = if orbiting { speed } else { 0.0 };
    let vx = v_t * ut_x + v_r * ur_x;
    let vy = v_t * ut_y + v_r * ur_y;

    // Heading: feed-forward of the orbit rate + correction towards the centre.
    let w = 0.0;
    if args.face_center {
        let ff = if orbiting { s * speed / r } else { 0.0 };
        w = ff + K_TH * wrap(phi + PI() - p.theta);
    } else {
        w = K_TH * wrap(hold - p.theta);
    }
    wvel(vx, vy, w);

    // Termination: count the angle swept around the centre.
    if orbiting {
        travelled += s * wrap(phi - last_phi);
        last_phi = phi;
        if goal > 0.0 && travelled >= goal { break; }
    }
    wait(DT);
}

stop();
print(`orbit: done, ${travelled / (2.0 * PI())} turns`);
