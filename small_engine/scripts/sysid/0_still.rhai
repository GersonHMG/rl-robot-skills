// 0_still.rhai: standing still.
//
// Sends a zero command for `duration` seconds.
// Gives: the vision noise level (spread of x, y, theta while nothing moves),
// and whether the robot creeps when told to stop (drift of x, y, theta).
//
// Before: press Record (or type `rec`), stop recording afterwards.
// At the end it prints the drift and the spread it saw, as a quick check.

fn params() {
    [
        #{ name: "duration", type: "float", value: 10.0, min: 1.0, max: 120.0, help: "s" },
    ]
}

stop();
wait(0.5);  // let any motion settle before measuring

let n = 0;
let sx = 0.0; let sy = 0.0; let st = 0.0;
let sxx = 0.0; let syy = 0.0;
let first = ();
let last = ();
let t_end = time() + args.duration;
print(`still: zero command for ${args.duration} s`);
while time() < t_end {
    stop();
    let p = pose();
    if p.seen {
        if first == () { first = p; }
        last = p;
        n += 1;
        sx += p.x; sy += p.y;
        sxx += p.x * p.x; syy += p.y * p.y;
    }
    wait(1.0 / 60.0);
}

if n < 2 {
    print("still: robot not seen");
} else {
    let mx = sx / n; let my = sy / n;
    let sdx = sqrt(max(0.0, sxx / n - mx * mx));
    let sdy = sqrt(max(0.0, syy / n - my * my));
    let drift = hypot(last.x - first.x, last.y - first.y);
    print(`still: ${n} frames  std x ${sdx * 1000.0} mm  std y ${sdy * 1000.0} mm`);
    print(`still: drift ${drift * 1000.0} mm  heading change ${(last.theta - first.theta) * 180.0 / PI()} deg`);
}
