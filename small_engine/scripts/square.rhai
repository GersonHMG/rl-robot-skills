// Rhai example: open-loop square, then drive forward until 0.5 m from the start.
// Run with:  rhai scripts/square.rhai      (type `stop` to abort)
// vel(vx, vy, w) is robot frame, m/s and rad/s, held until the next command.

let speed = 0.5;
let side = 0.5;

for i in 0..4 {
    vel(speed, 0.0, 0.0);
    wait(side / speed);
    vel(0.0, 0.0, 1.5708);      // quarter turn in 1 s
    wait(1.0);
}
stop();
wait(0.5);

// Closed loop on vision: read the pose while driving.
let start = pose();
vel(0.3, 0.0, 0.0);
while true {
    let p = pose();
    if !p.seen { print("lost the robot"); break; }
    let d = ((p.x - start.x) ** 2 + (p.y - start.y) ** 2).sqrt();
    if d >= 0.5 { break; }
    wait(0.02);
}
stop();
print(`ended at ${pose()} after ${time()} s`);
