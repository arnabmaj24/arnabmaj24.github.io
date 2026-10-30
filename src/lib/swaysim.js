// Truck-trailer sway demo: a planar two-body model (truck + trailer joined by a pinned hitch) and a re-implementation of
// the sway controller's published logic. Simplified for illustration; this is not Aston's production code or tuning.
//
// Each body is a rigid body in the plane with single-track (one tire per axle) lateral forces. The hitch is a pin joint:
// every step solves for the 2D hitch force that keeps the two hitch points together, S = J M^-1 J^T (2x2, inverted
// analytically), with Baumgarte stabilization so numerical drift can't pull the joint apart.

const G = 9.81;
const RHO = 1.2;
const DT = 0.001; // physics step [s]
const CTRL_EVERY = 10; // controller runs every 10 physics steps: 100 Hz
const SAMPLE_EVERY = 20; // recorded frames: 50 Hz

export const DEFAULTS = {
  speedKmh: 120,
  tongueWeight: 0.04, // fraction of trailer weight carried by the hitch; negative means tail-heavy
  trailerMass: 3000, // kg
  impulseDeg: 1.5, // peak front-wheel angle of the steering impulse
  duration: 25, // s: long enough to settle and get back up to speed
};

// A full-size pickup and a tandem-axle travel trailer. Round numbers, chosen to be typical, not a specific model.
export function buildVehicle(p) {
  const truck = {
    m: 2600,
    I: 4500,
    a: 1.6, // CG to front axle
    b: 2.1, // CG to rear axle
    c: 3.3, // CG to hitch ball (behind the rear axle)
    Cf: 140e3, // front axle cornering stiffness [N/rad]
    Cr: 160e3, // rear axle
    CdA: 1.3,
  };
  const l2 = 5.0; // hitch to trailer axle
  const e = p.tongueWeight * l2; // trailer CG ahead of its axle: tongue load = m g e / l2
  const trailer = {
    m: p.trailerMass,
    I: (p.trailerMass * (6.5 ** 2 + 2.4 ** 2)) / 12, // box of 6.5 m x 2.4 m
    d: l2 - e, // hitch to CG
    e,
    l2,
    Ct: 160e3,
    CdA: 2.6,
  };
  return { truck, trailer };
}

const MU = 0.9;
const MU_BRAKE = 0.7;
const CRR = 0.012;

const wrapPi = (a) => Math.atan2(Math.sin(a), Math.cos(a));

function tireForce(C, alpha, Fz, Fx = 0) {
  const cap = Math.sqrt(Math.max((MU * Fz) ** 2 - Fx * Fx, 1));
  return cap * Math.tanh((C * alpha) / cap);
}

const IMPULSE_T0 = 1.0; // s
const IMPULSE_T = 0.6; // s

function steerImpulse(t, amp) {
  if (t < IMPULSE_T0 || t > IMPULSE_T0 + IMPULSE_T) return 0;
  return amp * Math.sin((Math.PI * (t - IMPULSE_T0)) / IMPULSE_T);
}

// A lane-keeping driver, the same in both runs: pure pursuit. After a short reaction time it steers toward a point on
// the lane centre about 2.2 s ahead, which is the standard geometric driver model. There are no lagging filters in the
// loop (lag is what makes a driver weave), just light smoothing and a steering-rate limit. (A strict ISO 9815 run holds
// the wheel still after the impulse instead.)
export const DRIVER = { reaction: 0.4, lookaheadS: 2.2, minLookahead: 25, tau: 0.1, maxDeg: 3, rateDeg: 10 };

function makeDriver(wheelbase) {
  let filt = 0;
  let cmd = 0;
  return function step(t, y, psi, u, dt) {
    if (t < IMPULSE_T0 + IMPULSE_T + DRIVER.reaction) return 0;
    const ld = Math.max(DRIVER.minLookahead, DRIVER.lookaheadS * u);
    const alpha = Math.atan2(-y, ld) - psi; // bearing of the lookahead point on the lane centre (y = 0)
    const target = Math.atan((2 * wheelbase * Math.sin(alpha)) / ld);
    filt += ((target - filt) * dt) / DRIVER.tau;
    const maxStep = ((DRIVER.rateDeg * Math.PI) / 180) * dt;
    const lim = (DRIVER.maxDeg * Math.PI) / 180;
    cmd += Math.max(-maxStep, Math.min(maxStep, filt - cmd));
    cmd = Math.max(-lim, Math.min(lim, cmd));
    return cmd;
  };
}

// The driver's right foot, also the same in both runs. It holds the throttle still through the swerve and while the
// trailer sways (the usual advice: don't accelerate into sway). Once the rig has straightened out (controller off, hitch
// angle and trailer yaw rate small for a second), it accelerates back to the set speed like a driver resuming cruise,
// limited by engine power. If the trailer starts swaying again, it goes back to holding the throttle.
export const THROTTLE = {
  settleHitch: 1.0, // deg: straightened out below this hitch angle...
  settleYaw: 2.0, // deg/s: ...and this trailer yaw rate...
  settleTime: 1.0, // s: ...held this long
  swayHitch: 3.0, // deg: sway again above this (or the controller engaging): hold the throttle
  accel: 1.5, // m/s^2: the most it asks for (firm, for a pickup pulling 3 t)
  jerk: 1.0, // m/s^3: how quickly it gets on and off the throttle
  kp: 0.4, // 1/s: acceleration asked for per m/s below the set speed
  powerKW: 220, // at the wheels
};

// ---------------------------------------------------------------- sway controller (demo re-implementation)
// Watches the trailer's yaw rate at 100 Hz over a 4 s window. Three checks must agree: it oscillates (enough zero
// crossings), it has energy (yaw-rate RMS over the window and over the last second, so a swing that has already died
// out doesn't count), and the frequency is in the sway band. An asymmetric latch (+1 per agreeing
// tick, -3 otherwise, engage at 8, release at 3) keeps one-off bumps from triggering it. While engaged, it brakes the
// trailer in proportion to the sway energy. Thresholds here are demo values.
// Tuned by searching the whole slider range (tongue weight -5% to 15%, 60 to 140 km/h) for settings where every
// controlled run stays out of a jackknife, stays on the road and settles. Saving the worst corner (a tail-heavy trailer
// at 140 km/h) takes early, firm braking, so these thresholds also brake briefly after any sharp swerve at speed.
// (Re-checked with the THROTTLE driver over 25 s: no jackknife or road exit anywhere; at 135-140 km/h with a light or
// tail-heavy trailer, 9 settings start swaying again once back at speed, which is the physics, not the controller.)
export const CONTROLLER = {
  windowN: 400, // 4 s at 100 Hz
  deadband: 0.5, // deg/s: zero crossings are counted outside this band
  minCrossings: 1,
  rmsMin: 6.0, // deg/s over the window
  recentN: 100, // last 1 s
  recentMin: 3.0, // deg/s over the last second
  fMin: 0.2, // Hz
  fMax: 1.5,
  gain: 0.12, // brake per deg/s of RMS
  maxBrake: 1.0, // fraction of the trailer's maximum braking
  rate: 5.0, // brake change per second
};

export function makeController() {
  const cfg = { ...CONTROLLER };
  const N = cfg.windowN;
  const buf = new Float32Array(N);
  let n = 0;
  let head = 0;
  let count = 0;
  let engaged = false;
  let brake = 0;

  return function step(yawRateDeg, dt) {
    buf[head] = yawRateDeg;
    head = (head + 1) % N;
    n = Math.min(n + 1, N);

    let sumsq = 0;
    let recentsq = 0;
    let crossings = 0;
    let lastSign = 0;
    for (let i = 0; i < n; i++) {
      const v = buf[(head - n + i + N) % N];
      sumsq += v * v;
      if (i >= n - cfg.recentN) recentsq += v * v;
      const s = v > cfg.deadband ? 1 : v < -cfg.deadband ? -1 : 0;
      if (s !== 0) {
        if (lastSign !== 0 && s !== lastSign) crossings++;
        lastSign = s;
      }
    }
    const rms = Math.sqrt(sumsq / Math.max(n, 1));
    const recent = Math.sqrt(recentsq / Math.max(Math.min(n, cfg.recentN), 1));
    const freq = crossings / (2 * (n / 100));
    const osc = crossings >= cfg.minCrossings;
    const energy = rms >= cfg.rmsMin && recent >= cfg.recentMin;
    const band = freq >= cfg.fMin && freq <= cfg.fMax;

    count = osc && energy && band ? Math.min(count + 1, 30) : Math.max(count - 3, 0);
    if (!engaged && count >= 8) engaged = true;
    if (engaged && count <= 3) engaged = false;

    const target = engaged ? Math.min(cfg.gain * rms, cfg.maxBrake) : 0;
    const maxStep = cfg.rate * dt;
    brake += Math.max(-maxStep, Math.min(maxStep, target - brake));
    return { brake, engaged, count, osc, energy, band, rms, freq };
  };
}

// ---------------------------------------------------------------- simulation
export function simulate(params, { controlled, driver = true }) {
  const p = { ...DEFAULTS, ...params };
  const { truck: T, trailer: R } = buildVehicle(p);
  const u0 = p.speedKmh / 3.6;

  // Static axle loads, including the tongue load the trailer puts on the truck's hitch.
  const Ft = (R.m * G * R.e) / R.l2;
  const L = T.a + T.b;
  const Fzf = (T.m * G * T.b - Ft * (T.c - T.b)) / L;
  const Fzr = T.m * G + Ft - Fzf;
  const Fzt = R.m * G - Ft;

  // Drive force at the truck's rear axle. It starts at whatever holds the set speed (see THROTTLE).
  const M = T.m + R.m;
  const resist = (u) => 0.5 * RHO * (T.CdA + R.CdA) * u * u + CRR * M * G;
  let Fdrive = resist(u0);
  let resuming = false; // accelerating back to (or holding) the set speed
  let calm = 0; // how long the rig has been straight [s]
  let aCmd = 0; // acceleration the driver is asking for while resuming [m/s^2]

  // State: positions, velocities (world frame), headings, yaw rates.
  const s = {
    x1: 0, y1: 0, vx1: u0, vy1: 0, p1: 0, w1: 0,
    x2: -T.c - R.d, y2: 0, vx2: u0, vy2: 0, p2: 0, w2: 0,
  };

  const ctrl = controlled ? makeController() : null;
  const drive = driver ? makeDriver(T.a + T.b) : null;
  let steerDriver = 0;
  let cs = { brake: 0, engaged: false, count: 0, osc: false, energy: false, band: false, rms: 0, freq: 0 };
  let ay2 = 0;

  const frames = [];
  const steps = Math.round(p.duration / DT);
  // The swerve is sized so the rig moves about one lane at any speed (lateral travel grows with speed squared), as a
  // driver dodging the same object would. impulseDeg is the size at 120 km/h.
  const ampDeg = Math.min(4, Math.max(0.8, p.impulseDeg * (120 / p.speedKmh) ** 2));
  const amp = (ampDeg * Math.PI) / 180;
  let jackknife = null;

  for (let k = 0; k <= steps; k++) {
    const t = k * DT;
    if (ctrl && k % CTRL_EVERY === 0) cs = ctrl((s.w2 * 180) / Math.PI, DT * CTRL_EVERY);
    if (drive && k % CTRL_EVERY === 0) {
      const u1w = s.vx1 * Math.cos(s.p1) + s.vy1 * Math.sin(s.p1);
      steerDriver = drive(t, s.y1, s.p1, u1w, DT * CTRL_EVERY);
    }
    if (k % CTRL_EVERY === 0 && t > IMPULSE_T0 + IMPULSE_T) {
      const dtc = DT * CTRL_EVERY;
      const u = Math.max(s.vx1 * Math.cos(s.p1) + s.vy1 * Math.sin(s.p1), 0.5);
      const hitchDeg = Math.abs((wrapPi(s.p2 - s.p1) * 180) / Math.PI);
      const yawDeg = Math.abs((s.w2 * 180) / Math.PI);
      const straight = !cs.engaged && cs.brake < 0.01 && hitchDeg < THROTTLE.settleHitch && yawDeg < THROTTLE.settleYaw;
      calm = straight ? calm + dtc : 0;
      if (resuming && (cs.engaged || hitchDeg > THROTTLE.swayHitch)) {
        resuming = false; // swaying again: back off to a steady throttle and hold it
        Fdrive = Math.min(Fdrive, resist(u));
      } else if (!resuming && calm >= THROTTLE.settleTime) {
        resuming = true;
        aCmd = (Fdrive - resist(u)) / M; // pick up from the current throttle, no jump
      }
      if (resuming) {
        const want = Math.max(-0.3, Math.min(THROTTLE.accel, THROTTLE.kp * (u0 - u)));
        aCmd += Math.max(-THROTTLE.jerk * dtc, Math.min(THROTTLE.jerk * dtc, want - aCmd));
        Fdrive = Math.max(0, Math.min(resist(u) + M * aCmd, (THROTTLE.powerKW * 1000) / u));
      }
    }
    const delta = steerImpulse(t, amp) + steerDriver;

    // Body-frame velocities.
    const c1 = Math.cos(s.p1), s1 = Math.sin(s.p1);
    const c2 = Math.cos(s.p2), s2 = Math.sin(s.p2);
    const u1 = s.vx1 * c1 + s.vy1 * s1, v1 = -s.vx1 * s1 + s.vy1 * c1;
    const u2 = s.vx2 * c2 + s.vy2 * s2, v2 = -s.vx2 * s2 + s.vy2 * c2;
    const uu1 = Math.max(u1, 0.5), uu2 = Math.max(u2, 0.5);

    // Truck tires.
    const af = delta - Math.atan2(v1 + T.a * s.w1, uu1);
    const ar = -Math.atan2(v1 - T.b * s.w1, uu1);
    const Fyf = tireForce(T.Cf, af, Fzf);
    const Fyr = tireForce(T.Cr, ar, Fzr); // simplification: drive force doesn't use up rear-tire grip
    const drag1 = 0.5 * RHO * T.CdA * u1 * Math.abs(u1) + CRR * T.m * G * Math.sign(u1);
    const fbx1 = Fdrive - Fyf * Math.sin(delta) - drag1;
    const fby1 = Fyf * Math.cos(delta) + Fyr;
    const M1 = T.a * Fyf * Math.cos(delta) - T.b * Fyr;

    // Trailer tires and brakes.
    const Fxt = -cs.brake * MU_BRAKE * Fzt * Math.sign(u2);
    const at = -Math.atan2(v2 - R.e * s.w2, uu2);
    const Fyt = tireForce(R.Ct, at, Fzt, Fxt);
    const drag2 = 0.5 * RHO * R.CdA * u2 * Math.abs(u2) + CRR * R.m * G * Math.sign(u2);
    const fbx2 = Fxt - drag2;
    const fby2 = Fyt;
    const M2 = -R.e * Fyt;

    // World-frame applied forces.
    const F1x = fbx1 * c1 - fby1 * s1, F1y = fbx1 * s1 + fby1 * c1;
    const F2x = fbx2 * c2 - fby2 * s2, F2y = fbx2 * s2 + fby2 * c2;

    // Hitch constraint. r1, r2: CG-to-hitch vectors; q = perpendicular of r.
    const r1x = -T.c * c1, r1y = -T.c * s1;
    const r2x = R.d * c2, r2y = R.d * s2;
    const q1x = -r1y, q1y = r1x, q2x = -r2y, q2y = r2x;
    const Cx = s.x1 + r1x - (s.x2 + r2x), Cy = s.y1 + r1y - (s.y2 + r2y);
    const Cdx = s.vx1 + s.w1 * q1x - (s.vx2 + s.w2 * q2x);
    const Cdy = s.vy1 + s.w1 * q1y - (s.vy2 + s.w2 * q2y);
    const wn = 60, zb = 1;
    const bx = -2 * zb * wn * Cdx - wn * wn * Cx, by = -2 * zb * wn * Cdy - wn * wn * Cy;
    const freeX = F1x / T.m + (q1x * M1) / T.I - s.w1 * s.w1 * r1x - (F2x / R.m + (q2x * M2) / R.I - s.w2 * s.w2 * r2x);
    const freeY = F1y / T.m + (q1y * M1) / T.I - s.w1 * s.w1 * r1y - (F2y / R.m + (q2y * M2) / R.I - s.w2 * s.w2 * r2y);
    const im = 1 / T.m + 1 / R.m;
    const k11 = im + (q1x * q1x) / T.I + (q2x * q2x) / R.I;
    const k12 = (q1x * q1y) / T.I + (q2x * q2y) / R.I;
    const k22 = im + (q1y * q1y) / T.I + (q2y * q2y) / R.I;
    const rx = bx - freeX, ry = by - freeY;
    const det = k11 * k22 - k12 * k12;
    const lx = (k22 * rx - k12 * ry) / det;
    const ly = (-k12 * rx + k11 * ry) / det;

    // Accelerations (truck gets +lambda, trailer -lambda).
    const a1x = (F1x + lx) / T.m, a1y = (F1y + ly) / T.m;
    const al1 = (M1 + q1x * lx + q1y * ly) / T.I;
    const a2x = (F2x - lx) / R.m, a2y = (F2y - ly) / R.m;
    const al2 = (M2 - q2x * lx - q2y * ly) / R.I;
    ay2 = -a2x * s2 + a2y * c2; // trailer lateral acceleration, hitch force included

    const hitch = wrapPi(s.p2 - s.p1);
    if (k % SAMPLE_EVERY === 0) {
      frames.push({
        t, x1: s.x1, y1: s.y1, p1: s.p1, x2: s.x2, y2: s.y2, p2: s.p2,
        hitch: (hitch * 180) / Math.PI, speed: u1 * 3.6, delta: (delta * 180) / Math.PI,
        yaw2: (s.w2 * 180) / Math.PI, ay2: ay2 / G,
        brake: cs.brake, engaged: cs.engaged, count: cs.count, osc: cs.osc, energy: cs.energy, band: cs.band,
        resuming,
      });
    }
    if (Math.abs(hitch) > (55 * Math.PI) / 180) {
      jackknife = t;
      break;
    }

    // Semi-implicit Euler.
    s.vx1 += a1x * DT; s.vy1 += a1y * DT; s.w1 += al1 * DT;
    s.vx2 += a2x * DT; s.vy2 += a2y * DT; s.w2 += al2 * DT;
    s.x1 += s.vx1 * DT; s.y1 += s.vy1 * DT; s.p1 += s.w1 * DT;
    s.x2 += s.vx2 * DT; s.y2 += s.vy2 * DT; s.p2 += s.w2 * DT;
  }

  return { frames, jackknife, vehicle: { truck: T, trailer: R }, params: p };
}

// Damping ratio from the hitch-angle peaks after the steering impulse: fit ln|peak| = sigma t + c, take the damped
// frequency from the peak spacing, zeta = -sigma / sqrt(sigma^2 + wd^2). Null when there are too few peaks to fit.
export function dampingRatio(frames, tStart = 2.0) {
  const pk = [];
  for (let i = 1; i < frames.length - 1; i++) {
    const f = frames[i];
    if (f.t < tStart) continue;
    const a = Math.abs(frames[i - 1].hitch), b = Math.abs(f.hitch), c = Math.abs(frames[i + 1].hitch);
    if (b > a && b >= c && b > 0.05) pk.push({ t: f.t, v: b });
  }
  if (pk.length < 3) return null;
  const n = pk.length;
  const mt = pk.reduce((s, q) => s + q.t, 0) / n;
  const ml = pk.reduce((s, q) => s + Math.log(q.v), 0) / n;
  let num = 0, den = 0;
  for (const q of pk) { num += (q.t - mt) * (Math.log(q.v) - ml); den += (q.t - mt) ** 2; }
  const sigma = num / den;
  const halfPeriod = (pk[n - 1].t - pk[0].t) / (n - 1); // |angle| peaks come every half period
  const wd = Math.PI / halfPeriod;
  return -sigma / Math.sqrt(sigma * sigma + wd * wd);
}

// The tire the rigs swerve around: in the middle lane, a little right of centre. Its position along the road is chosen
// from the simulated runs: the first spot where the truck and the trailer of both runs pass it with at least 0.6 m to
// spare (the trailer cuts inside the truck's line, so both are checked), or the roomiest spot if none does.
export const TIRE = { y: -0.4, r: 0.45 };

export function placeObstacle(runs) {
  const half = (run) => [
    { x: (r) => r.x1, y: (r) => r.y1, p: (r) => r.p1, front: run.vehicle.truck.a + 0.9, back: run.vehicle.truck.c, w: 1.0 },
    { x: (r) => r.x2, y: (r) => r.y2, p: (r) => r.p2, front: run.vehicle.trailer.d, back: 6.5 - (run.vehicle.trailer.d - 1.3), w: 1.2 },
  ];
  const clearanceAt = (xo) => {
    let worst = Infinity;
    for (const run of runs) {
      for (const b of half(run)) {
        for (const f of run.frames) {
          const dx = xo - b.x(f);
          const along = dx / Math.max(Math.cos(b.p(f)), 0.2);
          if (along > b.front || along < -b.back) continue; // this body isn't alongside the tire in this frame
          const yb = b.y(f) + dx * Math.tan(b.p(f));
          worst = Math.min(worst, Math.abs(yb - TIRE.y) - b.w - TIRE.r);
        }
      }
    }
    return worst;
  };
  const truckPath = runs[0].frames.filter((f) => f.t > 1 && f.t < 6);
  const x0 = truckPath.find((f) => f.y1 > 1.0)?.x1 ?? truckPath[0].x1;
  const peak = truckPath.reduce((a, f) => (f.y1 > a.y1 ? f : a), truckPath[0]);
  let best = { x: peak.x1, clearance: -Infinity };
  for (let xo = x0; xo <= peak.x1 + 15; xo += 1) {
    const c = clearanceAt(xo);
    if (c >= 0.6) {
      best = { x: xo, clearance: c };
      break;
    }
    if (c > best.clearance) best = { x: xo, clearance: c };
  }
  return { x: best.x, y: TIRE.y, r: TIRE.r, clearance: best.clearance };
}

