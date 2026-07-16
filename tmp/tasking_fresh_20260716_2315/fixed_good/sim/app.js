const CONFIG = {
  dt: 0.01,
  recordIntervalMs: 50,
  gpsPeriodMs: 200,
  timeLookaheadMs: 300,
  pathMaxPoints: 1800,
  minTargetSpeed: 0.20,
  maxTargetSpeed: 0.70,
  rearDutyLimit: 5200,
  rearMinMoveDuty: 1500,
  rearPwmSlewStep: 140,
  steerTargetLimit: 900,
  steerPwmLimit: 6000,
  steerFfGain: 1.0,
  gpsCteLimit: 1.2,
  gpsCteMinSpeed: 0.15,
  finishRemain: 6,
  ekfGateM: 5.0,
  qPos: 0.0025,
  qYaw: 0.000030,
  qGyroBias: 0.0000008,
  qAccBias: 0.000080,
  rYaw: 0.035,
  rStillSpeed: 0.004,
  minCov: 0.000001,
};

const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");

const ui = {
  btnTeach: document.getElementById("btnTeach"),
  btnGuide: document.getElementById("btnGuide"),
  btnReset: document.getElementById("btnReset"),
  gpsToggle: document.getElementById("gpsToggle"),
  noiseToggle: document.getElementById("noiseToggle"),
  speedScale: document.getElementById("speedScale"),
  yawGain: document.getElementById("yawGain"),
  rateGain: document.getElementById("rateGain"),
  cteGain: document.getElementById("cteGain"),
  steerResp: document.getElementById("steerResp"),
  rPos: document.getElementById("rPos"),
  rSpeed: document.getElementById("rSpeed"),
  qSpeed: document.getElementById("qSpeed"),
};

const readouts = {};
for (const id of [
  "hudState", "hudMode", "hudGps", "hudPath", "hudEkf",
  "speedScaleValue", "yawGainValue", "rateGainValue", "cteGainValue", "steerRespValue",
  "rPosValue", "rSpeedValue", "qSpeedValue",
  "mYaw", "mGyro", "mVel", "mIdx", "mHead", "mRate", "mCte", "mTs", "mSteer",
  "mRearPwm", "mSteerPwm", "mDir", "mEkf", "mEr", "mEst", "mTrue"
]) {
  readouts[id] = document.getElementById(id);
}

const keys = new Set();
let state = "READY";
let elapsedMs = 0;
let lastRecordMs = -Infinity;
let lastGpsMs = -Infinity;
let path = [];
let nearestIndex = 0;
let targetIndex = 0;
let rearPwmRamped = 0;
let steerIntegral = 0;
let speedIntegral = 0;
let lastSteerErr = 0;
let lastSpeedErr = 0;
let gpsPending = null;
let manualAccCmd = 0;

const trueCar = {
  x: -4,
  y: 0,
  yawDeg: 0,
  speed: 0,
  acc: 0,
  gyroZ: 0,
};

const car = {
  x: -4,
  y: 0,
  yawDeg: 0,
  gyroZ: 0,
  speed: 0,
  targetSpeed: 0,
  acc: 0,
  steerEnc: 0,
  steerTarget: 0,
  rearPwm: 0,
  steerPwm: 0,
  headingError: 0,
  yawRateError: 0,
  cte: 0,
  dir: 0,
  replayS: 0,
};

const ekf = {
  x: [0, 0, 0, 0, 0, 0],
  p: Array.from({ length: 6 }, () => Array(6).fill(0)),
  gpsCount: 0,
  posResidual: 0,
  speedResidual: 0,
  yawResidual: 0,
};

const world = { scale: 62, ox: 0, oy: 0 };

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function wrapDeg(angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

function angle360(angle) {
  while (angle >= 360) angle -= 360;
  while (angle < 0) angle += 360;
  return angle;
}

function signSpeed(speed) {
  if (speed > 0.06) return 1;
  if (speed < -0.06) return -1;
  return 0;
}

function normalNoise(std) {
  if (!ui.noiseToggle.checked) return 0;
  let u = 0;
  let v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v) * std;
}

function pid(target, current, store, kp, ki, kd, integralLimit, outputLimit) {
  const err = target - current;
  store.integral = clamp(store.integral + err, -integralLimit, integralLimit);
  const out = kp * err + ki * store.integral + kd * (err - store.last);
  store.last = err;
  return clamp(out, -outputLimit, outputLimit);
}

function worldToScreen(x, y) {
  return { x: world.ox + x * world.scale, y: world.oy - y * world.scale };
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  world.ox = rect.width * 0.5;
  world.oy = rect.height * 0.56;
}

function ekfSyncOutputs() {
  car.x = ekf.x[0];
  car.y = ekf.x[1];
  car.yawDeg = angle360(ekf.x[2] * 180 / Math.PI);
  car.speed = clamp(ekf.x[3], -1.5, 1.5);
}

function ekfInit(x, y, yawDeg, speed) {
  ekf.x = [x, y, yawDeg * Math.PI / 180, speed, 0, 0];
  ekf.p = Array.from({ length: 6 }, () => Array(6).fill(0));
  ekf.p[0][0] = 0.25;
  ekf.p[1][1] = 0.25;
  ekf.p[2][2] = 0.030;
  ekf.p[3][3] = 0.20;
  ekf.p[4][4] = 0.010;
  ekf.p[5][5] = 0.25;
  ekf.gpsCount = 0;
  ekf.posResidual = 0;
  ekf.speedResidual = 0;
  ekf.yawResidual = 0;
  ekfSyncOutputs();
}

function matMul(a, b) {
  const out = Array.from({ length: a.length }, () => Array(b[0].length).fill(0));
  for (let i = 0; i < a.length; i++) {
    for (let j = 0; j < b[0].length; j++) {
      for (let k = 0; k < b.length; k++) out[i][j] += a[i][k] * b[k][j];
    }
  }
  return out;
}

function transpose(a) {
  return a[0].map((_, c) => a.map(row => row[c]));
}

function ekfPredict(gyroDps, accMps2) {
  const dt = CONFIG.dt;
  const yaw = ekf.x[2];
  const v = ekf.x[3];
  const acc = accMps2 - ekf.x[5];
  const vMid = v + 0.5 * acc * dt;
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);

  ekf.x[0] += vMid * cy * dt;
  ekf.x[1] += vMid * sy * dt;
  ekf.x[2] = wrapDeg((ekf.x[2] + (gyroDps - ekf.x[4]) * Math.PI / 180 * dt) * 180 / Math.PI) * Math.PI / 180;
  ekf.x[3] = clamp(ekf.x[3] + acc * dt, -1.5, 1.5);

  const f = Array.from({ length: 6 }, (_, i) => Array.from({ length: 6 }, (_, j) => i === j ? 1 : 0));
  f[0][2] = -vMid * sy * dt;
  f[0][3] = cy * dt;
  f[0][5] = -0.5 * cy * dt * dt;
  f[1][2] = vMid * cy * dt;
  f[1][3] = sy * dt;
  f[1][5] = -0.5 * sy * dt * dt;
  f[2][4] = -Math.PI / 180 * dt;
  f[3][5] = -dt;

  const q = [CONFIG.qPos, CONFIG.qPos, CONFIG.qYaw, Number(ui.qSpeed.value), CONFIG.qGyroBias, CONFIG.qAccBias];
  const fp = matMul(f, ekf.p);
  ekf.p = matMul(fp, transpose(f));
  for (let i = 0; i < 6; i++) ekf.p[i][i] = Math.max(ekf.p[i][i] + q[i], CONFIG.minCov);
  ekfSyncOutputs();
}

function ekfUpdateScalar(h, innovation, r) {
  const ph = Array(6).fill(0);
  let s = r;
  const oldP = ekf.p.map(row => row.slice());
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 6; j++) ph[i] += ekf.p[i][j] * h[j];
    s += h[i] * ph[i];
  }
  if (s < CONFIG.minCov) return;
  const gain = ph.map(v => v / s);
  for (let i = 0; i < 6; i++) ekf.x[i] += gain[i] * innovation;
  ekf.x[2] = wrapDeg(ekf.x[2] * 180 / Math.PI) * Math.PI / 180;
  ekf.x[3] = clamp(ekf.x[3], -1.5, 1.5);
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 6; j++) {
      let hp = 0;
      for (let k = 0; k < 6; k++) hp += h[k] * oldP[k][j];
      ekf.p[i][j] = oldP[i][j] - gain[i] * hp;
    }
    ekf.p[i][i] = Math.max(ekf.p[i][i], CONFIG.minCov);
  }
  ekfSyncOutputs();
}

function ekfUpdateGps(gps) {
  const dx = gps.x - ekf.x[0];
  const dy = gps.y - ekf.x[1];
  ekf.posResidual = Math.hypot(dx, dy);
  if (ekf.posResidual < CONFIG.ekfGateM) {
    ekfUpdateScalar([1, 0, 0, 0, 0, 0], dx, Number(ui.rPos.value));
    ekfUpdateScalar([0, 1, 0, 0, 0, 0], gps.y - ekf.x[1], Number(ui.rPos.value));
  }

  let dir = signSpeed(ekf.x[3]);
  if (!dir) dir = car.dir < 0 ? -1 : 1;
  const signedSpeed = gps.speed * dir;
  ekf.speedResidual = signedSpeed - ekf.x[3];
  ekfUpdateScalar([0, 0, 0, 1, 0, 0], ekf.speedResidual, Number(ui.rSpeed.value));

  if (gps.speed > 0.33) {
    const yawInnovation = wrapDeg(gps.yawDeg - ekf.x[2] * 180 / Math.PI) * Math.PI / 180;
    ekf.yawResidual = yawInnovation * 180 / Math.PI;
    ekfUpdateScalar([0, 0, 1, 0, 0, 0], yawInnovation, CONFIG.rYaw);
  }
  ekf.gpsCount++;
}

function maybeGenerateGps() {
  if (!ui.gpsToggle.checked || elapsedMs - lastGpsMs < CONFIG.gpsPeriodMs) return;
  lastGpsMs = elapsedMs;
  gpsPending = {
    x: trueCar.x + normalNoise(0.65),
    y: trueCar.y + normalNoise(0.65),
    speed: Math.abs(trueCar.speed) + normalNoise(0.04),
    yawDeg: angle360(trueCar.yawDeg + normalNoise(3.0)),
  };
}

function resetCar() {
  elapsedMs = 0;
  nearestIndex = 0;
  targetIndex = 0;
  rearPwmRamped = 0;
  steerIntegral = 0;
  speedIntegral = 0;
  lastSteerErr = 0;
  lastSpeedErr = 0;
  lastGpsMs = -Infinity;
  gpsPending = null;
  manualAccCmd = 0;
  Object.assign(trueCar, { x: -4, y: 0, yawDeg: 0, speed: 0, acc: 0, gyroZ: 0 });
  Object.assign(car, {
    x: -4, y: 0, yawDeg: 0, gyroZ: 0, speed: 0, targetSpeed: 0, acc: 0,
    steerEnc: 0, steerTarget: 0, rearPwm: 0, steerPwm: 0,
    headingError: 0, yawRateError: 0, cte: 0, dir: 0, replayS: 0,
  });
  ekfInit(-4, 0, 0, 0);
}

function clearAll() {
  path = [];
  state = "READY";
  resetCar();
}

function startRecord() {
  path = [];
  resetCar();
  state = "REC";
  lastRecordMs = -Infinity;
}

function finishRecord() {
  state = path.length >= 20 ? "READY" : "REC";
  car.targetSpeed = 0;
  car.rearPwm = 0;
}

function startGuide() {
  if (path.length < 20) return;
  resetCar();
  state = "GUIDE";
}

function stopGuide() {
  state = "READY";
  car.targetSpeed = 0;
  car.rearPwm = 0;
  car.steerPwm = 0;
}

function pushPoint() {
  if (elapsedMs - lastRecordMs < CONFIG.recordIntervalMs) return;
  if (path.length >= CONFIG.pathMaxPoints) {
    state = "FULL";
    return;
  }
  const last = path[path.length - 1];
  const ds = last ? Math.hypot(car.x - last.x, car.y - last.y) : 0;
  const s = last ? last.s + ds : 0;
  let speed = car.speed;
  if (Math.abs(speed) < CONFIG.minTargetSpeed) speed = CONFIG.minTargetSpeed;
  path.push({
    x: car.x,
    y: car.y,
    s,
    yawDeg: car.yawDeg,
    yawRateDps: car.gyroZ,
    speedMps: clamp(speed, -CONFIG.maxTargetSpeed, CONFIG.maxTargetSpeed),
    steerCount: car.steerEnc,
    timeMs: elapsedMs,
    gpsValid: ui.gpsToggle.checked,
  });
  lastRecordMs = elapsedMs;
}

function findTimeIndex(timeMs) {
  const start = clamp(nearestIndex, 0, Math.max(0, path.length - 1));
  for (let i = start; i < path.length; i++) {
    if (path[i].timeMs >= timeMs) return i;
  }
  return path.length - 1;
}

function findNearestIndex() {
  const start = clamp(nearestIndex - 20, 0, path.length - 1);
  const end = clamp(nearestIndex + 80, 0, path.length);
  let best = nearestIndex;
  let bestD2 = Infinity;
  for (let i = start; i < end; i++) {
    const dx = path[i].x - car.x;
    const dy = path[i].y - car.y;
    const d2 = dx * dx + dy * dy;
    if (d2 < bestD2) {
      bestD2 = d2;
      best = i;
    }
  }
  car.cte = Math.sqrt(bestD2);
  return best;
}

function signedCrossTrack(index) {
  const p = path[index];
  if (!p) return 0;
  const yaw = p.yawDeg * Math.PI / 180;
  const dx = car.x - p.x;
  const dy = car.y - p.y;
  return clamp(-Math.sin(yaw) * dx + Math.cos(yaw) * dy, -CONFIG.gpsCteLimit, CONFIG.gpsCteLimit);
}

function slewPwm(target) {
  const delta = clamp(target - rearPwmRamped, -CONFIG.rearPwmSlewStep, CONFIG.rearPwmSlewStep);
  rearPwmRamped += delta;
  return Math.round(rearPwmRamped);
}

function speedToPwm(targetSpeed, currentSpeed) {
  const dir = signSpeed(targetSpeed);
  if (!dir) return slewPwm(0);
  const store = { integral: speedIntegral, last: lastSpeedErr };
  const out = pid(Math.abs(targetSpeed), Math.abs(currentSpeed), store, 2600, 0, 120, 3, CONFIG.rearDutyLimit);
  speedIntegral = store.integral;
  lastSpeedErr = store.last;
  let pwm = out + 2800 * Math.abs(targetSpeed);
  pwm = clamp(pwm, 0, CONFIG.rearDutyLimit);
  if (pwm > 0 && pwm < CONFIG.rearMinMoveDuty) pwm = CONFIG.rearMinMoveDuty;
  return slewPwm(pwm * dir);
}

function updateManual() {
  const throttle = (keys.has("KeyW") ? 1 : 0) - (keys.has("KeyS") ? 1 : 0);
  const steer = (keys.has("KeyD") ? 1 : 0) - (keys.has("KeyA") ? 1 : 0);
  car.rearPwm = 0;
  manualAccCmd = throttle * 1.45;
  if (keys.has("Space")) {
    trueCar.speed *= 0.88;
    manualAccCmd = 0;
  }
  car.steerTarget = steer * 520;
}

function updateGuide() {
  if (path.length < 20) {
    state = "ERROR";
    return;
  }

  const gpsMode = ui.gpsToggle.checked && path.some(p => p.gpsValid);
  if (gpsMode) {
    nearestIndex = findNearestIndex();
    targetIndex = findTimeIndex(path[nearestIndex].timeMs + CONFIG.timeLookaheadMs);
  } else {
    nearestIndex = findTimeIndex(elapsedMs);
    targetIndex = findTimeIndex(elapsedMs + CONFIG.timeLookaheadMs);
    car.cte = 0;
  }

  const ref = path[nearestIndex];
  const target = path[targetIndex];
  let refSpeed = ref.speedMps * Number(ui.speedScale.value);
  let dir = signSpeed(refSpeed);
  if (!dir) {
    dir = 1;
    refSpeed = CONFIG.minTargetSpeed * Number(ui.speedScale.value);
  }
  car.targetSpeed = clamp(refSpeed, -CONFIG.maxTargetSpeed, CONFIG.maxTargetSpeed);
  car.dir = dir;
  car.headingError = wrapDeg(target.yawDeg - car.yawDeg);
  car.yawRateError = ref.yawRateDps - car.gyroZ;

  let cteSigned = 0;
  if (gpsMode && Math.abs(car.targetSpeed) > CONFIG.gpsCteMinSpeed) {
    cteSigned = signedCrossTrack(nearestIndex);
    car.cte = Math.abs(cteSigned);
  }

  car.steerTarget = clamp(
    ref.steerCount * CONFIG.steerFfGain +
    car.headingError * Number(ui.yawGain.value) +
    car.yawRateError * Number(ui.rateGain.value) -
    cteSigned * Number(ui.cteGain.value),
    -CONFIG.steerTargetLimit,
    CONFIG.steerTargetLimit
  );

  const steerStore = { integral: steerIntegral, last: lastSteerErr };
  car.steerPwm = Math.round(pid(car.steerTarget, car.steerEnc, steerStore, 6, 0, 0.8, 800, CONFIG.steerPwmLimit));
  steerIntegral = steerStore.integral;
  lastSteerErr = steerStore.last;
  car.rearPwm = speedToPwm(car.targetSpeed, car.speed);

  if (nearestIndex + CONFIG.finishRemain >= path.length) {
    state = "FINISH";
    car.targetSpeed = 0;
    car.rearPwm = 0;
  }
}

function updateVehicle() {
  const dt = CONFIG.dt;
  const motorAcc = state === "GUIDE" ? car.rearPwm / CONFIG.rearDutyLimit * 1.4 : manualAccCmd;
  const drag = trueCar.speed * 0.9;
  trueCar.acc = motorAcc - drag + normalNoise(0.02);
  trueCar.speed += trueCar.acc * dt;
  trueCar.speed = clamp(trueCar.speed, -1.1, 1.3);

  car.steerEnc += (car.steerTarget - car.steerEnc) * Number(ui.steerResp.value);
  const steerNorm = clamp(car.steerEnc / CONFIG.steerTargetLimit, -1, 1);
  trueCar.gyroZ = steerNorm * trueCar.speed * 92;
  trueCar.yawDeg = angle360(trueCar.yawDeg + trueCar.gyroZ * dt);
  const yawRad = trueCar.yawDeg * Math.PI / 180;
  trueCar.x += trueCar.speed * Math.cos(yawRad) * dt;
  trueCar.y += trueCar.speed * Math.sin(yawRad) * dt;

  const imuGyro = trueCar.gyroZ + normalNoise(0.5);
  const imuAcc = trueCar.acc + normalNoise(0.10);
  car.gyroZ = imuGyro;
  car.acc = imuAcc;
  ekfPredict(imuGyro, imuAcc);

  if (Math.abs(imuAcc) < 0.16 && Math.abs(imuGyro) < 0.5 && Math.abs(trueCar.speed) < 0.04) {
    ekfUpdateScalar([0, 0, 0, 1, 0, 0], -ekf.x[3], CONFIG.rStillSpeed);
  }

  maybeGenerateGps();
  if (gpsPending) {
    ekfUpdateGps(gpsPending);
    gpsPending = null;
  }

  if (state === "GUIDE") car.replayS += Math.abs(car.targetSpeed) * dt;
}

function tick() {
  elapsedMs += CONFIG.dt * 1000;
  if (state === "REC") {
    updateManual();
    pushPoint();
    car.steerPwm = 0;
  } else if (state === "GUIDE") {
    updateGuide();
  } else {
    updateManual();
    car.steerPwm = 0;
    car.targetSpeed = 0;
  }
  updateVehicle();
}

function drawGrid(width, height) {
  ctx.fillStyle = "#0d1012";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#1d242a";
  ctx.lineWidth = 1;
  const step = world.scale;
  for (let x = world.ox % step; x < width; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = world.oy % step; y < height; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
}

function drawPath() {
  if (path.length < 2) return;
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#3ec6a8";
  ctx.beginPath();
  path.forEach((p, i) => {
    const s = worldToScreen(p.x, p.y);
    if (i === 0) ctx.moveTo(s.x, s.y);
    else ctx.lineTo(s.x, s.y);
  });
  ctx.stroke();
  if (path[nearestIndex]) drawMarker(path[nearestIndex], "#f1b34c", 5);
  if (path[targetIndex]) drawMarker(path[targetIndex], "#6bb7ff", 6);
}

function drawMarker(p, color, radius) {
  const s = worldToScreen(p.x, p.y);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(s.x, s.y, radius, 0, Math.PI * 2);
  ctx.fill();
}

function drawCones() {
  for (let i = -3; i <= 3; i++) {
    const x = i * 1.5;
    const y = i % 2 === 0 ? 0.9 : -0.9;
    const s = worldToScreen(x, y);
    ctx.fillStyle = "#e45757";
    ctx.beginPath();
    ctx.moveTo(s.x, s.y - 10);
    ctx.lineTo(s.x - 8, s.y + 8);
    ctx.lineTo(s.x + 8, s.y + 8);
    ctx.closePath();
    ctx.fill();
  }
}

function drawCarBody(x, y, yawDeg, color, alpha) {
  const s = worldToScreen(x, y);
  const yaw = -yawDeg * Math.PI / 180;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.translate(s.x, s.y);
  ctx.rotate(yaw);
  ctx.fillStyle = color;
  ctx.strokeStyle = "#15191e";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-18, -10, 36, 20, 4);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = "#3ec6a8";
  ctx.fillRect(6, -8, 8, 16);
  const steer = clamp(car.steerEnc / CONFIG.steerTargetLimit, -1, 1) * 0.5;
  ctx.strokeStyle = "#111418";
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(-12, -12);
  ctx.lineTo(-12, -17);
  ctx.moveTo(-12, 12);
  ctx.lineTo(-12, 17);
  ctx.stroke();
  ctx.rotate(steer);
  ctx.beginPath();
  ctx.moveTo(13, -13);
  ctx.lineTo(13, -20);
  ctx.moveTo(13, 13);
  ctx.lineTo(13, 20);
  ctx.stroke();
  ctx.restore();
}

function render() {
  const rect = canvas.getBoundingClientRect();
  drawGrid(rect.width, rect.height);
  drawCones();
  drawPath();
  drawCarBody(trueCar.x, trueCar.y, trueCar.yawDeg, "#6bb7ff", 0.35);
  drawCarBody(car.x, car.y, car.yawDeg, "#e9eef2", 1);
}

function updateUi() {
  const gpsMode = ui.gpsToggle.checked && path.some(p => p.gpsValid);
  readouts.hudState.textContent = state;
  readouts.hudMode.textContent = gpsMode && state === "GUIDE" ? "EKF+GPS" : "EKF";
  readouts.hudGps.textContent = ui.gpsToggle.checked ? "ON" : "OFF";
  readouts.hudPath.textContent = String(path.length);
  readouts.hudEkf.textContent = String(ekf.gpsCount);

  readouts.speedScaleValue.textContent = Number(ui.speedScale.value).toFixed(2);
  readouts.yawGainValue.textContent = Number(ui.yawGain.value).toFixed(1);
  readouts.rateGainValue.textContent = Number(ui.rateGain.value).toFixed(1);
  readouts.cteGainValue.textContent = Number(ui.cteGain.value).toFixed(0);
  readouts.steerRespValue.textContent = Number(ui.steerResp.value).toFixed(2);
  readouts.rPosValue.textContent = Number(ui.rPos.value).toFixed(2);
  readouts.rSpeedValue.textContent = Number(ui.rSpeed.value).toFixed(2);
  readouts.qSpeedValue.textContent = Number(ui.qSpeed.value).toFixed(3);

  readouts.mYaw.textContent = `${car.yawDeg.toFixed(1)} deg`;
  readouts.mGyro.textContent = `${car.gyroZ.toFixed(1)} dps`;
  readouts.mVel.textContent = `${car.speed.toFixed(2)} m/s`;
  readouts.mIdx.textContent = `${nearestIndex} / ${targetIndex}`;
  readouts.mHead.textContent = `${car.headingError.toFixed(1)} deg`;
  readouts.mRate.textContent = `${car.yawRateError.toFixed(1)} dps`;
  readouts.mCte.textContent = `${car.cte.toFixed(2)} m`;
  readouts.mTs.textContent = `${car.targetSpeed.toFixed(2)} m/s`;
  readouts.mSteer.textContent = `${car.steerTarget.toFixed(0)} / ${car.steerEnc.toFixed(0)}`;
  readouts.mRearPwm.textContent = `${car.rearPwm}`;
  readouts.mSteerPwm.textContent = `${car.steerPwm}`;
  readouts.mDir.textContent = `${car.dir}`;
  readouts.mEkf.textContent = `${ekf.gpsCount}`;
  readouts.mEr.textContent = `${ekf.posResidual.toFixed(2)} m`;
  readouts.mEst.textContent = `${car.x.toFixed(1)}, ${car.y.toFixed(1)}`;
  readouts.mTrue.textContent = `${trueCar.x.toFixed(1)}, ${trueCar.y.toFixed(1)}`;

  ui.btnTeach.classList.toggle("active", state === "REC");
  ui.btnGuide.classList.toggle("active", state === "GUIDE");
  ui.btnTeach.textContent = state === "REC" ? "STOP REC" : "REC";
  ui.btnGuide.textContent = state === "GUIDE" ? "STOP" : "GUIDE";
}

function loop() {
  for (let i = 0; i < 2; i++) tick();
  render();
  updateUi();
  requestAnimationFrame(loop);
}

ui.btnTeach.addEventListener("click", () => {
  if (state === "REC") finishRecord();
  else startRecord();
});

ui.btnGuide.addEventListener("click", () => {
  if (state === "GUIDE") stopGuide();
  else startGuide();
});

ui.btnReset.addEventListener("click", clearAll);
ui.btnReset.classList.add("danger");

window.addEventListener("keydown", event => {
  if (["KeyW", "KeyA", "KeyS", "KeyD", "Space"].includes(event.code)) event.preventDefault();
  if (event.code === "KeyR") state === "REC" ? finishRecord() : startRecord();
  else if (event.code === "KeyG") state === "GUIDE" ? stopGuide() : startGuide();
  else keys.add(event.code);
});

window.addEventListener("keyup", event => keys.delete(event.code));
window.addEventListener("resize", resizeCanvas);

if (!CanvasRenderingContext2D.prototype.roundRect) {
  CanvasRenderingContext2D.prototype.roundRect = function roundRect(x, y, w, h, r) {
    const radius = Math.min(r, Math.abs(w) / 2, Math.abs(h) / 2);
    this.beginPath();
    this.moveTo(x + radius, y);
    this.lineTo(x + w - radius, y);
    this.quadraticCurveTo(x + w, y, x + w, y + radius);
    this.lineTo(x + w, y + h - radius);
    this.quadraticCurveTo(x + w, y + h, x + w - radius, y + h);
    this.lineTo(x + radius, y + h);
    this.quadraticCurveTo(x, y + h, x, y + h - radius);
    this.lineTo(x, y + radius);
    this.quadraticCurveTo(x, y, x + radius, y);
    this.closePath();
  };
}

resizeCanvas();
clearAll();
loop();
