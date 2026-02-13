// 简单双精度 D2Q9 格子玻尔兹曼流场模拟
// 左侧定常来流，右侧近似自由出流，鼠标左键拖动添加障碍物

const canvas = document.getElementById("lbmCanvas");
const ctx = canvas.getContext("2d");

// 网格设置（默认值，可通过界面调整）
let NX = 200; // x 方向格点数
let NY = 100; // y 方向格点数

// D2Q9 方向
const Q = 9;
const cx = [0, 1, 0, -1, 0, 1, -1, -1, 1];
const cy = [0, 0, 1, 0, -1, 1, 1, -1, -1];
const w = [4 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 36, 1 / 36, 1 / 36, 1 / 36];
const opp = [0, 3, 4, 1, 2, 7, 8, 5, 6];

// 流体参数（可通过界面调整）
let tau = 0.6; // 松弛时间
let omega = 1 / tau;
let uIn = 0.08; // 左侧入口速度
let rho0 = 1.0; // 参考密度

// 数据数组（会随 NX、NY 改变而重新分配）
let sizeCell;
let sizeDist;
let f;
let fNext;
let solid; // 1 表示障碍物/固体

// 可视化参数
const uMaxVis = 0.15; // 颜色映射最大速度
let stepCounter = 0;
let paused = false;

const gridInfo = document.getElementById("gridInfo");
const stepInfo = document.getElementById("stepInfo");
const pauseBtn = document.getElementById("pauseBtn");
const resetBtn = document.getElementById("resetBtn");
const nxInput = document.getElementById("nxInput");
const nyInput = document.getElementById("nyInput");
const uInInput = document.getElementById("uInInput");
const rhoInput = document.getElementById("rhoInput");
const tauInput = document.getElementById("tauInput");
const applyParamsBtn = document.getElementById("applyParamsBtn");

gridInfo.textContent = `${NX} × ${NY}`;

// 根据当前参数调整画布和数组
function resizeAndAllocate() {
  canvas.width = NX;
  canvas.height = NY;
  sizeCell = NX * NY;
  sizeDist = sizeCell * Q;
  f = new Float32Array(sizeDist);
  fNext = new Float32Array(sizeDist);
  solid = new Uint8Array(sizeCell);
  gridInfo.textContent = `${NX} × ${NY}`;
}

// 工具函数
function idx(x, y) {
  return y * NX + x;
}

function idxQ(x, y, q) {
  return (y * NX + x) * Q + q;
}

function feq(q, rho, ux, uy) {
  const cu = 3 * (cx[q] * ux + cy[q] * uy);
  const u2 = ux * ux + uy * uy;
  return w[q] * rho * (1 + cu + 0.5 * cu * cu - 1.5 * u2);
}

// 初始化分布函数
function initLbm() {
  solid.fill(0);

  // 上下边界设为固体壁面
  for (let x = 0; x < NX; x++) {
    solid[idx(x, 0)] = 1;
    solid[idx(x, NY - 1)] = 1;
  }

  const ux0 = 0.0;
  const uy0 = 0.0;

  for (let y = 0; y < NY; y++) {
    for (let x = 0; x < NX; x++) {
      const isSolid = solid[idx(x, y)];
      const uxInit = isSolid ? 0 : ux0;
      const uyInit = isSolid ? 0 : uy0;
      for (let q = 0; q < Q; q++) {
        const val = feq(q, rho0, uxInit, uyInit);
        f[idxQ(x, y, q)] = val;
        fNext[idxQ(x, y, q)] = val;
      }
    }
  }

  stepCounter = 0;
  stepInfo.textContent = "0";
}

// 碰撞步骤
function collide() {
  for (let y = 0; y < NY; y++) {
    for (let x = 0; x < NX; x++) {
      if (solid[idx(x, y)]) continue;

      let rho = 0;
      let ux = 0;
      let uy = 0;
      const base = idx(x, y) * Q;

      for (let q = 0; q < Q; q++) {
        const fq = f[base + q];
        rho += fq;
        ux += fq * cx[q];
        uy += fq * cy[q];
      }

      if (rho <= 0) rho = 1e-6;
      ux /= rho;
      uy /= rho;

      const u2 = ux * ux + uy * uy;

      for (let q = 0; q < Q; q++) {
        const cu = 3 * (cx[q] * ux + cy[q] * uy);
        const feqVal = w[q] * rho * (1 + cu + 0.5 * cu * cu - 1.5 * u2);
        const idxF = base + q;
        f[idxF] = f[idxF] - omega * (f[idxF] - feqVal);
      }
    }
  }
}

// 迁移 + 反弹边界
function stream() {
  fNext.fill(0);

  for (let y = 0; y < NY; y++) {
    for (let x = 0; x < NX; x++) {
      const cellSolid = solid[idx(x, y)];
      const base = idx(x, y) * Q;

      for (let q = 0; q < Q; q++) {
        const fq = f[base + q];
        if (fq === 0) continue;

        const nxp = x + cx[q];
        const nyp = y + cy[q];

        if (nxp < 0 || nxp >= NX || nyp < 0 || nyp >= NY) {
          // 超出边界，当做固体反弹
          const destBase = base;
          const qq = opp[q];
          fNext[destBase + qq] += fq;
          continue;
        }

        const destSolid = solid[idx(nxp, nyp)];
        if (cellSolid || destSolid) {
          // 在固体内或撞到固体，反弹到当前格点的对向方向
          const destBase = base;
          const qq = opp[q];
          fNext[destBase + qq] += fq;
        } else {
          const destBase = idx(nxp, nyp) * Q;
          fNext[destBase + q] += fq;
        }
      }
    }
  }
}

// 边界条件：左侧入口速度，右侧近似零梯度出口
function applyBoundaries() {
  // 左侧入口：指定水平速度 uIn，重新设定为平衡分布
  for (let y = 1; y < NY - 1; y++) {
    const id = idx(0, y);
    if (solid[id]) continue;
    const base = id * Q;
    for (let q = 0; q < Q; q++) {
      fNext[base + q] = feq(q, rho0, uIn, 0);
    }
  }

  // 右侧出口：拷贝前一列分布函数，近似零梯度
  for (let y = 1; y < NY - 1; y++) {
    const idOut = idx(NX - 1, y);
    const idInner = idx(NX - 2, y);
    if (solid[idOut]) continue;
    const baseOut = idOut * Q;
    const baseInner = idInner * Q;
    for (let q = 0; q < Q; q++) {
      fNext[baseOut + q] = fNext[baseInner + q];
    }
  }

  // 上下边界已经在 solid 中设置为固体，由 migrate 阶段的反弹处理
}

// 单步时间推进
function stepSimulation() {
  // 为了稳定性，每帧可多步
  const stepsPerFrame = 2;

  for (let i = 0; i < stepsPerFrame; i++) {
    collide();
    stream();
    applyBoundaries();

    // 交换 f 和 fNext
    const tmp = f;
    f = fNext;
    fNext = tmp;

    stepCounter++;
  }

  stepInfo.textContent = String(stepCounter);
}

// 速度场可视化
function drawField() {
  const imageData = ctx.createImageData(NX, NY);
  const data = imageData.data;

  for (let y = 0; y < NY; y++) {
    for (let x = 0; x < NX; x++) {
      const id = idx(x, y);
      const base = id * Q;
      const offset = (y * NX + x) * 4;

      if (solid[id]) {
        // 障碍物：黑色
        data[offset] = 0;
        data[offset + 1] = 0;
        data[offset + 2] = 0;
        data[offset + 3] = 255;
        continue;
      }

      let rho = 0;
      let ux = 0;
      let uy = 0;
      for (let q = 0; q < Q; q++) {
        const fq = f[base + q];
        rho += fq;
        ux += fq * cx[q];
        uy += fq * cy[q];
      }
      if (rho <= 0) rho = 1e-6;
      ux /= rho;
      uy /= rho;

      const uMag = Math.sqrt(ux * ux + uy * uy);
      const t = Math.min(uMag / uMaxVis, 1.0);

      // 简单蓝 -> 青 -> 黄 -> 红 渐变
      const r = t < 0.5 ? 0 : Math.round(510 * (t - 0.5));
      const g =
        t < 0.25
          ? Math.round(4 * 255 * t)
          : t < 0.75
          ? 255
          : Math.round(255 * (1 - (t - 0.75) * 4));
      const b = t < 0.5 ? Math.round(255 * (1 - 2 * t)) : 0;

      data[offset] = r;
      data[offset + 1] = g;
      data[offset + 2] = b;
      data[offset + 3] = 255;
    }
  }

  ctx.putImageData(imageData, 0, 0);
}

// 鼠标交互：左键拖动画障碍物
let isDrawing = false;

function handlePointer(ev, drawing) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const x = Math.floor((ev.clientX - rect.left) * scaleX);
  const y = Math.floor((ev.clientY - rect.top) * scaleY);

  if (x < 1 || x >= NX - 1 || y < 1 || y >= NY - 1) return;

  const brushRadius = 2;
  for (let j = -brushRadius; j <= brushRadius; j++) {
    for (let i = -brushRadius; i <= brushRadius; i++) {
      const xx = x + i;
      const yy = y + j;
      if (xx < 1 || xx >= NX - 1 || yy < 1 || yy >= NY - 1) continue;
      if ((i * i + j * j) > brushRadius * brushRadius) continue;
      solid[idx(xx, yy)] = drawing ? 1 : 0;

      // 将新设为固体的格点速度清零，对应分布设为静止平衡
      if (drawing) {
        const base = idx(xx, yy) * Q;
        for (let q = 0; q < Q; q++) {
          const val = feq(q, rho0, 0, 0);
          f[base + q] = val;
          fNext[base + q] = val;
        }
      }
    }
  }
}

canvas.addEventListener("mousedown", (ev) => {
  if (ev.button !== 0) return; // 只处理左键
  isDrawing = true;
  handlePointer(ev, true);
});

canvas.addEventListener("mousemove", (ev) => {
  if (!isDrawing) return;
  handlePointer(ev, true);
});

canvas.addEventListener("mouseup", () => {
  isDrawing = false;
});

canvas.addEventListener("mouseleave", () => {
  isDrawing = false;
});

// 可选：右键擦除障碍物
canvas.addEventListener("contextmenu", (ev) => {
  ev.preventDefault();
});

canvas.addEventListener("mousedown", (ev) => {
  if (ev.button === 2) {
    handlePointer(ev, false);
  }
});

pauseBtn.addEventListener("click", () => {
  paused = !paused;
  pauseBtn.textContent = paused ? "继续" : "暂停";
});

resetBtn.addEventListener("click", () => {
  initLbm();
});

applyParamsBtn.addEventListener("click", () => {
  // 读取并限制参数
  const nxVal = Math.round(parseFloat(nxInput.value));
  const nyVal = Math.round(parseFloat(nyInput.value));
  const uVal = parseFloat(uInInput.value);
  const rhoVal = parseFloat(rhoInput.value);
  const tauVal = parseFloat(tauInput.value);

  const nxClamped = Math.min(Math.max(isFinite(nxVal) ? nxVal : 200, 50), 400);
  const nyClamped = Math.min(Math.max(isFinite(nyVal) ? nyVal : 100, 30), 200);
  const uClamped = Math.min(Math.max(isFinite(uVal) ? uVal : 0.08, 0), 0.3);
  const rhoClamped = Math.min(Math.max(isFinite(rhoVal) ? rhoVal : 1.0, 0.1), 5.0);
  const tauClamped = Math.min(Math.max(isFinite(tauVal) ? tauVal : 0.6, 0.51), 2.0);

  NX = nxClamped;
  NY = nyClamped;
  uIn = uClamped;
  rho0 = rhoClamped;
  tau = tauClamped;
  omega = 1 / tau;

  nxInput.value = NX;
  nyInput.value = NY;
  uInInput.value = uIn.toFixed(3);
  rhoInput.value = rho0.toFixed(2);
  tauInput.value = tau.toFixed(2);

  resizeAndAllocate();
  initLbm();
});

// 主循环
function animate() {
  if (!paused) {
    stepSimulation();
  }
  drawField();
  requestAnimationFrame(animate);
}

// 启动
resizeAndAllocate();
initLbm();
animate();

