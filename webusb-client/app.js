const GS_USB_FILTERS = [
  { vendorId: 0x1d50, productId: 0x606f },
  { vendorId: 0x1209, productId: 0x606f },
  { vendorId: 0x1209, productId: 0x81fe },
  { vendorId: 0x1cd2, productId: 0x606f },
  { vendorId: 0x16d0, productId: 0x10b8 }
];

const GS_USB_BREQ = {
  bittiming: { request: 1, len: 20 },
  mode: { request: 2, len: 8 },
  bt_const: { request: 4, len: 40 }
};

const GS_DEVICE_FLAGS = {
  listenOnly: 0x01,
  loopBack: 0x02,
  oneShot: 0x08,
  hwTimeStamp: 0x10
};

const EFF_FLAG = 0x80000000;
const WATCHED_PGNS = new Set([127257, 127252]);
const PGN_LABELS = {
  127257: "Attitude (127257)",
  127252: "Heave (127252)"
};
const SPECIAL_SOURCE_LABELS = {
  101: "CM7 (101)",
  102: "ICM (102)"
};

class GSUsbWeb {
  constructor() {
    this.device = null;
    this.running = false;
    this.frameLength = 24;
    this.lastTiming = null;
    this.lastCaps = null;
  }

  async connect(bitrate) {
    this.device = await navigator.usb.requestDevice({ filters: GS_USB_FILTERS });
    await this.device.open();
    await this.device.reset();

    if (!this.device.configuration) {
      await this.device.selectConfiguration(1);
    }
    await this.device.claimInterface(0);

    this.capabilities = await this.#controlRead(GS_USB_BREQ.bt_const);
    const caps = this.#decodeCapabilities(this.capabilities);
    this.lastCaps = caps;
    const features = caps.features;

    this.lastTiming = await this.#setBitrate(bitrate, caps);

    const flags = features & GS_DEVICE_FLAGS.hwTimeStamp;
    this.frameLength = (flags & GS_DEVICE_FLAGS.hwTimeStamp) === GS_DEVICE_FLAGS.hwTimeStamp ? 24 : 20;

    const mode = new DataView(new ArrayBuffer(8));
    mode.setUint32(0, 1, true);
    mode.setUint32(4, flags, true);
    await this.#controlWrite(GS_USB_BREQ.mode, mode.buffer);

    this.running = true;
  }

  async disconnect() {
    if (!this.device) return;
    this.running = false;

    const mode = new DataView(new ArrayBuffer(8));
    mode.setUint32(0, 0, true);
    mode.setUint32(4, 0, true);

    try {
      await this.#controlWrite(GS_USB_BREQ.mode, mode.buffer);
    } catch {
      // best effort
    }

    try {
      await this.device.releaseInterface(0);
    } catch {
      // best effort
    }

    await this.device.close();
    this.device = null;
  }

  async readLoop(onFrame, onError) {
    while (this.running && this.device?.opened) {
      try {
        const result = await this.device.transferIn(1, this.frameLength);
        if (result.status !== "ok" || !result.data) continue;

        const frame = parseFrame(result.data, this.frameLength);
        onFrame(frame);
      } catch (err) {
        onError(err);
      }
    }
  }

  async #setBitrate(bitrate, caps) {
    const timing = this.#findBitTiming(bitrate, caps);
    if (!timing) {
      throw new Error(`No valid bit timing found for bitrate ${bitrate} with CAN clock ${caps.clock}`);
    }

    const out = new DataView(new ArrayBuffer(20));
    out.setUint32(0, timing.propSeg, true);
    out.setUint32(4, timing.phaseSeg1, true);
    out.setUint32(8, timing.phaseSeg2, true);
    out.setUint32(12, timing.sjw, true);
    out.setUint32(16, timing.brp, true);

    await this.#controlWrite(GS_USB_BREQ.bittiming, out.buffer);
    return timing;
  }

  #decodeCapabilities(data) {
    return {
      features: data.getUint32(0, true),
      clock: data.getUint32(4, true),
      tseg1Min: data.getUint32(8, true),
      tseg1Max: data.getUint32(12, true),
      tseg2Min: data.getUint32(16, true),
      tseg2Max: data.getUint32(20, true),
      sjwMax: data.getUint32(24, true),
      brpMin: data.getUint32(28, true),
      brpMax: data.getUint32(32, true),
      brpInc: data.getUint32(36, true)
    };
  }

  #findBitTiming(targetBitrate, caps) {
    const propSeg = 1;
    const inc = Math.max(1, caps.brpInc || 1);
    let best = null;

    for (let brp = caps.brpMin; brp <= caps.brpMax; brp += inc) {
      for (let phaseSeg1 = caps.tseg1Min; phaseSeg1 <= caps.tseg1Max; phaseSeg1 += 1) {
        for (let phaseSeg2 = caps.tseg2Min; phaseSeg2 <= caps.tseg2Max; phaseSeg2 += 1) {
          if (phaseSeg2 > phaseSeg1) {
            continue;
          }

          const totalTq = 1 + propSeg + phaseSeg1 + phaseSeg2;
          const actualBitrate = caps.clock / (brp * totalTq);
          const relError = Math.abs((actualBitrate - targetBitrate) / targetBitrate);
          const samplePoint = (1 + propSeg + phaseSeg1) / totalTq;
          const sampleErr = Math.abs(samplePoint - 0.875);

          if (
            !best ||
            relError < best.relError ||
            (relError === best.relError && sampleErr < best.sampleErr)
          ) {
            best = {
              propSeg,
              phaseSeg1,
              phaseSeg2,
              sjw: Math.max(1, Math.min(caps.sjwMax, phaseSeg2, 2)),
              brp,
              actualBitrate,
              relError,
              samplePoint,
              sampleErr
            };
          }
        }
      }
    }

    if (!best) {
      return null;
    }

    return best;
  }

  async #controlRead(req) {
    const result = await this.device.controlTransferIn(
      {
        requestType: "vendor",
        recipient: "interface",
        request: req.request,
        value: 0,
        index: 0
      },
      req.len
    );

    if (result.status !== "ok" || !result.data) {
      throw new Error(`controlTransferIn failed: ${result.status}`);
    }

    return result.data;
  }

  async #controlWrite(req, buffer) {
    const result = await this.device.controlTransferOut(
      {
        requestType: "vendor",
        recipient: "interface",
        request: req.request,
        value: 0,
        index: 0
      },
      buffer
    );

    if (result.status !== "ok") {
      throw new Error(`controlTransferOut failed: ${result.status}`);
    }
  }
}

function parseFrame(dataView, frameLength) {
  const canId = dataView.getUint32(4, true);
  const dlc = dataView.getUint8(8);
  const payload = [];
  for (let i = 0; i < Math.min(dlc, 8); i += 1) {
    payload.push(dataView.getUint8(12 + i));
  }

  const header = parseN2KHeader(canId);

  return {
    canId,
    dlc,
    payload,
    pgn: header?.pgn,
    source: header?.source,
    priority: header?.priority,
    timestampUs: frameLength === 24 ? dataView.getUint32(20, true) : null
  };
}

function parseN2KHeader(canId) {
  if ((canId & EFF_FLAG) === 0) {
    return null;
  }

  const canIdUnsigned = canId >>> 0;
  const pduFormat = (canIdUnsigned >>> 16) & 0xff;
  const pduSpecific = (canIdUnsigned >>> 8) & 0xff;
  const dataPage = (canIdUnsigned >>> 24) & 0x01;

  return {
    source: canIdUnsigned & 0xff,
    priority: (canIdUnsigned >>> 26) & 0x7,
    pgn: pduFormat < 240 ? (dataPage << 16) | (pduFormat << 8) : (dataPage << 16) | (pduFormat << 8) | pduSpecific
  };
}

const connectBtn = document.getElementById("connect");
const disconnectBtn = document.getElementById("disconnect");
const clearBtn = document.getElementById("clear");
const statusEl = document.getElementById("status");
const countEl = document.getElementById("count");
const logEl = document.getElementById("log");
const latestRowsEl = document.getElementById("latestRows");
const rawRowsEl = document.getElementById("rawFrames");
const rawSectionEl = document.getElementById("rawSection");
const bitrateEl = document.getElementById("bitrate");
const showRawEl = document.getElementById("showRaw");
const heavePlotEl = document.getElementById("heavePlot");
const pitchPlotEl = document.getElementById("pitchPlot");
const rollPlotEl = document.getElementById("rollPlot");

let driver = null;
let frameCount = 0;
const latestBySourceAndPgn = new Map();
let ageTimerId = null;
const PLOT_WINDOW_MS = 60000;

function log(msg) {
  const line = `[${new Date().toLocaleTimeString()}] ${msg}`;
  logEl.textContent = `${line}\n${logEl.textContent}`.slice(0, 4000);
}

function formatNumber(value, digits = 4) {
  if (value === null || value === undefined || Number.isNaN(value)) return "-";
  return value.toFixed(digits);
}

function formatSource(source) {
  return SPECIAL_SOURCE_LABELS[source] ?? String(source);
}

function formatPgn(pgn) {
  return PGN_LABELS[pgn] ?? String(pgn);
}

function formatAgeMs(ageMs) {
  const sec = Math.max(0, Math.floor(ageMs / 1000));
  return `${sec} s`;
}

function formatValueCell(row) {
  if (row.pgn === 127257) {
    const yawDeg = row.decoded?.yawRad != null ? (row.decoded.yawRad * 180) / Math.PI : null;
    const pitchDeg = row.decoded?.pitchRad != null ? (row.decoded.pitchRad * 180) / Math.PI : null;
    const rollDeg = row.decoded?.rollRad != null ? (row.decoded.rollRad * 180) / Math.PI : null;
    return `yaw=${formatNumber(yawDeg, 1)} deg, pitch=${formatNumber(pitchDeg, 1)} deg, roll=${formatNumber(rollDeg, 1)} deg`;
  }
  if (row.pgn === 127252) {
    return `heave=${formatNumber(row.decoded?.heaveM, 3)} m`;
  }
  return "-";
}

function formatAvgHz(row) {
  if (!row.periodsMs || row.periodsMs.length === 0) return "-";
  const sum = row.periodsMs.reduce((acc, v) => acc + v, 0);
  const avgPeriodMs = sum / row.periodsMs.length;
  if (avgPeriodMs <= 0) return "-";
  return `${(1000 / avgPeriodMs).toFixed(2)} Hz`;
}

function sourceColor(source) {
  const palette = [
    "#1d4ed8",
    "#dc2626",
    "#7c3aed",
    "#0f766e",
    "#ea580c",
    "#0891b2",
    "#be123c",
    "#4338ca",
    "#65a30d",
    "#b45309"
  ];
  return palette[Math.abs(source) % palette.length];
}

function decodeWatchedPgn(frame) {
  if (frame.pgn === 127257 && frame.payload.length >= 7) {
    const bytes = new Uint8Array(frame.payload);
    const dv = new DataView(bytes.buffer);
    return {
      sid: dv.getUint8(0),
      yawRad: dv.getInt16(1, true) * 0.0001,
      pitchRad: dv.getInt16(3, true) * 0.0001,
      rollRad: dv.getInt16(5, true) * 0.0001
    };
  }
  if (frame.pgn === 127252 && frame.payload.length >= 3) {
    const bytes = new Uint8Array(frame.payload);
    const dv = new DataView(bytes.buffer);
    return {
      sid: dv.getUint8(0),
      heaveM: dv.getInt16(1, true) * 0.01
    };
  }
  return null;
}

function renderLatestTable() {
  latestRowsEl.innerHTML = "";

  const rows = Array.from(latestBySourceAndPgn.values()).sort((a, b) => {
    if (a.source !== b.source) return a.source - b.source;
    return a.pgn - b.pgn;
  });

  for (const row of rows) {
    const ageMs = Date.now() - row.updatedAt;
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${formatSource(row.source)}</td>
      <td>${formatPgn(row.pgn)}</td>
      <td>${row.priority}</td>
      <td><input class="plot-toggle" type="checkbox" data-key="${row.key}" ${row.plotEnabled ? "checked" : ""}></td>
      <td>${formatValueCell(row)}</td>
      <td>${formatAgeMs(ageMs)}</td>
      <td>${formatAvgHz(row)}</td>
    `;
    latestRowsEl.appendChild(tr);
  }
}

function resizeCanvasToDisplaySize(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(canvas.clientWidth * ratio));
  const height = Math.max(1, Math.floor(canvas.clientHeight * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return ctx;
}

function drawSeriesChart(canvas, title, series) {
  const ctx = resizeCanvasToDisplaySize(canvas);
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const pad = { left: 50, right: 16, top: 20, bottom: 26 };

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, width, height);

  const now = Date.now();
  const minT = now - PLOT_WINDOW_MS;
  const plotW = width - pad.left - pad.right;
  const plotH = height - pad.top - pad.bottom;

  const visibleSeries = series
    .map((s) => ({
      ...s,
      points: s.points.filter((p) => p.t >= minT)
    }))
    .filter((s) => s.points.length > 0);

  ctx.strokeStyle = "#dbe5f5";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = pad.top + (plotH * i) / 4;
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(width - pad.right, y);
    ctx.stroke();
  }

  ctx.beginPath();
  ctx.moveTo(pad.left, pad.top);
  ctx.lineTo(pad.left, height - pad.bottom);
  ctx.lineTo(width - pad.right, height - pad.bottom);
  ctx.strokeStyle = "#6b7280";
  ctx.stroke();

  ctx.fillStyle = "#111827";
  ctx.font = "12px IBM Plex Sans, sans-serif";
  ctx.fillText(title, 8, 14);

  if (visibleSeries.length === 0) {
    ctx.fillStyle = "#6b7280";
    ctx.fillText("No selected data", pad.left + 8, pad.top + 18);
    return;
  }

  let yMin = Infinity;
  let yMax = -Infinity;
  for (const s of visibleSeries) {
    for (const p of s.points) {
      yMin = Math.min(yMin, p.v);
      yMax = Math.max(yMax, p.v);
    }
  }
  if (yMin === yMax) {
    yMin -= 1;
    yMax += 1;
  }
  const yPad = (yMax - yMin) * 0.1;
  yMin -= yPad;
  yMax += yPad;

  const xFromT = (t) => pad.left + ((t - minT) / PLOT_WINDOW_MS) * plotW;
  const yFromV = (v) => pad.top + ((yMax - v) / (yMax - yMin)) * plotH;

  for (const s of visibleSeries) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    s.points.forEach((p, idx) => {
      const x = xFromT(p.t);
      const y = yFromV(p.v);
      if (idx === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  ctx.fillStyle = "#4b5563";
  ctx.font = "11px IBM Plex Mono, monospace";
  ctx.fillText(`${yMin.toFixed(2)}`, 4, height - pad.bottom + 3);
  ctx.fillText(`${yMax.toFixed(2)}`, 4, pad.top + 3);
  ctx.fillText("60s", width - pad.right - 20, height - 8);

  let legendY = pad.top + 12;
  for (const s of visibleSeries.slice(0, 8)) {
    ctx.fillStyle = s.color;
    ctx.fillRect(width - pad.right - 170, legendY - 8, 10, 10);
    ctx.fillStyle = "#1f2937";
    ctx.fillText(s.label, width - pad.right - 156, legendY);
    legendY += 14;
  }
}

function renderPlots() {
  const heaveSeries = [];
  const pitchSeries = [];
  const rollSeries = [];
  const rows = Array.from(latestBySourceAndPgn.values());

  for (const row of rows) {
    if (!row.plotEnabled) continue;
    const sourceLabel = formatSource(row.source);

    if (row.pgn === 127252) {
      heaveSeries.push({
        label: `${sourceLabel}`,
        color: sourceColor(row.source),
        points: row.heaveHistory || []
      });
    }

    if (row.pgn === 127257) {
      pitchSeries.push({
        label: `${sourceLabel}`,
        color: sourceColor(row.source),
        points: row.pitchHistory || []
      });
      rollSeries.push({
        label: `${sourceLabel}`,
        color: sourceColor(row.source),
        points: row.rollHistory || []
      });
    }
  }

  drawSeriesChart(heavePlotEl, "Heave (m)", heaveSeries);
  drawSeriesChart(pitchPlotEl, "Pitch (deg)", pitchSeries);
  drawSeriesChart(rollPlotEl, "Roll (deg)", rollSeries);
}

function addRawFrameRow(frame) {
  if (!showRawEl.checked) return;

  const tr = document.createElement("tr");
  tr.innerHTML = `
    <td>${new Date().toLocaleTimeString()}</td>
    <td>0x${frame.canId.toString(16).padStart(8, "0")}</td>
    <td>${frame.pgn ?? "-"}</td>
    <td>${frame.source ?? "-"}</td>
    <td>${frame.priority ?? "-"}</td>
    <td>${frame.dlc}</td>
    <td>${frame.payload.map((b) => b.toString(16).padStart(2, "0")).join(" ")}</td>
  `;

  rawRowsEl.prepend(tr);

  while (rawRowsEl.children.length > 250) {
    rawRowsEl.removeChild(rawRowsEl.lastChild);
  }
}

function onFrame(frame) {
  frameCount += 1;
  countEl.textContent = String(frameCount);
  addRawFrameRow(frame);

  if (!WATCHED_PGNS.has(frame.pgn)) return;

  const decoded = decodeWatchedPgn(frame);
  if (!decoded) return;

  const key = `${frame.source}:${frame.pgn}`;
  const now = Date.now();
  const prev = latestBySourceAndPgn.get(key);
  const periodsMs = prev?.periodsMs ? [...prev.periodsMs] : [];
  if (prev?.updatedAt) {
    periodsMs.push(now - prev.updatedAt);
    if (periodsMs.length > 10) {
      periodsMs.shift();
    }
  }
  const heaveHistory = prev?.heaveHistory ? [...prev.heaveHistory] : [];
  const pitchHistory = prev?.pitchHistory ? [...prev.pitchHistory] : [];
  const rollHistory = prev?.rollHistory ? [...prev.rollHistory] : [];

  if (frame.pgn === 127252 && decoded.heaveM != null) {
    heaveHistory.push({ t: now, v: decoded.heaveM });
  }
  if (frame.pgn === 127257) {
    if (decoded.pitchRad != null) {
      pitchHistory.push({ t: now, v: (decoded.pitchRad * 180) / Math.PI });
    }
    if (decoded.rollRad != null) {
      rollHistory.push({ t: now, v: (decoded.rollRad * 180) / Math.PI });
    }
  }

  const minT = now - PLOT_WINDOW_MS;
  const prune = (arr) => arr.filter((p) => p.t >= minT);

  latestBySourceAndPgn.set(key, {
    key,
    source: frame.source,
    pgn: frame.pgn,
    priority: frame.priority,
    decoded,
    plotEnabled: prev?.plotEnabled ?? true,
    periodsMs,
    heaveHistory: prune(heaveHistory),
    pitchHistory: prune(pitchHistory),
    rollHistory: prune(rollHistory),
    updatedAt: now
  });
  renderPlots();
}

connectBtn.addEventListener("click", async () => {
  if (!navigator.usb) {
    log("WebUSB is not available in this browser.");
    return;
  }

  try {
    driver = new GSUsbWeb();
    const bitrate = Number.parseInt(bitrateEl.value, 10);

    statusEl.textContent = "Connecting...";
    await driver.connect(bitrate);

    statusEl.textContent = "Connected";
    connectBtn.disabled = true;
    disconnectBtn.disabled = false;
    const timing = driver.lastTiming;
    if (timing) {
      log(
        `Connected at ${bitrate} bps; clock=${driver.lastCaps?.clock ?? "?"}; ` +
          `actual=${Math.round(timing.actualBitrate)} brp=${timing.brp} tseg1=${timing.phaseSeg1} tseg2=${timing.phaseSeg2}`
      );
    } else {
      log(`Connected at ${bitrate} bps`);
    }

    driver.readLoop((frame) => onFrame(frame), (err) => log(`Read error: ${err.message ?? err}`));
  } catch (err) {
    statusEl.textContent = "Error";
    log(`Connect failed: ${err.message ?? err}`);
  }
});

disconnectBtn.addEventListener("click", async () => {
  if (!driver) return;

  await driver.disconnect();
  statusEl.textContent = "Disconnected";
  connectBtn.disabled = false;
  disconnectBtn.disabled = true;
  log("Disconnected");
});

clearBtn.addEventListener("click", () => {
  frameCount = 0;
  countEl.textContent = "0";
  latestBySourceAndPgn.clear();
  latestRowsEl.innerHTML = "";
  rawRowsEl.innerHTML = "";
  logEl.textContent = "";
  renderPlots();
});

showRawEl.addEventListener("change", () => {
  rawSectionEl.classList.toggle("hidden", !showRawEl.checked);
});

latestRowsEl.addEventListener("change", (event) => {
  const target = event.target;
  if (!(target instanceof Element)) return;
  if (!target.classList.contains("plot-toggle")) return;
  const key = target.dataset.key;
  if (!key) return;
  const row = latestBySourceAndPgn.get(key);
  if (!row) return;
  row.plotEnabled = Boolean(target.checked);
  latestBySourceAndPgn.set(key, row);
  renderLatestTable();
  renderPlots();
});

ageTimerId = setInterval(() => {
  if (latestBySourceAndPgn.size > 0) {
    renderLatestTable();
    renderPlots();
  }
}, 250);

window.addEventListener("resize", () => {
  renderPlots();
});
