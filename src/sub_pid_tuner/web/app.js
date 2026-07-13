const $ = id => document.getElementById(id);
const colors = ["#55a7ff", "#ffffff", "#25d8ff", "#8d7cff", "#78e6b2", "#f5c451", "#ff7f9f", "#b4c8e8"];
const state = {
  socket: null,
  server: {parameters: {}, signals: {}, profile: {available: false, values: {}}},
  staged: new Map(),
  pending: new Map(),
  selected: new Set(JSON.parse(localStorage.getItem("avbotz-signals") || "[]")),
  history: [],
  paused: false,
  saving: false,
  nodeKey: "",
  parameterKey: "",
  signalKey: "",
  legendKey: "",
  drawPending: false,
  lastTelemetryRender: 0,
  feedTimes: [],
  serverClockOffset: null,
  lastServerTime: null,
  lastArrivalTime: null,
  controlsSynced: false
};
let toastTimer;

function toast(message, error) {
  const el = $("toast");
  el.textContent = message;
  el.className = "show " + (error ? "error" : "");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.className = ""; }, 3200);
}

function setPill(id, text, kind) {
  const el = $(id);
  el.textContent = text;
  el.className = "pill " + (kind || "");
}

function request(type, payload) {
  if (!state.socket || state.socket.readyState !== WebSocket.OPEN) {
    return Promise.reject(new Error("Dashboard is disconnected"));
  }
  const requestId = crypto.randomUUID();
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      state.pending.delete(requestId);
      reject(new Error("The robot did not acknowledge the save in time"));
    }, 10000);
    state.pending.set(requestId, {resolve: resolve, reject: reject, timeout: timeout});
    state.socket.send(JSON.stringify(Object.assign({
      type: type,
      request_id: requestId
    }, payload || {})));
  });
}

function connect() {
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(protocol + "://" + location.host + "/api/ws");
  state.socket = socket;
  socket.onopen = () => setPill("connection", "Connected", "good");
  socket.onclose = () => {
    setPill("connection", "Disconnected", "bad");
    for (const pending of state.pending.values()) {
      clearTimeout(pending.timeout);
      pending.reject(new Error("Dashboard disconnected during the save"));
    }
    state.pending.clear();
    setTimeout(connect, 800);
  };
  socket.onerror = () => socket.close();
  socket.onmessage = event => handleMessage(JSON.parse(event.data));
}

function handleMessage(message) {
  if (message.type === "state") {
    const receivedAt = performance.now() / 1000;
    if (Number.isFinite(message.server_time)) {
      if (state.serverClockOffset === null) state.serverClockOffset = receivedAt - message.server_time;
      state.lastServerTime = message.server_time;
      state.lastArrivalTime = receivedAt;
    }
    state.feedTimes.push(receivedAt);
    while (state.feedTimes.length && state.feedTimes[0] < receivedAt - 2) state.feedTimes.shift();
    const hadParameters = Object.prototype.hasOwnProperty.call(message, "parameters");
    state.server = Object.assign({}, state.server, message);
    updateStatus();
    updateNodeSelect();
    if (hadParameters && state.staged.size === 0 && !$("parameter-list").contains(document.activeElement)) {
      renderParameters(false);
    }
    recordTelemetry(receivedAt, message.server_time);
    renderSignals();
    if (receivedAt - state.lastTelemetryRender >= 0.2) {
      renderTelemetry();
      state.lastTelemetryRender = receivedAt;
    }
    renderFeedRate();
    updateSaveControls();
    return;
  }

  if (message.request_id && state.pending.has(message.request_id)) {
    const pending = state.pending.get(message.request_id);
    state.pending.delete(message.request_id);
    clearTimeout(pending.timeout);
    if (message.type === "error") pending.reject(new Error(message.message));
    else pending.resolve(message);
    return;
  }

  if (message.type === "error") toast(message.message, true);
}

function updateStatus() {
  const s = state.server;
  setPill("robot", "/" + (s.robot_name || "—"), s.demo ? "warn" : "");
  const live = s.telemetry_age !== null && s.telemetry_age !== undefined && s.telemetry_age < 1;
  setPill("telemetry", live ? "Telemetry live" : "Telemetry stale", live ? "good" : "bad");
  setPill("kill", s.killed === null || s.killed === undefined ? "Kill unknown" : s.killed ? "Killed" : "Armed",
    s.killed === null || s.killed === undefined ? "warn" : s.killed ? "good" : "warn");
}

function updateNodeSelect() {
  const nodes = [...new Set([
    ...(state.server.nodes || []),
    ...Object.keys(state.server.parameters || {})
  ])].sort();
  const key = nodes.join("|");
  if (key === state.nodeKey) return;
  state.nodeKey = key;
  const select = $("node-select");
  const previous = select.value;
  select.innerHTML = "";
  for (const node of nodes) {
    const option = document.createElement("option");
    option.value = node;
    option.textContent = node;
    select.append(option);
  }
  const controller = "/" + (state.server.robot_name || "marlin_v2") + "/sub_control";
  if (nodes.includes(previous)) select.value = previous;
  else if (nodes.includes(controller)) select.value = controller;
  state.parameterKey = "";
  renderParameters(true);
}

function stableValue(value) {
  return JSON.stringify(value);
}

function escapeHtml(value) {
  return String(value).replaceAll("&", "&amp;").replaceAll('"', "&quot;").replaceAll("<", "&lt;");
}

function isPid(name, metadata) {
  return /^(pos|vel|att|ang)_pid.[xyz]$/.test(name) &&
    metadata.type === "double_array" && Array.isArray(metadata.value) && metadata.value.length >= 3;
}

function groupName(name) {
  if (name.startsWith("pos_pid.")) return "Position PID";
  if (name.startsWith("vel_pid.")) return "Velocity PID";
  if (name.startsWith("att_pid.")) return "Attitude PID";
  if (name.startsWith("ang_pid.")) return "Angular-rate PID";
  return "Other parameters";
}

function editorHtml(name, metadata, value) {
  if (isPid(name, metadata)) {
    const labels = value.length === 4 ? ["Kp", "Ki", "Kd", "Limit"] : ["Kp", "Ti", "Td"];
    return labels.map((label, index) =>
      '<label class="array-label">' + label +
      '<input data-index="' + index + '" type="number" step="any" value="' +
      escapeHtml(value[index]) + '"></label>'
    ).join("");
  }
  if (metadata.type === "bool") {
    return '<select><option value="true" ' + (value === true ? "selected" : "") +
      '>true</option><option value="false" ' + (value === false ? "selected" : "") +
      '>false</option></select>';
  }
  const isArray = metadata.type.endsWith("_array");
  const numeric = metadata.type === "integer" || metadata.type === "double";
  return '<input ' + (numeric ? 'type="number" step="any" ' : "") + 'value="' +
    escapeHtml(isArray ? JSON.stringify(value) : value === null ? "" : value) + '">';
}

function readRow(row, metadata) {
  if (isPid(row.dataset.name, metadata)) {
    const values = [...row.querySelectorAll(".parameter-editor input")].map(input => Number(input.value));
    if (values.some(value => !Number.isFinite(value))) throw new Error(row.dataset.name + " must contain finite numbers");
    return values;
  }
  const control = row.querySelector(".parameter-editor input, .parameter-editor select");
  if (metadata.type === "bool") return control.value === "true";
  if (metadata.type.endsWith("_array")) {
    const value = JSON.parse(control.value);
    if (!Array.isArray(value)) throw new Error(row.dataset.name + " must be a JSON array");
    return value;
  }
  if (metadata.type === "integer") {
    const value = Number(control.value);
    if (!Number.isInteger(value)) throw new Error(row.dataset.name + " must be an integer");
    return value;
  }
  if (metadata.type === "double") {
    const value = Number(control.value);
    if (!Number.isFinite(value)) throw new Error(row.dataset.name + " must be finite");
    return value;
  }
  return control.value;
}

function metadataForRow(row) {
  return state.server.parameters?.[row.dataset.node]?.[row.dataset.name];
}

function syncRow(row) {
  const metadata = metadataForRow(row);
  if (!metadata || metadata.read_only) return true;
  const key = row.dataset.node + "|" + row.dataset.name;
  try {
    const value = readRow(row, metadata);
    row.classList.remove("invalid");
    if (stableValue(value) === stableValue(metadata.value)) {
      state.staged.delete(key);
      row.classList.remove("dirty");
    } else {
      state.staged.set(key, value);
      row.classList.add("dirty");
    }
    const button = row.querySelector(".apply-one");
    if (button) button.disabled = !state.staged.has(key) || state.saving;
    updateSaveControls();
    return true;
  } catch (error) {
    state.staged.delete(key);
    row.classList.add("dirty", "invalid");
    const button = row.querySelector(".apply-one");
    if (button) button.disabled = true;
    updateSaveControls();
    return false;
  }
}

function harvestVisibleRows() {
  let valid = true;
  for (const row of document.querySelectorAll(".parameter-row:not(.readonly)")) {
    if (!syncRow(row)) valid = false;
  }
  if (!valid) throw new Error("Fix the highlighted invalid value before saving");
}

function profileDiff(node, name, metadata) {
  const profile = state.server.profile || {};
  return node === profile.controller &&
    Object.prototype.hasOwnProperty.call(profile.values || {}, name) &&
    stableValue(profile.values[name]) !== stableValue(metadata.value);
}

function renderParameters(force) {
  const node = $("node-select").value;
  const parameters = state.server.parameters?.[node] || {};
  const filter = $("parameter-filter").value.toLowerCase();
  const modifiedOnly = $("modified-only").checked;
  const signature = JSON.stringify([
    node, filter, modifiedOnly,
    Object.entries(parameters).map(entry => [entry[0], entry[1].value, entry[1].read_only]),
    [...state.staged.entries()]
  ]);
  if (!force && signature === state.parameterKey) return;
  state.parameterKey = signature;

  const root = $("parameter-list");
  root.innerHTML = "";
  let lastGroup = "";
  for (const [name, metadata] of Object.entries(parameters).sort((a, b) => {
    const groupCompare = groupName(a[0]).localeCompare(groupName(b[0]));
    return groupCompare || a[0].localeCompare(b[0]);
  })) {
    const key = node + "|" + name;
    const dirty = state.staged.has(key);
    if (!name.toLowerCase().includes(filter) || (modifiedOnly && !dirty)) continue;
    const group = groupName(name);
    if (group !== lastGroup) {
      const heading = document.createElement("div");
      heading.className = "parameter-group";
      heading.textContent = group;
      root.append(heading);
      lastGroup = group;
    }

    const value = dirty ? state.staged.get(key) : metadata.value;
    const differs = profileDiff(node, name, metadata);
    const row = document.createElement("div");
    row.className = "parameter-row " + (dirty ? "dirty " : "") +
      (metadata.read_only ? "readonly " : "") + (differs ? "profile-diff" : "");
    row.dataset.node = node;
    row.dataset.name = name;
    row.dataset.testid = "parameter-" + name;
    row.innerHTML =
      '<div class="parameter-name"><strong title="' + escapeHtml(name) + '">' + escapeHtml(name) +
      '</strong><small>' + escapeHtml(metadata.type) +
      (metadata.read_only ? " · read only" : "") +
      (differs ? " · differs from profile" : "") +
      '</small></div><div class="parameter-editor">' +
      editorHtml(name, metadata, value) +
      '</div><button class="apply-one" title="Apply this value to the running node" ' +
      (metadata.read_only || !dirty || state.saving ? "disabled" : "") + '>Apply</button>';

    if (!metadata.read_only) {
      row.querySelectorAll("input, select").forEach(control => {
        control.addEventListener("input", () => syncRow(row));
        control.addEventListener("change", () => syncRow(row));
      });
    }
    row.querySelector(".apply-one").addEventListener("click", () => applyRuntime([key]));
    root.append(row);
  }

  if (!root.querySelector(".parameter-row")) {
    root.innerHTML = '<div class="empty">No matching parameters.</div>';
  }
  updateSaveControls();
}

function updateSaveControls() {
  const count = state.staged.size;
  $("edit-count").textContent = count ? count + " staged edit" + (count === 1 ? "" : "s") : "No staged edits";
  $("edit-count").className = "edit-count " + (count ? "active" : "");
  $("apply-all").disabled = state.saving || count === 0 || !state.socket || state.socket.readyState !== WebSocket.OPEN;
  const profile = state.server.profile || {};
  $("permanent-save").disabled = state.saving || !profile.available || !state.socket ||
    state.socket.readyState !== WebSocket.OPEN;
  document.querySelectorAll(".apply-one").forEach(button => {
    const row = button.closest(".parameter-row");
    const key = row.dataset.node + "|" + row.dataset.name;
    button.disabled = state.saving || !state.staged.has(key) || row.classList.contains("invalid");
  });

  const summary = $("config-summary");
  if (state.saving) summary.textContent = "Waiting for ROS acknowledgement and readback…";
  else if (profile.available) summary.textContent =
    "Apply updates the running system now. Save profile writes the verified runtime values to " + profile.path + ".";
  else summary.textContent = "Apply updates the running system now. Persistent profile saving is unavailable for this launch.";
}

function changesForKeys(keys) {
  const changes = [];
  for (const key of keys) {
    if (!state.staged.has(key)) continue;
    const split = key.indexOf("|");
    changes.push({node: key.slice(0, split), name: key.slice(split + 1), value: state.staged.get(key)});
  }
  return changes;
}

function acceptApplied(applied) {
  for (const item of applied || []) {
    const key = item.node + "|" + item.name;
    state.staged.delete(key);
    if (state.server.parameters?.[item.node]?.[item.name]) {
      state.server.parameters[item.node][item.name].value = item.value;
    }
  }
  state.parameterKey = "";
  renderParameters(true);
}

async function applyRuntime(onlyKeys) {
  if (state.saving) {
    toast("A save is already in progress");
    return;
  }
  try {
    harvestVisibleRows();
    const keys = onlyKeys || [...state.staged.keys()];
    const changes = changesForKeys(keys);
    if (!changes.length) {
      toast("No runtime changes to apply");
      return;
    }
    state.saving = true;
    updateSaveControls();
    const result = await request("set_parameters", {changes: changes});
    acceptApplied(result.applied);
    toast(result.applied.length + " value" + (result.applied.length === 1 ? "" : "s") +
      " applied to the running system and verified");
  } catch (error) {
    toast(error.message, true);
  } finally {
    state.saving = false;
    updateSaveControls();
  }
}

async function saveProfile() {
  if (state.saving) {
    toast("A save is already in progress");
    return;
  }
  try {
    harvestVisibleRows();
    const profile = state.server.profile || {};
    if (!profile.available) throw new Error("This launch did not configure a persistent profile");
    const changes = changesForKeys([...state.staged.keys()]);
    state.saving = true;
    updateSaveControls();
    const result = await request("permanent_save", {changes: changes});
    acceptApplied(result.applied);
    state.server.profile = Object.assign({}, profile, {values: result.values, path: result.path});
    toast("Runtime values verified and saved to " + result.path);
  } catch (error) {
    toast(error.message, true);
  } finally {
    state.saving = false;
    updateSaveControls();
  }
}

async function reloadConfiguration() {
  state.staged.clear();
  state.parameterKey = "";
  renderParameters(true);
  try {
    await request("refresh");
    toast("Staged edits discarded; refreshing ROS values");
  } catch (error) {
    toast(error.message, true);
  }
}

function recordTelemetry(receivedAt, serverTime) {
  if (state.paused) return;
  const time = Number.isFinite(serverTime) && state.serverClockOffset !== null ?
    serverTime + state.serverClockOffset : receivedAt;
  state.history.push({time: time, wallTime: Date.now(), signals: Object.assign({}, state.server.signals || {})});
  const cutoff = time - 65;
  while (state.history.length && state.history[0].time < cutoff) state.history.shift();
  scheduleGraphDraw();
}

function scheduleGraphDraw() {
  if (state.drawPending) return;
  state.drawPending = true;
  let completed = false;
  const run = () => {
    if (completed) return;
    completed = true;
    state.drawPending = false;
    drawGraph();
  };
  requestAnimationFrame(run);
  setTimeout(run, 34);
}

function graphNow() {
  if (state.lastServerTime !== null && state.serverClockOffset !== null && state.lastArrivalTime !== null) {
    return state.lastServerTime + state.serverClockOffset + (performance.now() / 1000 - state.lastArrivalTime);
  }
  return performance.now() / 1000;
}

function renderFeedRate() {
  if (state.feedTimes.length < 2) {
    $("feed-rate").textContent = "Waiting for data";
    return;
  }
  const duration = state.feedTimes[state.feedTimes.length - 1] - state.feedTimes[0];
  const hz = duration > 0 ? (state.feedTimes.length - 1) / duration : 0;
  $("feed-rate").textContent = "Live · " + hz.toFixed(0) + " Hz";
}

function renderSignals() {
  const catalog = Object.keys(state.server.signals || {}).sort();
  normalizeSignalSelection(catalog);
  const filter = $("signal-filter").value.toLowerCase();
  const key = catalog.join("|") + "|" + filter + "|" + [...state.selected].join("|");
  if (key === state.signalKey) return;
  state.signalKey = key;
  const root = $("signal-list");
  root.innerHTML = "";
  for (const name of catalog.filter(item => item.includes(filter))) {
    const label = document.createElement("label");
    label.className = "signal-option";
    label.innerHTML = '<input type="checkbox" ' + (state.selected.has(name) ? "checked" : "") +
      '><span title="' + escapeHtml(name) + '">' + escapeHtml(name) + "</span>";
    label.querySelector("input").addEventListener("change", event => {
      if (event.target.checked) state.selected.add(name);
      else state.selected.delete(name);
      localStorage.setItem("avbotz-signals", JSON.stringify([...state.selected]));
      state.signalKey = "";
      scheduleGraphDraw();
    });
    root.append(label);
  }
}

function normalizeSignalSelection(catalog) {
  if (!catalog.length) return;
  const available = new Set(catalog);
  let selectionReset = false;
  if (![...state.selected].some(name => available.has(name))) {
    const defaults = ["target", "current", "error"]
      .map(kind => "pid.position.z." + kind)
      .filter(name => available.has(name));
    state.selected = new Set(defaults.length ? defaults : catalog.slice(0, 3));
    localStorage.setItem("avbotz-signals", JSON.stringify([...state.selected]));
    selectionReset = true;
  }
  if (state.controlsSynced && !selectionReset) return;
  const parsed = [...state.selected]
    .map(name => name.match(/^pid\.(position|velocity|attitude|angular_velocity)\.([xyz])\.(target|current|error)$/))
    .filter(Boolean);
  if (parsed.length && parsed.every(match => match[1] === parsed[0][1] && match[2] === parsed[0][2])) {
    $("loop-select").value = parsed[0][1];
    updateGraphAxisLabels();
    $("axis-select").value = parsed[0][2];
  }
  state.controlsSynced = true;
}

function updateGraphAxisLabels() {
  const rotational = $("loop-select").value === "attitude" ||
    $("loop-select").value === "angular_velocity";
  const values = ["x", "y", "z"];
  const labels = rotational ? ["Roll", "Pitch", "Yaw"] : ["X", "Y", "Z"];
  [...$("axis-select").options].forEach((option, index) => {
    option.value = values[index];
    option.textContent = labels[index];
  });
}

function selectPidPreset() {
  const group = $("loop-select").value;
  const axis = $("axis-select").value;
  state.selected = new Set(["target", "current", "error"].map(kind =>
    "pid." + group + "." + axis + "." + kind));
  localStorage.setItem("avbotz-signals", JSON.stringify([...state.selected]));
  state.signalKey = "";
  renderSignals();
  scheduleGraphDraw();
}

function drawGraph() {
  const canvas = $("chart");
  const rect = canvas.getBoundingClientRect();
  const dpr = devicePixelRatio || 1;
  const width = Math.max(1, rect.width);
  const height = Math.max(1, rect.height);
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#080d16";
  ctx.fillRect(0, 0, width, height);

  const pad = {left: 58, right: 14, top: 16, bottom: 25};
  const plotWidth = width - pad.left - pad.right;
  const plotHeight = height - pad.top - pad.bottom;
  const names = [...state.selected];
  const seconds = Number($("window-select").value);
  const now = graphNow();
  const start = now - seconds;
  const values = [];
  for (const sample of state.history) {
    if (sample.time < start) continue;
    for (const name of names) {
      if (Number.isFinite(sample.signals[name])) values.push(sample.signals[name]);
    }
  }
  const selectedScale = $("scale-select").value;
  let min;
  let max;
  if (selectedScale === "auto") {
    min = values.length ? Math.min(...values) : -1;
    max = values.length ? Math.max(...values) : 1;
    if (Math.abs(max - min) < 1e-9) { min -= 1; max += 1; }
    const margin = (max - min) * 0.1;
    min -= margin;
    max += margin;
  } else if (selectedScale === "custom") {
    const customMin = Number($("scale-min").value);
    const customMax = Number($("scale-max").value);
    if (Number.isFinite(customMin) && Number.isFinite(customMax) && customMin < customMax) {
      min = customMin;
      max = customMax;
      $("custom-scale").classList.remove("invalid");
    } else {
      min = Number(canvas.dataset.yMin) || -1;
      max = Number(canvas.dataset.yMax) || 1;
      $("custom-scale").classList.add("invalid");
    }
  } else {
    const limit = Number(selectedScale);
    min = -limit;
    max = limit;
  }
  canvas.dataset.yMin = String(min);
  canvas.dataset.yMax = String(max);
  const latestSample = state.history[state.history.length - 1];
  canvas.dataset.sampleAgeMs = latestSample ? String(Math.max(0, (now - latestSample.time) * 1000)) : "";
  canvas.dataset.sourceAgeMs = Number.isFinite(state.server.telemetry_age) ?
    String(Math.max(0, state.server.telemetry_age * 1000)) : "";
  canvas.dataset.latestValues = JSON.stringify(Object.fromEntries(names.map(name => [
    name, latestSample?.signals?.[name] ?? null
  ])));
  $("range-readout").textContent = (selectedScale === "auto" ? "Y: auto · " : "Y: ") +
    formatAxisValue(min) + " to " + (max > 0 ? "+" : "") + formatAxisValue(max);

  ctx.strokeStyle = "#20304a";
  ctx.fillStyle = "#8798b3";
  ctx.font = "10px ui-monospace";
  for (let index = 0; index <= 5; index++) {
    const y = pad.top + plotHeight * index / 5;
    const value = max - (max - min) * index / 5;
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(width - pad.right, y);
    ctx.stroke();
    ctx.fillText(value.toPrecision(3), 5, y + 3);
  }

  ctx.fillText("-" + seconds + "s", pad.left, height - 7);
  ctx.fillText("now", width - 36, height - 7);

  names.forEach((name, index) => {
    ctx.save();
    ctx.beginPath();
    ctx.rect(pad.left, pad.top, plotWidth, plotHeight);
    ctx.clip();
    ctx.strokeStyle = colors[index % colors.length];
    ctx.lineWidth = 2;
    ctx.beginPath();
    let begun = false;
    for (const sample of state.history) {
      const value = sample.signals[name];
      if (!Number.isFinite(value)) continue;
      const x = pad.left + (sample.time - start) / seconds * plotWidth;
      const y = pad.top + (max - value) / (max - min) * plotHeight;
      if (!begun) { ctx.moveTo(x, y); begun = true; }
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.restore();
  });
  const legendKey = names.join("|");
  if (legendKey !== state.legendKey) {
    state.legendKey = legendKey;
    $("legend").innerHTML = names.map((name, index) =>
      '<span><i class="legend-dot" style="background:' + colors[index % colors.length] +
      '"></i>' + escapeHtml(name) + "</span>").join("");
  }
}

function formatAxisValue(value) {
  if (Math.abs(value) >= 1000 || (Math.abs(value) > 0 && Math.abs(value) < 0.001)) {
    return value.toExponential(2);
  }
  return Number(value.toFixed(3)).toString();
}

function renderTelemetry() {
  const filter = $("telemetry-filter").value.toLowerCase();
  const signals = state.server.signals || {};
  const root = $("telemetry-list");
  root.innerHTML = "";
  for (const name of Object.keys(signals).sort().filter(item => item.includes(filter))) {
    const row = document.createElement("div");
    row.className = "telemetry-row";
    row.innerHTML = "<span>" + escapeHtml(name) + "</span><span>" +
      Number(signals[name]).toPrecision(7) + "</span>";
    root.append(row);
  }
  if (!root.children.length) root.innerHTML = '<div class="empty">No matching live values.</div>';
}

function updateSetpointLabels() {
  const mode = $("setpoint-mode").value;
  const rotational = mode === "attitude" || mode === "angular_velocity";
  ["axis-a", "axis-b", "axis-c"].forEach((id, index) => {
    $(id).textContent = (rotational ? ["Roll", "Pitch", "Yaw"] : ["X", "Y", "Z"])[index];
  });
  $("altitude-wrap").style.display = mode === "position" ? "flex" : "none";
  $("setpoint-hint").textContent = rotational ?
    "Controller convention: roll/pitch/yaw in radians; positive yaw is counterclockwise." :
    "Controller convention: x forward/world x, y left/world y, z up. Values are metres or m/s.";
}

async function publishSetpoint() {
  const values = [...document.querySelectorAll(".setpoint-value")].map(input => Number(input.value));
  if (values.some(value => !Number.isFinite(value))) {
    toast("Enter three finite setpoint values", true);
    return;
  }
  try {
    const result = await request("publish_setpoint", {
      mode: $("setpoint-mode").value,
      values: values,
      altitude: $("altitude-mode").checked
    });
    const labels = result.mode === "attitude" || result.mode === "angular_velocity" ?
      ["roll", "pitch", "yaw"] : ["x", "y", "z"];
    $("last-command").textContent = result.mode + ": " +
      labels.map((label, index) => label + "=" + Number(result.values[index]).toPrecision(5)).join(", ") +
      (result.altitude ? " · altitude mode" : "");
    toast("Setpoint published");
  } catch (error) {
    toast(error.message, true);
  }
}

function exportCsv() {
  const names = [...state.selected];
  const rows = [["time", ...names].join(",")];
  for (const sample of state.history) {
    rows.push([new Date(sample.wallTime).toISOString(),
      ...names.map(name => sample.signals[name] ?? "")].join(","));
  }
  const url = URL.createObjectURL(new Blob([rows.join(String.fromCharCode(10)) + String.fromCharCode(10)], {type: "text/csv"}));
  const link = document.createElement("a");
  link.href = url;
  link.download = "avbotz-dashboard-" + new Date().toISOString().replaceAll(":", "-") + ".csv";
  link.click();
  URL.revokeObjectURL(url);
}

function saveLayout() {
  const layout = [...document.querySelectorAll(".tile")].map(tile => ({
    id: tile.id, width: tile.dataset.w, height: tile.dataset.h
  }));
  localStorage.setItem("avbotz-layout-v2", JSON.stringify(layout));
}

function sizeTile(tile) {
  tile.style.setProperty("--w", tile.dataset.w);
  tile.style.setProperty("--h", tile.dataset.h);
}

function setupLayout() {
  const root = $("dashboard");
  const saved = JSON.parse(localStorage.getItem("avbotz-layout-v2") || "null");
  if (saved) {
    for (const item of saved) {
      const tile = $(item.id);
      if (tile) {
        tile.dataset.w = item.width;
        tile.dataset.h = item.height;
        root.append(tile);
      }
    }
  }
  document.querySelectorAll(".tile").forEach(tile => {
    sizeTile(tile);
    const title = tile.querySelector(".tile-title");
    title.addEventListener("dragstart", () => tile.classList.add("dragging"));
    title.addEventListener("dragend", () => { tile.classList.remove("dragging"); saveLayout(); });
    tile.addEventListener("dragover", event => {
      event.preventDefault();
      const dragging = document.querySelector(".dragging");
      if (dragging && dragging !== tile) root.insertBefore(dragging, tile);
    });

    const handle = tile.querySelector(".resize-handle");
    handle.addEventListener("pointerdown", event => {
      event.preventDefault();
      handle.setPointerCapture(event.pointerId);
      const startX = event.clientX;
      const startY = event.clientY;
      const startWidth = Number(tile.dataset.w);
      const startHeight = Number(tile.dataset.h);
      const move = moveEvent => {
        tile.dataset.w = Math.max(3, Math.min(12,
          startWidth + Math.round((moveEvent.clientX - startX) / (root.clientWidth / 12))));
        tile.dataset.h = Math.max(3, Math.min(12,
          startHeight + Math.round((moveEvent.clientY - startY) / 84)));
        sizeTile(tile);
        scheduleGraphDraw();
      };
      const up = () => { handle.removeEventListener("pointermove", move); saveLayout(); };
      handle.addEventListener("pointermove", move);
      handle.addEventListener("pointerup", up, {once: true});
    });
  });
}

$("node-select").addEventListener("change", () => {
  try { harvestVisibleRows(); } catch (error) { toast(error.message, true); }
  state.parameterKey = "";
  renderParameters(true);
  request("load_parameters", {node: $("node-select").value}).catch(error => toast(error.message, true));
});
$("parameter-filter").addEventListener("input", () => {
  try { harvestVisibleRows(); } catch (_error) {}
  state.parameterKey = "";
  renderParameters(true);
});
$("modified-only").addEventListener("change", () => {
  try { harvestVisibleRows(); } catch (_error) {}
  state.parameterKey = "";
  renderParameters(true);
});
$("reload").addEventListener("click", reloadConfiguration);
$("apply-all").addEventListener("click", () => applyRuntime());
$("permanent-save").addEventListener("click", saveProfile);
$("signal-filter").addEventListener("input", () => { state.signalKey = ""; renderSignals(); });
$("pid-preset").addEventListener("click", selectPidPreset);
$("loop-select").addEventListener("change", () => {
  updateGraphAxisLabels();
  selectPidPreset();
});
$("axis-select").addEventListener("change", selectPidPreset);
$("pause").addEventListener("click", () => {
  state.paused = !state.paused;
  $("pause").textContent = state.paused ? "Resume" : "Pause";
});
$("clear").addEventListener("click", () => { state.history = []; scheduleGraphDraw(); });
$("window-select").addEventListener("change", scheduleGraphDraw);
const savedGraphScale = localStorage.getItem("avbotz-graph-scale") || "auto";
$("scale-select").value = [...$("scale-select").options].some(option => option.value === savedGraphScale) ?
  savedGraphScale : "auto";
$("scale-select").addEventListener("change", () => {
  localStorage.setItem("avbotz-graph-scale", $("scale-select").value);
  $("custom-scale").hidden = $("scale-select").value !== "custom";
  scheduleGraphDraw();
});
const savedCustomScale = JSON.parse(localStorage.getItem("avbotz-custom-scale") || "null");
if (savedCustomScale && Number.isFinite(savedCustomScale.min) && Number.isFinite(savedCustomScale.max) && savedCustomScale.min < savedCustomScale.max) {
  $("scale-min").value = savedCustomScale.min;
  $("scale-max").value = savedCustomScale.max;
}
$("custom-scale").hidden = $("scale-select").value !== "custom";
function updateCustomScale() {
  $("scale-select").value = "custom";
  $("custom-scale").hidden = false;
  const min = Number($("scale-min").value);
  const max = Number($("scale-max").value);
  if (Number.isFinite(min) && Number.isFinite(max) && min < max) {
    localStorage.setItem("avbotz-graph-scale", "custom");
    localStorage.setItem("avbotz-custom-scale", JSON.stringify({min: min, max: max}));
  }
  scheduleGraphDraw();
}
$("scale-min").addEventListener("input", updateCustomScale);
$("scale-max").addEventListener("input", updateCustomScale);
function zoomGraph(factor) {
  const canvas = $("chart");
  const min = Number(canvas.dataset.yMin);
  const max = Number(canvas.dataset.yMax);
  const center = (min + max) / 2;
  const half = Math.max((max - min) * factor / 2, 1e-9);
  $("scale-min").value = Number((center - half).toPrecision(8));
  $("scale-max").value = Number((center + half).toPrecision(8));
  updateCustomScale();
}
$("zoom-in").addEventListener("click", () => zoomGraph(0.67));
$("zoom-out").addEventListener("click", () => zoomGraph(1.5));
$("telemetry-filter").addEventListener("input", renderTelemetry);
$("setpoint-mode").addEventListener("change", updateSetpointLabels);
$("publish-setpoint").addEventListener("click", publishSetpoint);
$("export").addEventListener("click", exportCsv);
$("reset-layout").addEventListener("click", () => {
  localStorage.removeItem("avbotz-layout-v2");
  location.reload();
});
document.addEventListener("keydown", event => {
  if ((event.ctrlKey || event.metaKey) && !event.altKey && event.key.toLowerCase() === "s") {
    event.preventDefault();
    if (event.shiftKey) saveProfile();
    else applyRuntime();
  }
});
window.addEventListener("resize", scheduleGraphDraw);

setupLayout();
updateGraphAxisLabels();
updateSetpointLabels();
updateSaveControls();
connect();
