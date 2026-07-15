const $ = id => document.getElementById(id);
const colors = ["#55a7ff", "#ffffff", "#25d8ff", "#8d7cff", "#78e6b2", "#f5c451", "#ff7f9f", "#b4c8e8"];
const savedPathView = localStorage.getItem("avbotz-path-view");
const state = {
  socket: null,
  server: {parameters: {}, signals: {}, profile: {available: false, values: {}}},
  staged: new Map(),
  pending: new Map(),
  selected: new Set(JSON.parse(localStorage.getItem("avbotz-signals") || "[]")),
  history: [],
  paused: false,
  saving: false,
  stoppingAll: false,
  nodeKey: "",
  parameterKey: "",
  signalKey: "",
  legendKey: "",
  drawPending: false,
  lastDrawAt: 0,
  lastTelemetryRender: 0,
  feedTimes: [],
  serverClockOffset: null,
  lastServerTime: null,
  lastArrivalTime: null,
  controlsSynced: false,
  pathTrail: [],
  pathTargetTrail: [],
  pathView: ["xy", "xz", "yz"].includes(savedPathView) ? savedPathView : "xy"
};
let toastTimer;

function duplicateSources(server = state.server) {
  return Array.isArray(server.duplicate_sources) ? server.duplicate_sources : [];
}

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

if (typeof window !== "undefined" && window.crypto && !window.crypto.randomUUID) {
  window.crypto.randomUUID = function () {
    return ([1e7]+-1e3+-4e3+-8e3+-1e11).replace(/[018]/g, c =>
      (c ^ crypto.getRandomValues(new Uint8Array(1))[0] & 15 >> c / 4).toString(16)
    );
  };
}

function request(type, payload) {
  if (!state.socket || state.socket.readyState !== WebSocket.OPEN) {
    return Promise.reject(new Error("Dashboard is disconnected"));
  }
  const requestId = crypto.randomUUID();
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      state.pending.delete(requestId);
      reject(new Error("The robot did not acknowledge the request in time"));
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
  socket.onopen = () => {
    setPill("connection", "Connected", "good");
    $("stop-all").disabled = false;
  };
  socket.onclose = () => {
    setPill("connection", "Disconnected", "bad");
    $("stop-all").disabled = true;
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
    if (message.tracking && !Object.prototype.hasOwnProperty.call(message.tracking, "path_preview") &&
        Array.isArray(state.server.tracking?.path_preview)) {
      message.tracking.path_preview = state.server.tracking.path_preview;
    }
    const wasUnsafe = duplicateSources().length > 0;
    const hadParameters = Object.prototype.hasOwnProperty.call(message, "parameters");
    state.server = Object.assign({}, state.server, message);
    const topologyUnsafe = duplicateSources().length > 0;
    if (topologyUnsafe !== wasUnsafe) state.history = [];
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
    renderTracking();
    renderCharacterization();
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
  const duplicates = duplicateSources(s);
  const live = s.telemetry_age !== null && s.telemetry_age !== undefined &&
    s.telemetry_age < 1 && duplicates.length === 0;
  const telemetryText = duplicates.length ? "Duplicate ROS sources" :
    live ? "Telemetry live" : "Telemetry stale";
  setPill("telemetry", telemetryText, live ? "good" : "bad");
  $("telemetry").title = duplicates.length ?
    "Multiple publishers: " + duplicates.join(", ") :
    "Odometry age: " + formatAge(s.odometry_age) + "; controller error age: " +
      formatAge(s.control_error_age);
  setPill("kill", s.killed === null || s.killed === undefined ? "Kill unknown" : s.killed ? "Killed" : "Armed",
    s.killed === null || s.killed === undefined ? "warn" : s.killed ? "good" : "warn");
  $("stop-all").disabled = state.stoppingAll || !state.socket ||
    state.socket.readyState !== WebSocket.OPEN;
}

function formatAge(value) {
  return Number.isFinite(value) ? Math.round(value * 1000) + " ms" : "unavailable";
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
  const controller = state.server.profile?.controller ||
    "/" + (state.server.robot_name || "marlin_v2") + "/sub_control";
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

const rotationNames = {x: "roll", y: "pitch", z: "yaw"};

function displayName(raw) {
  const parameter = String(raw).match(/^(att_pid|ang_pid)\.([xyz])$/);
  if (parameter) return parameter[1] + "." + rotationNames[parameter[2]];
  const signal = String(raw).match(/^pid\.(attitude|angular_velocity)\.([xyz])\.(.+)$/);
  if (signal) return "pid." + signal[1] + "." + rotationNames[signal[2]] + "." + signal[3];
  return String(raw);
}

function isPid(name, metadata) {
  return /^(pos|vel|att|ang)_pid.[xyz]$/.test(name) &&
    metadata.type === "double_array" && Array.isArray(metadata.value) && metadata.value.length >= 3;
}

function structuredArrayLabels(name, metadata) {
  if (metadata.type !== "double_array" || !Array.isArray(metadata.value)) return null;
  if (name === "reference.position_kp") return ["X", "Y", "Z"];
  if (name === "reference.attitude_kp") return ["Roll", "Pitch", "Yaw"];
  if (/^(reference\.(max_velocity|max_acceleration)|model\.|feedback\.)/.test(name) &&
      metadata.value.length === 6) {
    return ["X", "Y", "Z", "Roll", "Pitch", "Yaw"];
  }
  return null;
}

function groupName(name) {
  if (name.startsWith("pos_pid.")) return "Position PID";
  if (name.startsWith("vel_pid.")) return "Velocity PID";
  if (name.startsWith("att_pid.")) return "Attitude PID";
  if (name.startsWith("ang_pid.")) return "Angular-rate PID";
  if (name.startsWith("reference.")) return "Reference shaping";
  if (name.startsWith("model.")) return "Feedforward model";
  if (name.startsWith("feedback.")) return "Feedback PI";
  if (name.startsWith("spin.")) return "Spin control";
  if (["power_limit", "odom_timeout_s", "cmd_vel_timeout_s", "antiwindup_gain",
      "feedforward_enabled"].includes(name)) return "Controller and safety";
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
  const structuredLabels = structuredArrayLabels(name, metadata);
  if (structuredLabels) {
    return structuredLabels.map((label, index) =>
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
  if (isPid(row.dataset.name, metadata) || structuredArrayLabels(row.dataset.name, metadata)) {
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
    if (!displayName(name).toLowerCase().includes(filter) || (modifiedOnly && !dirty)) continue;
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
    const structuredLabels = structuredArrayLabels(name, metadata);
    const structuredClass = structuredLabels ? " structured-array structured-array-" + structuredLabels.length : "";
    const row = document.createElement("div");
    row.className = "parameter-row " + (dirty ? "dirty " : "") +
      (metadata.read_only ? "readonly " : "") + (differs ? "profile-diff " : "") +
      (structuredLabels ? "structured-row structured-row-" + structuredLabels.length : "");
    row.dataset.node = node;
    row.dataset.name = name;
    row.dataset.testid = "parameter-" + name;
    row.innerHTML =
      '<div class="parameter-name"><strong title="' + escapeHtml(displayName(name)) + '">' + escapeHtml(displayName(name)) +
      '</strong><small>' + escapeHtml(metadata.type) +
      (metadata.read_only ? " · read only" : "") +
      (differs ? " · differs from profile" : "") +
      '</small></div><div class="parameter-editor' + structuredClass + '">' +
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
  if (state.paused || duplicateSources().length) return;
  const time = Number.isFinite(serverTime) && state.serverClockOffset !== null ?
    serverTime + state.serverClockOffset : receivedAt;
  state.history.push({time: time, wallTime: Date.now(), signals: Object.assign({}, state.server.signals || {})});
  const cutoff = time - 65;
  while (state.history.length && state.history[0].time < cutoff) state.history.shift();
  const signals = state.server.signals || {};
  const point = [signals["pid.position.x.current"], signals["pid.position.y.current"], signals["pid.position.z.current"]];
  if (point.every(Number.isFinite)) {
    const previous = state.pathTrail[state.pathTrail.length - 1];
    const moved = !previous || point.some((value, index) => Math.abs(value - previous[index]) > 0.001);
    if (moved) state.pathTrail.push(point);
    if (state.pathTrail.length > 1200) state.pathTrail.splice(0, state.pathTrail.length - 1200);
  }
  const trackingTarget = state.server.tracking?.current_reference;
  const signalTarget = [signals["pid.position.x.target"], signals["pid.position.y.target"], signals["pid.position.z.target"]];
  const targetPoint = Array.isArray(trackingTarget) && trackingTarget.every(Number.isFinite) ?
    trackingTarget : signalTarget;
  const recordingPathTarget = Boolean(state.server.tracking?.active) &&
    pathExperiments.has(state.server.tracking?.experiment);
  if (recordingPathTarget && targetPoint.every(Number.isFinite)) {
    const previousTarget = state.pathTargetTrail[state.pathTargetTrail.length - 1];
    const moved = !previousTarget || targetPoint.some((value, index) => Math.abs(value - previousTarget[index]) > 0.0001);
    if (moved) state.pathTargetTrail.push([...targetPoint]);
    if (state.pathTargetTrail.length > 1200) state.pathTargetTrail.splice(0, state.pathTargetTrail.length - 1200);
  }
  scheduleGraphDraw();
}

function scheduleGraphDraw() {
  if (state.drawPending) return;
  state.drawPending = true;
  const minimumFrameMs = 50;
  const delay = Math.max(0, minimumFrameMs - (performance.now() - state.lastDrawAt));
  const run = () => {
    state.drawPending = false;
    state.lastDrawAt = performance.now();
    drawGraph();
    drawPathViewer();
  };
  if (delay <= 1) requestAnimationFrame(run);
  else setTimeout(() => requestAnimationFrame(run), delay);
}

function graphNow() {
  if (state.lastServerTime !== null && state.serverClockOffset !== null && state.lastArrivalTime !== null) {
    return state.lastServerTime + state.serverClockOffset + (performance.now() / 1000 - state.lastArrivalTime);
  }
  return performance.now() / 1000;
}

function renderFeedRate() {
  if (duplicateSources().length) {
    $("feed-rate").textContent = "Blocked · duplicate ROS sources";
    return;
  }
  if (!Number.isFinite(state.server.telemetry_age) || state.server.telemetry_age >= 1) {
    $("feed-rate").textContent = "Telemetry stale";
    return;
  }
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
  for (const name of catalog.filter(item => displayName(item).toLowerCase().includes(filter))) {
    const label = document.createElement("label");
    label.className = "signal-option";
    label.innerHTML = '<input type="checkbox" ' + (state.selected.has(name) ? "checked" : "") +
      '><span title="' + escapeHtml(displayName(name)) + '">' + escapeHtml(displayName(name)) + "</span>";
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
      '"></i>' + escapeHtml(displayName(name)) + "</span>").join("");
  }
}

function formatAxisValue(value) {
  if (Math.abs(value) >= 1000 || (Math.abs(value) > 0 && Math.abs(value) < 0.001)) {
    return value.toExponential(2);
  }
  return Number(value.toFixed(3)).toString();
}

function projectPathPoint(point, view) {
  const [x, y, z] = point;
  if (view === "xy") return [x, y];
  if (view === "xz") return [x, z];
  return [y, z];
}

function pathOrientationTip(current, view, signals, length) {
  if (view === "xy") {
    const yaw = signals["pid.attitude.z.current"];
    return Number.isFinite(yaw) ?
      [current[0] + Math.cos(yaw) * length, current[1] + Math.sin(yaw) * length, current[2]] : null;
  }
  if (view === "xz") {
    const pitch = signals["pid.attitude.y.current"];
    return Number.isFinite(pitch) ?
      [current[0] + Math.cos(pitch) * length, current[1], current[2] - Math.sin(pitch) * length] : null;
  }
  const roll = signals["pid.attitude.x.current"];
  return Number.isFinite(roll) ?
    [current[0], current[1] + Math.cos(roll) * length, current[2] + Math.sin(roll) * length] : null;
}

function drawPathViewer() {
  const canvas = $("path-canvas");
  if (!canvas) return;
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
  const tracking = state.server.tracking || {};
  const planned = Array.isArray(tracking.path_preview) ?
    tracking.path_preview.filter(point => Array.isArray(point) && point.every(Number.isFinite)) : [];
  const signals = state.server.signals || {};
  const current = [signals["pid.position.x.current"], signals["pid.position.y.current"], signals["pid.position.z.current"]];
  const signalTarget = [signals["pid.position.x.target"], signals["pid.position.y.target"], signals["pid.position.z.target"]];
  const liveReference = Array.isArray(tracking.current_reference) && tracking.current_reference.every(Number.isFinite) ?
    tracking.current_reference : null;
  const recordedReference = state.pathTargetTrail[state.pathTargetTrail.length - 1];
  const target = liveReference || (planned.length && recordedReference ? recordedReference : signalTarget);
  const validCurrent = current.every(Number.isFinite) ? current : null;
  const validTarget = target.every(Number.isFinite) ? target : null;
  const all = [...planned, ...state.pathTargetTrail, ...state.pathTrail];
  if (validCurrent) all.push(validCurrent);
  if (validTarget) all.push(validTarget);

  ctx.fillStyle = "#06111f";
  ctx.fillRect(0, 0, width, height);
  if (!all.length) {
    ctx.fillStyle = "#8798b3";
    ctx.font = "13px system-ui";
    ctx.fillText("Waiting for live position telemetry", 20, 30);
    $("path-status").textContent = "Waiting for position";
    return;
  }

  const projected = all.map(point => projectPathPoint(point, state.pathView));
  let minX = Math.min(...projected.map(point => point[0]));
  let maxX = Math.max(...projected.map(point => point[0]));
  let minY = Math.min(...projected.map(point => point[1]));
  let maxY = Math.max(...projected.map(point => point[1]));
  const deficitX = Math.max(0, 0.5 - (maxX - minX));
  const deficitY = Math.max(0, 0.5 - (maxY - minY));
  minX -= deficitX / 2; maxX += deficitX / 2;
  minY -= deficitY / 2; maxY += deficitY / 2;
  const marginX = Math.max((maxX - minX) * 0.16, 0.08);
  const marginY = Math.max((maxY - minY) * 0.16, 0.08);
  minX -= marginX; maxX += marginX; minY -= marginY; maxY += marginY;
  const plot = {left: 46, top: 25, right: width - 18, bottom: height - 35};
  const plotWidth = Math.max(1, plot.right - plot.left);
  const plotHeight = Math.max(1, plot.bottom - plot.top);
  const scale = Math.max(1, Math.min(plotWidth / (maxX - minX), plotHeight / (maxY - minY)));
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  const screen = point => {
    const projectedPoint = projectPathPoint(point, state.pathView);
    return [plot.left + plotWidth / 2 + (projectedPoint[0] - centerX) * scale,
      plot.top + plotHeight / 2 - (projectedPoint[1] - centerY) * scale];
  };

  const poolGradient = ctx.createLinearGradient(0, plot.top, 0, plot.bottom);
  poolGradient.addColorStop(0, "#0b3555");
  poolGradient.addColorStop(1, "#08253e");
  ctx.fillStyle = poolGradient;
  ctx.fillRect(plot.left, plot.top, plotWidth, plotHeight);
  ctx.strokeStyle = "#1b5378";
  ctx.lineWidth = 1;
  for (let index = 1; index < 8; index++) {
    const x = plot.left + plotWidth * index / 8;
    ctx.beginPath(); ctx.moveTo(x, plot.top); ctx.lineTo(x, plot.bottom); ctx.stroke();
  }
  for (let index = 1; index < 6; index++) {
    const y = plot.top + plotHeight * index / 6;
    ctx.beginPath(); ctx.moveTo(plot.left, y); ctx.lineTo(plot.right, y); ctx.stroke();
  }
  ctx.strokeStyle = "#58aee6";
  ctx.lineWidth = 2;
  ctx.strokeRect(plot.left, plot.top, plotWidth, plotHeight);

  const strokePath = (points, color, lineWidth, dashed) => {
    if (points.length < 2) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.setLineDash(dashed || []);
    ctx.beginPath();
    points.forEach((point, index) => {
      const [x, y] = screen(point);
      if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.restore();
  };
  strokePath(planned, "#72e6a9", 3);
  strokePath(state.pathTargetTrail, "#b4f4cf", 1.5, [5, 4]);
  strokePath(state.pathTrail, "#3195ff", 2.8);

  if (validTarget) {
    const [x, y] = screen(validTarget);
    ctx.fillStyle = "#72e6a9";
    ctx.beginPath();
    ctx.moveTo(x, y - 7); ctx.lineTo(x + 7, y); ctx.lineTo(x, y + 7); ctx.lineTo(x - 7, y); ctx.closePath();
    ctx.fill();
  }
  if (validCurrent) {
    const [x, y] = screen(validCurrent);
    ctx.fillStyle = "#0d2e54";
    ctx.strokeStyle = "#57adff";
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(x, y, 9, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    const tip = pathOrientationTip(validCurrent, state.pathView, signals, Math.max(0.08, 24 / scale));
    if (tip) {
      const [tipX, tipY] = screen(tip);
      ctx.strokeStyle = "#9ed2ff"; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(tipX, tipY); ctx.stroke();
    }
  }

  ctx.fillStyle = "#9fc2dc";
  ctx.font = "11px ui-monospace";
  const axes = state.pathView === "xy" ? "X forward / Y left · yaw" : state.pathView === "xz" ?
    "X forward / Z up · pitch" : "Y left / Z up · roll";
  ctx.fillText(axes + " · metres", plot.left, height - 11);
  ctx.fillText("local pool plane · auto fit", plot.left + 6, plot.top + 16);
  const pathNames = {follower_pid: "Follower PID", square_test: "Square Test", spline_test: "Spline Test"};
  const rate = Number(tracking.reference_rate);
  const manualStatus = tracking.manual_advance ?
    tracking.ready_for_next ? " · press Next corner" : " · target held" : "";
  const governorStatus = tracking.reference_limited ?
    rate < 0.05 ? " · catching up" : " · target " + Math.round(rate * 100) + "% speed" : "";
  $("path-status").textContent = tracking.active && pathNames[tracking.experiment] ?
    pathNames[tracking.experiment] + " · " + Math.round((tracking.progress || 0) * 100) + "% · " +
      (validTarget ? "target live" : "target unavailable") + manualStatus + governorStatus :
    planned.length ? "Last target path" : validTarget ? "Live target and pose" : "Live pose trail";
}

function renderTelemetry() {
  const filter = $("telemetry-filter").value.toLowerCase();
  const signals = state.server.signals || {};
  const root = $("telemetry-list");
  root.innerHTML = "";
  for (const name of Object.keys(signals).sort().filter(item => displayName(item).toLowerCase().includes(filter))) {
    const row = document.createElement("div");
    row.className = "telemetry-row";
    row.innerHTML = "<span>" + escapeHtml(displayName(name)) + "</span><span>" +
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

const experimentNames = {
  minimum_jerk: "Smooth move",
  trapezoid: "Trapezoid cruise",
  step: "Step + settle",
  sine: "Windowed sine",
  hold: "Hold / disturbance",
  follower_pid: "Follower PID loop",
  square_test: "Square Test",
  spline_test: "Spline Test"
};

const pathExperiments = new Set(["follower_pid", "square_test", "spline_test"]);

const trackingDefaults = {
  position: {
    minimum_jerk: {amplitude: 0.5, ramp: 4, hold: 2, cycles: 2},
    step: {amplitude: 0.25, ramp: 3, hold: 3, cycles: 1},
    sine: {amplitude: 0.25, ramp: 8, hold: 3, cycles: 3},
    hold: {amplitude: 0, ramp: 1, hold: 20, cycles: 1},
    follower_pid: {amplitude: 0.5, ramp: 75, hold: 5, cycles: 1, maxError: 0.08},
    square_test: {amplitude: 0.5, ramp: 60, hold: 5, cycles: 1, maxError: 0.05},
    spline_test: {amplitude: 0.5, ramp: 40, hold: 5, cycles: 1, maxError: 0.12}
  },
  velocity: {
    trapezoid: {amplitude: 0.25, ramp: 0.125, hold: 2, cycles: 2},
    step: {amplitude: 0.25, ramp: 3, hold: 3, cycles: 1},
    sine: {amplitude: 0.2, ramp: 8, hold: 3, cycles: 3}
  },
  attitude: {
    minimum_jerk: {amplitude: 0.2, ramp: 4, hold: 2, cycles: 2},
    step: {amplitude: 0.12, ramp: 3, hold: 3, cycles: 1},
    sine: {amplitude: 0.12, ramp: 8, hold: 3, cycles: 3},
    hold: {amplitude: 0, ramp: 1, hold: 20, cycles: 1}
  },
  angular_velocity: {
    trapezoid: {amplitude: 0.3, ramp: 0.15, hold: 2, cycles: 2},
    step: {amplitude: 0.35, ramp: 3, hold: 3, cycles: 1},
    sine: {amplitude: 0.25, ramp: 8, hold: 3, cycles: 3}
  }
};

const trackingHints = {
  minimum_jerk: "Quintic out-and-back move with zero velocity and acceleration at each endpoint.",
  trapezoid: "Uses the selected acceleration slope, cruises, ramps to zero, settles, then repeats in the opposite direction.",
  step: "Tests positive and negative commands separately, returning to zero to settle between them.",
  sine: "Smooth zero-mean tracking with a fade-in/out; shorten the period gradually to expose phase lag.",
  hold: "Captures the measured pose. Gently disturb one axis and watch recovery without commanding a move.",
  follower_pid: "Repeats a slow, smooth planar loop from the measured pose. Tune from both position errors and the pool trail.",
  square_test: "Commands one corner at a time and waits for the vehicle to arrive and settle before enabling Next corner.",
  spline_test: "Runs a slow planar Bezier out and back to validate coupled tracking without target jumps."
};

function updateTrackingHint() {
  const experiment = $("tracking-experiment").value;
  let hint = trackingHints[experiment] || "";
  if (experiment === "trapezoid") {
    const amplitude = Math.abs(Number($("tracking-amplitude").value));
    const slope = Number($("tracking-ramp").value);
    if (Number.isFinite(amplitude) && Number.isFinite(slope) && slope > 0) {
      hint += " Current ramp duration: " + (amplitude / slope).toFixed(2) + " s.";
    }
  }
  if (experiment === "square_test") {
    const arrivalRadius = Number($("tracking-max-error").value);
    if (Number.isFinite(arrivalRadius)) {
      hint += " Arrival requires position error within " + arrivalRadius.toFixed(2) +
        " m and planar speed below 0.05 m/s.";
    }
  } else if (pathExperiments.has(experiment)) {
    const amplitude = Math.abs(Number($("tracking-amplitude").value));
    const traversal = Number($("tracking-ramp").value);
    const maxError = Number($("tracking-max-error").value);
    const lengthFactor = {follower_pid: 5.25, square_test: 4, spline_test: 2.3}[experiment];
    if ([amplitude, traversal, maxError].every(Number.isFinite) && traversal > 0) {
      hint += " Approx. target speed: " + (lengthFactor * amplitude / traversal).toFixed(3) +
        " m/s. The target slows and then pauses as lag approaches " + maxError.toFixed(2) + " m.";
    }
  }
  $("tracking-hint").textContent = hint;
}

function updateTrackingLabels(loadDefaults) {
  const mode = $("tracking-mode").value;
  const experimentSelect = $("tracking-experiment");
  const previousExperiment = experimentSelect.value;
  const experiments = Object.keys(trackingDefaults[mode]);
  experimentSelect.innerHTML = experiments.map(value =>
    '<option value="' + value + '">' + experimentNames[value] + "</option>"
  ).join("");
  experimentSelect.value = experiments.includes(previousExperiment) ? previousExperiment : experiments[0];
  const experiment = experimentSelect.value;
  const pathExperiment = pathExperiments.has(experiment);
  const rotational = mode === "attitude" || mode === "angular_velocity";
  const previousAxis = $("tracking-axis").value;
  if (pathExperiment) {
    $("tracking-axis").innerHTML = '<option value="xy">XY · horizontal</option><option value="xz">XZ · vertical</option><option value="yz">YZ · vertical</option>';
  } else {
    const labels = rotational ? ["Roll", "Pitch", "Yaw"] : ["X", "Y", "Z"];
    $("tracking-axis").innerHTML = labels.map((label, index) =>
      '<option value="' + ["x", "y", "z"][index] + '">' + label + "</option>").join("");
  }
  if ([...$("tracking-axis").options].some(option => option.value === previousAxis)) $("tracking-axis").value = previousAxis;
  const unit = {position: "m", velocity: "m/s", attitude: "rad", angular_velocity: "rad/s"}[mode];
  const limit = {position: 2, velocity: 1, attitude: 0.523598, angular_velocity: 1}[mode];
  $("tracking-amplitude").min = pathExperiment ? "0.05" : -limit;
  $("tracking-amplitude").max = limit;
  $("tracking-amplitude-label").textContent = (pathExperiment ? "Path span" : mode === "position" || mode === "attitude" ? "Travel" : "Command") + " (" + unit + ")";
  $("tracking-amplitude-wrap").hidden = experiment === "hold";
  $("tracking-primary-wrap").hidden = experiment === "hold" || experiment === "square_test";
  $("tracking-max-error-wrap").hidden = !pathExperiment;
  $("tracking-max-error-label").textContent = experiment === "square_test" ? "Arrival radius (m)" : "Max lag (m)";
  $("tracking-cycles-wrap").hidden = experiment === "hold" || experiment === "square_test";
  $("next-waypoint").hidden = experiment !== "square_test";
  const slopeUnit = mode === "angular_velocity" ? "rad/s²" : "m/s²";
  $("tracking-primary-label").textContent = pathExperiment ? "Nominal traversal (s)" : experiment === "minimum_jerk" ? "Move (s)" : experiment === "trapezoid" ? "Ramp slope (" + slopeUnit + ")" : experiment === "step" ? "Step (s)" : "Period (s)";
  $("tracking-ramp").min = pathExperiment ? "4" : experiment === "trapezoid" ? "0.005" : "0.5";
  $("tracking-ramp").max = pathExperiment ? "180" : experiment === "trapezoid" ? "5" : "30";
  $("tracking-ramp").step = pathExperiment ? "1" : experiment === "trapezoid" ? "0.005" : "0.1";
  $("tracking-secondary-label").textContent = experiment === "trapezoid" ? "Cruise / settle (s)" : experiment === "step" ? "Settle (s)" : experiment === "sine" ? "Final settle (s)" : experiment === "hold" ? "Duration (s)" : pathExperiment ? "Home settle (s)" : "Hold (s)";
  if (loadDefaults) {
    const defaults = trackingDefaults[mode][experiment];
    $("tracking-amplitude").value = defaults.amplitude;
    $("tracking-ramp").value = defaults.ramp;
    $("tracking-hold").value = defaults.hold;
    $("tracking-cycles").value = defaults.cycles;
    if (pathExperiment) $("tracking-max-error").value = defaults.maxError;
  }
  updateTrackingHint();
}

function selectTrackingGraph() {
  $("loop-select").value = $("tracking-mode").value;
  updateGraphAxisLabels();
  const experiment = $("tracking-experiment").value;
  if (pathExperiments.has(experiment)) {
    const plane = $("tracking-axis").value;
    const axes = plane.split("");
    state.selected = new Set(axes.map(axis => "pid.position." + axis + ".error"));
    localStorage.setItem("avbotz-signals", JSON.stringify([...state.selected]));
    state.pathView = plane;
    localStorage.setItem("avbotz-path-view", plane);
    document.querySelectorAll("[data-path-view]").forEach(button =>
      button.classList.toggle("active", button.dataset.pathView === plane));
    state.signalKey = "";
    renderSignals();
  } else {
    $("axis-select").value = $("tracking-axis").value;
    selectPidPreset();
  }
  state.history = [];
  state.paused = false;
  $("pause").textContent = "Pause";
  scheduleGraphDraw();
}

async function startTracking() {
  const experiment = $("tracking-experiment").value;
  const amplitude = Number($("tracking-amplitude").value);
  const primary = Number($("tracking-ramp").value);
  const rampTime = experiment === "trapezoid" && primary > 0 ? Math.abs(amplitude) / primary : primary;
  const values = {
    experiment: experiment,
    mode: $("tracking-mode").value,
    axis: $("tracking-axis").value,
    amplitude: amplitude,
    ramp_time: rampTime,
    hold_time: Number($("tracking-hold").value),
    cycles: Number($("tracking-cycles").value),
    max_tracking_error: pathExperiments.has(experiment) ? Number($("tracking-max-error").value) : null
  };
  if (experiment === "trapezoid" && (!Number.isFinite(primary) || primary <= 0)) {
    toast("Ramp slope must be greater than zero", true);
    return;
  }
  if (experiment === "trapezoid" && (rampTime < 0.5 || rampTime > 30)) {
    const minimumSlope = Math.abs(amplitude) / 30;
    const maximumSlope = Math.abs(amplitude) / 0.5;
    toast("For this command, use a ramp slope from " + minimumSlope.toPrecision(3) +
      " to " + maximumSlope.toPrecision(3), true);
    return;
  }
  const numericValues = [values.amplitude, values.ramp_time, values.hold_time, values.cycles];
  if (pathExperiments.has(experiment)) numericValues.push(values.max_tracking_error);
  if (!numericValues.every(Number.isFinite)) {
    toast("Enter finite tracking routine values", true);
    return;
  }
  try {
    const result = await request("start_tracking", values);
    state.server.tracking = result.tracking;
    state.pathTrail = [];
    state.pathTargetTrail = Array.isArray(result.tracking.current_reference) &&
      result.tracking.current_reference.every(Number.isFinite) ? [[...result.tracking.current_reference]] : [];
    selectTrackingGraph();
    renderTracking();
    toast("Tracking routine started");
  } catch (error) {
    toast(error.message, true);
  }
}

async function stopTracking() {
  try {
    const result = await request("stop_tracking");
    state.server.tracking = result.tracking;
    renderTracking();
    toast("Tracking routine stopped; holding measured pose");
  } catch (error) {
    toast(error.message, true);
  }
}

async function advanceTracking() {
  try {
    const result = await request("advance_tracking");
    state.server.tracking = result.tracking;
    renderTracking();
  } catch (error) {
    toast(error.message, true);
  }
}

const characterizationDefaults = {
  x: [0.08, 0.02, 0.06, 3.0],
  y: [0.08, 0.02, 0.06, 3.0],
  z: [0.06, 0.012, 0.045, 3.0],
  roll: [0.08, 0.015, 0.06, 2.0],
  pitch: [0.08, 0.015, 0.06, 2.0],
  yaw: [0.15, 0.03, 0.11, 3.0]
};

function updateCharacterizationLabels(loadDefaults) {
  const axis = $("characterization-axis").value;
  const rotational = ["roll", "pitch", "yaw"].includes(axis);
  const velocityUnit = rotational ? "rad/s" : "m/s";
  const accelerationUnit = rotational ? "rad/s²" : "m/s²";
  $("characterization-amplitude-label").textContent = "Maximum speed (" + velocityUnit + ")";
  $("characterization-slow-label").textContent = "Slow slope (" + accelerationUnit + ")";
  $("characterization-fast-label").textContent = "Fast slope (" + accelerationUnit + ")";
  if (loadDefaults) {
    const defaults = characterizationDefaults[axis];
    $("characterization-amplitude").value = defaults[0];
    $("characterization-slow").value = defaults[1];
    $("characterization-fast").value = defaults[2];
    $("characterization-dwell").value = defaults[3];
  }
}

function selectCharacterizationGraph(axis) {
  const rotational = ["roll", "pitch", "yaw"].includes(axis);
  $("loop-select").value = rotational ? "angular_velocity" : "velocity";
  updateGraphAxisLabels();
  $("axis-select").value = rotational ? {roll: "x", pitch: "y", yaw: "z"}[axis] : axis;
  selectPidPreset();
  state.history = [];
}

async function startCharacterization() {
  const values = {
    axis: $("characterization-axis").value,
    amplitude: Number($("characterization-amplitude").value),
    slow_slope: Number($("characterization-slow").value),
    fast_slope: Number($("characterization-fast").value),
    dwell: Number($("characterization-dwell").value)
  };
  if (!Object.values(values).slice(1).every(Number.isFinite)) {
    toast("Enter finite feedforward identification values", true);
    return;
  }
  try {
    const result = await request("start_characterization", values);
    state.server.characterization = result.characterization;
    selectCharacterizationGraph(values.axis);
    renderCharacterization();
    renderTracking();
    toast("Feedforward identification started");
  } catch (error) {
    toast(error.message, true);
  }
}

async function stopCharacterization() {
  try {
    const result = await request("stop_characterization");
    state.server.characterization = result.characterization;
    renderCharacterization();
    renderTracking();
    toast("Identification stopped; holding measured pose");
  } catch (error) {
    toast(error.message, true);
  }
}

async function stopAll() {
  if (state.stoppingAll) return;
  state.stoppingAll = true;
  $("stop-all").disabled = true;
  try {
    const result = await request("stop_all");
    state.server.tracking = result.tracking;
    state.server.characterization = result.characterization;
    renderTracking();
    renderCharacterization();
    toast(result.message || "All dashboard routines stopped");
  } catch (error) {
    toast(error.message, true);
  } finally {
    state.stoppingAll = false;
    $("stop-all").disabled = !state.socket || state.socket.readyState !== WebSocket.OPEN;
  }
}

async function applyCharacterization() {
  try {
    const result = await request("apply_characterization");
    acceptApplied(result.applied);
    state.server.characterization = result.characterization;
    renderCharacterization();
    toast("Feedforward estimates applied and verified at runtime");
  } catch (error) {
    toast(error.message, true);
  }
}

function characterizationNumber(value) {
  return Number.isFinite(Number(value)) ? Number(value).toPrecision(6) : "—";
}

function renderCharacterization() {
  const characterization = state.server.characterization || {
    active: false, status: "idle", message: "Ready to characterize one body axis"
  };
  const active = Boolean(characterization.active);
  const result = characterization.result;
  const passed = Boolean(result && result.passed);
  const telemetryReady = Number.isFinite(state.server.telemetry_age) && state.server.telemetry_age < 0.5;
  const robotReady = state.server.killed === false && telemetryReady && duplicateSources().length === 0;
  const statusText = active ? "Running" : characterization.status === "complete" ? "Fit passed" :
    characterization.status === "rejected" ? "Fit rejected" :
    characterization.status === "idle" ? "Idle" :
    characterization.status ? characterization.status[0].toUpperCase() + characterization.status.slice(1) : "Idle";
  setPill("characterization-status", statusText,
    active ? "warn" : passed ? "good" : characterization.status === "rejected" ? "bad" : "");
  document.querySelectorAll(".characterization-form input, .characterization-form select").forEach(control => {
    control.disabled = active;
  });
  const trackingActive = Boolean(state.server.tracking?.active);
  $("start-characterization").disabled = active || trackingActive || !robotReady ||
    !state.socket || state.socket.readyState !== WebSocket.OPEN;
  $("stop-characterization").disabled = !active || !state.socket || state.socket.readyState !== WebSocket.OPEN;
  $("apply-characterization").disabled = active || !passed || Boolean(characterization.applied) ||
    !state.socket || state.socket.readyState !== WebSocket.OPEN;
  if (active) {
    const progress = Math.round(Number(characterization.progress || 0) * 100);
    $("characterization-progress").textContent =
      (characterization.phase || "running") + " · " + progress + "% · " +
      Number(characterization.samples || 0) + " samples";
    $("characterization-quality").textContent =
      "command " + Number(characterization.command || 0).toFixed(3);
  } else {
    $("characterization-progress").textContent = characterization.message || "Ready";
    $("characterization-quality").textContent = result ?
      "steady R² " + Number(result.quality.drag_r_squared).toFixed(3) + " · " +
      Math.round(Number(result.quality.rejected_fraction) * 100) + "% rejected" : "No fit yet";
  }
  const root = $("characterization-results");
  root.hidden = !result;
  if (!result) {
    root.innerHTML = "";
    return;
  }
  const coefficients = result.coefficients || {};
  const entries = [
    ["bias / trim", coefficients.trim],
    ["kV · linear drag", coefficients.linear_drag],
    ["kQ · quadratic drag", coefficients.quadratic_drag],
    ["kA · retained mass", coefficients.effective_mass]
  ];
  if (["roll", "pitch"].includes(characterization.axis)) {
    entries.push(["restoring", coefficients.restoring_stiffness]);
  }
  root.innerHTML = entries.map(entry =>
    '<div class="characterization-result"><small>' + escapeHtml(entry[0]) +
    '</small><strong>' + characterizationNumber(entry[1]) + '</strong></div>'
  ).join("") + (result.warnings?.length ?
    '<div class="characterization-warnings">' +
      result.warnings.map(escapeHtml).join("; ") + '</div>' : "") +
    (result.failures?.length ?
    '<div class="characterization-failures">Not safe to apply: ' +
      result.failures.map(escapeHtml).join("; ") + '</div>' : "");
}

function renderTracking() {
  const tracking = state.server.tracking || {active: false, status: "idle", message: "Ready"};
  const active = Boolean(tracking.active);
  const duplicates = duplicateSources();
  const telemetryReady = Number.isFinite(state.server.telemetry_age) && state.server.telemetry_age < 1;
  const topologyReady = duplicates.length === 0;
  const robotReady = state.server.killed === false && telemetryReady && topologyReady;
  const statusKind = active ? "warn" : !topologyReady || !telemetryReady ? "bad" :
    tracking.status === "aborted" ? "bad" :
    tracking.status === "complete" ? "good" : "";
  const inactiveStatus = !topologyReady ? "Conflict" : !telemetryReady ? "Telemetry stale" :
    state.server.killed !== false ? "Killed" : tracking.status === "stopped" ? "Ready" :
      tracking.status === "idle" ? "Idle" : tracking.status[0].toUpperCase() + tracking.status.slice(1);
  const referenceRate = Number(tracking.reference_rate);
  const waypointIndex = Number(tracking.waypoint_index || 0);
  const waypointCount = Number(tracking.waypoint_count || 4);
  const activeStatus = tracking.manual_advance ?
    tracking.ready_for_next ? "Corner reached" : waypointIndex === 0 ? "Settling" : "Moving" :
    Number.isFinite(referenceRate) && referenceRate < 0.05 ? "Catching up" :
    Number.isFinite(referenceRate) && referenceRate < 0.999 ? "Target slowed" : "Running";
  setPill("tracking-status", active ? activeStatus : inactiveStatus, statusKind);
  const characterizationActive = Boolean(state.server.characterization?.active);
  document.querySelectorAll(".tracking-form input, .tracking-form select").forEach(control => {
    control.disabled = active || characterizationActive;
  });
  $("start-tracking").disabled = active || characterizationActive || !robotReady || !state.socket || state.socket.readyState !== WebSocket.OPEN;
  $("stop-tracking").disabled = !active || !state.socket || state.socket.readyState !== WebSocket.OPEN;
  const squareSelected = $("tracking-experiment").value === "square_test";
  $("next-waypoint").hidden = !squareSelected;
  $("next-waypoint").disabled = !active || !tracking.manual_advance || !tracking.ready_for_next ||
    !state.socket || state.socket.readyState !== WebSocket.OPEN;
  $("next-waypoint").textContent = waypointIndex === 0 ? "First corner" :
    waypointIndex >= waypointCount - 1 ? "Return home" : "Next corner";
  const progress = Number.isFinite(tracking.progress) ? Math.round(tracking.progress * 100) : 0;
  const waypointProgress = tracking.manual_advance ?
    (waypointIndex === 0 ? "At start" : waypointIndex === waypointCount ? "Returning home" :
      "Corner " + waypointIndex + " / " + (waypointCount - 1)) + " · error " +
      Number(tracking.tracking_error || 0).toFixed(2) + " m · speed " +
      Number(tracking.vehicle_speed || 0).toFixed(2) + " m/s" : null;
  const pathProgress = pathExperiments.has(tracking.experiment) ?
    progress + "% path · " + Number(tracking.wall_elapsed || 0).toFixed(1) + " s wall · lag " +
      Number(tracking.tracking_error || 0).toFixed(2) + " m · target " +
      Math.round(Math.max(0, Math.min(1, Number.isFinite(referenceRate) ? referenceRate : 1)) * 100) + "%" : null;
  $("tracking-progress").textContent = active ?
    waypointProgress || pathProgress || progress + "% · " + Number(tracking.elapsed || 0).toFixed(1) + " / " + Number(tracking.duration || 0).toFixed(1) + " s" :
    !topologyReady ? "Stop the duplicate sim/controller launch: " + duplicates.join(", ") :
      !telemetryReady ? "Waiting for live telemetry" : state.server.killed !== false ?
      "Release kill switch to tune" : tracking.message || "Ready";
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
  const rows = [["time", ...names.map(displayName)].join(",")];
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
  localStorage.setItem("avbotz-layout-v4", JSON.stringify(layout));
}

function sizeTile(tile) {
  tile.style.setProperty("--w", tile.dataset.w);
  tile.style.setProperty("--h", tile.dataset.h);
}

function setupLayout() {
  const root = $("dashboard");
  const saved = JSON.parse(localStorage.getItem("avbotz-layout-v4") || "null");
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
document.querySelectorAll("[data-path-view]").forEach(button => {
  button.classList.toggle("active", button.dataset.pathView === state.pathView);
  button.addEventListener("click", () => {
    state.pathView = button.dataset.pathView;
    localStorage.setItem("avbotz-path-view", state.pathView);
    document.querySelectorAll("[data-path-view]").forEach(item =>
      item.classList.toggle("active", item === button));
    scheduleGraphDraw();
  });
});
$("clear-path").addEventListener("click", () => {
  state.pathTrail = [];
  state.pathTargetTrail = [];
  scheduleGraphDraw();
});
$("setpoint-mode").addEventListener("change", updateSetpointLabels);
$("publish-setpoint").addEventListener("click", publishSetpoint);
$("tracking-mode").addEventListener("change", () => updateTrackingLabels(true));
$("tracking-experiment").addEventListener("change", () => updateTrackingLabels(true));
$("tracking-amplitude").addEventListener("input", updateTrackingHint);
$("tracking-ramp").addEventListener("input", updateTrackingHint);
$("tracking-max-error").addEventListener("input", updateTrackingHint);
$("start-tracking").addEventListener("click", startTracking);
$("next-waypoint").addEventListener("click", advanceTracking);
$("stop-tracking").addEventListener("click", stopTracking);
$("characterization-axis").addEventListener("change", () => updateCharacterizationLabels(true));
$("start-characterization").addEventListener("click", startCharacterization);
$("stop-characterization").addEventListener("click", stopCharacterization);
$("apply-characterization").addEventListener("click", applyCharacterization);
$("stop-all").addEventListener("click", stopAll);
$("export").addEventListener("click", exportCsv);
$("reset-layout").addEventListener("click", () => {
  localStorage.removeItem("avbotz-layout-v4");
  location.reload();
});
document.addEventListener("keydown", event => {
  if (event.key === "Escape" && !event.ctrlKey && !event.metaKey && !event.altKey) {
    event.preventDefault();
    stopAll();
    return;
  }
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
updateTrackingLabels(true);
renderTracking();
updateCharacterizationLabels(true);
renderCharacterization();
updateSaveControls();
connect();
