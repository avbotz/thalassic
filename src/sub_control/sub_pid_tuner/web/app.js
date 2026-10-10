// AVBotz dashboard: live parameters, telemetry graphs and tuning routines for
// sub_control. The server (sub_pid_tuner/server.py) streams state over one
// websocket and answers requests on it. Frames are those of ControllerStatus:
// translation in odom ENU (z up), rotation as ZYX Euler angles; the wrench,
// integral and disturbance are body FLU.

const $ = id => document.getElementById(id);
const AXES = ["x", "y", "z", "roll", "pitch", "yaw"];
const AXIS_LABELS = {x: "X", y: "Y", z: "Z", roll: "Roll", pitch: "Pitch", yaw: "Yaw"};
const ROTATION = new Set(["roll", "pitch", "yaw"]);
const COLORS = ["#55a7ff", "#ffffff", "#25d8ff", "#8d7cff", "#78e6b2", "#f5c451", "#ff7f9f", "#b4c8e8"];
// Graph presets: signals shown for the selected axis.
const VIEWS = {
  pose: ["target", "reference", "measured"],
  tracking: ["tracking_error"],
  velocity: ["reference_velocity", "velocity"],
  wrench: ["wrench_command", "wrench_achieved", "disturbance"],
  integral: ["integral"]
};
const TRAJECTORY_GROUP = {x: "horizontal", y: "horizontal", z: "vertical", roll: "roll", pitch: "pitch", yaw: "yaw"};

const savedPathView = localStorage.getItem("dashboard-path-view");
const state = {
  socket: null,
  server: {parameters: {}, signals: {}, routine: {}, profile: null},
  staged: new Map(),  // "node|name" -> value
  pending: new Map(),  // request id -> {resolve, reject, timeout}
  nextRequest: 1,
  selected: new Set(JSON.parse(localStorage.getItem("dashboard-signals") || "[]")),
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
  clockOffset: null,
  lastServerTime: null,
  lastArrival: null,
  measuredTrail: [],
  referenceTrail: [],
  pathView: ["xy", "xz", "yz"].includes(savedPathView) ? savedPathView : "xy"
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

function escapeHtml(value) {
  return String(value).replaceAll("&", "&amp;").replaceAll('"', "&quot;").replaceAll("'", "&#39;")
    .replaceAll("<", "&lt;").replaceAll(">", "&gt;");
}

function connected() {
  return Boolean(state.socket) && state.socket.readyState === WebSocket.OPEN;
}

function telemetryLive() {
  return Number.isFinite(state.server.telemetry_age) && state.server.telemetry_age < 1;
}

function conflicts() {
  return Array.isArray(state.server.conflicts) ? state.server.conflicts : [];
}

// Connection

function request(type, payload) {
  if (!connected()) return Promise.reject(new Error("The dashboard is disconnected"));
  const requestId = state.nextRequest++;
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      state.pending.delete(requestId);
      reject(new Error("The robot did not answer in time"));
    }, 10000);
    state.pending.set(requestId, {resolve: resolve, reject: reject, timeout: timeout});
    state.socket.send(JSON.stringify(Object.assign({type: type, request_id: requestId}, payload || {})));
  });
}

function connect() {
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(protocol + "://" + location.host + "/api/ws");
  state.socket = socket;
  socket.onopen = () => {
    setPill("connection", "Connected", "good");
    // A restarted server may count time from another origin (a rebooted vehicle) and
    // watches only the controller: anchor its clock afresh and ask for the node shown.
    state.clockOffset = null;
    const node = $("node-select").value;
    if (node) request("load_parameters", {node: node}).catch(error => toast(error.message, true));
  };
  socket.onclose = () => {
    setPill("connection", "Disconnected", "bad");
    for (const pending of state.pending.values()) {
      clearTimeout(pending.timeout);
      pending.reject(new Error("The dashboard disconnected"));
    }
    state.pending.clear();
    renderRoutine();
    updateSaveControls();
    setTimeout(connect, 800);
  };
  socket.onerror = () => socket.close();
  socket.onmessage = event => handleMessage(JSON.parse(event.data));
}

function handleMessage(message) {
  if (message.type === "state") {
    const receivedAt = performance.now() / 1000;
    if (Number.isFinite(message.server_time)) {
      if (state.clockOffset === null) state.clockOffset = receivedAt - message.server_time;
      state.lastServerTime = message.server_time;
      state.lastArrival = receivedAt;
    }
    state.feedTimes.push(receivedAt);
    while (state.feedTimes.length && state.feedTimes[0] < receivedAt - 2) state.feedTimes.shift();
    const hadParameters = "parameters" in message;
    state.server = Object.assign({}, state.server, message);
    updateStatus();
    updateNodeSelect();
    if (hadParameters && state.staged.size === 0 && !$("parameter-list").contains(document.activeElement)) {
      renderParameters(false);
    }
    if (hadParameters) updateRoutineHint();
    recordTelemetry(receivedAt, message.server_time);
    renderSignals();
    if (receivedAt - state.lastTelemetryRender >= 0.2) {
      renderTelemetry();
      state.lastTelemetryRender = receivedAt;
    }
    renderFeedRate();
    updateSaveControls();
    renderRoutine();
    return;
  }

  const pending = state.pending.get(message.request_id);
  if (pending) {
    state.pending.delete(message.request_id);
    clearTimeout(pending.timeout);
    if (message.type === "error") pending.reject(new Error(message.message));
    else pending.resolve(message);
    return;
  }
  if (message.type === "error") toast(message.message, true);
}

// Header

function formatAge(value) {
  return Number.isFinite(value) ? Math.round(value * 1000) + " ms" : "never";
}

function updateStatus() {
  const s = state.server;
  setPill("robot", s.namespace || "—");
  const live = telemetryLive();
  setPill("telemetry", conflicts().length ? "Conflicting publishers" : live ? "Telemetry live" : "Telemetry stale",
    live && !conflicts().length ? "good" : "bad");
  $("telemetry").title = conflicts().length ? "More publishers than expected on " + conflicts().join(", ") :
    "odometry/filtered " + formatAge(s.odometry_age) + " old; control/status " + formatAge(s.status_age) + " old";

  const c = s.controller_state;
  if (!c) setPill("controller-health", "Controller —", "warn");
  else if (c.failsafe) setPill("controller-health", "Failsafe", "bad");
  else if (!c.depth_ok) setPill("controller-health", "No depth", "bad");
  else if (c.axis_controllable.some(ok => !ok)) setPill("controller-health", "Axis lost", "bad");
  else if (!c.dvl_ok) setPill("controller-health", "No DVL", "warn");
  else if (c.thruster_saturated.some(Boolean)) setPill("controller-health", "Saturated", "warn");
  else setPill("controller-health", "Controller OK", "good");

  if (s.killed === null || s.killed === undefined) setPill("kill", "Kill unknown", "warn");
  else setPill("kill", s.killed ? "Killed" : "Unkilled", s.killed ? "good" : "warn");
}

// Configuration

function updateNodeSelect() {
  const nodes = [...new Set([...(state.server.nodes || []), ...Object.keys(state.server.parameters || {})])].sort();
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
  if (nodes.includes(previous)) select.value = previous;
  else if (nodes.includes(state.server.controller)) select.value = state.server.controller;
  state.parameterKey = "";
  renderParameters(true);
}

const GROUPS = [
  ["gains", "Gains"],
  ["trajectory", "Trajectory"],
  ["", "General"],
  ["feedforward", "Feedforward"],
  ["observer", "Disturbance observer"],
  ["model", "Vehicle model"],
  ["allocation", "Allocation"],
  ["thrusters", "Thrusters"],
  ["safety", "Safety"]
];

function groupOf(name) {
  const prefix = ["thruster_health", "failed_thrusters"].includes(name) ? "thrusters" :
    name.includes(".") ? name.slice(0, name.indexOf(".")) : "";
  const index = GROUPS.findIndex(group => group[0] === prefix);
  return index >= 0 ? {order: index, title: GROUPS[index][1]} :
    {order: GROUPS.length, title: prefix.replaceAll("_", " ")};
}

function isGain(name, metadata) {
  return /^gains\.(x|y|z|roll|pitch|yaw)$/.test(name) && metadata.type === "double_array" &&
    Array.isArray(metadata.value) && metadata.value.length === 3;
}

// Labels of a short double array's elements, or null to edit it as JSON.
function elementLabels(name, metadata) {
  if (metadata.type !== "double_array" || !Array.isArray(metadata.value)) return null;
  const length = metadata.value.length;
  if (length === 0 || length > 8) return null;
  if (isGain(name, metadata)) return ["kp", "ki", "kd"];
  if (/^trajectory\.(horizontal|vertical|roll|pitch|yaw)$/.test(name) && length === 3) return ["v max", "a max", "j max"];
  if (name === "model.inertia" && length === 3) return ["roll", "pitch", "yaw"];
  if (length === 6) return AXES;
  if (length === 3) return ["x", "y", "z"];
  return [...Array(length).keys()].map(String);
}

// A critically damped triple pole at -w has gains [3w^2, w^3, 3w] (docs/control.md).
function gainsFor(bandwidth) {
  return [3 * bandwidth * bandwidth, bandwidth ** 3, 3 * bandwidth].map(value => Number(value.toPrecision(4)));
}

function bandwidthOf(gains) {
  const bandwidth = gains[2] / 3;
  if (!(bandwidth > 0)) return null;
  const expected = gainsFor(bandwidth);
  const close = (a, b) => Math.abs(a - b) <= 0.02 * Math.abs(b);
  return close(gains[0], expected[0]) && close(gains[1], expected[1]) ? Number(bandwidth.toPrecision(4)) : null;
}

function editorHtml(name, metadata, value) {
  const labels = elementLabels(name, metadata);
  if (labels) {
    const inputs = labels.map((label, index) =>
      '<label class="array-label">' + escapeHtml(label) + '<input data-index="' + index +
      '" type="number" step="any" value="' + escapeHtml(value[index]) + '"></label>').join("");
    if (!isGain(name, metadata)) return inputs;
    const bandwidth = bandwidthOf(value);
    return inputs + '<label class="array-label bandwidth" title="Bandwidth: sets [kp, ki, kd] to [3ω², ω³, 3ω]">ω rad/s' +
      '<input class="bandwidth-input" type="number" min="0" step="any" placeholder="custom" value="' +
      (bandwidth === null ? "" : bandwidth) + '"></label>';
  }
  if (metadata.type === "bool") {
    return '<select><option value="true" ' + (value === true ? "selected" : "") + '>true</option>' +
      '<option value="false" ' + (value === false ? "selected" : "") + ">false</option></select>";
  }
  const numeric = metadata.type === "integer" || metadata.type === "double";
  const text = metadata.type.endsWith("_array") ? JSON.stringify(value) : value === null ? "" : value;
  return "<input " + (numeric ? 'type="number" step="any" ' : "") + 'value="' + escapeHtml(text) + '">';
}

function readRow(row, metadata) {
  const name = row.dataset.name;
  if (elementLabels(name, metadata)) {
    // Number("") is 0: an emptied field must not apply a zero gain.
    const values = [...row.querySelectorAll("input[data-index]")].map(input =>
      input.value.trim() === "" ? NaN : Number(input.value));
    if (values.some(value => !Number.isFinite(value))) throw new Error(name + " must contain finite numbers");
    return values;
  }
  const control = row.querySelector(".parameter-editor input, .parameter-editor select");
  if (metadata.type === "bool") return control.value === "true";
  if (metadata.type.endsWith("_array")) {
    const value = JSON.parse(control.value);
    if (!Array.isArray(value)) throw new Error(name + " must be a JSON array");
    return value;
  }
  if (metadata.type === "integer" || metadata.type === "double") {
    const value = Number(control.value);
    if (control.value.trim() === "" || !Number.isFinite(value)) throw new Error(name + " must be a finite number");
    if (metadata.type === "integer" && !Number.isInteger(value)) throw new Error(name + " must be an integer");
    return value;
  }
  return control.value;
}

function metadataFor(row) {
  return state.server.parameters?.[row.dataset.node]?.[row.dataset.name];
}

function syncRow(row) {
  const metadata = metadataFor(row);
  if (!metadata || metadata.read_only) return true;
  const key = row.dataset.node + "|" + row.dataset.name;
  const button = row.querySelector(".apply-one");
  try {
    const value = readRow(row, metadata);
    row.classList.remove("invalid");
    if (JSON.stringify(value) === JSON.stringify(metadata.value)) {
      state.staged.delete(key);
      row.classList.remove("dirty");
    } else {
      state.staged.set(key, value);
      row.classList.add("dirty");
    }
    if (button) button.disabled = !state.staged.has(key) || state.saving;
    updateSaveControls();
    return true;
  } catch (error) {
    state.staged.delete(key);
    row.classList.add("dirty", "invalid");
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
  if (!valid) throw new Error("Fix the highlighted value first");
}

function differsFromProfile(node, name, metadata) {
  const values = state.server.profile?.values || {};
  return node === state.server.controller && name in values &&
    JSON.stringify(values[name]) !== JSON.stringify(metadata.value);
}

function renderParameters(force) {
  const node = $("node-select").value;
  const parameters = state.server.parameters?.[node] || {};
  const filter = $("parameter-filter").value.toLowerCase();
  const modifiedOnly = $("modified-only").checked;
  const signature = JSON.stringify([
    node, filter, modifiedOnly, state.server.profile?.values,
    Object.entries(parameters).map(entry => [entry[0], entry[1].value, entry[1].read_only]),
    [...state.staged.entries()]
  ]);
  if (!force && signature === state.parameterKey) return;
  state.parameterKey = signature;

  const root = $("parameter-list");
  root.innerHTML = "";
  let lastGroup = "";
  // Grouped, then in the gains file's order (x, y, z, roll, pitch, yaw), then by name.
  const fileOrder = Object.keys(node === state.server.controller ? state.server.profile?.values || {} : {});
  const position = name => fileOrder.includes(name) ? fileOrder.indexOf(name) : fileOrder.length;
  const sorted = Object.entries(parameters).sort((a, b) =>
    groupOf(a[0]).order - groupOf(b[0]).order || groupOf(a[0]).title.localeCompare(groupOf(b[0]).title) ||
    position(a[0]) - position(b[0]) || a[0].localeCompare(b[0]));
  for (const [name, metadata] of sorted) {
    const key = node + "|" + name;
    const dirty = state.staged.has(key);
    if (!name.toLowerCase().includes(filter) || (modifiedOnly && !dirty)) continue;
    const group = groupOf(name).title;
    if (group !== lastGroup) {
      const heading = document.createElement("div");
      heading.className = "parameter-group";
      heading.textContent = group;
      root.append(heading);
      lastGroup = group;
    }

    const value = dirty ? state.staged.get(key) : metadata.value;
    const differs = differsFromProfile(node, name, metadata);
    const row = document.createElement("div");
    row.className = "parameter-row" + (dirty ? " dirty" : "") + (metadata.read_only ? " readonly" : "") +
      (differs ? " profile-diff" : "");
    row.dataset.node = node;
    row.dataset.name = name;
    row.innerHTML =
      '<div class="parameter-name"><strong title="' + escapeHtml(metadata.description || name) + '">' +
      escapeHtml(name) + "</strong><small>" + escapeHtml(metadata.type) +
      (metadata.read_only ? " · read only" : "") + (differs ? " · differs from profile" : "") +
      '</small></div><div class="parameter-editor">' + editorHtml(name, metadata, value) +
      '</div><button class="apply-one" title="Apply this value to the running node" disabled>Apply</button>';

    if (metadata.read_only) {
      row.querySelectorAll("input, select").forEach(control => { control.disabled = true; });
    } else {
      row.querySelectorAll("input[data-index], .parameter-editor > input, .parameter-editor > select").forEach(control => {
        const changed = () => {
          syncRow(row);
          const bandwidthInput = row.querySelector(".bandwidth-input");
          if (bandwidthInput && document.activeElement !== bandwidthInput) {
            const values = [...row.querySelectorAll("input[data-index]")].map(input => Number(input.value));
            const bandwidth = bandwidthOf(values);
            bandwidthInput.value = bandwidth === null ? "" : bandwidth;
          }
        };
        control.addEventListener("input", changed);
        control.addEventListener("change", changed);
      });
      const bandwidthInput = row.querySelector(".bandwidth-input");
      if (bandwidthInput) {
        bandwidthInput.addEventListener("input", () => {
          const bandwidth = Number(bandwidthInput.value);
          if (!(bandwidth > 0) || !Number.isFinite(bandwidth)) return;
          gainsFor(bandwidth).forEach((gain, index) => {
            row.querySelector('input[data-index="' + index + '"]').value = gain;
          });
          syncRow(row);
        });
      }
    }
    row.querySelector(".apply-one").addEventListener("click", () => applyRuntime([key]));
    root.append(row);
  }
  if (!root.querySelector(".parameter-row")) root.innerHTML = '<div class="empty">No matching parameters.</div>';
  updateSaveControls();
}

function updateSaveControls() {
  const count = state.staged.size;
  $("edit-count").textContent = count ? count + " staged edit" + (count === 1 ? "" : "s") : "No staged edits";
  $("edit-count").className = "edit-count" + (count ? " active" : "");
  $("apply-all").disabled = state.saving || count === 0 || !connected();
  const profile = state.server.profile;
  $("save-profile").disabled = state.saving || !profile || !connected();
  document.querySelectorAll(".apply-one").forEach(button => {
    const row = button.closest(".parameter-row");
    button.disabled = state.saving || !state.staged.has(row.dataset.node + "|" + row.dataset.name) ||
      row.classList.contains("invalid");
  });
  $("configuration").classList.toggle("configuration-saving", state.saving);
  $("config-summary").textContent = state.saving ? "Waiting for ROS to apply and read back…" :
    profile ? "Apply changes the running nodes. Save profile also writes the controller's values to " + profile.path + "." :
      "Apply changes the running nodes. Saving is off: the dashboard was started without a profile.";
}

function changesFor(keys) {
  return keys.filter(key => state.staged.has(key)).map(key => {
    const split = key.indexOf("|");
    return {node: key.slice(0, split), name: key.slice(split + 1), value: state.staged.get(key)};
  });
}

function acceptApplied(applied) {
  for (const item of applied || []) {
    // A row clears only once the node reads back what is staged; one edited since stays.
    const key = item.node + "|" + item.name;
    if (JSON.stringify(state.staged.get(key)) === JSON.stringify(item.value)) state.staged.delete(key);
    const metadata = state.server.parameters?.[item.node]?.[item.name];
    if (metadata) metadata.value = item.value;
  }
  renderParameters(true);
}

async function applyRuntime(onlyKeys) {
  if (state.saving) return;
  try {
    harvestVisibleRows();
    const changes = changesFor(onlyKeys || [...state.staged.keys()]);
    if (!changes.length) {
      toast("Nothing to apply");
      return;
    }
    state.saving = true;
    updateSaveControls();
    const result = await request("set_parameters", {changes: changes});
    acceptApplied(result.applied);
    toast(result.applied.length + " value" + (result.applied.length === 1 ? "" : "s") + " applied and read back");
  } catch (error) {
    toast(error.message, true);
  } finally {
    state.saving = false;
    updateSaveControls();
  }
}

async function saveProfile() {
  if (state.saving) return;
  try {
    harvestVisibleRows();
    if (!state.server.profile) throw new Error("The dashboard was started without a profile");
    const changes = changesFor([...state.staged.keys()]);
    state.saving = true;
    updateSaveControls();
    const result = await request("save_profile", {changes: changes});
    acceptApplied(result.applied);
    const written = Object.keys(result.written);
    state.server.profile = {path: result.path, values: Object.assign({}, state.server.profile.values, result.written)};
    renderParameters(true);
    toast(written.length ? "Saved " + written.join(", ") + " to " + result.path.split("/").pop() :
      "The profile already matches the controller");
  } catch (error) {
    toast(error.message, true);
  } finally {
    state.saving = false;
    updateSaveControls();
  }
}

async function reloadConfiguration() {
  state.staged.clear();
  renderParameters(true);
  try {
    await request("refresh");
    toast("Staged edits discarded; reloading");
  } catch (error) {
    toast(error.message, true);
  }
}

// Graph

function recordTelemetry(receivedAt, serverTime) {
  if (state.paused) return;
  const time = Number.isFinite(serverTime) && state.clockOffset !== null ? serverTime + state.clockOffset : receivedAt;
  const signals = state.server.signals || {};
  state.history.push({time: time, wallTime: Date.now(), signals: Object.assign({}, signals)});
  while (state.history.length && state.history[0].time < time - 65) state.history.shift();
  record(state.measuredTrail, ["x", "y", "z"].map(axis => signals[axis + ".measured"]));
  record(state.referenceTrail, ["x", "y", "z"].map(axis => signals[axis + ".reference"]));
  scheduleDraw();
}

function record(trail, point) {
  if (!point.every(Number.isFinite)) return;
  const previous = trail[trail.length - 1];
  if (!previous || point.some((value, index) => Math.abs(value - previous[index]) > 0.001)) trail.push(point);
  if (trail.length > 1200) trail.splice(0, trail.length - 1200);
}

function scheduleDraw() {
  if (state.drawPending) return;
  state.drawPending = true;
  let done = false;
  const run = () => {
    if (done) return;
    done = true;
    state.drawPending = false;
    drawGraph();
    drawPath();
  };
  requestAnimationFrame(run);
  setTimeout(run, 34);  // background tabs get no animation frames
}

function graphNow() {
  if (state.lastServerTime !== null && state.clockOffset !== null && state.lastArrival !== null) {
    return state.lastServerTime + state.clockOffset + (performance.now() / 1000 - state.lastArrival);
  }
  return performance.now() / 1000;
}

function renderFeedRate() {
  if (!telemetryLive()) {
    $("feed-rate").textContent = "Telemetry stale";
  } else if (state.feedTimes.length < 2) {
    $("feed-rate").textContent = "Waiting for data";
  } else {
    const span = state.feedTimes[state.feedTimes.length - 1] - state.feedTimes[0];
    $("feed-rate").textContent = "Live · " + (span > 0 ? (state.feedTimes.length - 1) / span : 0).toFixed(0) + " Hz";
  }
}

function saveSelection() {
  localStorage.setItem("dashboard-signals", JSON.stringify([...state.selected]));
  state.signalKey = "";
}

function viewSignals() {
  return VIEWS[$("graph-view").value].map(kind => $("graph-axis").value + "." + kind);
}

function renderSignals() {
  const catalog = Object.keys(state.server.signals || {}).sort();
  if (catalog.length && ![...state.selected].some(name => catalog.includes(name))) {
    const defaults = viewSignals();
    state.selected = new Set(defaults.some(name => catalog.includes(name)) ? defaults : catalog.slice(0, 3));
    saveSelection();
  }
  const filter = $("signal-filter").value.toLowerCase();
  const key = catalog.join("|") + "|" + filter + "|" + [...state.selected].join("|");
  if (key === state.signalKey) return;
  state.signalKey = key;
  const root = $("signal-list");
  root.innerHTML = "";
  for (const name of catalog.filter(item => item.toLowerCase().includes(filter))) {
    const label = document.createElement("label");
    label.className = "signal-option";
    label.innerHTML = '<input type="checkbox" ' + (state.selected.has(name) ? "checked" : "") +
      '><span title="' + escapeHtml(name) + '">' + escapeHtml(name) + "</span>";
    label.querySelector("input").addEventListener("change", event => {
      if (event.target.checked) state.selected.add(name);
      else state.selected.delete(name);
      saveSelection();
      scheduleDraw();
    });
    root.append(label);
  }
}

function selectView(axis, view) {
  if (axis) $("graph-axis").value = axis;
  if (view) $("graph-view").value = view;
  state.selected = new Set(viewSignals());
  saveSelection();
  renderSignals();
  scheduleDraw();
}

// Point the axis and view menus at a restored selection that is one of the views.
function syncViewControls() {
  const names = [...state.selected];
  const axis = names.length ? names[0].slice(0, names[0].indexOf(".")) : "";
  if (!AXES.includes(axis)) return;
  const kinds = names.map(name => name.slice(axis.length + 1)).sort().join("|");
  const view = Object.keys(VIEWS).find(key => [...VIEWS[key]].sort().join("|") === kinds &&
    names.every(name => name.startsWith(axis + ".")));
  if (!view) return;
  $("graph-axis").value = axis;
  $("graph-view").value = view;
}

function yRange(values, canvas) {
  const scale = $("scale-select").value;
  if (scale === "auto") {
    // A loop: Math.min(...values) overflows the stack past about 125k samples.
    let min = values.length ? Infinity : -1;
    let max = values.length ? -Infinity : 1;
    for (const value of values) {
      if (value < min) min = value;
      if (value > max) max = value;
    }
    if (Math.abs(max - min) < 1e-9) { min -= 1; max += 1; }
    const margin = (max - min) * 0.1;
    return [min - margin, max + margin];
  }
  if (scale === "custom") {
    const min = Number($("scale-min").value);
    const max = Number($("scale-max").value);
    const valid = Number.isFinite(min) && Number.isFinite(max) && min < max;
    $("custom-scale").classList.toggle("invalid", !valid);
    return valid ? [min, max] : [Number(canvas.dataset.yMin) || -1, Number(canvas.dataset.yMax) || 1];
  }
  return [-Number(scale), Number(scale)];
}

function sizeCanvas(canvas) {
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
  return {ctx: ctx, width: width, height: height};
}

function formatAxisValue(value) {
  if (Math.abs(value) >= 1000 || (Math.abs(value) > 0 && Math.abs(value) < 0.001)) return value.toExponential(2);
  return Number(value.toFixed(3)).toString();
}

function drawGraph() {
  const canvas = $("chart");
  const {ctx, width, height} = sizeCanvas(canvas);
  ctx.fillStyle = "#080d16";
  ctx.fillRect(0, 0, width, height);

  const pad = {left: 58, right: 14, top: 16, bottom: 25};
  const plotWidth = width - pad.left - pad.right;
  const plotHeight = height - pad.top - pad.bottom;
  const names = [...state.selected];
  const seconds = Number($("window-select").value);
  const start = graphNow() - seconds;
  const values = [];
  for (const sample of state.history) {
    if (sample.time < start) continue;
    for (const name of names) if (Number.isFinite(sample.signals[name])) values.push(sample.signals[name]);
  }
  const [min, max] = yRange(values, canvas);
  canvas.dataset.yMin = String(min);
  canvas.dataset.yMax = String(max);
  $("range-readout").textContent = ($("scale-select").value === "auto" ? "Y: auto · " : "Y: ") +
    formatAxisValue(min) + " to " + formatAxisValue(max);

  ctx.strokeStyle = "#20304a";
  ctx.fillStyle = "#8798b3";
  ctx.font = "10px ui-monospace";
  for (let index = 0; index <= 5; index++) {
    const y = pad.top + plotHeight * index / 5;
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(width - pad.right, y);
    ctx.stroke();
    ctx.fillText((max - (max - min) * index / 5).toPrecision(3), 5, y + 3);
  }
  ctx.fillText("-" + seconds + " s", pad.left, height - 7);
  ctx.fillText("now", width - 36, height - 7);

  ctx.save();
  ctx.beginPath();
  ctx.rect(pad.left, pad.top, plotWidth, plotHeight);
  ctx.clip();
  names.forEach((name, index) => {
    ctx.strokeStyle = COLORS[index % COLORS.length];
    ctx.lineWidth = 2;
    ctx.beginPath();
    let begun = false;
    for (const sample of state.history) {
      const value = sample.signals[name];
      if (!Number.isFinite(value)) continue;
      const x = pad.left + (sample.time - start) / seconds * plotWidth;
      const y = pad.top + (max - value) / (max - min) * plotHeight;
      if (begun) ctx.lineTo(x, y);
      else ctx.moveTo(x, y);
      begun = true;
    }
    ctx.stroke();
  });
  ctx.restore();

  const legendKey = names.join("|");
  if (legendKey !== state.legendKey) {
    state.legendKey = legendKey;
    $("legend").innerHTML = names.map((name, index) => '<span><i class="legend-dot" style="background:' +
      COLORS[index % COLORS.length] + '"></i>' + escapeHtml(name) + "</span>").join("");
  }
}

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

function updateCustomScale() {
  $("scale-select").value = "custom";
  $("custom-scale").hidden = false;
  const min = Number($("scale-min").value);
  const max = Number($("scale-max").value);
  if (Number.isFinite(min) && Number.isFinite(max) && min < max) {
    localStorage.setItem("dashboard-graph-scale", "custom");
    localStorage.setItem("dashboard-custom-scale", JSON.stringify({min: min, max: max}));
  }
  scheduleDraw();
}

function exportCsv() {
  const names = [...state.selected];
  const rows = [["time", ...names].join(",")];
  for (const sample of state.history) {
    rows.push([new Date(sample.wallTime).toISOString(), ...names.map(name => sample.signals[name] ?? "")].join(","));
  }
  const url = URL.createObjectURL(new Blob([rows.join("\n") + "\n"], {type: "text/csv"}));
  const link = document.createElement("a");
  link.href = url;
  link.download = "dashboard-" + new Date().toISOString().replaceAll(":", "-") + ".csv";
  link.click();
  URL.revokeObjectURL(url);
}

// Path

const PATH_AXES = {xy: [0, 1], xz: [0, 2], yz: [1, 2]};
const PATH_CAPTIONS = {xy: "x right, y up · from above", xz: "x right, z up · from the side", yz: "y right, z up · from the front"};

// The vehicle's nose (xy, xz) or left side (yz) in the view's plane.
function headingTip(point, signals, length) {
  const [x, y, z] = point;
  if (state.pathView === "xy") {
    const yaw = signals["yaw.measured"];
    return Number.isFinite(yaw) ? [x + Math.cos(yaw) * length, y + Math.sin(yaw) * length, z] : null;
  }
  if (state.pathView === "xz") {
    // Positive pitch turns the nose down.
    const pitch = signals["pitch.measured"];
    return Number.isFinite(pitch) ? [x + Math.cos(pitch) * length, y, z - Math.sin(pitch) * length] : null;
  }
  const roll = signals["roll.measured"];
  return Number.isFinite(roll) ? [x, y + Math.cos(roll) * length, z + Math.sin(roll) * length] : null;
}

function drawPath() {
  const canvas = $("path-canvas");
  const {ctx, width, height} = sizeCanvas(canvas);
  const routine = state.server.routine || {};
  const planned = Array.isArray(routine.preview) ? routine.preview : [];
  const signals = state.server.signals || {};
  const point = kind => {
    const value = ["x", "y", "z"].map(axis => signals[axis + "." + kind]);
    return value.every(Number.isFinite) ? value : null;
  };
  const measured = point("measured");
  const target = point("target");
  const all = [...planned, ...state.referenceTrail, ...state.measuredTrail];
  if (measured) all.push(measured);
  if (target) all.push(target);

  ctx.fillStyle = "#06111f";
  ctx.fillRect(0, 0, width, height);
  if (!all.length) {
    ctx.fillStyle = "#8798b3";
    ctx.font = "13px system-ui";
    ctx.fillText("Waiting for odometry", 20, 50);
    $("path-status").textContent = "Waiting for position";
    return;
  }

  const [u, v] = PATH_AXES[state.pathView];
  let minU = Math.min(...all.map(p => p[u]));
  let maxU = Math.max(...all.map(p => p[u]));
  let minV = Math.min(...all.map(p => p[v]));
  let maxV = Math.max(...all.map(p => p[v]));
  const growU = Math.max(0, 0.5 - (maxU - minU)) / 2 + Math.max((maxU - minU) * 0.16, 0.08);
  const growV = Math.max(0, 0.5 - (maxV - minV)) / 2 + Math.max((maxV - minV) * 0.16, 0.08);
  minU -= growU; maxU += growU; minV -= growV; maxV += growV;
  const plot = {left: 46, top: 40, right: width - 18, bottom: height - 35};
  const plotWidth = Math.max(1, plot.right - plot.left);
  const plotHeight = Math.max(1, plot.bottom - plot.top);
  const scale = Math.max(1, Math.min(plotWidth / (maxU - minU), plotHeight / (maxV - minV)));
  const centerU = (minU + maxU) / 2;
  const centerV = (minV + maxV) / 2;
  const screen = p => [plot.left + plotWidth / 2 + (p[u] - centerU) * scale, plot.top + plotHeight / 2 - (p[v] - centerV) * scale];

  const water = ctx.createLinearGradient(0, plot.top, 0, plot.bottom);
  water.addColorStop(0, "#0b3555");
  water.addColorStop(1, "#08253e");
  ctx.fillStyle = water;
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

  const stroke = (points, color, lineWidth, dash) => {
    if (points.length < 2) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.setLineDash(dash || []);
    ctx.beginPath();
    points.forEach((p, index) => {
      const [x, y] = screen(p);
      if (index) ctx.lineTo(x, y);
      else ctx.moveTo(x, y);
    });
    ctx.stroke();
    ctx.restore();
  };
  stroke(planned, "#b4f4cf", 1.5, [5, 4]);
  stroke(state.referenceTrail, "#72e6a9", 2.2);
  stroke(state.measuredTrail, "#3195ff", 2.8);

  if (target) {
    const [x, y] = screen(target);
    ctx.fillStyle = "#72e6a9";
    ctx.beginPath();
    ctx.moveTo(x, y - 7); ctx.lineTo(x + 7, y); ctx.lineTo(x, y + 7); ctx.lineTo(x - 7, y); ctx.closePath();
    ctx.fill();
  }
  if (measured) {
    const [x, y] = screen(measured);
    ctx.fillStyle = "#0d2e54";
    ctx.strokeStyle = "#57adff";
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(x, y, 9, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    const tip = headingTip(measured, signals, Math.max(0.08, 24 / scale));
    if (tip) {
      const [tipX, tipY] = screen(tip);
      ctx.strokeStyle = "#9ed2ff";
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(tipX, tipY); ctx.stroke();
    }
  }

  ctx.fillStyle = "#9fc2dc";
  ctx.font = "11px ui-monospace";
  ctx.fillText("odom " + PATH_CAPTIONS[state.pathView] + " · m", plot.left, height - 11);
  $("path-status").textContent = routine.active ? ROUTINE_NAMES[routine.kind] + " · " +
    Math.round((routine.progress || 0) * 100) + "%" : planned.length ? "Last routine" : "Live";
}

// Telemetry

function telemetryRows() {
  const rows = [];
  const c = state.server.controller_state;
  if (c) {
    const yesNo = value => value ? "yes" : "no";
    AXES.forEach((axis, index) => rows.push(["mode." + axis, c.mode[index] +
      (c.axis_controllable[index] ? "" : " (not controllable)"), true]));
    rows.push(["altitude_hold", yesNo(c.altitude), true], ["dvl_ok", yesNo(c.dvl_ok), true],
      ["depth_ok", yesNo(c.depth_ok), true], ["altitude_ok", yesNo(c.altitude_ok), true],
      ["failsafe", yesNo(c.failsafe), true]);
    const saturated = c.thruster_saturated.flatMap((value, index) => value ? [index] : []);
    rows.push(["thrusters_saturated", saturated.length ? saturated.join(", ") : "none", true]);
    // null where the server had NaN, which JSON cannot carry.
    rows.push(["thruster_health", c.thruster_health.map(value => Number.isFinite(value) ? value.toFixed(2) : "—").join(" "), true]);
  }
  const signals = state.server.signals || {};
  for (const name of Object.keys(signals).sort()) rows.push([name, Number(signals[name]).toPrecision(6), false]);
  return rows;
}

function renderTelemetry() {
  const filter = $("telemetry-filter").value.toLowerCase();
  const root = $("telemetry-list");
  root.innerHTML = "";
  for (const [name, value, status] of telemetryRows()) {
    if (!name.toLowerCase().includes(filter)) continue;
    const row = document.createElement("div");
    row.className = "telemetry-row" + (status ? " status" : "");
    row.innerHTML = "<span>" + escapeHtml(name) + "</span><span>" + escapeHtml(value) + "</span>";
    root.append(row);
  }
  if (!root.children.length) root.innerHTML = '<div class="empty">No matching live values.</div>';
}

// Routines (sub_pid_tuner/routines.py)

const ROUTINE_NAMES = {step: "Step", cruise: "Cruise", hold: "Hold", square: "Square"};
const ROUTINE_AXES = {step: AXES, cruise: ["x", "y", "z", "yaw"], hold: AXES, square: []};
const ROUTINE_DEFAULTS = {
  step: {x: 1, y: 1, z: 0.5, roll: 0.2, pitch: 0.2, yaw: 0.8, hold: 10, cycles: 1},
  cruise: {x: 0.2, y: 0.2, z: 0.1, yaw: 0.3, hold: 8, settle: 6, cycles: 1},
  hold: {hold: 30},
  square: {amplitude: 1, hold: 12, cycles: 1}
};
const ROUTINE_HINTS = {
  step: "POSITION steps out, back, the other way and back, holding each.",
  cruise: "VELOCITY one way, stop and hold, then the other way.",
  hold: "Holds every axis where it is. Push the vehicle and watch it return.",
  square: "POSITION through the corners of a square in odom x/y: forward, then left."
};

function trajectoryLimits(axis) {
  const parameter = state.server.parameters?.[state.server.controller]?.["trajectory." + TRAJECTORY_GROUP[axis]];
  return Array.isArray(parameter?.value) && parameter.value.length === 3 ? parameter.value : null;
}

// Rough duration of a rest-to-rest move of `distance` within [v max, a max, j max].
function moveTime(distance, limits) {
  const [velocity, acceleration, jerk] = limits;
  const d = Math.abs(distance);
  const cruise = d > velocity * velocity / acceleration ? d / velocity + velocity / acceleration :
    2 * Math.sqrt(d / acceleration);
  return cruise + acceleration / jerk;
}

function updateRoutineForm(loadDefaults) {
  const kind = $("routine-kind").value;
  const axisSelect = $("routine-axis");
  const previous = axisSelect.value;
  axisSelect.innerHTML = ROUTINE_AXES[kind].map(axis => '<option value="' + axis + '">' + AXIS_LABELS[axis] + "</option>").join("");
  if (ROUTINE_AXES[kind].includes(previous)) axisSelect.value = previous;
  else if (ROUTINE_AXES[kind].includes("z")) axisSelect.value = "z";
  const axis = axisSelect.value;
  const unit = ROTATION.has(axis) ? "rad" : "m";

  $("routine-axis-wrap").hidden = kind === "square";
  $("routine-axis-label").textContent = kind === "hold" ? "Graph" : "Axis";
  $("routine-amplitude-wrap").hidden = kind === "hold";
  $("routine-amplitude-label").textContent = kind === "step" ? "Step (" + unit + ")" :
    kind === "cruise" ? "Speed (" + unit + "/s)" : "Side (m)";
  $("routine-hold-label").textContent = kind === "cruise" ? "At speed (s)" : kind === "hold" ? "Duration (s)" :
    kind === "square" ? "Per corner (s)" : "Hold (s)";
  $("routine-settle-wrap").hidden = kind !== "cruise";
  $("routine-cycles-wrap").hidden = kind === "hold";

  if (loadDefaults) {
    const defaults = ROUTINE_DEFAULTS[kind];
    $("routine-amplitude").value = defaults[axis] ?? defaults.amplitude ?? 0;
    $("routine-hold").value = defaults.hold;
    $("routine-settle").value = defaults.settle ?? 5;
    $("routine-cycles").value = defaults.cycles ?? 1;
  }
  updateRoutineHint();
}

function updateRoutineHint() {
  const kind = $("routine-kind").value;
  const axis = kind === "square" ? "x" : $("routine-axis").value;
  const amplitude = Math.abs(Number($("routine-amplitude").value));
  const unit = ROTATION.has(axis) ? "rad" : "m";
  let hint = ROUTINE_HINTS[kind];
  const limits = trajectoryLimits(axis);
  if (limits && kind !== "hold" && amplitude > 0) {
    const group = "trajectory." + TRAJECTORY_GROUP[axis];
    if (kind === "cruise") {
      hint += amplitude > limits[0] ? " The reference is limited to " + group + " v max, " + limits[0] + " " + unit + "/s." :
        " Up to speed in about " + (amplitude / limits[1] + limits[1] / limits[2]).toFixed(1) + " s.";
    } else {
      hint += " Each move takes about " + moveTime(amplitude, limits).toFixed(1) + " s at " + group + ".";
    }
  }
  if (ROTATION.has(axis) && kind !== "hold" && amplitude > 0) {
    hint += " " + amplitude + " rad = " + (amplitude * 180 / Math.PI).toFixed(1) + "°.";
  }
  $("routine-hint").textContent = hint;
}

function graphRoutine(kind, axis) {
  state.history = [];
  state.measuredTrail = [];
  state.referenceTrail = [];
  state.paused = false;
  $("pause").textContent = "Pause";
  if (kind === "square") {
    state.selected = new Set(["x.reference", "x.measured", "y.reference", "y.measured"]);
    saveSelection();
    setPathView("xy");
    renderSignals();
  } else {
    selectView(axis, kind === "cruise" ? "velocity" : "pose");
  }
  scheduleDraw();
}

async function startRoutine() {
  const kind = $("routine-kind").value;
  const values = {
    kind: kind,
    axis: $("routine-axis").value,
    amplitude: Number($("routine-amplitude").value),
    hold: Number($("routine-hold").value),
    settle: Number($("routine-settle").value),
    cycles: Number($("routine-cycles").value)
  };
  try {
    const result = await request("start_routine", values);
    state.server.routine = result.routine;
    graphRoutine(kind, values.axis);
    renderRoutine();
    toast(ROUTINE_NAMES[kind] + " started");
  } catch (error) {
    toast(error.message, true);
  }
}

async function routineRequest(type, message) {
  try {
    const result = await request(type);
    state.server.routine = result.routine;
    renderRoutine();
    toast(message);
  } catch (error) {
    toast(error.message, true);
  }
}

async function zeroPose() {
  if (!confirm("Make the vehicle's current position and heading the odom origin?")) return;
  try {
    const result = await request("zero_pose");
    toast(result.message, !result.success);
  } catch (error) {
    toast(error.message, true);
  }
}

function routineProblem() {
  if (!connected()) return "Disconnected";
  if (conflicts().length) return "Other publishers on " + conflicts().join(", ") + ": stop the mission, teleop or second stack";
  if (!telemetryLive()) return "Waiting for odometry and control/status";
  if (state.server.killed !== false) return "Release the kill switch to run a routine";
  if (state.server.controller_state?.failsafe) return "sub_control is in failsafe";
  return null;
}

function renderRoutine() {
  const routine = state.server.routine || {};
  const active = Boolean(routine.active);
  const problem = routineProblem();
  const kind = active ? "warn" : problem ? "bad" : routine.status === "aborted" ? "bad" :
    routine.status === "complete" ? "good" : "";
  const status = active ? "Running" : problem ? "Not ready" : routine.status === "aborted" ? "Aborted" :
    routine.status === "complete" ? "Complete" : routine.status === "stopped" ? "Stopped" : "Idle";
  setPill("routine-status", status, kind);
  document.querySelectorAll(".routine-form input, .routine-form select").forEach(control => { control.disabled = active; });
  $("start-routine").disabled = active || Boolean(problem);
  $("stop-routine").disabled = !active || !connected();
  $("zero-pose").disabled = active || !connected();
  $("send-command").disabled = active || !connected();
  $("routine-progress").textContent = active ?
    Math.round((routine.progress || 0) * 100) + "% · " + Number(routine.elapsed || 0).toFixed(1) + " / " +
      Number(routine.duration || 0).toFixed(1) + " s · " + routine.message :
    problem || routine.message || "Ready";
}

// Manual command

const COMMAND_UNITS = {
  KEEP: ["", ""], HOLD: ["", ""], POSITION: ["m", "rad"], VELOCITY: ["m/s", "rad/s"], EFFORT: ["N", "N·m"]
};

function buildCommandGrid() {
  const modes = Object.keys(COMMAND_UNITS);
  $("command-grid").innerHTML = AXES.map(axis =>
    "<span>" + AXIS_LABELS[axis] + '</span><select class="command-mode" data-axis="' + axis + '">' +
    modes.map(mode => '<option value="' + mode + '">' + mode[0] + mode.slice(1).toLowerCase() + "</option>").join("") +
    '</select><input class="command-value" data-axis="' + axis + '" type="number" step="0.05" value="0">' +
    '<span class="command-unit" data-axis="' + axis + '"></span>').join("");
  document.querySelectorAll(".command-mode").forEach(select => {
    select.addEventListener("change", () => {
      // x and y always share a mode.
      const partner = {x: "y", y: "x"}[select.dataset.axis];
      if (partner) document.querySelector('.command-mode[data-axis="' + partner + '"]').value = select.value;
      updateCommandUnits();
    });
  });
  updateCommandUnits();
}

function updateCommandUnits() {
  for (const axis of AXES) {
    const mode = document.querySelector('.command-mode[data-axis="' + axis + '"]').value;
    document.querySelector('.command-unit[data-axis="' + axis + '"]').textContent = COMMAND_UNITS[mode][ROTATION.has(axis) ? 1 : 0];
    document.querySelector('.command-value[data-axis="' + axis + '"]').disabled = mode === "KEEP" || mode === "HOLD";
  }
}

async function sendCommand() {
  const modes = AXES.map(axis => document.querySelector('.command-mode[data-axis="' + axis + '"]').value);
  try {
    const values = AXES.map(axis => {
      const input = document.querySelector('.command-value[data-axis="' + axis + '"]');
      // Number("") is 0: an emptied field must not command a zero.
      if (!input.disabled && input.value.trim() === "") throw new Error(AXIS_LABELS[axis] + " needs a value");
      return Number(input.value);
    });
    const result = await request("command", {
      modes: modes,
      values: values,
      altitude: $("command-altitude").checked,
      frame: $("command-frame").value,
      timeout: Number($("command-timeout").value)
    });
    const parts = AXES.flatMap((axis, index) => result.modes[index] === "KEEP" ? [] :
      [axis + " " + result.modes[index].toLowerCase() +
        (result.modes[index] === "HOLD" ? "" : " " + Number(result.values[index]).toPrecision(4))]);
    $("last-command").textContent = parts.join(", ") + (result.altitude ? " · z altitude" : "") +
      (result.timeout ? " · " + result.frame + " frame · " + result.timeout + " s timeout" : "");
    toast("Command sent");
  } catch (error) {
    toast(error.message, true);
  }
}

// Layout

function saveLayout() {
  const layout = [...document.querySelectorAll(".tile")].map(tile => ({id: tile.id, width: tile.dataset.w, height: tile.dataset.h}));
  localStorage.setItem("dashboard-layout", JSON.stringify(layout));
}

function sizeTile(tile) {
  tile.style.setProperty("--w", tile.dataset.w);
  tile.style.setProperty("--h", tile.dataset.h);
}

function setupLayout() {
  const root = $("dashboard");
  for (const item of JSON.parse(localStorage.getItem("dashboard-layout") || "[]")) {
    const tile = $(item.id);
    if (!tile) continue;
    tile.dataset.w = item.width;
    tile.dataset.h = item.height;
    root.append(tile);
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
        tile.dataset.w = Math.max(3, Math.min(12, startWidth + Math.round((moveEvent.clientX - startX) / (root.clientWidth / 12))));
        tile.dataset.h = Math.max(3, Math.min(12, startHeight + Math.round((moveEvent.clientY - startY) / 84)));
        sizeTile(tile);
        scheduleDraw();
      };
      handle.addEventListener("pointermove", move);
      // Not pointerup: a cancelled pointer (touch) never sends one, and move would stay attached.
      handle.addEventListener("lostpointercapture", () => {
        handle.removeEventListener("pointermove", move);
        saveLayout();
      }, {once: true});
    });
  });
}

function setPathView(view) {
  state.pathView = view;
  localStorage.setItem("dashboard-path-view", view);
  document.querySelectorAll("[data-path-view]").forEach(button => button.classList.toggle("active", button.dataset.pathView === view));
  scheduleDraw();
}

// Wiring

function rerenderParameters() {
  try { harvestVisibleRows(); } catch (_error) { /* keep the invalid row's edit out of the staged set */ }
  renderParameters(true);
}

$("node-select").addEventListener("change", () => {
  rerenderParameters();
  request("load_parameters", {node: $("node-select").value}).catch(error => toast(error.message, true));
});
$("parameter-filter").addEventListener("input", rerenderParameters);
$("modified-only").addEventListener("change", rerenderParameters);
$("reload").addEventListener("click", reloadConfiguration);
$("apply-all").addEventListener("click", () => applyRuntime());
$("save-profile").addEventListener("click", saveProfile);

$("graph-axis").innerHTML = AXES.map(axis => '<option value="' + axis + '">' + AXIS_LABELS[axis] + "</option>").join("");
$("graph-axis").value = "z";
syncViewControls();
$("graph-axis").addEventListener("change", () => selectView());
$("graph-view").addEventListener("change", () => selectView());
$("signal-filter").addEventListener("input", () => { state.signalKey = ""; renderSignals(); });
$("pause").addEventListener("click", () => {
  state.paused = !state.paused;
  $("pause").textContent = state.paused ? "Resume" : "Pause";
});
$("clear").addEventListener("click", () => { state.history = []; scheduleDraw(); });
$("window-select").addEventListener("change", scheduleDraw);
const savedScale = localStorage.getItem("dashboard-graph-scale") || "auto";
$("scale-select").value = [...$("scale-select").options].some(option => option.value === savedScale) ? savedScale : "auto";
const savedCustomScale = JSON.parse(localStorage.getItem("dashboard-custom-scale") || "null");
if (savedCustomScale && savedCustomScale.min < savedCustomScale.max) {
  $("scale-min").value = savedCustomScale.min;
  $("scale-max").value = savedCustomScale.max;
}
$("custom-scale").hidden = $("scale-select").value !== "custom";
$("scale-select").addEventListener("change", () => {
  localStorage.setItem("dashboard-graph-scale", $("scale-select").value);
  $("custom-scale").hidden = $("scale-select").value !== "custom";
  scheduleDraw();
});
$("scale-min").addEventListener("input", updateCustomScale);
$("scale-max").addEventListener("input", updateCustomScale);
$("zoom-in").addEventListener("click", () => zoomGraph(0.67));
$("zoom-out").addEventListener("click", () => zoomGraph(1.5));
$("export").addEventListener("click", exportCsv);

document.querySelectorAll("[data-path-view]").forEach(button => {
  button.addEventListener("click", () => setPathView(button.dataset.pathView));
});
$("clear-path").addEventListener("click", () => {
  state.measuredTrail = [];
  state.referenceTrail = [];
  scheduleDraw();
});
$("telemetry-filter").addEventListener("input", renderTelemetry);

$("routine-kind").addEventListener("change", () => updateRoutineForm(true));
$("routine-axis").addEventListener("change", () => updateRoutineForm(true));
$("routine-amplitude").addEventListener("input", updateRoutineHint);
$("start-routine").addEventListener("click", startRoutine);
$("stop-routine").addEventListener("click", () => routineRequest("stop_routine", "Stopped; holding here"));
$("hold-here").addEventListener("click", () => routineRequest("hold", "Holding here"));
$("zero-pose").addEventListener("click", zeroPose);
$("send-command").addEventListener("click", sendCommand);

$("reset-layout").addEventListener("click", () => {
  localStorage.removeItem("dashboard-layout");
  location.reload();
});
document.addEventListener("keydown", event => {
  if ((event.ctrlKey || event.metaKey) && !event.altKey && event.key.toLowerCase() === "s") {
    event.preventDefault();
    if (event.shiftKey) saveProfile();
    else applyRuntime();
  }
});
window.addEventListener("resize", scheduleDraw);

setupLayout();
setPathView(state.pathView);
buildCommandGrid();
updateRoutineForm(true);
renderRoutine();
updateSaveControls();
connect();
