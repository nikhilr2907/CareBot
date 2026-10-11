"use strict";

const elements = {
  connectionBadge: document.getElementById("connectionBadge"),
  serialPort: document.getElementById("serialPort"),
  armToggle: document.getElementById("armToggle"),
  stopButton: document.getElementById("stopButton"),
  motionState: document.getElementById("motionState"),
  contextLabel: document.getElementById("contextLabel"),
  heightInput: document.getElementById("heightInput"),
  speedInput: document.getElementById("speedInput"),
  durationInput: document.getElementById("durationInput"),
  trimInput: document.getElementById("trimInput"),
  trimValue: document.getElementById("trimValue"),
  m2Speed: document.getElementById("m2Speed"),
  m5Speed: document.getElementById("m5Speed"),
  learnedTrials: document.getElementById("learnedTrials"),
  smoothTrials: document.getElementById("smoothTrials"),
  runUp: document.getElementById("runUp"),
  runDown: document.getElementById("runDown"),
  feedbackTiming: document.getElementById("feedbackTiming"),
  noteInput: document.getElementById("noteInput"),
  recommendationText: document.getElementById("recommendationText"),
  jogSpeed: document.getElementById("jogSpeed"),
  jogDuration: document.getElementById("jogDuration"),
  logPath: document.getElementById("logPath"),
  trialRows: document.getElementById("trialRows"),
  terminal: document.getElementById("terminal"),
  toast: document.getElementById("toast"),
};

const app = {
  state: null,
  eventId: 0,
  lastDirection: "UP",
  trials: new Map(),
  trialOrder: [],
  toastTimer: null,
  recommendationRequest: 0,
};

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const payload = await response.json();
  if (!response.ok || payload.ok === false) {
    throw new Error(payload.error || `Request failed (${response.status})`);
  }
  return payload;
}

function numericValue(element, fallback) {
  const value = Number(element.value);
  return Number.isFinite(value) ? value : fallback;
}

function clamp(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

function calculatedSpeeds() {
  const base = clamp(Math.round(numericValue(elements.speedInput, 1000)), 300, 4000);
  const trim = clamp(numericValue(elements.trimInput, 0), -30, 30) / 100;
  if (trim >= 0) {
    return { m2: base, m5: Math.max(100, Math.round(base * (1 - trim))) };
  }
  return { m2: Math.max(100, Math.round(base * (1 + trim))), m5: base };
}

function updateSpeedReadout() {
  const speeds = calculatedSpeeds();
  const trim = numericValue(elements.trimInput, 0);
  elements.trimValue.textContent = `${trim > 0 ? "+" : ""}${trim}%`;
  elements.m2Speed.textContent = `${speeds.m2} Hz`;
  elements.m5Speed.textContent = `${speeds.m5} Hz`;
}

function showToast(message) {
  elements.toast.textContent = message;
  elements.toast.classList.add("visible");
  window.clearTimeout(app.toastTimer);
  app.toastTimer = window.setTimeout(() => elements.toast.classList.remove("visible"), 3200);
}

function setControlsEnabled() {
  const connected = Boolean(app.state && app.state.connected);
  const armed = Boolean(app.state && app.state.armed);
  elements.armToggle.disabled = !connected;
  elements.armToggle.checked = armed;
  document.querySelectorAll(".motion-button, .jog-side button").forEach((button) => {
    button.disabled = !connected || !armed;
  });
  document.querySelectorAll(".feedback-button").forEach((button) => {
    button.disabled = !app.state || !app.state.last_trial;
  });
}

function updateState(state) {
  app.state = state;
  elements.connectionBadge.textContent = state.connected ? "Connected" : "Disconnected";
  elements.connectionBadge.className = `status-badge ${state.connected ? "connected" : "disconnected"}`;
  elements.serialPort.textContent = state.connected
    ? `${state.serial_port} at 115200 baud`
    : "Waiting for STM32";
  elements.logPath.textContent = state.log_path || "";

  if (state.active_trial) {
    const trial = state.active_trial;
    const elapsed = Math.max(0, Date.now() / 1000 - trial.started_at);
    const remaining = Math.max(0, trial.duration_ms / 1000 - elapsed);
    elements.motionState.textContent = `${trial.direction} ${remaining.toFixed(1)} s`;
    elements.motionState.classList.add("running");
    elements.feedbackTiming.textContent = `Trial ${trial.id} is active`;
  } else {
    elements.motionState.textContent = state.armed ? "Armed" : "Idle";
    elements.motionState.classList.remove("running");
    elements.feedbackTiming.textContent = state.last_trial
      ? `Latest trial ${state.last_trial.id}`
      : "No trial selected";
  }
  setControlsEnabled();
}

function terminalLine(event) {
  const line = document.createElement("div");
  const timeText = new Date(event.time * 1000).toLocaleTimeString([], {
    hour12: false,
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
  line.className = event.kind;
  line.textContent = `${timeText}  ${event.message}`;
  elements.terminal.appendChild(line);
  while (elements.terminal.childElementCount > 250) {
    elements.terminal.firstElementChild.remove();
  }
  elements.terminal.scrollTop = elements.terminal.scrollHeight;
}

function rememberTrial(trial) {
  if (!trial || !trial.id) return;
  const current = app.trials.get(trial.id) || {};
  app.trials.set(trial.id, { ...current, ...trial });
  if (!app.trialOrder.includes(trial.id)) {
    app.trialOrder.unshift(trial.id);
    app.trialOrder = app.trialOrder.slice(0, 20);
  }
  renderTrials();
}

function renderTrials() {
  elements.trialRows.replaceChildren();
  if (app.trialOrder.length === 0) {
    const row = document.createElement("tr");
    const cell = document.createElement("td");
    cell.colSpan = 5;
    cell.className = "empty-state";
    cell.textContent = "No trials recorded";
    row.appendChild(cell);
    elements.trialRows.appendChild(row);
    return;
  }

  for (const id of app.trialOrder) {
    const trial = app.trials.get(id);
    const row = document.createElement("tr");
    const values = [
      new Date(trial.started_at * 1000).toLocaleTimeString([], { hour12: false }),
      `${trial.kind === "pair" ? trial.direction : trial.kind.replace("_", " ")}`,
      `${trial.m2_hz} Hz`,
      `${trial.m5_hz} Hz`,
      trial.feedback ? trial.feedback.replaceAll("_", " ") : "-",
    ];
    for (const value of values) {
      const cell = document.createElement("td");
      cell.textContent = value;
      row.appendChild(cell);
    }
    elements.trialRows.appendChild(row);
  }
}

function processEvent(event) {
  terminalLine(event);
  if (event.kind === "trial" && event.data && event.data.trial) {
    rememberTrial(event.data.trial);
  }
  if (event.kind === "feedback" && event.data && event.data.trial_id) {
    const trial = app.trials.get(event.data.trial_id);
    if (trial) {
      trial.feedback = event.message.toLowerCase().replaceAll(" ", "_");
      renderTrials();
    }
  }
}

async function pollEvents() {
  try {
    const payload = await api(`/api/events?after=${app.eventId}`);
    for (const event of payload.events) {
      app.eventId = Math.max(app.eventId, event.id);
      processEvent(event);
    }
  } catch (error) {
    showToast(error.message);
  }
}

async function pollState() {
  try {
    const state = await api("/api/state");
    updateState(state);
  } catch (error) {
    showToast(error.message);
  }
}

async function loadRecommendation(direction = app.lastDirection, applyValues = false) {
  const requestId = ++app.recommendationRequest;
  const height = clamp(numericValue(elements.heightInput, 0), 0, 100);
  try {
    const recommendation = await api(
      `/api/recommendation?direction=${encodeURIComponent(direction)}&height=${height}`
    );
    if (requestId !== app.recommendationRequest) return;
    elements.contextLabel.textContent = recommendation.context;
    elements.learnedTrials.textContent = recommendation.trials;
    elements.smoothTrials.textContent = recommendation.smooth;
    if (applyValues) {
      elements.speedInput.value = recommendation.base_hz;
      elements.trimInput.value = recommendation.trim_percent;
      updateSpeedReadout();
    }
    if (recommendation.trials > 0) {
      const signedTrim = recommendation.trim_percent > 0
        ? `+${recommendation.trim_percent}`
        : `${recommendation.trim_percent}`;
      elements.recommendationText.textContent =
        `${recommendation.context}: ${recommendation.base_hz} Hz base, ${signedTrim}% right trim, ` +
        `${recommendation.smooth}/${recommendation.trials} smooth.`;
    } else {
      elements.recommendationText.textContent =
        "No learned adjustment for this height band yet.";
    }
  } catch (error) {
    showToast(error.message);
  }
}

async function setArmed(armed) {
  try {
    const payload = await api("/api/arm", {
      method: "POST",
      body: JSON.stringify({ armed }),
    });
    updateState(payload.result);
  } catch (error) {
    elements.armToggle.checked = false;
    showToast(error.message);
  }
}

async function stopAll(reason = "operator") {
  try {
    const payload = await api("/api/stop", {
      method: "POST",
      body: JSON.stringify({ reason }),
    });
    updateState(payload.result);
  } catch (error) {
    showToast(error.message);
  }
}

async function runTrial(direction) {
  app.lastDirection = direction;
  const request = {
    direction,
    height_percent: numericValue(elements.heightInput, 0),
    base_hz: numericValue(elements.speedInput, 1000),
    trim_percent: numericValue(elements.trimInput, 0),
    duration_ms: numericValue(elements.durationInput, 1000),
  };
  try {
    const payload = await api("/api/trial", {
      method: "POST",
      body: JSON.stringify(request),
    });
    rememberTrial(payload.result);
    await pollState();
    await loadRecommendation(direction, false);
  } catch (error) {
    showToast(error.message);
  }
}

async function jog(side, direction) {
  try {
    const payload = await api("/api/jog", {
      method: "POST",
      body: JSON.stringify({
        side,
        direction,
        hz: numericValue(elements.jogSpeed, 500),
        duration_ms: numericValue(elements.jogDuration, 250),
      }),
    });
    rememberTrial(payload.result);
    await pollState();
  } catch (error) {
    showToast(error.message);
  }
}

async function recordFeedback(label) {
  try {
    const payload = await api("/api/feedback", {
      method: "POST",
      body: JSON.stringify({ label, note: elements.noteInput.value }),
    });
    const result = payload.result;
    const id = result.trial.id;
    const trial = app.trials.get(id);
    if (trial) {
      trial.feedback = label;
      renderTrials();
    }
    elements.noteInput.value = "";
    if (result.recommendation) {
      elements.speedInput.value = result.recommendation.base_hz;
      elements.trimInput.value = result.recommendation.trim_percent;
      updateSpeedReadout();
      await loadRecommendation(result.trial.direction, false);
    }
  } catch (error) {
    showToast(error.message);
  }
}

elements.armToggle.addEventListener("change", () => setArmed(elements.armToggle.checked));
elements.stopButton.addEventListener("click", () => stopAll("stop button"));
elements.runUp.addEventListener("click", () => runTrial("UP"));
elements.runDown.addEventListener("click", () => runTrial("DOWN"));
elements.speedInput.addEventListener("input", updateSpeedReadout);
elements.trimInput.addEventListener("input", updateSpeedReadout);
elements.heightInput.addEventListener("change", () => loadRecommendation(app.lastDirection, true));

document.querySelectorAll("[data-feedback]").forEach((button) => {
  button.addEventListener("click", () => recordFeedback(button.dataset.feedback));
});

document.querySelectorAll("[data-jog-side]").forEach((button) => {
  button.addEventListener("click", () => jog(button.dataset.jogSide, button.dataset.jogDirection));
});

document.addEventListener("keydown", (event) => {
  const target = event.target;
  if (target instanceof HTMLInputElement || target instanceof HTMLTextAreaElement) return;
  if (event.repeat) return;
  const key = event.key.toLowerCase();
  if (event.code === "Space") {
    event.preventDefault();
    stopAll("keyboard stop");
  } else if (event.key === "ArrowUp") {
    event.preventDefault();
    runTrial("UP");
  } else if (event.key === "ArrowDown") {
    event.preventDefault();
    runTrial("DOWN");
  } else if (key === "a") {
    recordFeedback("left_stall");
  } else if (key === "d") {
    recordFeedback("right_stall");
  } else if (key === "s") {
    recordFeedback("smooth");
  } else if (key === "b") {
    recordFeedback("both_stall");
  } else if (key === "v") {
    recordFeedback("vibration");
  }
});

window.addEventListener("beforeunload", () => {
  const body = new Blob([JSON.stringify({ reason: "browser closed" })], {
    type: "application/json",
  });
  navigator.sendBeacon("/api/stop", body);
});

updateSpeedReadout();
pollState().then(() => loadRecommendation("UP", true));
pollEvents();
window.setInterval(pollState, 400);
window.setInterval(pollEvents, 250);
