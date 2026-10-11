"use strict";

const $ = (id) => document.getElementById(id);
const elements = {
  serialPort: $("serialPort"), connection: $("connection"), armToggle: $("armToggle"), stopButton: $("stopButton"),
  studyTimer: $("studyTimer"), studyTimerLabel: $("studyTimerLabel"),
  studyTimerDetail: $("studyTimerDetail"), studyTimerValue: $("studyTimerValue"),
  safetyBanner: $("safetyBanner"), safetyTitle: $("safetyTitle"), safetyDetail: $("safetyDetail"),
  pausedRecovery: $("pausedRecovery"), pausedRecoveryTitle: $("pausedRecoveryTitle"),
  pausedRecoveryDetail: $("pausedRecoveryDetail"), pausedRecoveryHelp: $("pausedRecoveryHelp"),
  resumeMotion: $("resumeMotion"), returnMotion: $("returnMotion"),
  actionStrip: $("actionStrip"), motionState: $("motionState"),
  group: $("group"), preset: $("preset"), presetButtons: $("presetButtons"), presetField: $("presetField"),
  mechanismSignalHint: $("mechanismSignalHint"), mechanismSignalComparison: $("mechanismSignalComparison"),
  useBenchSignal: $("useBenchSignal"),
  directionA: $("directionA"), directionB: $("directionB"), directionAField: $("directionAField"),
  directionBField: $("directionBField"), motorALabel: $("motorALabel"), motorBLabel: $("motorBLabel"),
  steps: $("steps"), maxHz: $("maxHz"), ramp: $("ramp"), rampValue: $("rampValue"),
  duration: $("duration"), movementSummary: $("movementSummary"), runMotion: $("runMotion"), runReverse: $("runReverse"),
  resetCounters: $("resetCounters"), m1Steps: $("m1Steps"), m3Steps: $("m3Steps"), m4Steps: $("m4Steps"), m6Steps: $("m6Steps"), m7Steps: $("m7Steps"),
  tofDistance: $("tofDistance"), tofDetail: $("tofDetail"), readTof: $("readTof"),
  flagLabel: $("flagLabel"), addFlag: $("addFlag"), clearFlags: $("clearFlags"), comparison: $("comparison"), flagTimeline: $("flagTimeline"),
  lac1State: $("lac1State"), lac2State: $("lac2State"), lac1: $("lac1"), lac2: $("lac2"), lacBoth: $("lacBoth"), lacSpeed: $("lacSpeed"),
  note: $("note"), addNote: $("addNote"), logPath: $("logPath"), terminal: $("terminal"), toast: $("toast"),
  actionEditor: $("actionEditor"), editorTitle: $("editorTitle"), editActionId: $("editActionId"), editName: $("editName"),
  editGroup: $("editGroup"), editPreset: $("editPreset"), editSteps: $("editSteps"), editHz: $("editHz"),
  editGroupField: $("editGroupField"), editPresetField: $("editPresetField"), editStepsField: $("editStepsField"),
  editHzField: $("editHzField"), editRampField: $("editRampField"), lacEditorFields: $("lacEditorFields"),
  editMotorOffsets: $("editMotorOffsets"), editM1ExtraSteps: $("editM1ExtraSteps"),
  editM6ExtraSteps: $("editM6ExtraSteps"), motorOffsetSummary: $("motorOffsetSummary"),
  editLacPercent: $("editLacPercent"), editLacSpeed: $("editLacSpeed"), editorLacCurrent: $("editorLacCurrent"),
  editLacClampSteps: $("editLacClampSteps"), editLacClampHz: $("editLacClampHz"),
  lacSupportSummary: $("lacSupportSummary"),
  editorRampPreview: $("editorRampPreview"), editorRampSummary: $("editorRampSummary"),
  editorRampLine: $("editorRampLine"), editorRampDetail: $("editorRampDetail"),
  editorRuntime: $("editorRuntime"), editorEstimatedDuration: $("editorEstimatedDuration"),
  editorEstimateDetail: $("editorEstimateDetail"),
  clampProfileEditor: $("clampProfileEditor"), clampProfileToggle: $("clampProfileToggle"),
  clampProfileBody: $("clampProfileBody"), clampProfileGraph: $("clampProfileGraph"),
  clampProfileLine: $("clampProfileLine"), clampProfileMarkers: $("clampProfileMarkers"),
  clampGraphEndLabel: $("clampGraphEndLabel"), clampProfileScale: $("clampProfileScale"),
  clampProfileRows: $("clampProfileRows"), addClampPoint: $("addClampPoint"),
  clampProfileSummary: $("clampProfileSummary"),
  clampPairOptions: $("clampPairOptions"), clampPairName: $("clampPairName"), copyClampFromPair: $("copyClampFromPair"),
  syncClampToPair: $("syncClampToPair"),
  editRamp: $("editRamp"), editTofField: $("editTofField"), editTofMode: $("editTofMode"),
  editTofMin: $("editTofMin"), editTofIdeal: $("editTofIdeal"), editTofMax: $("editTofMax"),
  tofRangeFields: $("tofRangeFields"), editorLiveTof: $("editorLiveTof"), editorLiveTofDetail: $("editorLiveTofDetail"),
  editorTofRaw: $("editorTofRaw"), editorTofSamples: $("editorTofSamples"), editorTofErrors: $("editorTofErrors"),
  editorTofUpdated: $("editorTofUpdated"), refreshEditorTof: $("refreshEditorTof"),
  captureTof: $("captureTof"), editInstructions: $("editInstructions"), deleteAction: $("deleteAction"), saveEditedAction: $("saveEditedAction"),
  demoPosition: $("demoPosition"), demoDelta: $("demoDelta"), demoTarget: $("demoTarget"),
  demoZero: $("demoZero"), rebaseDemo: $("rebaseDemo"), demoTofResult: $("demoTofResult"), pairHelp: $("pairHelp"),
  tofReferenceHelp: $("tofReferenceHelp"),
};

const mappings = {
  mechanism: {
    label: "Side mechanisms", motors: [1, 6], motorLabels: ["M1 · physical right", "M6 · physical left"],
    presets: {
      clamp: { label: "Close / clamp", dirs: ["R", "R"] },
      release: { label: "Open / release", dirs: ["F", "F"] },
      left: { label: "Shift system left", dirs: ["R", "F"] },
      right: { label: "Shift system right", dirs: ["F", "R"] },
    },
  },
  belts: {
    label: "Conveyor belts", motors: [3, 4], motorLabels: ["M3 belt", "M4 belt"],
    presets: {
      out: { label: "Feed box out", dirs: ["R", "F"] },
      in: { label: "Feed box in", dirs: ["F", "R"] },
    },
  },
  m1: { label: "M1 only", motors: [1], motorLabels: ["M1 · physical right"], presets: { f: { label: "F", dirs: ["F"] }, r: { label: "R", dirs: ["R"] } } },
  m6: { label: "M6 only", motors: [6], motorLabels: ["M6 · physical left"], presets: { f: { label: "F", dirs: ["F"] }, r: { label: "R", dirs: ["R"] } } },
  m3: { label: "M3 only", motors: [3], motorLabels: ["M3 belt"], presets: { f: { label: "F", dirs: ["F"] }, r: { label: "R", dirs: ["R"] } } },
  m4: { label: "M4 only", motors: [4], motorLabels: ["M4 belt"], presets: { f: { label: "F", dirs: ["F"] }, r: { label: "R", dirs: ["R"] } } },
  m7: { label: "M7 geared angle", motors: [7], motorLabels: ["M7 · geared X-axis angle"], presets: { cw: { label: "Clockwise", dirs: ["F"] }, ccw: { label: "Counterclockwise", dirs: ["R"] } } },
};

const fixedActionDefinitions = {
  "clamp-box": "Fixed: Clamp/close moves M1 R + M6 R inward, towards each other.",
  "release-box": "Fixed: Release/open moves M1 F + M6 F outward, away from each other.",
  "move-left": "Fixed: Move system left uses M1 R + M6 F.",
  "move-right": "Fixed: Move system right uses M1 F + M6 R.",
  "box-out": "Fixed: Conveyor box out uses M3 R + M4 F.",
  "box-in": "Fixed: Conveyor box in uses M3 F + M4 R.",
};

const app = {
  state: null, eventId: 0, toastTimer: null, actionsSignature: "", flagsSignature: "",
  compareA: null, compareB: null, editorKind: "stepper", clampProfilePoints: [],
  clampProfileTotal: 100, clampProfileDrag: null, clampProfileJustDragged: false,
  editorPairAction: null,
  editorEstimateTimer: null, editorEstimateSequence: 0,
  trialEvents: [],
};

const MAX_ACTION_STEPS = 10_000_000;
const MIN_ACTION_HZ = 100;
const MAX_ACTION_HZ = 4_000;

async function api(path, options = {}) {
  const response = await fetch(path, { headers: { "Content-Type": "application/json" }, ...options });
  const payload = await response.json();
  if (!response.ok || payload.ok === false) throw new Error(payload.error || `Request failed (${response.status})`);
  return payload;
}

async function post(path, body) {
  try {
    const payload = await api(path, { method: "POST", body: JSON.stringify(body) });
    return payload.result;
  } catch (error) {
    showToast(error.message);
    throw error;
  }
}

function showToast(message) {
  elements.toast.textContent = message;
  elements.toast.classList.add("visible");
  clearTimeout(app.toastTimer);
  app.toastTimer = setTimeout(() => elements.toast.classList.remove("visible"), 3600);
}

function numberValue(element, fallback) {
  const value = Number(element.value);
  return Number.isFinite(value) ? value : fallback;
}

function signed(value) {
  const number = Number(value) || 0;
  return number > 0 ? `+${number.toLocaleString()}` : number.toLocaleString();
}

function trialClock(seconds) {
  const safe = Math.max(0, Number(seconds) || 0);
  const minutes = Math.floor(safe / 60);
  const remainder = (safe - minutes * 60).toFixed(1).padStart(4, "0");
  return `${String(minutes).padStart(2, "0")}:${remainder}`;
}

function currentTrialReference() {
  let reference = null;
  let excluded = false;
  for (const flag of app.state?.step_flags || []) {
    const label = String(flag.label || "");
    if (/timer starts on (?:the )?next conveyor(?:\s*:)? box out/i.test(label)) {
      reference = flag;
      excluded = false;
    } else if (reference && /\bexcluded\b/i.test(label) && Number(flag.captured_at) >= Number(reference.captured_at)) {
      reference = null;
      excluded = true;
    }
  }
  return { reference, excluded };
}

function trialMotionEvents(referenceAt) {
  return app.trialEvents
    .filter((event) => Number(event.time) >= referenceAt && event.kind === "motion" && event.data?.motion)
    .map((event) => ({ event, motion: event.data.motion }));
}

function trialTimerState() {
  const marker = currentTrialReference();
  if (!marker.reference) {
    // A saved Conveyor: box out is itself an unambiguous timer start. This
    // fallback lets a fresh trial begin after an EXCLUDED marker without
    // requiring the operator to place another timeline flag first.
    const actions = app.state?.actions || [];
    const boxOutName = actions.find((item) => item.id === "box-out")?.name || "Conveyor: box out";
    const boxInName = actions.find((item) => item.id === "box-in")?.name || "Conveyor: box in";
    const latestFlagAt = Math.max(0, ...(app.state?.step_flags || []).map((flag) => Number(flag.captured_at) || 0));
    const automaticStart = [...app.trialEvents].reverse().find((event) => {
      const motion = event.data?.motion;
      return event.kind === "motion" && motion?.name === boxOutName
        && motion.state === "running" && Number(motion.started_at) > latestFlagAt;
    });
    if (automaticStart) {
      const startAt = Number(automaticStart.data.motion.started_at);
      const automaticFinish = app.trialEvents.find((event) => {
        const motion = event.data?.motion;
        return event.kind === "motion" && motion?.name === boxInName
          && motion.state === "complete" && Number(motion.finished_at) >= startAt;
      });
      if (automaticFinish) {
        return {
          state: "complete",
          label: "Automatic trial",
          startAt,
          endAt: Number(automaticFinish.data.motion.finished_at),
        };
      }
      return { state: "running", label: "Automatic trial", startAt };
    }
    return { state: marker.excluded ? "excluded" : "unarmed" };
  }
  const referenceAt = Number(marker.reference.captured_at);
  const referenceLabel = String(marker.reference.label || "Active trial");
  const explicitStartMatch = referenceLabel.match(/\bbegan=([0-9]+(?:\.[0-9]+)?)/i);
  const explicitStartAt = explicitStartMatch ? Number(explicitStartMatch[1]) : null;
  // A trial can be flagged just after Box out has already begun (for example
  // while an endpoint note is being saved).  A short explicit `began=` epoch
  // preserves the accepted server timestamp so the live clock can recover
  // without stopping or replaying robot motion.
  const eventReferenceAt = Number.isFinite(explicitStartAt)
    ? Math.min(referenceAt, explicitStartAt) - 0.001
    : referenceAt;
  const actions = app.state?.actions || [];
  const boxOutName = actions.find((item) => item.id === "box-out")?.name || "Conveyor: box out";
  const boxInName = actions.find((item) => item.id === "box-in")?.name || "Conveyor: box in";
  const motions = trialMotionEvents(eventReferenceAt);
  const start = motions.find(({ motion }) =>
    motion.name === boxOutName && motion.state === "running" && Number(motion.started_at) >= eventReferenceAt
  );
  const label = referenceLabel
    .replace(/\s*;\s*timer starts.*$/i, "")
    .replace(/\s*timer starts.*$/i, "");
  if (!start && !Number.isFinite(explicitStartAt)) return { state: "waiting", label, referenceAt };
  const startAt = Number.isFinite(explicitStartAt)
    ? explicitStartAt
    : Number(start.motion.started_at);
  const finish = motions.find(({ motion }) =>
    motion.name === boxInName && motion.state === "complete" && Number(motion.finished_at) >= startAt
  );
  if (finish) {
    return { state: "complete", label, startAt, endAt: Number(finish.motion.finished_at) };
  }
  return { state: "running", label, startAt };
}

function renderStudyTimer() {
  if (!elements.studyTimer) return;
  const timer = trialTimerState();
  elements.studyTimer.className = `study-timer ${timer.state}`;
  if (timer.state === "waiting") {
    elements.studyTimerLabel.textContent = "Trial timer · READY";
    elements.studyTimerValue.textContent = "00:00.0";
    elements.studyTimerDetail.textContent = `${timer.label} · starts on Conveyor: box out`;
    return;
  }
  if (timer.state === "running") {
    const active = app.state?.active_motion?.name;
    elements.studyTimerLabel.textContent = "Trial timer · RUNNING";
    elements.studyTimerValue.textContent = trialClock(Date.now() / 1000 - timer.startAt);
    elements.studyTimerDetail.textContent = active
      ? `Current action: ${active} · stops only when Conveyor: box in completes`
      : "Between actions · stops only when Conveyor: box in completes";
    return;
  }
  if (timer.state === "complete") {
    elements.studyTimerLabel.textContent = "Trial timer · STOPPED";
    elements.studyTimerValue.textContent = trialClock(timer.endAt - timer.startAt);
    elements.studyTimerDetail.textContent = `${timer.label} · Conveyor: box in complete`;
    return;
  }
  elements.studyTimerLabel.textContent = timer.state === "excluded" ? "Trial timer · EXCLUDED" : "Trial timer · not armed";
  elements.studyTimerValue.textContent = "00:00.0";
  elements.studyTimerDetail.textContent = timer.state === "excluded"
    ? "Waiting for a fresh trial start flag"
    : "Flag a trial start to enable automatic timing";
}

function updateAge(timestamp) {
  if (!timestamp) return "No reply yet";
  const seconds = Math.max(0, Math.round(Date.now() / 1000 - Number(timestamp)));
  if (seconds < 2) return "Just now";
  if (seconds < 60) return `${seconds}s ago`;
  return `${Math.floor(seconds / 60)}m ago`;
}

function formatDuration(seconds) {
  const value = Math.max(0, Number(seconds) || 0);
  if (value < 1) return `${value.toFixed(2)} seconds`;
  if (value < 60) return `${value.toFixed(value < 10 ? 2 : 1)} seconds`;
  if (value < 3600) {
    const minutes = Math.floor(value / 60);
    const remainder = Math.round(value - minutes * 60);
    return `${minutes}m ${remainder}s`;
  }
  const hours = Math.floor(value / 3600);
  const minutes = Math.round((value - hours * 3600) / 60);
  return `${hours}h ${minutes}m`;
}

function selectedManualPreset() {
  return mappings[elements.group.value].presets[elements.preset.value];
}

function manualPayload(reverse = false) {
  return {
    name: `Direct positioning: ${mappings[elements.group.value].label}${reverse ? " opposite" : ""}`,
    group: elements.group.value,
    direction_a: elements.directionA.value,
    direction_b: elements.directionB.value,
    steps: Math.round(numberValue(elements.steps, 100)),
    max_hz: Math.round(numberValue(elements.maxHz, 300)),
    ramp_percent: Math.round(numberValue(elements.ramp, 20)),
    instructions: "Temporary direct positioning; not saved to Actions",
    reverse,
  };
}

function updateManualSummary() {
  const preset = selectedManualPreset();
  const movement = preset?.label || mappings[elements.group.value].label;
  const steps = Math.max(1, Math.round(numberValue(elements.steps, 100)));
  const speed = Math.max(100, Math.round(numberValue(elements.maxHz, 300)));
  const ramp = Math.max(0, Math.min(45, Math.round(numberValue(elements.ramp, 20))));
  const rampFactor = ramp / 100;
  const rampSteps = steps * rampFactor;
  const fractions = [.2, .4, .6, .8, 1];
  const rampTime = rampSteps ? fractions.reduce((sum, fraction) => sum + (rampSteps / 5) / Math.max(100, speed * fraction), 0) : 0;
  const seconds = (steps - 2 * rampSteps) / speed + 2 * rampTime;
  elements.rampValue.textContent = `${ramp}%`;
  elements.movementSummary.textContent = `${movement} · ${steps.toLocaleString()} steps · ${speed.toLocaleString()} Hz`;
  elements.duration.textContent = ramp
    ? `About ${seconds.toFixed(2)} seconds · linear ramp over ${ramp}% at each end`
    : `About ${seconds.toFixed(2)} seconds · one continuous command, no ramp`;
  const relative = steps / 20000 * 100;
  const relativeLabel = relative < 0.1 ? "less than 0.1" : relative.toFixed(relative < 10 ? 1 : 0);
  elements.mechanismSignalComparison.textContent = `${steps.toLocaleString()} pulses is ${relativeLabel}% of yesterday’s visible 20,000-pulse test; current rate ${speed.toLocaleString()} vs 4,000 Hz.`;
}

function updateManualGroup() {
  const group = mappings[elements.group.value];
  elements.mechanismSignalHint.hidden = elements.group.value !== "mechanism";
  elements.motorALabel.textContent = group.motorLabels[0];
  elements.motorBLabel.textContent = group.motorLabels[1] || "";
  elements.directionBField.hidden = group.motors.length === 1;
  elements.preset.replaceChildren();
  elements.presetButtons.replaceChildren();
  for (const [key, preset] of Object.entries(group.presets)) {
    const option = document.createElement("option");
    option.value = key; option.textContent = preset.label; elements.preset.appendChild(option);
    const button = document.createElement("button");
    button.type = "button"; button.className = "preset-button"; button.textContent = preset.label;
    button.dataset.manualPreset = key;
    button.onclick = () => { elements.preset.value = key; updateManualPreset(); };
    elements.presetButtons.appendChild(button);
  }
  updateManualPreset();
}

function updateManualPreset() {
  const preset = selectedManualPreset();
  if (preset) {
    elements.directionA.value = preset.dirs[0];
    if (preset.dirs.length > 1) elements.directionB.value = preset.dirs[1];
  }
  document.querySelectorAll("[data-manual-preset]").forEach((button) => {
    const active = button.dataset.manualPreset === elements.preset.value;
    button.classList.toggle("active", active);
    button.setAttribute("aria-pressed", String(active));
  });
  updateManualSummary();
}

function syncPresetFromDirections() {
  const group = mappings[elements.group.value];
  const dirs = [elements.directionA.value];
  if (group.motors.length > 1) dirs.push(elements.directionB.value);
  const match = Object.entries(group.presets).find(([, preset]) => preset.dirs.join() === dirs.join());
  if (match) elements.preset.value = match[0];
  document.querySelectorAll("[data-manual-preset]").forEach((button) => {
    button.classList.toggle("active", button.dataset.manualPreset === elements.preset.value && Boolean(match));
  });
  updateManualSummary();
}

function actionSubtitle(action) {
  const demoPosition = Number(app.state?.action_demo_positions?.[action.id] || 0);
  if (!action.configured) {
    return {
      saved: "Edit to set up",
      demo: action.kind !== "lac" && demoPosition !== 0
        ? `Demo position: ${demoPosition.toLocaleString()} steps · return to 0`
        : "",
    };
  }
  const tof = action.tof_mode === "range"
    ? ` · ToF ${action.tof_min_mm}–${action.tof_max_mm} (ideal ${action.tof_ideal_mm})`
    : action.tof_mode === "not_applicable" ? " · ToF n/a" : "";
  if (action.kind === "lac") {
    const support = Number(action.support_clamp_steps ?? 300);
    return { saved: `Saved: ${Number(action.lac_percent)}% · ${Number(action.lac_speed_percent_s || 25)}%/s · LAC1 + LAC2 · +${support.toLocaleString()} clamp across 100%${tof}`, demo: "" };
  }
  const clamp = action.clamp_profile_enabled ? " · clamp profile" : "";
  const offsets = action.motor_step_offsets || {};
  const trim = [["1", "M1 right"], ["6", "M6 left"]]
    .filter(([motor]) => Number(offsets[motor] || 0) > 0)
    .map(([motor, label]) => `${label} +${Number(offsets[motor]).toLocaleString()} clamp across 100%`)
    .join(" · ");
  return {
    saved: `Saved: ${Number(action.steps).toLocaleString()} base steps${trim ? ` · ${trim}` : ""} · ${Number(action.max_hz).toLocaleString()} Hz${clamp}${tof}`,
    demo: demoPosition !== 0
      ? `Demo position: ${demoPosition.toLocaleString()} steps · return to 0`
      : "",
  };
}

function renderActions(actions) {
  elements.actionStrip.replaceChildren();
  for (const action of actions) {
    const tile = document.createElement("article");
    tile.className = `action-tile${action.configured ? " configured" : ""}${action.kind === "lac" ? " lac-action" : ""}`;
    const run = document.createElement("button");
    run.type = "button"; run.className = "action-run"; run.dataset.runAction = action.id;
    run.dataset.configured = String(Boolean(action.configured));
    run.setAttribute("aria-label", action.name);
    const colon = action.name.indexOf(":");
    if (colon > 0) {
      const category = document.createElement("span"); category.className = "action-category";
      category.textContent = action.name.slice(0, colon); run.appendChild(category);
    }
    const name = document.createElement("strong"); name.textContent = colon > 0 ? action.name.slice(colon + 1).trim() : action.name;
    const subtitle = actionSubtitle(action);
    const details = document.createElement("small"); details.textContent = subtitle.saved;
    run.append(name, details);
    if (subtitle.demo) {
      const demo = document.createElement("small");
      demo.className = "action-demo-position";
      demo.textContent = subtitle.demo;
      run.appendChild(demo);
    }
    const edit = document.createElement("button");
    edit.type = "button"; edit.className = "edit-action"; edit.dataset.editAction = action.id;
    edit.setAttribute("aria-label", `Edit ${action.name}`); edit.textContent = "✏";
    tile.append(run, edit); elements.actionStrip.appendChild(tile);
  }
  document.querySelectorAll("[data-run-action]").forEach((button) => {
    button.onclick = () => runSaved(button.dataset.runAction);
  });
  document.querySelectorAll("[data-edit-action]").forEach((button) => {
    button.onclick = () => openActionEditor(actions.find((action) => action.id === button.dataset.editAction));
  });
}

function editorPresets(groupName, directions = null) {
  const group = mappings[groupName];
  elements.editPreset.replaceChildren();
  let selected = null;
  for (const [key, preset] of Object.entries(group.presets)) {
    const option = document.createElement("option"); option.value = key; option.textContent = preset.label;
    elements.editPreset.appendChild(option);
    if (directions && preset.dirs.join() === directions.join()) selected = key;
  }
  if (selected) elements.editPreset.value = selected;
  elements.editTofField.hidden = false;
}

function updateEditorRampPreview() {
  if (app.editorKind === "lac") return;
  const ramp = Math.max(0, Math.min(45, Math.round(numberValue(elements.editRamp, 20))));
  const topHz = Math.max(MIN_ACTION_HZ, Math.min(MAX_ACTION_HZ, Math.round(numberValue(elements.editHz, 300))));
  if (ramp === 0) {
    elements.editorRampLine.setAttribute("points", "10,8 310,8");
    elements.editorRampSummary.textContent = "No ramp · immediate full speed";
    elements.editorRampDetail.textContent = `The whole movement runs at ${topHz.toLocaleString()} Hz.`;
    scheduleEditorEstimate();
    return;
  }
  const fractions = [.2, .4, .6, .8, 1].map((fraction) => Math.max(100, Math.round(topHz * fraction)) / topHz);
  const y = (fraction) => 60 - 52 * Math.min(1, fraction);
  const rampWidth = 300 * ramp / 100;
  const points = [[10, y(fractions[0])]];
  fractions.forEach((fraction, index) => {
    const x = 10 + rampWidth * (index + 1) / fractions.length;
    points.push([x, y(fraction)]);
    if (index < fractions.length - 1) points.push([x, y(fractions[index + 1])]);
  });
  const decelStart = 310 - rampWidth;
  points.push([decelStart, y(1)]);
  const descending = [...fractions].reverse();
  descending.forEach((fraction, index) => {
    const x = decelStart + rampWidth * (index + 1) / descending.length;
    points.push([x, y(fraction)]);
    if (index < descending.length - 1) points.push([x, y(descending[index + 1])]);
  });
  elements.editorRampLine.setAttribute("points", points.map(([xValue, yValue]) => `${xValue.toFixed(1)},${yValue.toFixed(1)}`).join(" "));
  elements.editorRampSummary.textContent = `Linear · ${ramp}% at each end`;
  const lowestHz = Math.max(100, Math.round(topHz * .2));
  elements.editorRampDetail.textContent = `${ramp}% accelerates through five steady stages (${lowestHz.toLocaleString()}→${topHz.toLocaleString()} Hz), ${100 - 2 * ramp}% stays at full speed, then ${ramp}% decelerates. Not compounded.`;
  scheduleEditorEstimate();
}

function clampProfileIsAvailable() {
  return app.editorKind === "stepper" && elements.editGroup.value === "belts";
}

function clampProfileTotal() {
  return Math.max(1, Math.min(MAX_ACTION_STEPS, Math.round(numberValue(elements.editSteps, 1))));
}

function clampValueAt(points, atStep) {
  const step = Math.max(0, Math.min(points.at(-1)?.at_step || 0, atStep));
  for (let index = 1; index < points.length; index += 1) {
    const first = points[index - 1]; const second = points[index];
    if (step > second.at_step) continue;
    const fraction = (step - first.at_step) / Math.max(1, second.at_step - first.at_step);
    return Math.round(first.offset_steps + fraction * (second.offset_steps - first.offset_steps));
  }
  return Number(points.at(-1)?.offset_steps || 0);
}

function normalizedClampPoints(points, total) {
  const clean = (Array.isArray(points) ? points : []).map((point) => ({
    at_step: Math.max(0, Math.min(total, Math.round(Number(point.at_step) || 0))),
    offset_steps: Math.max(-100000, Math.min(100000, Math.round(Number(point.offset_steps) || 0))),
  })).sort((a, b) => a.at_step - b.at_step);
  const byStep = new Map(clean.map((point) => [point.at_step, point]));
  byStep.set(0, { at_step: 0, offset_steps: 0 });
  if (!byStep.has(total)) byStep.set(total, { at_step: total, offset_steps: clampValueAt(clean.length >= 2 ? clean : [{ at_step: 0, offset_steps: 0 }, { at_step: total, offset_steps: 0 }], total) });
  const result = [...byStep.values()].sort((a, b) => a.at_step - b.at_step);
  result[0] = { at_step: 0, offset_steps: 0 };
  result[result.length - 1].at_step = total;
  return result.slice(0, 24);
}

function rescaledClampPoints(points, oldTotal, newTotal) {
  const previousTotal = Math.max(1, Number(oldTotal) || newTotal);
  const scaled = points.map((point, index) => ({
    at_step: index === 0 ? 0 : index === points.length - 1 ? newTotal : Math.round(point.at_step / previousTotal * newTotal),
    offset_steps: point.offset_steps,
  }));
  return normalizedClampPoints(scaled, newTotal);
}

function reversedClampPoints(points, total) {
  const endOffset = Number(points.at(-1)?.offset_steps || 0);
  return [...points].reverse().map((point) => ({
    at_step: total - point.at_step,
    offset_steps: point.offset_steps - endOffset,
  }));
}

function profileGraphPoint(event) {
  const rect = elements.clampProfileGraph.getBoundingClientRect();
  const x = (event.clientX - rect.left) / Math.max(1, rect.width) * 720;
  const y = (event.clientY - rect.top) / Math.max(1, rect.height) * 250;
  const total = clampProfileTotal();
  const scale = Number(elements.clampProfileGraph.dataset.scale || 500);
  return {
    at_step: Math.max(0, Math.min(total, Math.round((x - 58) / 642 * total))),
    offset_steps: Math.max(-100000, Math.min(100000, Math.round((120 - y) / 96 * scale))),
  };
}

function clampSegmentLabel(first, second) {
  const delta = second.offset_steps - first.offset_steps;
  if (delta === 0) return `Hold ${signed(second.offset_steps)} through this range`;
  return `${delta > 0 ? "Clamp" : "Release"} ${signed(delta)} linearly over ${(second.at_step - first.at_step).toLocaleString()} conveyor steps`;
}

function renderClampProfile() {
  const available = clampProfileIsAvailable();
  elements.clampProfileEditor.hidden = !available;
  if (!available) {
    scheduleEditorEstimate();
    return;
  }
  const total = clampProfileTotal();
  app.clampProfilePoints = normalizedClampPoints(app.clampProfilePoints, total);
  app.clampProfileTotal = total;
  const enabled = elements.clampProfileToggle.checked;
  const moving = Boolean(app.state?.active_motion);
  const pair = app.editorPairAction;
  elements.clampPairOptions.hidden = !pair;
  if (pair) {
    elements.clampPairName.textContent = pair.name;
    elements.copyClampFromPair.textContent = pair.clamp_profile_enabled
      ? `Copy reverse from ${pair.name}`
      : `${pair.name} has no saved clamp curve`;
    elements.copyClampFromPair.disabled = moving || !pair.clamp_profile_enabled;
    elements.syncClampToPair.disabled = moving;
  }
  elements.clampProfileBody.classList.toggle("inactive", !enabled);
  elements.addClampPoint.disabled = !enabled || moving || app.clampProfilePoints.length >= 24 || total < 2;

  const maxAbs = Math.max(0, ...app.clampProfilePoints.map((point) => Math.abs(point.offset_steps)));
  const scale = Math.max(500, Math.ceil(maxAbs * 1.2 / 100) * 100);
  elements.clampProfileGraph.dataset.scale = String(scale);
  elements.clampProfileScale.textContent = `±${scale.toLocaleString()} clamp steps`;
  elements.clampGraphEndLabel.textContent = `${total.toLocaleString()} conveyor steps`;
  const graphPoints = app.clampProfilePoints.map((point) => {
    const x = 58 + point.at_step / total * 642;
    const y = 120 - point.offset_steps / scale * 96;
    return { x, y, ...point };
  });
  elements.clampProfileLine.setAttribute("points", graphPoints.map((point) => `${point.x.toFixed(1)},${point.y.toFixed(1)}`).join(" "));
  elements.clampProfileMarkers.replaceChildren();
  graphPoints.forEach((point, index) => {
    const marker = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    marker.setAttribute("cx", point.x.toFixed(1)); marker.setAttribute("cy", point.y.toFixed(1));
    marker.setAttribute("r", index === 0 ? "6" : "8"); marker.dataset.index = String(index);
    marker.setAttribute("tabindex", "0");
    marker.setAttribute("aria-label", `Point ${index + 1}: conveyor ${point.at_step} steps, clamp ${signed(point.offset_steps)} steps`);
    marker.classList.toggle("fixed-point", index === 0);
    marker.onpointerdown = (event) => {
      if (!enabled || moving || index === 0) return;
      event.preventDefault(); app.clampProfileDrag = index; app.clampProfileJustDragged = false;
    };
    elements.clampProfileMarkers.appendChild(marker);
  });

  elements.clampProfileRows.replaceChildren();
  app.clampProfilePoints.forEach((point, index) => {
    const row = document.createElement("div"); row.className = "clamp-point-row";
    const title = document.createElement("div");
    const strong = document.createElement("strong"); strong.textContent = index === 0 ? "Start" : index === app.clampProfilePoints.length - 1 ? "End" : `Point ${index + 1}`;
    const small = document.createElement("small"); small.textContent = index === 0 ? "Current clamping is the zero reference" : clampSegmentLabel(app.clampProfilePoints[index - 1], point);
    title.append(strong, small);
    const stepLabel = document.createElement("label"); stepLabel.textContent = "Conveyor step";
    const stepInput = document.createElement("input"); stepInput.type = "number"; stepInput.min = "0"; stepInput.max = String(total); stepInput.step = "1"; stepInput.value = String(point.at_step);
    const endpoint = index === 0 || index === app.clampProfilePoints.length - 1;
    stepInput.disabled = !enabled || moving || endpoint;
    stepInput.onchange = () => {
      const previous = app.clampProfilePoints[index - 1]?.at_step ?? -1;
      const next = app.clampProfilePoints[index + 1]?.at_step ?? total + 1;
      app.clampProfilePoints[index].at_step = Math.max(previous + 1, Math.min(next - 1, Math.round(numberValue(stepInput, point.at_step))));
      renderClampProfile(); updateEditorDemoState();
    };
    stepLabel.appendChild(stepInput);
    const offsetLabel = document.createElement("label"); offsetLabel.textContent = "Extra clamp steps";
    const offsetInput = document.createElement("input"); offsetInput.type = "number"; offsetInput.min = "-100000"; offsetInput.max = "100000"; offsetInput.step = "1"; offsetInput.value = String(point.offset_steps);
    offsetInput.disabled = !enabled || moving || index === 0;
    offsetInput.onchange = () => {
      app.clampProfilePoints[index].offset_steps = Math.max(-100000, Math.min(100000, Math.round(numberValue(offsetInput, point.offset_steps))));
      renderClampProfile(); updateEditorDemoState();
    };
    offsetLabel.appendChild(offsetInput);
    const remove = document.createElement("button"); remove.type = "button"; remove.textContent = "×"; remove.setAttribute("aria-label", `Delete point ${index + 1}`);
    remove.disabled = !enabled || moving || endpoint; remove.onclick = () => { app.clampProfilePoints.splice(index, 1); renderClampProfile(); updateEditorDemoState(); };
    row.append(title, stepLabel, offsetLabel, remove); elements.clampProfileRows.appendChild(row);
  });
  const changes = app.clampProfilePoints.slice(1).filter((point, index) => point.offset_steps !== app.clampProfilePoints[index].offset_steps).length;
  const endOffset = app.clampProfilePoints.at(-1).offset_steps;
  elements.clampProfileSummary.textContent = enabled
    ? `${changes} changing range${changes === 1 ? "" : "s"} · finishes ${signed(endOffset)} clamp steps from its start`
    : "Profile off — conveyor runs without M1/M6.";
  scheduleEditorEstimate();
}

function rescaleClampProfile() {
  const nextTotal = clampProfileTotal();
  const previousTotal = Math.max(1, app.clampProfileTotal || nextTotal);
  app.clampProfilePoints = app.clampProfilePoints.map((point, index, points) => ({
    at_step: index === 0 ? 0 : index === points.length - 1 ? nextTotal : Math.round(point.at_step / previousTotal * nextTotal),
    offset_steps: point.offset_steps,
  }));
  app.clampProfileTotal = nextTotal;
  renderClampProfile();
}

function addClampProfilePoint(point = null) {
  if (!elements.clampProfileToggle.checked || app.clampProfilePoints.length >= 24) return;
  const total = clampProfileTotal();
  let next = point;
  if (!next) {
    let largest = { width: -1, first: app.clampProfilePoints[0], second: app.clampProfilePoints[1] };
    for (let index = 1; index < app.clampProfilePoints.length; index += 1) {
      const first = app.clampProfilePoints[index - 1]; const second = app.clampProfilePoints[index];
      if (second.at_step - first.at_step > largest.width) largest = { width: second.at_step - first.at_step, first, second };
    }
    const atStep = Math.round((largest.first.at_step + largest.second.at_step) / 2);
    next = { at_step: atStep, offset_steps: clampValueAt(app.clampProfilePoints, atStep) };
  }
  next.at_step = Math.max(1, Math.min(total - 1, next.at_step));
  if (app.clampProfilePoints.some((existing) => existing.at_step === next.at_step)) return;
  app.clampProfilePoints.push(next); app.clampProfilePoints.sort((a, b) => a.at_step - b.at_step);
  renderClampProfile(); updateEditorDemoState();
}

function clampProfilePayload() {
  const enabled = clampProfileIsAvailable() && elements.clampProfileToggle.checked;
  return {
    clamp_profile_enabled: enabled,
    clamp_profile: enabled ? app.clampProfilePoints.map((point) => ({ ...point })) : null,
  };
}

function copyReversedPairProfile() {
  const pair = app.editorPairAction;
  if (!pair?.clamp_profile_enabled || !Array.isArray(pair.clamp_profile)) return;
  const total = clampProfileTotal();
  const pairTotal = Math.max(1, Number(pair.steps) || total);
  const rescaled = rescaledClampPoints(pair.clamp_profile, pairTotal, total);
  app.clampProfilePoints = reversedClampPoints(rescaled, total);
  elements.clampProfileToggle.checked = true;
  elements.syncClampToPair.checked = false;
  renderClampProfile(); updateEditorDemoState();
  showToast(`Copied the reverse of ${pair.name}. This editor is independent until you choose Update both.`);
}

function updateTofModeState() {
  const useRange = elements.editTofMode.value === "range";
  const moving = Boolean(app.state?.active_motion);
  elements.tofRangeFields.classList.toggle("inactive", !useRange);
  [elements.editTofMin, elements.editTofIdeal, elements.editTofMax].forEach((input) => {
    input.disabled = !useRange || moving;
  });
  elements.captureTof.disabled = moving || !app.state?.tof?.valid;
}

function editorUsesMotorOffsets() {
  return ["move-left", "move-right"].includes(elements.editActionId.value);
}

function editorMotorOffsetsPayload() {
  if (!editorUsesMotorOffsets()) return { motor_step_offsets: {} };
  return {
    motor_step_offsets: {
      "1": Math.max(0, Math.round(numberValue(elements.editM1ExtraSteps, 0))),
      "6": Math.max(0, Math.round(numberValue(elements.editM6ExtraSteps, 0))),
    },
  };
}

function updateMotorOffsetSummary() {
  if (!editorUsesMotorOffsets()) return;
  const base = Math.max(1, Math.round(numberValue(elements.editSteps, 1)));
  const offsets = editorMotorOffsetsPayload().motor_step_offsets;
  const m1 = base + offsets["1"];
  const m6 = base + offsets["6"];
  elements.motorOffsetSummary.textContent =
    `Full action: M1 ${m1.toLocaleString()} steps · M6 ${m6.toLocaleString()} steps. The difference accumulates steadily across 100% of travel; both sides start and finish together. The ramp setting above changes speed, not clamp coverage.`;
}

function openActionEditor(action = null) {
  const record = action || {
    id: "", name: "", kind: "stepper", group: "mechanism", directions: ["R", "R"], steps: 100,
    max_hz: 300, ramp_percent: 20, lac_speed_percent_s: 25, support_clamp_steps: 300,
    support_max_hz: 500, support_ramp_percent: 100, tof_mode: "unspecified", tof_min_mm: null,
    tof_ideal_mm: null, tof_max_mm: null, instructions: "", built_in: false,
  };
  elements.editorTitle.textContent = action ? `Edit ${action.name}` : "New action";
  app.editorKind = record.kind || "stepper";
  const isLac = app.editorKind === "lac";
  elements.editActionId.value = record.id || "";
  elements.editName.value = record.name || "";
  elements.editGroup.value = ["mechanism", "belts"].includes(record.group) ? record.group : "mechanism";
  editorPresets(elements.editGroup.value, record.directions);
  elements.editGroup.disabled = Boolean(record.built_in);
  elements.editPreset.disabled = Boolean(record.built_in);
  elements.editSteps.value = record.steps ?? 100;
  elements.editHz.value = record.max_hz ?? 300;
  elements.editRamp.value = record.ramp_percent ?? 20;
  elements.editM1ExtraSteps.value = record.motor_step_offsets?.["1"] ?? 0;
  elements.editM6ExtraSteps.value = record.motor_step_offsets?.["6"] ?? 0;
  app.clampProfileTotal = Math.max(1, Number(record.steps) || 100);
  app.clampProfilePoints = normalizedClampPoints(
    record.clamp_profile || [
      { at_step: 0, offset_steps: 0 },
      { at_step: app.clampProfileTotal, offset_steps: 0 },
    ],
    app.clampProfileTotal,
  );
  elements.clampProfileToggle.checked = Boolean(record.clamp_profile_enabled);
  elements.editLacPercent.value = record.lac_percent ?? 0;
  elements.editLacSpeed.value = record.lac_speed_percent_s ?? 25;
  elements.editLacClampSteps.value = record.support_clamp_steps ?? 300;
  elements.editLacClampHz.value = record.support_max_hz ?? 500;
  elements.editorLacCurrent.textContent = `LAC1 ${app.state?.lac?.["1"] ?? 0}% · LAC2 ${app.state?.lac?.["2"] ?? 0}%`;
  [elements.editGroupField, elements.editPresetField, elements.editStepsField, elements.editHzField, elements.editRampField].forEach((field) => {
    field.hidden = isLac;
  });
  elements.editorRampPreview.hidden = isLac;
  elements.editMotorOffsets.hidden = isLac || !["move-left", "move-right"].includes(record.id);
  elements.clampProfileEditor.hidden = isLac || record.group !== "belts";
  elements.lacEditorFields.hidden = !isLac;
  elements.editTofMode.value = record.tof_mode || "unspecified";
  elements.editTofMin.value = record.tof_min_mm ?? "";
  elements.editTofIdeal.value = record.tof_ideal_mm ?? record.tof_reference_mm ?? "";
  elements.editTofMax.value = record.tof_max_mm ?? "";
  elements.editInstructions.value = record.instructions || "";
  elements.deleteAction.hidden = !action || Boolean(record.built_in);
  const pair = record.pair_id && app.state?.actions?.find((item) => item.id === record.pair_id);
  app.editorPairAction = pair || null;
  elements.syncClampToPair.checked = false;
  const fixedDefinition = fixedActionDefinitions[record.id] || "";
  const pairDescription = isLac
    ? "Separate actuator action: this commands LAC1 + LAC2 and simultaneously adds the shown Clamp movement on M1 + M6. It never starts conveyor belts M3/M4. Both actuator directions use inward Clamp support to prevent slipping."
    : pair ? `Linked pair: saving this also sets “${pair.name}” to the same base steps, speed and ramp with every motor direction reversed.${record.group === "belts" ? " Its purple clamp curve stays independent unless you explicitly choose Update both." : editorUsesMotorOffsets() ? " Each direction keeps its own independently editable M1/M6 extra-step trim." : ""} Its ToF setting stays separate.` : "";
  elements.pairHelp.textContent = [fixedDefinition, pairDescription].filter(Boolean).join(" ");
  elements.tofReferenceHelp.textContent = isLac
    ? "The percentage is an absolute LAC1 + LAC2 command. The clamp amount is relative: every run adds it again. ToF is recorded; M3/M4 do not start."
    : "The range checks the finish reading; it does not stop movement. Step count remains the movement limit.";
  elements.actionEditor.showModal();
  updateEditorRampPreview();
  updateMotorOffsetSummary();
  renderClampProfile();
  updateTofModeState();
  updateEditorDemoState();
  if (app.state?.connected) {
    post("/api/tof", {}).then(() => setTimeout(pollState, 120)).catch(() => {});
  }
}

function editorMotionPayload(targetSteps, includeMotorOffsets = true) {
  const group = mappings[elements.editGroup.value];
  const preset = group.presets[elements.editPreset.value];
  return {
    action_id: elements.editActionId.value,
    name: elements.editName.value.trim() || "Action demo",
    group: elements.editGroup.value,
    direction_a: preset.dirs[0],
    direction_b: preset.dirs[1],
    steps: Math.max(1, Math.round(numberValue(elements.editSteps, 1))),
    target_steps: Math.round(targetSteps),
    max_hz: Math.round(numberValue(elements.editHz, 300)),
    ramp_percent: Math.round(numberValue(elements.editRamp, 20)),
    tof_mode: elements.editTofMode.value,
    tof_min_mm: elements.editTofMin.value,
    tof_ideal_mm: elements.editTofIdeal.value,
    tof_max_mm: elements.editTofMax.value,
    instructions: elements.editInstructions.value.trim(),
    ...(includeMotorOffsets ? editorMotorOffsetsPayload() : { motor_step_offsets: {} }),
    ...clampProfilePayload(),
  };
}

function scheduleEditorEstimate() {
  if (app.editorKind === "lac" || !elements.actionEditor.open) return;
  clearTimeout(app.editorEstimateTimer);
  elements.editorEstimatedDuration.textContent = "Calculating…";
  elements.editorEstimateDetail.textContent = "Includes the selected ramp and simultaneous clamp profile.";
  app.editorEstimateTimer = setTimeout(updateEditorEstimate, 120);
}

async function updateEditorEstimate() {
  if (app.editorKind === "lac" || !elements.actionEditor.open) return;
  const steps = Math.round(numberValue(elements.editSteps, 0));
  const hz = Math.round(numberValue(elements.editHz, 0));
  if (steps < 1 || steps > MAX_ACTION_STEPS || hz < MIN_ACTION_HZ || hz > MAX_ACTION_HZ) {
    elements.editorEstimatedDuration.textContent = "Check steps and Hz";
    elements.editorEstimateDetail.textContent = `Use 1–${MAX_ACTION_STEPS.toLocaleString()} steps and ${MIN_ACTION_HZ.toLocaleString()}–${MAX_ACTION_HZ.toLocaleString()} Hz.`;
    elements.editorRuntime.classList.add("invalid");
    return;
  }
  const sequence = ++app.editorEstimateSequence;
  try {
    const payload = await api("/api/motion/preview", {
      method: "POST",
      body: JSON.stringify(editorMotionPayload(steps)),
    });
    if (sequence !== app.editorEstimateSequence || !elements.actionEditor.open) return;
    const result = payload.result;
    const seconds = Number(result.estimated_duration_s || 0);
    elements.editorEstimatedDuration.textContent = formatDuration(seconds);
    const offsets = result.motor_step_offsets || {};
    const adjusted = Number(offsets["1"] || 0) > 0 || Number(offsets["6"] || 0) > 0;
    elements.editorEstimateDetail.textContent = result.clamp_profile_enabled
      ? `Exact preview: ramp plus simultaneous conveyor/clamp motion across ${Number(result.segment_count || 0).toLocaleString()} synchronized segments. Long runs are allowed; STOP remains available.`
      : adjusted
        ? `Exact synchronized preview: M1 ${Number(result.motor_step_totals["1"]).toLocaleString()} and M6 ${Number(result.motor_step_totals["6"]).toLocaleString()} pulses, proportionally spread across the same ramp and finish time.`
        : `Ramp-aware estimate for ${steps.toLocaleString()} commanded steps. Long runs are allowed; STOP remains available.`;
    elements.editorRuntime.classList.toggle("long", seconds > 60);
    elements.editorRuntime.classList.remove("invalid");
  } catch (error) {
    if (sequence !== app.editorEstimateSequence || !elements.actionEditor.open) return;
    elements.editorEstimatedDuration.textContent = "Cannot calculate";
    elements.editorEstimateDetail.textContent = error.message;
    elements.editorRuntime.classList.remove("long");
    elements.editorRuntime.classList.add("invalid");
  }
}

function editorLacPayload() {
  return {
    action_id: elements.editActionId.value,
    name: elements.editName.value.trim() || "Linear actuator action",
    kind: "lac",
    lac_percent: Math.max(0, Math.min(100, Math.round(numberValue(elements.editLacPercent, 0)))),
    lac_speed_percent_s: Math.max(2, Math.min(100, Math.round(numberValue(elements.editLacSpeed, 25)))),
    support_clamp_steps: Math.max(0, Math.min(MAX_ACTION_STEPS, Math.round(numberValue(elements.editLacClampSteps, 300)))),
    support_max_hz: Math.max(MIN_ACTION_HZ, Math.min(MAX_ACTION_HZ, Math.round(numberValue(elements.editLacClampHz, 500)))),
    support_ramp_percent: 100,
    tof_mode: elements.editTofMode.value,
    tof_min_mm: elements.editTofMin.value,
    tof_ideal_mm: elements.editTofIdeal.value,
    tof_max_mm: elements.editTofMax.value,
    instructions: elements.editInstructions.value.trim(),
  };
}

function updateEditorDemoState() {
  const key = elements.editActionId.value;
  const position = Number(app.state?.action_demo_positions?.[key] || 0);
  const target = Math.max(0, Math.round(numberValue(elements.editSteps, 0)));
  const delta = target - position;
  const connected = Boolean(app.state?.connected);
  const armed = Boolean(app.state?.controls_armed);
  const moving = Boolean(app.state?.active_motion);
  const tof = app.state?.tof || {};
  elements.editorLiveTof.textContent = tof.valid
    ? `${tof.distance_mm} mm`
    : tof.state === "offline" ? "Sensor offline — no distance"
    : tof.state === "waiting" ? "Waiting for distance samples"
    : "No valid distance";
  elements.editorLiveTofDetail.textContent = tof.valid
    ? `D13/PA5 · physical right · ${tof.avg_n || 0} sample average`
    : `D13/PA5 · physical right · ${tof.state || "unknown"}${tof.status == null ? "" : ` · status ${tof.status}`}`;
  elements.editorTofRaw.textContent = tof.raw_mm == null ? "—" : `${tof.raw_mm} mm`;
  elements.editorTofSamples.textContent = Number(tof.samples || 0).toLocaleString();
  elements.editorTofErrors.textContent = Number(tof.errors || 0).toLocaleString();
  elements.editorTofUpdated.textContent = updateAge(tof.updated_at);
  elements.refreshEditorTof.disabled = !connected || moving;
  if (app.editorKind === "lac") {
    const lac1 = Number(app.state?.lac?.["1"] || 0);
    const lac2 = Number(app.state?.lac?.["2"] || 0);
    const targetPercent = Math.max(0, Math.min(100, Math.round(numberValue(elements.editLacPercent, 0))));
    const signalSpeed = Math.max(2, Math.min(100, Math.round(numberValue(elements.editLacSpeed, 25))));
    const clampSteps = Math.max(0, Math.min(MAX_ACTION_STEPS, Math.round(numberValue(elements.editLacClampSteps, 300))));
    const clampHz = Math.max(MIN_ACTION_HZ, Math.min(MAX_ACTION_HZ, Math.round(numberValue(elements.editLacClampHz, 500))));
    const travelSeconds = Math.max(Math.abs(targetPercent - lac1), Math.abs(targetPercent - lac2)) / signalSpeed;
    const actionSeconds = Math.max(travelSeconds, clampSteps / clampHz);
    elements.lacSupportSummary.textContent = clampSteps
      ? `+${clampSteps.toLocaleString()} clamp steps across 100%`
      : "Clamp support disabled";
    elements.editorLacCurrent.textContent = `LAC1 ${lac1}% · LAC2 ${lac2}%`;
    elements.demoPosition.textContent = `Current command: LAC1 ${lac1}% · LAC2 ${lac2}%`;
    if (!key) {
      elements.demoDelta.textContent = "Save this actuator action before applying its target.";
      elements.demoTarget.textContent = "Save before applying";
    } else if (lac1 === targetPercent && lac2 === targetPercent) {
      elements.demoDelta.textContent = `LAC1 + LAC2 are already at ${targetPercent}%; pressing play still spreads ${clampSteps.toLocaleString()} clamp steps across ${actionSeconds.toFixed(1)}s.`;
      elements.demoTarget.textContent = clampSteps ? `▶ Add ${clampSteps.toLocaleString()} clamp` : `Reapply ${targetPercent}%`;
    } else {
      elements.demoDelta.textContent = `Moves LAC1 + LAC2 to ${targetPercent}% while the ${clampSteps.toLocaleString()} clamp steps accumulate evenly from start to finish across the full ${actionSeconds.toFixed(1)}s action. There is no release at the end; M3/M4 stay stopped.`;
      elements.demoTarget.textContent = `▶ Ramp to ${targetPercent}%`;
    }
    const canApply = Boolean(key && connected && armed && !moving);
    elements.demoTarget.disabled = !canApply;
    elements.demoZero.hidden = true;
    elements.rebaseDemo.hidden = true;
    elements.editLacPercent.disabled = moving;
    elements.editLacSpeed.disabled = moving;
    elements.editLacClampSteps.disabled = moving;
    elements.editLacClampHz.disabled = moving;
    elements.saveEditedAction.disabled = moving;
    const latestLacDemo = [...(app.state?.action_demo_history || [])].reverse().find((item) => item.action_id === key);
    elements.demoTofResult.className = "demo-tof-result";
    if (latestLacDemo) {
      const reading = latestLacDemo.tof_after?.valid ? `${latestLacDemo.tof_after.distance_mm} mm` : "unavailable";
      elements.demoTofResult.textContent = `Last actuator command: ${latestLacDemo.target_percent}% at ${latestLacDemo.speed_percent_s || 25}%/s · clamp ${Number(latestLacDemo.support_completed_steps || 0).toLocaleString()}/${Number(latestLacDemo.support_clamp_steps || 0).toLocaleString()} · ToF ${reading}`;
      elements.demoTofResult.classList.add(latestLacDemo.tof_result?.code || "recorded");
    } else {
      elements.demoTofResult.textContent = "Applying records ToF, ramps both actuator signals, and adds the clamp support shown above. M3/M4 remain stopped.";
    }
    updateTofModeState();
    return;
  }
  elements.demoZero.hidden = false;
  elements.rebaseDemo.hidden = false;
  const demoClampOffset = Number(app.state?.action_demo_clamp_offsets?.[key] || 0);
  const profileActive = clampProfileIsAvailable() && elements.clampProfileToggle.checked;
  elements.demoPosition.textContent = profileActive
    ? `${position.toLocaleString()} conveyor steps · clamp ${signed(demoClampOffset)}`
    : `${position.toLocaleString()} steps from start`;
  if (!key) {
    elements.demoDelta.textContent = "Save this new action before demoing it.";
    elements.demoTarget.textContent = "Save before demo";
  } else if (delta === 0) {
    elements.demoDelta.textContent = `The robot is already at this demo target.`;
    elements.demoTarget.textContent = "At target";
  } else if (delta > 0) {
    const targetClamp = profileActive ? clampValueAt(app.clampProfilePoints, target) : demoClampOffset;
    elements.demoDelta.textContent = profileActive
      ? `Next demo adds ${delta.toLocaleString()} conveyor steps and follows the clamp curve to ${signed(targetClamp)}.`
      : `Next demo adds ${delta.toLocaleString()} steps in the action direction.`;
    elements.demoTarget.textContent = `▶ Move +${delta.toLocaleString()}`;
  } else {
    const targetClamp = profileActive ? clampValueAt(app.clampProfilePoints, target) : demoClampOffset;
    elements.demoDelta.textContent = profileActive
      ? `Next demo reverses ${Math.abs(delta).toLocaleString()} conveyor steps and follows the clamp curve back to ${signed(targetClamp)}.`
      : `Next demo removes ${Math.abs(delta).toLocaleString()} steps by moving in the opposite direction.`;
    elements.demoTarget.textContent = `↶ Move ${delta.toLocaleString()}`;
  }
  const canMove = Boolean(key && connected && armed && !moving);
  elements.demoTarget.disabled = !canMove || delta === 0;
  elements.demoZero.disabled = !canMove || position === 0;
  elements.rebaseDemo.disabled = !key || moving || position === 0;
  const action = app.state?.actions?.find((item) => item.id === key);
  elements.editGroup.disabled = moving || position !== 0 || Boolean(action?.pair_id);
  elements.editPreset.disabled = moving || position !== 0;
  elements.saveEditedAction.disabled = moving;
  const latestDemo = [...(app.state?.action_demo_history || [])].reverse().find((item) => item.action_id === key);
  elements.demoTofResult.className = "demo-tof-result";
  if (latestDemo) {
    const before = latestDemo.tof_before?.valid ? `${latestDemo.tof_before.distance_mm} mm` : "unavailable";
    const after = latestDemo.tof_after?.valid ? `${latestDemo.tof_after.distance_mm} mm` : "unavailable";
    elements.demoTofResult.textContent = `Last demo ToF: ${before} → ${after} · ${latestDemo.tof_result.label}`;
    elements.demoTofResult.classList.add(latestDemo.tof_result.code);
  } else {
    elements.demoTofResult.textContent = "Each demo records the current ToF before movement and the finish reading.";
  }
  updateTofModeState();
}

async function demoEditorTarget(target) {
  try {
    if (app.editorKind === "lac") await post("/api/action/demo/lac", editorLacPayload());
    else await post("/api/action/demo", editorMotionPayload(target, false));
    await pollState();
  } catch { /* toast already shown */ }
}

async function saveEditedAction() {
  if (!elements.editName.value.trim()) { showToast("Give the action a button name"); return; }
  const body = {
    action_id: elements.editActionId.value,
    name: elements.editName.value.trim(), kind: app.editorKind,
    tof_mode: elements.editTofMode.value,
    tof_min_mm: elements.editTofMin.value,
    tof_ideal_mm: elements.editTofIdeal.value,
    tof_max_mm: elements.editTofMax.value,
    instructions: elements.editInstructions.value.trim(),
  };
  if (app.editorKind === "lac") {
    body.lac_percent = Math.max(0, Math.min(100, Math.round(numberValue(elements.editLacPercent, 0))));
    body.lac_speed_percent_s = Math.max(2, Math.min(100, Math.round(numberValue(elements.editLacSpeed, 25))));
    body.support_clamp_steps = Math.max(0, Math.min(MAX_ACTION_STEPS, Math.round(numberValue(elements.editLacClampSteps, 300))));
    body.support_max_hz = Math.max(MIN_ACTION_HZ, Math.min(MAX_ACTION_HZ, Math.round(numberValue(elements.editLacClampHz, 500))));
    body.support_ramp_percent = 100;
  } else {
    const group = mappings[elements.editGroup.value];
    const preset = group.presets[elements.editPreset.value];
    Object.assign(body, {
      group: elements.editGroup.value,
      direction_a: preset.dirs[0], direction_b: preset.dirs[1],
      steps: Math.round(numberValue(elements.editSteps, 100)),
      max_hz: Math.round(numberValue(elements.editHz, 300)),
      ramp_percent: Math.round(numberValue(elements.editRamp, 20)),
      ...editorMotorOffsetsPayload(),
      ...clampProfilePayload(),
      sync_clamp_profile_to_pair: clampProfileIsAvailable() && elements.syncClampToPair.checked,
    });
  }
  try {
    await post("/api/action/save", body);
    elements.actionEditor.close();
    await pollState();
    const paired = app.state?.actions?.find((item) => item.id === elements.editActionId.value)?.pair_id;
    const message = paired && app.editorKind === "stepper"
      ? body.group === "belts"
        ? body.sync_clamp_profile_to_pair
          ? `${body.name} saved; paired clamp curve replaced with its exact reverse`
          : `${body.name} saved; paired clamp curve kept independent`
        : `${body.name} and its opposite saved`
      : `${body.name} saved`;
    showToast(message);
  } catch { /* toast already shown */ }
}

async function runSaved(actionId) {
  try { await post("/api/action/run", { action_id: actionId, reverse: false }); await pollState(); }
  catch { /* toast already shown */ }
}

function flagDelta(first, second) {
  const compatible = first.counter_epoch === second.counter_epoch;
  const motors = {};
  for (const motor of [1, 3, 4, 6, 7]) motors[String(motor)] = Number(second.motor_steps[String(motor)] || 0) - Number(first.motor_steps[String(motor)] || 0);
  const tof = first.tof.valid && second.tof.valid ? Number(second.tof.distance_mm) - Number(first.tof.distance_mm) : null;
  return { compatible, motors, tof };
}

function renderComparison(flags) {
  elements.comparison.replaceChildren();
  if (flags.length < 2) {
    elements.comparison.className = "comparison empty-comparison";
    elements.comparison.textContent = "Place two flags to compare any two points.";
    return;
  }
  const ids = new Set(flags.map((flag) => flag.id));
  if (!ids.has(app.compareA) || !ids.has(app.compareB) || app.compareA === app.compareB) {
    app.compareA = flags.at(-2).id; app.compareB = flags.at(-1).id;
  }
  elements.comparison.className = "comparison";
  const controls = document.createElement("div"); controls.className = "compare-controls";
  const makeSelect = (label, selected, key) => {
    const wrapper = document.createElement("label"); wrapper.textContent = label;
    const select = document.createElement("select");
    for (const flag of flags) {
      const option = document.createElement("option"); option.value = flag.id; option.textContent = `${flag.number}. ${flag.label}`;
      select.appendChild(option);
    }
    select.value = selected;
    select.onchange = () => { app[key] = select.value; renderComparison(flags); };
    wrapper.appendChild(select); return wrapper;
  };
  controls.append(makeSelect("From flag", app.compareA, "compareA"));
  const arrow = document.createElement("span"); arrow.className = "compare-arrow"; arrow.textContent = "→"; controls.appendChild(arrow);
  controls.append(makeSelect("To flag", app.compareB, "compareB"));
  elements.comparison.appendChild(controls);
  const first = flags.find((flag) => flag.id === app.compareA);
  const second = flags.find((flag) => flag.id === app.compareB);
  const delta = flagDelta(first, second);
  const grid = document.createElement("div"); grid.className = "delta-grid";
  const values = [["M1 net", signed(delta.motors["1"])], ["M6 net", signed(delta.motors["6"])], ["M3 net", signed(delta.motors["3"])], ["M4 net", signed(delta.motors["4"])], ["ToF change", delta.tof === null ? "No valid data" : `${signed(delta.tof)} mm`]];
  for (const [label, value] of values) {
    const chip = document.createElement("div"); chip.className = "delta-chip";
    const small = document.createElement("span"); small.textContent = label;
    const strong = document.createElement("strong"); strong.textContent = delta.compatible ? value : "—";
    chip.append(small, strong); grid.appendChild(chip);
  }
  if (!delta.compatible) {
    const warning = document.createElement("p"); warning.className = "epoch-break"; warning.textContent = "These flags cross a counter reset, so their difference is not comparable.";
    elements.comparison.appendChild(warning);
  }
  elements.comparison.appendChild(grid);
}

function renderTimeline(flags) {
  renderComparison(flags);
  elements.flagTimeline.replaceChildren();
  if (!flags.length) {
    const empty = document.createElement("div"); empty.className = "empty-state";
    empty.innerHTML = "<span>⚑</span><strong>No flags yet</strong><p>Flag the counters before moving, then flag them again afterwards.</p>";
    elements.flagTimeline.appendChild(empty); return;
  }
  flags.forEach((flag, index) => {
    if (index > 0) {
      const previous = flags[index - 1]; const delta = flagDelta(previous, flag);
      const segment = document.createElement("div"); segment.className = "timeline-segment";
      segment.appendChild(document.createElement("span"));
      const line = document.createElement("div"); line.className = "segment-line"; segment.appendChild(line);
      const text = document.createElement("div"); text.className = `segment-delta${delta.compatible ? "" : " epoch-break"}`;
      if (delta.compatible) {
        for (const motor of [1, 6, 3, 4]) {
          const value = delta.motors[String(motor)];
          if (value !== 0) { const chip = document.createElement("b"); chip.textContent = `M${motor} ${signed(value)}`; text.appendChild(chip); }
        }
        if (!text.childElementCount) text.textContent = "No commanded steps";
        if (delta.tof !== null) { const tof = document.createElement("span"); tof.textContent = `ToF ${signed(delta.tof)} mm`; text.appendChild(tof); }
      } else text.textContent = "Counter reset · new timeline epoch";
      segment.appendChild(text); elements.flagTimeline.appendChild(segment);
    }
    const node = document.createElement("article"); node.className = "flag-node";
    const time = document.createElement("time"); time.className = "flag-time"; time.textContent = new Date(flag.captured_at * 1000).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false });
    const dot = document.createElement("div"); dot.className = "flag-dot"; dot.textContent = flag.number;
    const card = document.createElement("div"); card.className = "flag-card";
    const title = document.createElement("div"); const h3 = document.createElement("h3"); h3.textContent = flag.label; title.appendChild(h3); card.appendChild(title);
    const stats = [["M1", signed(flag.motor_steps["1"])], ["M6", signed(flag.motor_steps["6"])], ["M3", signed(flag.motor_steps["3"])], ["M4", signed(flag.motor_steps["4"])], ["M7", signed(flag.motor_steps["7"] || 0)], ["Right ToF", flag.tof.valid ? `${flag.tof.distance_mm} mm` : "No target"]];
    for (const [label, value] of stats) { const stat = document.createElement("div"); stat.className = "flag-stat"; const s = document.createElement("span"); s.textContent = label; const b = document.createElement("b"); b.textContent = value; stat.append(s,b); card.appendChild(stat); }
    node.append(time,dot,card); elements.flagTimeline.appendChild(node);
  });
}

function controlsEnabled() {
  const ready = Boolean(app.state?.connected && app.state?.controls_armed);
  const moving = Boolean(app.state?.active_motion);
  const paused = app.state?.paused_motion;
  elements.armToggle.disabled = !app.state?.connected || moving;
  elements.stopButton.disabled = !app.state?.connected;
  elements.runMotion.disabled = !ready || moving;
  elements.runReverse.disabled = !ready || moving;
  document.querySelectorAll("[data-run-action]").forEach((button) => { button.disabled = !ready || moving || button.dataset.configured !== "true"; });
  document.querySelectorAll("[data-lac]").forEach((button) => { button.disabled = !ready || moving; });
  elements.lacSpeed.disabled = moving;
  const lacPaused = paused?.kind === "lac";
  const supportCompleted = Number(paused?.support_completed_steps || 0);
  const supportTotal = Number(paused?.support_clamp_steps || 0);
  const lacCanResume = lacPaused && ((paused.selected_actuators || []).some((item) => Number(paused.current_values?.[item]) !== Number(paused.target_values?.[item])) || supportCompleted < supportTotal);
  const lacCanReturn = lacPaused && ((paused.selected_actuators || []).some((item) => Number(paused.current_values?.[item]) !== Number(paused.start_values?.[item])) || supportCompleted > 0);
  elements.resumeMotion.disabled = !ready || moving || !paused || (lacPaused ? !lacCanResume : Number(paused.remaining_steps) <= 0);
  elements.returnMotion.disabled = !ready || moving || !paused || (lacPaused ? !lacCanReturn : Number(paused.completed_steps) <= 0);
}

function updateState(state) {
  app.state = state;
  renderStudyTimer();
  elements.connection.textContent = state.connected ? "Connected" : "Disconnected";
  elements.connection.className = `connection-pill ${state.connected ? "online" : "offline"}`;
  elements.serialPort.textContent = state.connected ? `${state.serial_port} · 115200 baud` : "Waiting for Nucleo";
  elements.armToggle.checked = Boolean(state.controls_armed);
  elements.logPath.textContent = `Session: ${state.session_log}`;
  for (const motor of [1,3,4,6,7]) elements[`m${motor}Steps`].textContent = signed(state.motor_steps[String(motor)] || 0);
  elements.lac1State.textContent = `${state.lac["1"]}%`; elements.lac2State.textContent = `${state.lac["2"]}%`;
  const tof = state.tof;
  elements.tofDistance.textContent = tof.valid ? `${tof.distance_mm} mm` : tof.state === "offline" ? "Offline" : "No target";
  elements.tofDetail.textContent = tof.valid ? `Raw ${tof.raw_mm} mm · ${tof.errors} errors` : tof.state === "invalid" ? `Raw ${tof.raw_mm} mm · status ${tof.status}` : `State: ${tof.state}`;
  const paused = state.paused_motion;
  elements.pausedRecovery.hidden = !paused || Boolean(state.active_motion);
  if (paused && !state.active_motion) {
    elements.pausedRecoveryTitle.textContent = paused.name;
    if (paused.kind === "lac") {
      const selected = paused.selected_actuators || [];
      const currentText = selected.map((item) => `LAC${item} ${paused.current_values?.[item]}%`).join(" · ");
      const targetText = selected.map((item) => `LAC${item} ${paused.target_values?.[item]}%`).join(" · ");
      const startText = selected.map((item) => `LAC${item} ${paused.start_values?.[item]}%`).join(" · ");
      const clampDone = Number(paused.support_completed_steps || 0);
      const clampTotal = Number(paused.support_clamp_steps || 0);
      elements.pausedRecoveryDetail.textContent = `Signal ramp stopped at ${currentText} · clamp ${clampDone.toLocaleString()}/${clampTotal.toLocaleString()} steps`;
      elements.pausedRecoveryHelp.textContent = "STOP holds the last actuator command. Resume completes both the actuator target and remaining clamp support; Go back reverses the actuator command and the clamp steps already delivered.";
      elements.resumeMotion.textContent = `▶ Resume to ${targetText}`;
      elements.returnMotion.textContent = `↶ Go back to ${startText}`;
    } else {
      const completed = Number(paused.completed_steps || 0);
      const remaining = Number(paused.remaining_steps || 0);
      const total = Number(paused.steps || 0);
      const motorList = (paused.motors || []).map((motor) => `M${motor}`).join(" + ");
      elements.pausedRecoveryDetail.textContent = `Stopped after about ${completed.toLocaleString()} of ${total.toLocaleString()} steps · ${motorList}${paused.clamp_profile_enabled ? " + simultaneous M1/M6 clamp profile" : ""}`;
      elements.pausedRecoveryHelp.textContent = "Open-loop estimate: the robot has no stepper encoders. Both choices preserve every paired motor and any simultaneous clamp curve.";
      elements.resumeMotion.textContent = `▶ Resume remaining ${remaining.toLocaleString()} steps`;
      elements.returnMotion.textContent = `↶ Go back ${completed.toLocaleString()} steps to start`;
    }
  }
  if (state.active_motion) {
    elements.motionState.textContent = `${state.active_motion.name} running`; elements.motionState.className = "state-pill running";
    elements.safetyBanner.className = "safety-banner running"; elements.safetyTitle.textContent = "Movement running";
    elements.safetyDetail.textContent = "Press STOP ALL or Space to stop immediately.";
  } else if (state.controls_armed) {
    elements.motionState.textContent = "Armed"; elements.motionState.className = "state-pill";
    elements.safetyBanner.className = "safety-banner armed"; elements.safetyTitle.textContent = "Motion enabled";
    elements.safetyDetail.textContent = "Choose a direct move or saved action, and keep the area clear.";
  } else {
    elements.motionState.textContent = "Idle / locked"; elements.motionState.className = "state-pill";
    elements.safetyBanner.className = "safety-banner locked"; elements.safetyTitle.textContent = "Motion locked";
    elements.safetyDetail.textContent = "Set up movements freely. Enable motion only when the area is clear.";
  }
  const actionsSignature = JSON.stringify([state.actions || [], state.action_demo_positions || {}, state.action_demo_clamp_offsets || {}]);
  if (actionsSignature !== app.actionsSignature) { app.actionsSignature = actionsSignature; renderActions(state.actions || []); }
  const flagsSignature = JSON.stringify(state.step_flags || []);
  if (flagsSignature !== app.flagsSignature) {
    const previousCount = app.flagsSignature ? JSON.parse(app.flagsSignature).length : 0;
    const flags = state.step_flags || [];
    if (flags.length > previousCount && flags.length >= 2) { app.compareA = flags.at(-2).id; app.compareB = flags.at(-1).id; }
    app.flagsSignature = flagsSignature; renderTimeline(flags);
  }
  elements.clearFlags.disabled = !(state.step_flags || []).length;
  controlsEnabled();
  if (elements.actionEditor.open) updateEditorDemoState();
}

function appendEvent(event) {
  const line = document.createElement("div"); line.className = event.kind;
  const time = new Date(event.time * 1000).toLocaleTimeString([], { hour12: false });
  line.textContent = `${time}  ${event.message}`; elements.terminal.appendChild(line);
  while (elements.terminal.childElementCount > 300) elements.terminal.firstElementChild.remove();
  elements.terminal.scrollTop = elements.terminal.scrollHeight;
}

async function pollState() {
  try { updateState(await api("/api/state")); }
  catch (error) { showToast(error.message); }
}

async function pollEvents() {
  try {
    const payload = await api(`/api/events?after=${app.eventId}`);
    for (const event of payload.events) {
      app.eventId = Math.max(app.eventId, event.id);
      app.trialEvents.push(event);
      appendEvent(event);
    }
    if (app.trialEvents.length > 3000) app.trialEvents.splice(0, app.trialEvents.length - 3000);
    renderStudyTimer();
  } catch (error) { showToast(error.message); }
}

async function setArmed(armed) {
  if (armed && !confirm("Enable motion? The existing LAC targets will be preserved. Is the robot area clear?")) { elements.armToggle.checked = false; return; }
  try { updateState(await post("/api/arm", { armed })); }
  catch { elements.armToggle.checked = false; }
}

elements.group.onchange = updateManualGroup;
elements.preset.onchange = updateManualPreset;
elements.directionA.onchange = syncPresetFromDirections;
elements.directionB.onchange = syncPresetFromDirections;
[elements.steps,elements.maxHz,elements.ramp].forEach((input) => input.oninput = updateManualSummary);
elements.runMotion.onclick = async () => { try { await post("/api/motion", manualPayload(false)); await pollState(); } catch {} };
elements.runReverse.onclick = async () => { try { await post("/api/motion", manualPayload(true)); await pollState(); } catch {} };
elements.useBenchSignal.onclick = () => {
  elements.steps.value = "2000";
  elements.maxHz.value = "4000";
  elements.ramp.value = "0";
  updateManualSummary();
  showToast("Loaded only: 2,000 steps · 4,000 Hz · no ramp. Press Move when ready.");
};
elements.editGroup.onchange = () => { editorPresets(elements.editGroup.value); renderClampProfile(); scheduleEditorEstimate(); };
elements.editPreset.onchange = scheduleEditorEstimate;
elements.editTofMode.onchange = updateTofModeState;
elements.editSteps.oninput = () => { rescaleClampProfile(); updateMotorOffsetSummary(); updateEditorDemoState(); };
elements.editHz.oninput = updateEditorRampPreview;
elements.editRamp.oninput = updateEditorRampPreview;
elements.editM1ExtraSteps.oninput = () => { updateMotorOffsetSummary(); scheduleEditorEstimate(); };
elements.editM6ExtraSteps.oninput = () => { updateMotorOffsetSummary(); scheduleEditorEstimate(); };
elements.editLacPercent.oninput = updateEditorDemoState;
elements.editLacSpeed.oninput = updateEditorDemoState;
elements.editLacClampSteps.oninput = updateEditorDemoState;
elements.editLacClampHz.oninput = updateEditorDemoState;
elements.clampProfileToggle.onchange = () => { renderClampProfile(); updateEditorDemoState(); };
elements.addClampPoint.onclick = () => addClampProfilePoint();
elements.copyClampFromPair.onclick = copyReversedPairProfile;
elements.clampProfileGraph.onclick = (event) => {
  if (app.clampProfileJustDragged || event.target.closest("circle")) return;
  addClampProfilePoint(profileGraphPoint(event));
};
window.addEventListener("pointermove", (event) => {
  if (app.clampProfileDrag == null) return;
  const index = app.clampProfileDrag;
  const point = profileGraphPoint(event);
  const last = app.clampProfilePoints.length - 1;
  if (index !== last) {
    const previous = app.clampProfilePoints[index - 1].at_step;
    const next = app.clampProfilePoints[index + 1].at_step;
    app.clampProfilePoints[index].at_step = Math.max(previous + 1, Math.min(next - 1, point.at_step));
  }
  app.clampProfilePoints[index].offset_steps = point.offset_steps;
  app.clampProfileJustDragged = true;
  renderClampProfile(); updateEditorDemoState();
});
window.addEventListener("pointerup", () => {
  if (app.clampProfileDrag == null) return;
  app.clampProfileDrag = null;
  setTimeout(() => { app.clampProfileJustDragged = false; }, 0);
});
elements.demoTarget.onclick = () => demoEditorTarget(Math.max(0, Math.round(numberValue(elements.editSteps, 0))));
elements.demoZero.onclick = () => demoEditorTarget(0);
elements.rebaseDemo.onclick = async () => {
  if (!confirm("This will call the robot's current physical position 0 without moving it. Only do this when you know this is the action's true starting position.")) return;
  try { await post("/api/action/demo/rebase", { action_id: elements.editActionId.value }); await pollState(); } catch {}
};
elements.captureTof.onclick = () => {
  if (!app.state?.tof?.valid) return;
  const reading = Number(app.state.tof.distance_mm);
  elements.editTofMode.value = "range";
  elements.editTofIdeal.value = reading;
  if (!elements.editTofMin.value) elements.editTofMin.value = Math.max(20, reading - 10);
  if (!elements.editTofMax.value) elements.editTofMax.value = Math.min(4000, reading + 10);
  updateTofModeState();
};
elements.refreshEditorTof.onclick = async () => {
  try { await post("/api/tof", {}); setTimeout(pollState, 120); } catch {}
};
elements.saveEditedAction.onclick = saveEditedAction;
elements.deleteAction.onclick = async () => {
  if (!confirm("Delete this action button?")) return;
  try { await post("/api/action/delete", { action_id: elements.editActionId.value }); elements.actionEditor.close(); await pollState(); } catch {}
};
elements.armToggle.onchange = () => setArmed(elements.armToggle.checked);
elements.stopButton.onclick = () => post("/api/stop", { reason: "STOP button" }).then(updateState).catch(() => {});
elements.resumeMotion.onclick = async () => {
  try { await post("/api/motion/recover", { mode: "resume" }); await pollState(); } catch {}
};
elements.returnMotion.onclick = async () => {
  try { await post("/api/motion/recover", { mode: "return" }); await pollState(); } catch {}
};
elements.readTof.onclick = () => post("/api/tof", {}).then(() => setTimeout(pollState, 160)).catch(() => {});
elements.addFlag.onclick = async () => {
  try { await post("/api/flag", { label: elements.flagLabel.value.trim() }); elements.flagLabel.value = ""; await pollState(); }
  catch {}
};
elements.flagLabel.onkeydown = (event) => { if (event.key === "Enter") { event.preventDefault(); elements.addFlag.click(); } };
elements.clearFlags.onclick = async () => {
  if (!confirm("Clear the visible step timeline? The JSON log will still retain the flags.")) return;
  try { updateState(await post("/api/flags/clear", {})); app.compareA = null; app.compareB = null; } catch {}
};
elements.resetCounters.onclick = async () => {
  if (!confirm("Start a new counter session at zero? Existing flags stay visible but cannot be compared across the reset.")) return;
  try { updateState(await post("/api/counters/reset", {})); } catch {}
};
document.querySelectorAll("[data-lac]").forEach((button) => {
  button.onclick = () => {
    const target = button.dataset.lac; const input = target === "1" ? elements.lac1 : target === "2" ? elements.lac2 : elements.lacBoth;
    post("/api/lac", {
      target,
      percent: Math.round(numberValue(input, 0)),
      speed_percent_s: Math.round(numberValue(elements.lacSpeed, 25)),
    }).then(() => pollState()).catch(() => {});
  };
});
elements.addNote.onclick = async () => {
  const note = elements.note.value.trim(); if (!note) return;
  try { updateState(await post("/api/note", { note })); elements.note.value = ""; } catch {}
};
document.addEventListener("keydown", (event) => {
  if (event.code === "Space" && !(event.target instanceof HTMLInputElement) && !(event.target instanceof HTMLTextAreaElement) && !(event.target instanceof HTMLSelectElement)) {
    event.preventDefault(); post("/api/stop", { reason: "keyboard space" }).then(updateState).catch(() => {});
  }
});

updateManualGroup(); updateManualSummary(); pollState(); pollEvents();
setInterval(pollState, 400); setInterval(pollEvents, 250);
setInterval(renderStudyTimer, 100);
