// Linorobot2 Console frontend. Vanilla JS, no build step, no framework --
// mirrors robot_config_engine/web/app.js: this file builds full shell command
// strings and POSTs them to a generic /api/exec (or /api/agent/exec) SSE
// runner on the server. The server does not know what "bringup" or "teleop"
// mean -- it just runs bash.

const state = {
  config: null,
  status: null,
  mainBusy: false,
  agentBusy: false,
  robot_name: "linorobot2",
  robots: [],
  git_branch: "",
  git_branches: [],
};

const consolePane = document.getElementById("console-pane");
const consoleTitle = document.getElementById("console-title");
const consoleWrap = document.getElementById("console-wrap");

function escapeHtml(value) {
  return String(value == null ? "" : value)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#39;");
}

function openTerminal(title) {
  if (consoleWrap && consoleWrap.classList.contains("collapsed")) {
    consoleWrap.classList.remove("collapsed");
  }
  if (title) {
    setConsoleTitle(title);
  }
  if (consolePane) {
    consolePane.scrollTop = consolePane.scrollHeight;
  }
}

// The pane used to be a plain append onto the pane's textContent, which copies
// the entire buffer on every line and then reads scrollHeight, forcing a
// synchronous layout each time. That is quadratic, and a chatty slot makes it
// fatal: the micro-ROS agent emits a few hundred lines a second against a 50 Hz
// board, and on a fresh Jazzy box the tab pegged a core for 23 of its 24
// minutes of life. Timers stopped firing, so the 1-Click chain simply stopped
// between two steps -- no error, no failed request, nothing in any log.
// Keep a bounded ring buffer and repaint on a timer instead, so console volume
// can never starve the chain that is driving the robot.
const CONSOLE_MAX_LINES = 2000;
const CONSOLE_FLUSH_MS = 100;
const consoleLines = [];
let consoleFlushTimer = null;

function flushConsole() {
  consoleFlushTimer = null;
  if (!consolePane) return;
  consolePane.textContent = consoleLines.length ? consoleLines.join("\n") + "\n" : "";
  consolePane.scrollTop = consolePane.scrollHeight;
}

// setTimeout, not requestAnimationFrame: rAF does not fire in a background tab,
// and a user who switches away mid-SLAM must still get the log when they come
// back. Background throttling clamps this to ~1 Hz, which is plenty.
function queueConsoleFlush() {
  if (consoleFlushTimer !== null) return;
  consoleFlushTimer = setTimeout(flushConsole, CONSOLE_FLUSH_MS);
}

function logLine(text) {
  consoleLines.push(String(text));
  if (consoleLines.length > CONSOLE_MAX_LINES) {
    consoleLines.splice(0, consoleLines.length - CONSOLE_MAX_LINES);
  }
  queueConsoleFlush();
}

function clearConsole() {
  consoleLines.length = 0;
  queueConsoleFlush();
}

function setConsoleTitle(title) {
  consoleTitle.textContent = title;
}

document.getElementById("console-clear").addEventListener("click", clearConsole);
document.getElementById("console-collapse").addEventListener("click", () => {
  consoleWrap.classList.toggle("collapsed");
});

// ---------- tabs ----------
document.querySelectorAll(".tab-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab-btn").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".tab-pane").forEach((p) => p.classList.remove("active"));
    btn.classList.add("active");
    document.getElementById("tab-" + btn.dataset.tab).classList.add("active");
  });
});

// ---------- generic SSE command runner ----------
// slot: "main" -> /api/exec ; "agent" -> /api/agent/exec
function runCommand(command, { slot = "main", title = "Running", action, onDone, onLine } = {}) {
  const endpoint = slot === "agent" ? "/api/agent/exec" : (slot === "bringup" ? "/api/bringup/exec" : "/api/exec");
  openTerminal(title);
  logLine(`$ [${slot}] ${title}`);
  return fetch(endpoint, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ command, slot, action: action || title }),
  }).then(async (response) => {
    if (response.status === 409) {
      logLine("[console] that slot is already busy -- stop the running action first.");
      if (onDone) onDone(-1);
      return;
    }
    if (!response.ok || !response.body) {
      logLine(`[console] failed to start: HTTP ${response.status}`);
      if (onDone) onDone(-1);
      return;
    }
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buf = "";
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      buf += decoder.decode(value, { stream: true });
      const frames = buf.split("\n\n");
      buf = frames.pop();
      for (const frame of frames) {
        const evMatch = frame.match(/^event: (.+)$/m);
        const dataMatch = frame.match(/^data: (.+)$/m);
        if (!dataMatch) continue;
        const evType = evMatch ? evMatch[1] : "message";
        let payload;
        try {
          payload = JSON.parse(dataMatch[1]);
        } catch {
          continue;
        }
        if (evType === "output") {
          const prefix = (slot !== "main") ? `[${slot}] ` : "";
          logLine(`${prefix}${payload.line}`);
          if (onLine) onLine(payload.line);
        } else if (evType === "done") {
          logLine(`[console] exited with code ${payload.exit_code}`);
          if (onDone) onDone(payload.exit_code);
        }
      }
    }
  });
}

function killSlot(slot) {
  const endpoint = slot === "agent" ? "/api/agent/kill" : (slot === "bringup" ? "/api/bringup/kill" : "/api/kill");
  return fetch(endpoint, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ slot }),
  });
}

// pairs up a Start/Stop button with a command builder for long-running actions
function wireStartStop({ startBtn, stopBtn, slot, title, buildCommand, needsAgent, needsBringup }) {
  startBtn.addEventListener("click", async () => {
    startBtn.disabled = true;
    try {
      if (needsBringup) {
        await ensureBringupRunning(title);
      } else if (needsAgent) {
        await ensureAgentRunning();
      }
      const command = await buildCommand();
      stopBtn.disabled = false;
      runCommand(command, {
        slot,
        title,
        onDone: () => {
          startBtn.disabled = false;
          stopBtn.disabled = true;
        },
      });
    } catch (err) {
      console.error(`Failed to start ${title}:`, err);
      logLine(`[console] ✖ Failed to start ${title}: ${err.message || err}`);
      startBtn.disabled = false;
      stopBtn.disabled = true;
    }
  });
  stopBtn.addEventListener("click", () => {
    killSlot(slot);
  });
}

// ---------- workspace / ros-env prefix ----------
function getDistro() {
  return (state.config && state.config.ros_distro) || (state.status && state.status.ros_distro) || "jazzy";
}

function envPrefix() {
  const distro = getDistro();
  const ws = (state.config && state.config.workspace_path) || "~/linorobot2_ws";
  // Service replies are dropped when rmw_fastrtps cannot match the response
  // writer to the client's reader inside max_blocking_time (100 ms by default),
  // which strands nav2's lifecycle manager mid-transition. The XML raises that
  // ceiling for service endpoints only -- see config/fastdds_service_qos.xml.
  // Everything nav2/SLAM is launched through here, not through server.py's
  // ros_env_prefix(), so the export has to live in this string too.
  const webDir = (state.status && state.status.web_dir) || "";
  const qosPrefix = webDir
    ? `export FASTDDS_DEFAULT_PROFILES_FILE=${webDir}/../config/fastdds_service_qos.xml; `
    : "";
  return qosPrefix +
         `export PATH=/usr/bin:$PATH; export ROS_DISTRO=${distro}; ` +
         `if [ -f /opt/ros/${distro}/setup.bash ]; then source /opt/ros/${distro}/setup.bash 2>/dev/null || true; ` +
         `elif [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash 2>/dev/null || true; ` +
         `elif [ -f /opt/ros/rolling/setup.bash ]; then source /opt/ros/rolling/setup.bash 2>/dev/null || true; ` +
         `fi; ` +
         `[ -f ${ws}/install/setup.bash ] && source ${ws}/install/setup.bash 2>/dev/null || true; `;
}

function gitCloneDistroSnippet(repoUrl, targetDir) {
  const distro = getDistro();
  return `[ -d ${targetDir} ] || git clone -b ${distro} ${repoUrl} ${targetDir} 2>/dev/null || ` +
         `git clone -b main ${repoUrl} ${targetDir} 2>/dev/null || ` +
         `git clone -b jazzy ${repoUrl} ${targetDir} 2>/dev/null || ` +
         `git clone ${repoUrl} ${targetDir}`;
}

function ws() {
  return (state.config && state.config.workspace_path) || "~/linorobot2_ws";
}

// ---------- status polling ----------
async function refreshStatus() {
  try {
    const res = await fetch("/api/status");
    const s = await res.json();
    state.status = s;
    state.config = s.config;
    applyLaserConfigToPanel();
    state.mainBusy = s.main_busy;
    state.agentBusy = s.agent_busy_console;

    const distroSel = document.getElementById("hdr-distro-select");
    if (distroSel && s.ros_distro) {
      distroSel.value = s.ros_distro;
    }

    // Robot name + branch header (don't clobber a field the user is editing)
    if (s.robot_name) state.robot_name = s.robot_name;
    if (Array.isArray(s.robots)) state.robots = s.robots;
    if (typeof s.git_branch === "string") state.git_branch = s.git_branch;
    const robotInput = document.getElementById("hdr-robot-name");
    if (robotInput && document.activeElement !== robotInput) {
      robotInput.value = state.robot_name;
    }
    const branchInput = document.getElementById("hdr-git-branch");
    if (branchInput && document.activeElement !== branchInput) {
      branchInput.value = state.git_branch;
    }
    const cfgDistroSel = document.getElementById("cfg-ros-distro");
    if (cfgDistroSel && s.ros_distro) {
      cfgDistroSel.value = s.ros_distro;
    }

    document.getElementById("hdr-workspace").innerHTML =
      `Workspace: <b>${s.workspace_built ? "built" : "not built"}</b>`;

    const agentPill = document.getElementById("hdr-agent-pill");
    const agentAlive = s.agent_alive_external || s.agent_busy_console;
    agentPill.textContent = agentAlive ? "running" : "down";
    agentPill.className = "pill " + (agentAlive ? "pill-ok" : "pill-off");
    document.getElementById("btn-agent-stop").disabled = !s.agent_busy_console;

    const bringupPill = document.getElementById("hdr-bringup-pill");
    const bringupAlive = s.bringup_alive_external || s.bringup_busy_console;
    if (bringupPill) {
      bringupPill.textContent = bringupAlive ? "running" : "down";
      bringupPill.className = "pill " + (bringupAlive ? "pill-ok" : "pill-off");
    }
    const bringupStartBtn = document.getElementById("btn-bringup-start");
    const bringupStopBtn = document.getElementById("btn-bringup-stop");
    if (bringupStartBtn) bringupStartBtn.disabled = s.bringup_busy_console;
    if (bringupStopBtn) bringupStopBtn.disabled = !s.bringup_busy_console;

    const autoBringupCfg = document.getElementById("cfg-auto-bringup");
    const autoBringupToggle = document.getElementById("bringup-auto-toggle");
    if (s.config && typeof s.config.auto_bringup === "boolean") {
      if (autoBringupCfg) autoBringupCfg.checked = s.config.auto_bringup;
      if (autoBringupToggle) autoBringupToggle.checked = s.config.auto_bringup;
    }

    if (!document.getElementById("install-workspace").value) {
      document.getElementById("install-workspace").value = s.workspace_path;
    }
    if (!document.getElementById("cfg-workspace").value) {
      document.getElementById("cfg-workspace").value = s.workspace_path;
    }
    if (s.config) {
      const c = s.config;
      const setIfEmpty = (id, val) => {
        const el = document.getElementById(id);
        if (el && !el.value) el.value = val;
      };
      document.getElementById("cfg-agent-transport").value = c.agent_transport;
      setIfEmpty("cfg-agent-device", c.agent_device);
      setIfEmpty("cfg-agent-port", c.agent_port);
      setIfEmpty("cfg-agent-baud", c.agent_baud);

      if (c.install_mode) {
        const hdrM = document.getElementById("hdr-install-mode");
        const tabM = document.getElementById("install-mode");
        if (hdrM && !localStorage.getItem("linorobot2_install_mode")) hdrM.value = c.install_mode;
        if (tabM && !localStorage.getItem("linorobot2_install_mode")) {
          tabM.value = c.install_mode;
          const isNative = c.install_mode === "native";
          const natCards = document.getElementById("install-native-cards");
          const dkrCard = document.getElementById("install-docker-card");
          if (natCards) natCards.style.display = isNative ? "block" : "none";
          if (dkrCard) dkrCard.style.display = isNative ? "none" : "block";
        }
      }

      if (c.agent_engine) {
        const hdrA = document.getElementById("hdr-agent-engine");
        const cfgA = document.getElementById("cfg-agent-engine");
        if (hdrA && !localStorage.getItem("linorobot2_agent_engine")) hdrA.value = c.agent_engine;
        if (cfgA && !localStorage.getItem("linorobot2_agent_engine")) cfgA.value = c.agent_engine;
      }

      if (c.container_registry) {
        const hdrR = document.getElementById("hdr-container-registry");
        const cfgR = document.getElementById("cfg-container-registry");
        const regMode = (c.container_registry === "custom" || (!["auto", "cluster", "dockerhub"].includes(c.container_registry))) ? "custom" : c.container_registry;
        const customVal = c.custom_registry || (!["auto", "cluster", "dockerhub"].includes(c.container_registry) ? c.container_registry : "");
        if (hdrR && !localStorage.getItem("linorobot2_container_registry")) hdrR.value = regMode;
        if (cfgR && !localStorage.getItem("linorobot2_container_registry")) cfgR.value = regMode;
        const hdrCust = document.getElementById("hdr-custom-registry");
        const cfgCust = document.getElementById("cfg-custom-registry");
        if (hdrCust) {
          if (!localStorage.getItem("linorobot2_custom_registry")) hdrCust.value = customVal;
          hdrCust.style.display = (regMode === "custom") ? "inline-block" : "none";
        }
        if (cfgCust) {
          if (!localStorage.getItem("linorobot2_custom_registry")) cfgCust.value = customVal;
          cfgCust.style.display = (regMode === "custom") ? "block" : "none";
        }
      }
    }
  } catch (e) {
    // server not reachable yet / transient -- ignore, next poll will retry
  }
}
setInterval(refreshStatus, 4000);
refreshStatus();

// ---------- Robot Name + Branch header ----------
// config/<robot>_config.yaml is the single source of truth; it is git
// auto-committed on the active branch before every action (server-side).
async function loadGitInfo() {
  try {
    const gi = await fetch("/api/gitinfo", { cache: "no-cache" }).then((r) => r.json());
    state.git_branch = gi.branch || state.git_branch;
    state.git_branches = gi.branches || [];
    const branchInput = document.getElementById("hdr-git-branch");
    if (branchInput && document.activeElement !== branchInput) branchInput.value = state.git_branch;
    const text = document.getElementById("git-version-text");
    const badge = document.getElementById("git-version-badge");
    if (text && (gi.version_at_start || gi.version)) {
      text.textContent = gi.version_at_start || gi.version;
    }
    if (badge) {
      if (gi.dirty || gi.moved_since_start) badge.classList.add("is-dirty");
      else badge.classList.remove("is-dirty");
    }
  } catch (e) { /* ignore */ }
}

async function loadRobotList() {
  try {
    const r = await fetch("/api/robots").then((x) => x.json());
    state.robots = r.robots || [];
    state.robot_name = r.active || state.robot_name;
  } catch (e) { /* ignore */ }
}

// Dropdown picker shared by the Robot and Branch header fields. Ported from
// robot_config_engine's tested initBranchPicker(): the input itself opens the
// list, ArrowUp/Down opens it, Escape closes it, typing filters it live, and
// the entry matching the current value carries a green dot.
function initHeaderPicker({ inputId, caretId, menuId, loadItems, onPick, emptyText }) {
  const input = document.getElementById(inputId);
  const caret = document.getElementById(caretId);
  const menu = document.getElementById(menuId);
  if (!input || !menu) return;

  const close = () => {
    menu.hidden = true;
    if (caret) caret.setAttribute("aria-expanded", "false");
  };

  const pick = (name) => {
    input.value = name;
    close();
    onPick(name);
  };

  const render = ({ items, current }) => {
    if (!items.length) {
      menu.innerHTML = `<div class="branch-empty">${escapeHtml(emptyText)}</div>`;
      return;
    }
    const typed = input.value.trim();
    menu.innerHTML = items.map((b) => `
      <button type="button" role="option" class="branch-item${b === current ? " is-current" : ""}${b === typed ? " is-active" : ""}" data-name="${escapeHtml(b)}">
        <span class="branch-cur-dot"></span><span>${escapeHtml(b)}</span>${
          b === current ? '<span style="margin-left:auto;font-size:0.68rem;opacity:0.6">current</span>' : ""
        }
      </button>`).join("");
    menu.querySelectorAll(".branch-item").forEach((btn) => {
      btn.addEventListener("click", () => pick(btn.dataset.name));
    });
  };

  const open = async () => {
    menu.hidden = false;
    if (caret) caret.setAttribute("aria-expanded", "true");
    menu.innerHTML = `<div class="branch-empty">Loading…</div>`;
    render(await loadItems());
  };

  input.addEventListener("click", (e) => { e.stopPropagation(); if (menu.hidden) open(); });
  input.addEventListener("keydown", (e) => {
    if ((e.key === "ArrowDown" || e.key === "ArrowUp") && menu.hidden) { e.preventDefault(); open(); }
  });
  if (caret) {
    caret.addEventListener("click", (e) => {
      e.stopPropagation();
      if (menu.hidden) open(); else close();
    });
  }
  // Re-filter the visible list as the user types.
  input.addEventListener("input", () => {
    if (menu.hidden) return;
    const typed = input.value.trim().toLowerCase();
    menu.querySelectorAll(".branch-item").forEach((btn) => {
      btn.style.display = btn.dataset.name.toLowerCase().includes(typed) ? "" : "none";
      btn.classList.toggle("is-active", btn.dataset.name === input.value.trim());
    });
  });
  document.addEventListener("click", (e) => {
    if (!menu.hidden && !menu.contains(e.target) && e.target !== input && e.target !== caret) close();
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && !menu.hidden) { close(); input.blur(); }
  });
}

async function selectRobot(name) {
  name = (name || "").trim();
  if (!name || !/^[a-z0-9_]+$/.test(name)) {
    logLine(`[console] invalid robot name: "${name}" (use lowercase, digits, _)`);
    const ri = document.getElementById("hdr-robot-name");
    if (ri) ri.value = state.robot_name;
    return;
  }
  if (name === state.robot_name) return;
  try {
    const res = await fetch("/api/robot/select", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    }).then((r) => r.json());
    if (res.error) { logLine(`[console] ${res.error}`); return; }
    state.robot_name = res.active;
    state.robots = res.robots || [];
    state.config = res.config || state.config;
    // Push the switched robot's workflow settings into the header selects.
    const c = state.config || {};
    const put = (id, v) => { const el = document.getElementById(id); if (el && v != null) el.value = v; };
    put("hdr-distro-select", c.ros_distro);
    put("hdr-install-mode", c.install_mode);
    put("hdr-agent-engine", c.agent_engine);
    put("install-mode", c.install_mode);
    put("cfg-agent-engine", c.agent_engine);
    put("hdr-container-registry", c.container_registry);
    put("cfg-container-registry", c.container_registry);
    logLine(`[console] active robot -> ${res.active}  (${res.robot_config_path})`);
    refreshStatus();
  } catch (e) {
    logLine(`[console] robot select failed: ${e}`);
  }
}

function checkoutBranch(branch) {
  branch = (branch || "").trim();
  if (!branch || !/^[A-Za-z0-9._/-]+$/.test(branch)) {
    logLine(`[console] invalid branch name: "${branch}"`);
    return;
  }
  if (branch === state.git_branch) return;
  setConsoleTitle(`git checkout ${branch}`);
  logLine(`$ git checkout ${branch}`);
  fetch("/api/gitinfo/branch", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ branch }),
  }).then(async (response) => {
    if (!response.ok || !response.body) {
      logLine(`[console] checkout failed: HTTP ${response.status}`);
      return;
    }
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buf = "";
    for (;;) {
      const { done, value } = await reader.read();
      if (done) break;
      buf += decoder.decode(value, { stream: true });
      const parts = buf.split("\n\n");
      buf = parts.pop();
      for (const chunk of parts) {
        const m = /^data: (.*)$/m.exec(chunk);
        if (!m) continue;
        try {
          const payload = JSON.parse(m[1]);
          if (payload.line) logLine(payload.line);
          if (typeof payload.exit_code === "number") {
            logLine(`[console] checkout exited ${payload.exit_code}`);
            loadGitInfo();
            refreshStatus();
          }
        } catch (e) { /* ignore */ }
      }
    }
  }).catch((e) => logLine(`[console] checkout error: ${e}`));
}

function setupRobotBranchHeader() {
  const robotInput = document.getElementById("hdr-robot-name");
  const branchInput = document.getElementById("hdr-git-branch");
  const toNameBtn = document.getElementById("btn-branch-to-name");
  if (!robotInput || !branchInput) return;

  robotInput.addEventListener("keydown", (e) => {
    if (e.key === "Enter") { e.preventDefault(); robotInput.blur(); }
  });
  robotInput.addEventListener("blur", () => selectRobot(robotInput.value));

  branchInput.addEventListener("keydown", (e) => {
    if (e.key === "Enter") { e.preventDefault(); checkoutBranch(branchInput.value); }
  });

  initHeaderPicker({
    inputId: "hdr-robot-name",
    caretId: "btn-robot-menu",
    menuId: "robot-menu",
    emptyText: "No saved robot configs.",
    loadItems: async () => {
      await loadRobotList();
      return { items: state.robots.map((r) => r.name), current: state.robot_name };
    },
    onPick: selectRobot,
  });

  initHeaderPicker({
    inputId: "hdr-git-branch",
    caretId: "btn-branch-menu",
    menuId: "branch-menu",
    emptyText: "No local git branches.",
    loadItems: async () => {
      await loadGitInfo();
      return { items: state.git_branches, current: state.git_branch };
    },
    onPick: checkoutBranch,
  });

  toNameBtn.addEventListener("click", () => {
    const n = (robotInput.value || state.robot_name || "").trim();
    if (!n) return;
    branchInput.value = n;
    checkoutBranch(n);
  });

  loadRobotList();
  loadGitInfo();
}
setupRobotBranchHeader();

// =============================================================================
// Header Git Version Badge — shows the 7-char commit the web server booted on;
// click to reveal the branch, remotes and last 10 commits (GET /api/gitinfo).
// =============================================================================
function initGitVersionBadge() {
  const badge = document.getElementById("git-version-badge");
  const text = document.getElementById("git-version-text");
  const popover = document.getElementById("git-version-popover");
  if (!badge || !text || !popover) return;

  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => (
    { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]
  ));

  let loaded = null;

  const render = (info) => {
    const remotes = (info.remotes || []).map((r) => `
      <div class="gv-line">
        <span class="gv-remote-name">${esc(r.name)}</span>
        <span class="gv-val">${esc(r.url)}</span>
      </div>`).join("") || `<div class="gv-line"><span class="gv-val">(no remotes)</span></div>`;

    const commits = (info.commits || []).map((c) => `
      <li>
        <div><span class="gv-hash">${esc(c.hash)}</span> <span class="gv-subject">${esc(c.subject)}</span></div>
        <div class="gv-meta">${esc(c.author)} · ${esc(c.date)} (${esc(c.reldate)})</div>
      </li>`).join("") || `<li><span class="gv-meta">(no commit history)</span></li>`;

    const movedNote = info.moved_since_start
      ? `<div class="gv-note">⚠ HEAD is now at <code>${esc(info.version)}</code> — the server is still running the <code>${esc(info.version_at_start)}</code> build. Restart server.py to pick up the new code.</div>`
      : "";
    const dirtyNote = info.dirty
      ? `<div class="gv-note">Working tree has uncommitted changes.</div>`
      : "";

    popover.innerHTML = `
      <h4>Version</h4>
      <div class="gv-line"><span class="gv-key">server @</span><span class="gv-val">${esc(info.version_at_start || info.version)}</span></div>
      <div class="gv-line"><span class="gv-key">branch</span><span class="gv-val">${esc(info.branch)}</span></div>
      <h4>Remotes</h4>
      ${remotes}
      <h4>Last 10 commits</h4>
      <ol class="gv-commits">${commits}</ol>
      ${movedNote}
      ${dirtyNote}`;
  };

  const load = async () => {
    try {
      const res = await fetch("/api/gitinfo", { cache: "no-cache" });
      if (!res.ok) return null;
      return await res.json();
    } catch (e) {
      return null;
    }
  };

  const closePopover = () => {
    popover.hidden = true;
    badge.setAttribute("aria-expanded", "false");
  };

  const openPopover = async () => {
    const fresh = await load();
    if (fresh) {
      loaded = fresh;
      render(loaded);
      const branchInput = document.getElementById("hdr-git-branch");
      if (branchInput && document.activeElement !== branchInput) {
        branchInput.value = fresh.branch;
      }
    }
    if (!loaded) return;
    popover.hidden = false;
    badge.setAttribute("aria-expanded", "true");
  };

  badge.addEventListener("click", (e) => {
    e.stopPropagation();
    if (popover.hidden) openPopover();
    else closePopover();
  });
  document.addEventListener("click", (e) => {
    if (!popover.hidden && !popover.contains(e.target) && e.target !== badge) closePopover();
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && !popover.hidden) closePopover();
  });

  // Prime the badge label at startup.
  load().then((info) => {
    if (!info) { text.textContent = "no-git"; return; }
    loaded = info;
    render(info);
    text.textContent = info.version_at_start || info.version || "unknown";
    if (info.dirty || info.moved_since_start) badge.classList.add("is-dirty");
    const branchInput = document.getElementById("hdr-git-branch");
    if (branchInput && document.activeElement !== branchInput) {
      branchInput.value = info.branch;
    }
  });
}
initGitVersionBadge();

// ---------- sensor registry (single source of truth) ----------
// Everything sensor-related -- Install driver list, Bringup model codes, the
// Docker/.env choices, the Sensors/LiDAR launcher, install/udev commands --
// comes from ONE /api/sensors payload (server.py's LASER_SENSORS/DEPTH_SENSORS).
// No parallel copies live in this file anymore.
let SENSORS = { laser: {}, depth: {} };
let SERIAL_PORTS = [];

function addOpt(sel, value, text) {
  if (!sel) return;
  const o = document.createElement("option");
  o.value = value;
  o.textContent = text;
  sel.appendChild(o);
}

function laserEntryForModel(code) {
  return Object.entries(SENSORS.laser).find(
    ([, e]) => (e.models || []).some((m) => m.code === code)
  );
}

function populateSensorSelects() {
  // ... and re-apply the saved laser at the end, because this and the /api/status
  // fetch race: whichever lands last has to be the one that decides the panel.
  // Install tab -- one entry per driver package
  Object.entries(SENSORS.laser).forEach(([k, e]) => addOpt(document.getElementById("install-laser"), k, e.label));
  Object.entries(SENSORS.depth).forEach(([k, e]) => addOpt(document.getElementById("install-depth"), k, e.label));

  // Bringup tab -- one entry per model code (LINOROBOT2_*_SENSOR value)
  Object.values(SENSORS.laser).forEach((e) =>
    (e.models || []).forEach((m) => addOpt(document.getElementById("bringup-laser-sensor"), m.code, `${m.code} — ${m.label}`)));
  Object.values(SENSORS.depth).forEach((e) =>
    (e.models || []).forEach((m) => addOpt(document.getElementById("bringup-depth-sensor"), m.code, `${m.code} — ${m.label}`)));

  // Docker tab -- only drivers that have a docker/.env key
  Object.values(SENSORS.laser).forEach((e) => e.docker_key && addOpt(document.getElementById("docker-laser-sensor"), e.docker_key, `${e.docker_key} (${e.label})`));
  Object.values(SENSORS.depth).forEach((e) => e.docker_key && addOpt(document.getElementById("docker-depth-sensor"), e.docker_key, `${e.docker_key} (${e.label})`));

  // Sensors/LiDAR tab launcher -- one entry per model code
  const lm = document.getElementById("laser-driver-model");
  Object.values(SENSORS.laser).forEach((e) =>
    (e.models || []).forEach((m) => addOpt(lm, m.code, `${m.code} — ${m.label}`)));
  if (typeof updateLaserDriverFieldsVisibility === "function") updateLaserDriverFieldsVisibility();

  // Robot Environment tab selects
  const envLaser = document.getElementById("env-laser-sensor");
  if (envLaser) {
    Object.values(SENSORS.laser).forEach((e) =>
      (e.models || []).forEach((m) => addOpt(envLaser, m.code, `${m.code} — ${m.label}`)));
  }
  const envDepth = document.getElementById("env-depth-sensor");
  if (envDepth) {
    Object.values(SENSORS.depth).forEach((e) =>
      (e.models || []).forEach((m) => addOpt(envDepth, m.code, `${m.code} — ${m.label}`)));
  }
  applyLaserConfigToPanel();
}

function renderSerialPortList() {
  const dl = document.getElementById("serial-ports-list");
  if (dl) {
    dl.innerHTML = "";
    SERIAL_PORTS.forEach((p) => {
      const o = document.createElement("option");
      o.value = p.preferred;
      o.label = `${p.vendor || "?"} ${p.model || ""} ${p.usb_id ? "[" + p.usb_id + "]" : ""} → ${p.tty}`.trim();
      dl.appendChild(o);
    });
  }
  const box = document.getElementById("serial-ports-detected");
  if (box) {
    if (!SERIAL_PORTS.length) {
      box.textContent = "No USB serial devices detected.";
    } else {
      box.innerHTML = SERIAL_PORTS.map((p) => {
        // Most lidars use a generic CP2102/CH340/FTDI bridge, so VID:PID and
        // the model string rarely tell devices apart -- the by-path (physical
        // USB port) is the reliable identifier and what we store.
        const idbits = [p.usb_id, p.vendor, p.model].filter(Boolean).join(" · ") || "generic UART";
        const sn = p.serial ? ` · SN ${p.serial}` : " · no serial#";
        return `<div style="margin-bottom:6px">` +
          `<code>${escapeHtml(p.preferred)}</code><br>` +
          `<span class="hint">${escapeHtml(idbits)}${escapeHtml(sn)} · now ${escapeHtml(p.tty)}</span>` +
          `</div>`;
      }).join("");
    }
  }
}

async function refreshSerialPorts() {
  try {
    const r = await fetch("/api/serial_ports");
    SERIAL_PORTS = (await r.json()).ports || [];
  } catch (e) {
    SERIAL_PORTS = [];
  }
  renderSerialPortList();
}

fetch("/api/sensors")
  .then((r) => r.json())
  .then((data) => {
    SENSORS = data;
    populateSensorSelects();
  })
  .catch(() => {});
refreshSerialPorts();

// ---------- import config ----------
document.getElementById("btn-import").addEventListener("click", async () => {
  const path = document.getElementById("import-path").value.trim();
  if (!path) return;
  const res = await fetch("/api/import_config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ path }),
  });
  const data = await res.json();
  const resultEl = document.getElementById("import-result");
  if (data.error) {
    resultEl.textContent = data.error;
    return;
  }
  if (data.type === "unified_yaml") {
    resultEl.innerHTML = `<span style="color: var(--success);">✓ Imported Unified Configuration! Linorobot2, Nav2, EKF, and SLAM synchronized.</span>`;
    loadRobotEnv();
    loadUnifiedConfig();
    return;
  }
  if (data.type === "robot_env") {
    resultEl.innerHTML = `<span style="color: var(--success);">✓ Imported robot.env! Environment settings updated and synced to ~/.bashrc.</span>`;
    loadRobotEnv();
    return;
  }
  if (data.base) document.getElementById("install-base").value = data.base;
  if (data.transport) document.getElementById("cfg-agent-transport").value = data.transport;
  if (data.agent_baud) document.getElementById("cfg-agent-baud").value = data.agent_baud;
  if (data.agent_port) document.getElementById("cfg-agent-port").value = data.agent_port;
  if (data.agent_ip) document.getElementById("cfg-agent-device").placeholder = data.agent_ip;
  // The header also settles how the scan reaches this computer. The server has
  // already persisted that, so re-read the config and let the laser panel
  // follow it instead of mapping the same fields a second time here.
  if (data.lidar_transport) {
    fetch("/api/config")
      .then((r) => r.json())
      .then((c) => { state.config = c; applyLaserConfigToPanel(); })
      .catch(() => {});
  }
  const lidarBit = data.lidar_transport
    ? ` lidar=${data.lidar_transport}` +
      (data.lidar_transport === "udp_bridge" ? `:${data.lidar_udp_port || "8889"}` : "") +
      (data.lidar_baud ? `@${data.lidar_baud}` : "")
    : "";
  resultEl.textContent =
    `Imported: base=${data.base || "?"} transport=${data.transport} ` +
    `has_imu=${data.has_imu} has_mag=${data.has_mag}` + lidarBit +
    (data.mag_bias ? ` mag_bias=[${data.mag_bias.join(", ")}]` : "");
  (data.warnings || []).forEach((w) => {
    const line = document.createElement("div");
    line.style.color = "var(--accent-warn)";
    line.textContent = "\u26a0 " + w;
    resultEl.appendChild(line);
  });
});

// ---------- install actions ----------
document.getElementById("btn-install-base").addEventListener("click", () => {
  const workspace = document.getElementById("install-workspace").value.trim();
  if (workspace) {
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ workspace_path: workspace }),
    });
  }
  checkAndBuildWorkspace();
});

// Install/udev command assembly lives on the server now (build_sensor_install_cmd);
// the client asks for the joined string via POST /api/sensor_install_cmd. No
// mirrored command table here.
async function fetchSensorInstallCmd(kind, key, { skipUdev = false, udevOnly = false } = {}) {
  const r = await fetch("/api/sensor_install_cmd", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ kind, key, skip_udev: skipUdev, udev_only: udevOnly, workspace_path: ws() }),
  });
  if (!r.ok) return null;
  return (await r.json()).command || null;
}

async function runSensorInstall(kind, selId, skipId, titlePrefix) {
  const key = document.getElementById(selId).value;
  if (!key) return;
  const skip = document.getElementById(skipId).checked;
  const cmd = await fetchSensorInstallCmd(kind, key, { skipUdev: skip });
  if (!cmd) {
    logLine(`[console] no install steps defined for ${kind} "${key}" -- see its own driver docs.`);
    return;
  }
  runCommand(envPrefix() + `cd ${ws()} && ` + cmd, { title: `${titlePrefix}: ${key}` });
}

document.getElementById("btn-install-laser").addEventListener("click", () =>
  runSensorInstall("laser", "install-laser", "laser-skip-udev", "Install laser"));
document.getElementById("btn-install-depth").addEventListener("click", () =>
  runSensorInstall("depth", "install-depth", "depth-skip-udev", "Install depth camera"));

// ---------- Docker / Podman install mode ----------
// The console has its OWN compose stack at tools/console/docker/
// (docker-compose.yaml + a generated .env + devices.generated.yaml). It reuses
// the upstream linorobot2 image + docker/Dockerfile (built, not modified) but
// no ROS install on this host at all -- sensor drivers install *inside* the
// image via the Dockerfile's own `bash install.bash ...` step (that's
// linorobot2's documented build process, not something Console runs on the
// host, so it doesn't conflict with the "no install.bash on the host" rule
// the native install path follows).
const installModeSel = document.getElementById("install-mode");
installModeSel.addEventListener("change", () => {
  const isNative = installModeSel.value === "native";
  document.getElementById("install-native-cards").style.display = isNative ? "block" : "none";
  document.getElementById("install-docker-card").style.display = isNative ? "none" : "block";
});

// The console's OWN compose dir -- it never writes into the repo's upstream
// docker/ dir. `.env` + `devices.generated.yaml` are written here by
// btn-docker-build; the checked-in docker-compose.yaml drives nav/SLAM
// through the console's own launch_nav2.py / launch_bringup.py.
function dockerDir() {
  return `${ws()}/src/linorobot2/tools/console/docker`;
}
// The -f overlay + --env-file the console always passes to compose.
function dockerComposeFlags() {
  return `--env-file .env -f docker-compose.yaml -f devices.generated.yaml`;
}

// Resolved at command-run time (not build time) since we can't be sure which
// of `podman compose` (the compose plugin) or the standalone `podman-compose`
// tool is actually installed -- docker itself only has one real option.
function composeResolveSnippet() {
  if (installModeSel.value === "podman") {
    return `if command -v podman-compose >/dev/null 2>&1; then COMPOSE="podman-compose"; else COMPOSE="podman compose"; fi; `;
  }
  return `COMPOSE="docker compose"; `;
}

// Docker/.env's LASER_SENSOR/DEPTH_SENSOR use a coarser name than
// lasers.launch.py's per-model `sensor` codes (e.g. "rplidar", not a1/.../s3).
// Look up the driver-package key from the registry's docker_key so the
// *udev-rules-only* step below (driver install itself happens inside the
// image) can reuse the same server-side command builder.
function sensorKeyForDockerValue(kind, dockerVal) {
  const table = kind === "laser" ? SENSORS.laser : SENSORS.depth;
  const hit = Object.entries(table).find(([, e]) => e.docker_key === dockerVal);
  return hit ? hit[0] : null;
}
function dockerLaserDevice(dockerVal) {
  const k = sensorKeyForDockerValue("laser", dockerVal);
  return k ? SENSORS.laser[k].symlink : null;
}

function cloneLinorobot2Command() {
  const workspace = document.getElementById("install-workspace").value.trim() || ws();
  return [
    `mkdir -p ${workspace}/src`,
    `cd ${workspace}/src`,
    gitCloneDistroSnippet("https://github.com/linorobot/linorobot2", "linorobot2"),
  ].join(" && ");
}

document.getElementById("btn-docker-build").addEventListener("click", () => {
  const baseImage = document.getElementById("docker-base-image").value;
  const robotBase = document.getElementById("install-base").value;
  const laser = document.getElementById("docker-laser-sensor").value;
  const depth = document.getElementById("docker-depth-sensor").value;
  const serialPort = document.getElementById("docker-base-serial-port").value.trim() || "/dev/ttyACM0";
  const domainId = document.getElementById("docker-ros-domain-id").value.trim() || "0";
  const gpuId = document.getElementById("docker-gpu-id").value.trim() || "0";
  const distro = (state.status && state.status.ros_distro) || "jazzy";

  const envBody =
    `DOCKER_ROS_DISTRO=${distro}\n` +
    `BASE_IMAGE=${baseImage}\n` +
    `ROBOT_BASE=${robotBase}\n` +
    `LASER_SENSOR=${laser}\n` +
    `DEPTH_SENSOR=${depth}\n` +
    `BASE_SERIAL_PORT=${serialPort}\n` +
    `ODOM_TOPIC=/odom\n` +
    `ROBOT_NAME=${state.robot_name || "linorobot2"}\n` +
    `ROS_DOMAIN_ID=${domainId}\n` +
    `CUSTOM_ROBOT=false\n` +
    `LAUNCH_EXTRA=false\n` +
    `LAUNCH_JOYSTICK=false\n` +
    `GPU_ID=${gpuId}\n` +
    `VIRTUALGL_VER=3.1.4\n`;

  // Only the two device mappings docs/docker.md itself documents as a
  // Base-serial + lidar device mappings for the console compose's `bringup`
  // service, written as its OWN overlay file inside tools/console/
  // docker/ -- the repo's docker/ dir is never written.
  const laserDevice = dockerLaserDevice(laser);
  const deviceLines = [`      - ${serialPort}:${serialPort}`];
  if (laserDevice) deviceLines.push(`      - ${laserDevice}:${laserDevice}`);
  const overrideBody =
    `services:\n  bringup:\n    devices:\n${deviceLines.join("\n")}\n`;

  const dir = dockerDir();
  // Steps are newline-joined, not `&&`-joined: a heredoc's closing delimiter
  // must be alone on its own line, so anything appended right after it on
  // the SAME line (like " && next-command") is never recognized as the
  // terminator -- bash just keeps reading everything that follows as more
  // heredoc body, silently swallowing every later step. `set -e` keeps the
  // fail-fast behavior `&&` would have given.
  const cmd = "set -e\n" + [
    cloneLinorobot2Command(),
    `mkdir -p ${dir}`,
    `cat > ${dir}/.env << 'CONSOLE_DOCKER_ENV_EOF'\n${envBody}CONSOLE_DOCKER_ENV_EOF`,
    `cat > ${dir}/devices.generated.yaml << 'CONSOLE_DOCKER_OVERRIDE_EOF'\n${overrideBody}CONSOLE_DOCKER_OVERRIDE_EOF`,
    `cd ${dir}`,
    `${composeResolveSnippet()}HOST_UID=$(id -u) HOST_GID=$(id -g) $COMPOSE ${dockerComposeFlags()} build`,
  ].join("\n");
  runCommand(cmd, { title: `Docker/Podman build (${baseImage})` });
});

document.getElementById("btn-docker-udev").addEventListener("click", async () => {
  const laser = document.getElementById("docker-laser-sensor").value;
  const depth = document.getElementById("docker-depth-sensor").value;
  const cmds = [];
  for (const [kind, dockerVal] of [["laser", laser], ["depth", depth]]) {
    if (!dockerVal) continue;
    const key = sensorKeyForDockerValue(kind, dockerVal);
    if (!key) {
      logLine(`[console] no registry entry for ${kind} "${dockerVal}" -- check its own driver docs (e.g. ZED SDK).`);
      continue;
    }
    const c = await fetchSensorInstallCmd(kind, key, { udevOnly: true });
    if (c) cmds.push(c);
    else logLine(`[console] "${dockerVal}" has no persistent udev symlink -- it'll enumerate as a plain /dev/ttyUSBx or /dev/ttyACMx.`);
  }
  if (!cmds.length) return;
  runCommand(envPrefix() + cmds.join(" && "), { title: "Install udev rules (host)" });
});

const btnDockerServiceStart = document.getElementById("btn-docker-service-start");
const btnDockerServiceStop = document.getElementById("btn-docker-service-stop");
btnDockerServiceStart.addEventListener("click", async () => {
  const service = document.getElementById("docker-service").value;
  if (["slam", "navigate"].includes(service)) {
    if (isAutoBringupEnabled()) {
      await ensureBringupRunning();
    }
  }
  // linorobot2's own Tmuxinator profiles (docker/profiles/*.yml) always
  // `export DISPLAY=:200` before `docker compose up` -- GUI services
  // (gazebo, rviz, slam/navigate with rviz:=true) render into that
  // virtual display, which the kasmvnc service then streams to a browser.
  // Skipping it isn't just "no picture" -- gz sim's GUI process crashes
  // outright trying to open an unset/invalid display.
  const cmd = `${composeResolveSnippet()}cd ${dockerDir()} && DISPLAY=:200 $COMPOSE ${dockerComposeFlags()} up ${service}`;
  btnDockerServiceStart.disabled = true;
  btnDockerServiceStop.disabled = false;
  runCommand(cmd, {
    title: `Docker/Podman service: ${service}`,
    onDone: () => {
      btnDockerServiceStart.disabled = false;
      btnDockerServiceStop.disabled = true;
    },
  });
});
btnDockerServiceStop.addEventListener("click", () => killSlot("main"));

const vncBtn = document.getElementById("btn-open-vnc");
if (vncBtn) {
  const host = window.location.hostname || "localhost";
  vncBtn.href = `http://${host}:3000/`;
}

document.getElementById("btn-docker-down").addEventListener("click", () => {
  const cmd = `${composeResolveSnippet()}cd ${dockerDir()} && $COMPOSE ${dockerComposeFlags()} down`;
  runCommand(cmd, { title: "Docker/Podman: stop + remove all containers" });
});

// ---------- micro-ROS agent: find-or-build, then launch ----------
// Blank means "follow the workflow". Defaulting to "docker" here is what made a
// fresh native install on a box without Docker announce "using docker container
// image" and then die with "ERROR: docker is not installed" -- the user had
// chosen nothing, and the selects ship with docker pre-selected, so the
// configured value never got a say. Derive it from install_mode instead.
function defaultAgentEngine() {
  const el = document.getElementById("hdr-install-mode")
    || document.getElementById("install-mode");
  const mode = (el && el.value)
    || (state.config && state.config.install_mode)
    || localStorage.getItem("linorobot2_install_mode")
    || "native";
  if (mode === "docker") return "docker";
  if (mode === "podman") return "podman";
  return "native";
}

function getAgentEngine() {
  const hdr = document.getElementById("hdr-agent-engine");
  if (hdr && hdr.value) return hdr.value;
  const cfg = document.getElementById("cfg-agent-engine");
  if (cfg && cfg.value) return cfg.value;
  return (state.config && state.config.agent_engine) || defaultAgentEngine();
}

function getContainerRegistry() {
  const hdr = document.getElementById("hdr-container-registry");
  const hdrCustom = document.getElementById("hdr-custom-registry");
  if (hdr && hdr.value === "custom" && hdrCustom && hdrCustom.value.trim()) {
    return hdrCustom.value.trim();
  }
  if (hdr && hdr.value) return hdr.value;
  const cfg = document.getElementById("cfg-container-registry");
  const cfgCustom = document.getElementById("cfg-custom-registry");
  if (cfg && cfg.value === "custom" && cfgCustom && cfgCustom.value.trim()) {
    return cfgCustom.value.trim();
  }
  if (cfg && cfg.value) return cfg.value;
  return (state.config && state.config.container_registry) || "auto";
}

function findOrBuildAgentCommand() {
  const engine = getAgentEngine();
  if (engine === "docker" || engine === "podman" || engine === "podman_systemd") {
    const bin = (engine === "docker") ? "docker" : "podman";
    const regMode = getContainerRegistry();
    const customReg = (document.getElementById("hdr-custom-registry")?.value || document.getElementById("cfg-custom-registry")?.value || (state.config && state.config.custom_registry) || "").trim();

    // Every mode probes whatever host the user configured -- nothing is
    // hardcoded. A shipped hostname would be a private address to everyone
    // except the machine it was written on, and it leaks that machine's name
    // into a public repository.
    let probeList = [];
    if (regMode === "dockerhub") {
      probeList = [];
    } else if (regMode === "custom" || regMode === "cluster" || regMode === "auto") {
      probeList = customReg ? [`"${customReg}"`] : [];
    } else {
      // An explicit host:port typed into the selector.
      probeList = [`"${regMode}"`];
    }

    let pullBlock = "";
    if (probeList.length > 0) {
      pullBlock = `REG_PULLED=0; ` +
        `for reg in ${probeList.join(" ")}; do ` +
        `  if curl -fsSL -m 2 "https://$reg/v2/" >/dev/null 2>&1 || curl -fsSL -m 2 "http://$reg/v2/" >/dev/null 2>&1; then ` +
        `    echo ">>> Container registry active at $reg. Pulling $reg/$IMG..."; ` +
        `    if ${bin} pull "$reg/$IMG" >/dev/null 2>&1; then ` +
        `      ${bin} tag "$reg/$IMG" "$IMG"; REG_PULLED=1; break; ` +
        `    fi; ` +
        `  fi; ` +
        `done; ` +
        `if [ "$REG_PULLED" -eq 0 ]; then ` +
        `  echo ">>> Pulling $IMG from upstream..."; ` +
        `  ${bin} pull "$IMG" 2>/dev/null || { echo ">>> no '$IMG' tag on Docker Hub, trying ':rolling'"; IMG="microros/micro-ros-agent:rolling"; ${bin} pull "$IMG" 2>/dev/null || true; }; ` +
        `fi; `;
    } else {
      pullBlock = `echo ">>> Pulling $IMG from Docker Hub directly..."; ` +
        `${bin} pull "$IMG" 2>/dev/null || { echo ">>> no '$IMG' tag on Docker Hub, trying ':rolling'"; IMG="microros/micro-ros-agent:rolling"; ${bin} pull "$IMG" 2>/dev/null || true; }; `;
    }

    return `echo ">>> micro-ROS agent: using ${bin} container image (skipping build from source)"; ` +
      `if ! command -v ${bin} >/dev/null 2>&1; then echo "ERROR: ${bin} is not installed" >&2; exit 1; fi; ` +
      `IMG="microros/micro-ros-agent:${state.status?.ros_distro || "jazzy"}"; ` +
      pullBlock +
      `echo AGENT_DOCKER_READY`;
  }
  return envPrefix() + [
    `[ -f ~/uros_ws/install/setup.bash ] && source ~/uros_ws/install/setup.bash`,
    `if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then echo AGENT_FOUND; ` +
      `else ` +
        `sudo apt-get install -y ros-$ROS_DISTRO-micro-ros-agent >/dev/null 2>&1; ` +
        `source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null; ` +
        `if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then echo AGENT_APT_OK; ` +
        `else ` +
          `mkdir -p ~/uros_ws/src && cd ~/uros_ws/src && ` +
          `{ [ -d micro_ros_agent ] || git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent || git clone -b rolling https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent; } && ` +
          `{ [ -d micro_ros_msgs ] || git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_msgs.git micro_ros_msgs || git clone -b rolling https://github.com/micro-ROS/micro_ros_msgs.git micro_ros_msgs; } && ` +
          `cd ~/uros_ws && colcon build && echo AGENT_BUILT; ` +
        `fi; ` +
      `fi`,
  ].join("; ");
}

function agentLaunchCommand() {
  const c = state.config || {};
  const transport = document.getElementById("cfg-agent-transport").value || c.agent_transport || "serial";
  const device = document.getElementById("cfg-agent-device").value || c.agent_device || "/dev/ttyACM0";
  const port = document.getElementById("cfg-agent-port").value || c.agent_port || "8888";
  const baud = document.getElementById("cfg-agent-baud").value || c.agent_baud || "921600";
  const engine = getAgentEngine();
  const distro = state.status?.ros_distro || "jazzy";

  const preClean = transport === "udp4" ? "" : `fuser -k -TERM ${device} 2>/dev/null || true; sleep 0.5; `;
  const devFlags = transport === "udp4" ? "" : `--device ${device}`;
  const agentArgs = transport === "udp4"
    ? `udp4 --port ${port}`
    : `serial --dev ${device} -b ${baud}`;

  if (engine === "podman_systemd") {
    const mode = transport === "udp4" ? "udp4" : "serial";
    return preClean + [
      `IMG="microros/micro-ros-agent:${distro}"`,
      `echo ">>> micro-ROS agent: Podman + systemd user service (${distro})"`,
      `podman pull "$IMG" 2>/dev/null || IMG="microros/micro-ros-agent:rolling"`,
      `podman run -d --replace --name "microros_agent_${mode}" --net=host ${devFlags} "$IMG" ${agentArgs}`,
      `mkdir -p "$HOME/.config/systemd/user"`,
      `podman generate systemd --new --name "microros_agent_${mode}" > "$HOME/.config/systemd/user/microros-agent.service" 2>/dev/null || true`,
      `systemctl --user daemon-reload 2>/dev/null || true`,
      `systemctl --user enable --now microros-agent.service 2>/dev/null || true`,
      `loginctl enable-linger "$USER" 2>/dev/null || true`,
      `echo ">>> micro-ROS agent running as persistent systemd user service: microros-agent.service"`
    ].join(" && ");
  }

  if (engine === "podman") {
    const mode = transport === "udp4" ? "udp4" : "serial";
    return `${preClean}podman run --rm --replace -it --name "uros_agent_${mode}" --net=host --privileged -v /dev:/dev ${devFlags} -e ROS_DOMAIN_ID=0 microros/micro-ros-agent:${distro} ${agentArgs}`;
  }

  if (engine === "docker") {
    const mode = transport === "udp4" ? "udp4" : "serial";
    return `${preClean}docker run --rm --net=host --privileged -v /dev:/dev ${devFlags} -e ROS_DOMAIN_ID=0 microros/micro-ros-agent:${distro} ${agentArgs}`;
  }

  // native
  const runLine = transport === "udp4"
    ? `ros2 run micro_ros_agent micro_ros_agent udp4 -p ${port}`
    : `ros2 run micro_ros_agent micro_ros_agent serial --dev ${device} -b ${baud}`;
  return envPrefix() + `[ -f ~/uros_ws/install/setup.bash ] && source ~/uros_ws/install/setup.bash; ` + preClean + runLine;
}

function _unused_old_agentLaunchCommand() {
  const c = state.config || {};
  const transport = document.getElementById("cfg-agent-transport").value || c.agent_transport || "serial";
  const device = document.getElementById("cfg-agent-device").value || c.agent_device || "/dev/ttyUSB0";
  const port = document.getElementById("cfg-agent-port").value || c.agent_port || "8888";
  const baud = document.getElementById("cfg-agent-baud").value || c.agent_baud || "921600";
  const isDocker = document.getElementById("cfg-agent-use-docker") ? document.getElementById("cfg-agent-use-docker").checked : (installModeSel?.value !== "native");

  if (isDocker) {
    const engine = (installModeSel?.value === "podman") ? "podman" : "docker";
    const devFlags = transport === "udp4" ? "" : `--device ${device}`;
    const agentArgs = transport === "udp4"
      ? `udp4 --port ${port}`
      : `serial --dev ${device} -b ${baud}`;
    return `${engine} run --rm --net=host --privileged -v /dev:/dev ${devFlags} -e ROS_DOMAIN_ID=0 microros/micro-ros-agent:${state.status?.ros_distro || "jazzy"} ${agentArgs}`;
  }
  const runLine = transport === "udp4"
    ? `ros2 run micro_ros_agent micro_ros_agent udp4 -p ${port}`
    : `ros2 run micro_ros_agent micro_ros_agent serial --dev ${device} -b ${baud}`;
  return envPrefix() + `[ -f ~/uros_ws/install/setup.bash ] && source ~/uros_ws/install/setup.bash; ` + runLine;
}

function ensureAgentRunning() {
  if (state.status && (state.status.agent_alive_external || state.status.agent_busy_console)) {
    return Promise.resolve();
  }
  return new Promise((resolve) => {
    runCommand(findOrBuildAgentCommand(), {
      title: "Preparing micro-ROS agent",
      onDone: (code) => {
        if (code !== 0) {
          resolve();
          return;
        }
        document.getElementById("btn-agent-stop").disabled = false;
        runCommand(agentLaunchCommand(), { slot: "agent", title: "micro-ROS agent" });
        setTimeout(resolve, 2500); // give the agent a moment to bind before the caller launches
      },
    });
  });
}

document.getElementById("btn-agent-ensure").addEventListener("click", () => ensureAgentRunning());
document.getElementById("btn-agent-stop").addEventListener("click", () => killSlot("agent"));

// ---------- auto-bringup & bringup launch helpers ----------
function isAutoBringupEnabled() {
  const el = document.getElementById("cfg-auto-bringup");
  return el ? el.checked : true;
}


function isAgentAlive() {
  return Boolean(state.status && (state.status.agent_alive_external || state.status.agent_busy_console));
}

// Install mode decides where things actually run. In Docker/Podman mode the
// workspace, the ROS packages and the sensor drivers all live inside the image,
// so the native pre-flight checks -- "is the workspace built", "is the LiDAR
// driver installed", "is nav2_bringup present" -- do not apply, and running
// them would try to build and apt-install on the host instead.
//
// The header select and the Install tab select mirror each other; fall back to
// the persisted config, then localStorage, then native.
function isDockerMode() {
  const el = document.getElementById("hdr-install-mode")
    || document.getElementById("install-mode");
  const mode = (el && el.value)
    || (state.config && state.config.install_mode)
    || localStorage.getItem("linorobot2_install_mode")
    || "native";
  return mode === "docker" || mode === "podman";
}

// The launch files need more than one package each, and finding out one at a
// time -- launch, read the "package 'x' not found" exception, install x, launch
// again -- is exactly what 1-Click is supposed to spare the user. Check the
// whole set and install what is missing in a single apt call.
//
// linorobot2_bringup pulls in robot_localization (ekf_node), imu_filter_madgwick
// (only when madgwick is on, but it is by default), robot_state_publisher and
// xacro. slam.launch.py pulls in slam_toolbox *and* nav2_bringup; navigation
// needs nav2_bringup. rosdep is supposed to cover these during the workspace
// build, but its failures are non-fatal there, so a missing one only surfaces
// as a launch exception much later.
const BRINGUP_PACKAGES = [
  "robot_localization",
  "imu_filter_madgwick",
  "robot_state_publisher",
  "joint_state_publisher",
  "xacro",
];
const SLAM_PACKAGES = ["slam_toolbox", "nav2_bringup"];
const NAV_PACKAGES = ["nav2_bringup"];

async function ensureRosPackages(pkgs, label) {
  if (isDockerMode()) return true;
  const distro = getDistro();
  const apt = [];      // available as ros-<distro>-<pkg>: one batched apt call
  const fromSource = []; // not published for this distro: build into the workspace
  for (const pkg of pkgs) {
    try {
      const res = await fetch(
        `/api/package/check?pkg=${encodeURIComponent(pkg)}` +
        `&distro=${encodeURIComponent(distro)}&ws=${encodeURIComponent(ws())}`
      );
      if (!res.ok) continue;
      const data = await res.json();
      if (data.installed) continue;
      if (data.source === "source" && data.install_cmd) fromSource.push(data);
      else apt.push(data.apt_package || `ros-${distro}-${pkg.replace(/_/g, "-")}`);
    } catch (err) {
      console.warn(`package check failed for ${pkg}:`, err);
    }
  }
  if (apt.length === 0 && fromSource.length === 0) return true;

  openTerminal(`Installing ${label} prerequisites`);
  logLine("[console] -------------------------------------------------------------");
  logLine(`[console] [1-Click] ${label} is missing: ${[...apt, ...fromSource.map((d) => d.package)].join(", ")}`);
  logLine("[console] -------------------------------------------------------------");

  let ok = true;
  if (apt.length) {
    ok = await new Promise((resolve) => {
      runCommand(`sudo apt-get update && sudo apt-get install -y ${apt.join(" ")}`, {
        title: `Install ${label} prerequisites`,
        action: `install ${label} prerequisites`,
        onDone: (exitCode) => resolve(exitCode === 0),
      });
    });
    if (!ok) logLine(`[console] ⚠ apt install of ${label} prerequisites exited with an error.`);
  }
  for (const d of fromSource) {
    logLine(`[console] '${d.package}' has no binary package on ${distro} -- building it from source.`);
    const built = await new Promise((resolve) => {
      runCommand(d.install_cmd, {
        title: `Build ${d.package} from source`,
        action: `build ${d.package}`,
        onDone: (exitCode) => resolve(exitCode === 0),
      });
    });
    if (!built) {
      ok = false;
      logLine(`[console] ⚠ Source build of '${d.package}' failed.`);
    }
  }
  // Trusting exit codes is not enough: apt can exit non-zero for one package
  // while the rest installed, and a source build can exit 0 having built
  // nothing ("Summary: 0 packages finished"). Ask again what is actually on
  // disk, and report the packages by name.
  const stillMissing = [];
  for (const pkg of pkgs) {
    try {
      const res = await fetch(
        `/api/package/check?pkg=${encodeURIComponent(pkg)}` +
        `&distro=${encodeURIComponent(distro)}&ws=${encodeURIComponent(ws())}`
      );
      if (!res.ok) continue;
      const data = await res.json();
      if (!data.installed) stillMissing.push(pkg);
    } catch (err) {
      console.warn(`re-check failed for ${pkg}:`, err);
    }
  }
  if (stillMissing.length) {
    logLine(`[console] ✖ ${label} prerequisites are still missing: ${stillMissing.join(", ")}`);
    return false;
  }
  return true;
}

// nav2_bringup only launches the rest of the stack, so having it says nothing
// about whether the nodes it starts exist. On Lyrical they are published one by
// one with no ros-lyrical-navigation2 metapackage to pull them in, and each
// missing one costs another failed launch ("package 'nav2_waypoint_follower'
// not found", then the next). Install the whole stack in one go instead.
async function ensureNav2Stack(label) {
  if (isDockerMode()) return true;
  let data = null;
  try {
    const res = await fetch(
      `/api/nav2/stack?distro=${encodeURIComponent(getDistro())}&ws=${encodeURIComponent(ws())}`
    );
    if (res.ok) data = await res.json();
  } catch (err) {
    console.warn("nav2 stack check failed:", err);
  }
  if (!data || data.installed || !data.command) return true;

  openTerminal(`Installing the nav2 stack for ${label}`);
  logLine("[console] -------------------------------------------------------------");
  logLine(`[console] [1-Click] nav2 runtime packages missing: ${data.missing.join(", ")}`);
  logLine("[console] Installing the published nav2 packages for this distro...");
  logLine("[console] -------------------------------------------------------------");
  const ok = await new Promise((resolve) => {
    runCommand(data.command, {
      title: "Install nav2 stack",
      action: "install nav2 stack",
      onDone: (exitCode) => resolve(exitCode === 0),
    });
  });
  // Not fatal on its own and not a decision point: this is a bulk apt install
  // of whatever the distro publishes, and what matters is which packages end
  // up on disk, which ensureRosPackages re-checks by name afterwards.
  if (!ok) logLine("[console] ⚠ nav2 stack install exited with an error -- checking what actually installed...");
  return ok;
}

async function ensureNav2Prerequisites(targetTitle = "Navigation") {
  if (isDockerMode()) return true;
  const isSlam = (targetTitle || "").toLowerCase().includes("slam");
  const label = isSlam ? "SLAM" : "Nav2";
  await ensureNav2Stack(label);
  const ok = await ensureRosPackages(isSlam ? SLAM_PACKAGES : NAV_PACKAGES, label);
  if (!ok) {
    // Launching regardless produced a stack with no mapper in it and no error
    // to explain it: on Rolling/Ubuntu 26.04 the ROS index carries 2205
    // packages of which exactly three are nav2-* (all TurtleBot sim assets),
    // and neither slam_toolbox nor nav2_bringup exists at all. Say so and stop
    // rather than bringing up something that cannot work.
    logLine(`[console] ✖ Cannot start ${label}: the packages above are not published`);
    logLine(`[console]   for ${getDistro()} on this Ubuntu, and could not be built from source.`);
    logLine("[console]   This is a ROS index gap, not a fault in your setup --");
    logLine("[console]   a distro whose nav2/slam_toolbox packages are released will work.");
    throw new Error(`${label} prerequisites unavailable for ${getDistro()}`);
  }
  return ok;
}

function isBringupAlive() {
  return Boolean(state.status && (state.status.bringup_alive_external || state.status.bringup_busy_console));
}

function bringupLaunchCommand() {
  if (isDockerMode()) {
    return `${composeResolveSnippet()}cd ${dockerDir()} && $COMPOSE ${dockerComposeFlags()} up bringup`;
  }
  const launcher = `${state.status?.web_dir || "."}/../launch_bringup.py`;
  const cfgPath = (state.robot_config && state.robot_config.path) || "~/.config/linorobot2/robot_config.yaml";
  const base = document.getElementById("bringup-base-type")?.value || (state.config && state.config.base_type) || "2wd";
  const dev = document.getElementById("bringup-agent-device")?.value || (state.config && state.config.agent_device) || "/dev/ttyACM0";
  const baud = document.getElementById("bringup-agent-baud")?.value || (state.config && state.config.agent_baud) || "1500000";
  const madgwick = document.getElementById("bringup-madgwick-toggle")?.checked ? "true" : "false";

  // Bringup would otherwise start its own native micro_ros_agent. Skip that
  // when an agent is already up, or when the agent engine is a container --
  // in container mode the package is usually not installed natively at all, so
  // including it fails the whole launch while the real agent runs happily.
  const nativeAgent = getAgentEngine() === "native";
  const microRos = (nativeAgent && !isAgentAlive()) ? "true" : "false";

  return envPrefix() +
    `ros2 launch ${launcher} config_file:=${cfgPath} base:=${base} base_serial_port:=${dev} micro_ros_baudrate:=${baud} madgwick:=${madgwick} micro_ros:=${microRos}`;
}

async function ensureBringupRunning(targetTitle = "requested action") {
  if (!isAutoBringupEnabled()) return;
  await refreshStatus();

  // 1-Click Intermediate Step 0: ROS 2 itself
  if (!isDockerMode() && state.status && state.status.ros2_installed === false) {
    logLine(`[console] [1-Click] Step 0: ROS 2 ${getDistro()} not installed -- installing it first...`);
    const rosOk = await ensureRos2Installed();
    if (!rosOk) {
      logLine("[console] \u2716 [1-Click] ROS 2 install failed. Aborting " + targetTitle + ".");
      throw new Error("ROS 2 install failed");
    }
  }

  // 1-Click Intermediate Step 1: Ensure micro-ROS agent is active
  if (!isAgentAlive()) {
    logLine("[console] [1-Click] Step 1: micro-ROS Agent is down -- auto-starting agent...");
    await ensureAgentRunning();
  }

  // 1-Click Intermediate Step 2: Auto-build base workspace if not built yet
  if (!isDockerMode() && state.status && state.status.workspace_built === false) {
    logLine("[console] [1-Click] Step 2: Workspace not built -- auto-building linorobot2 base...");
    const buildOk = await checkAndBuildWorkspace();
    if (!buildOk) {
      logLine("[console] ✖ [1-Click] Workspace build failed. Aborting " + targetTitle + ".");
      throw new Error("Workspace build failed");
    }
  }

  // 1-Click Intermediate Step 3: Auto-detect and auto-install missing LiDAR driver
  const laser = document.getElementById("bringup-laser-sensor")?.value || (state.config && state.config.laser_sensor);
  if (laser && !isDockerMode()) {
    logLine(`[console] [1-Click] Step 3: Checking LiDAR driver for '${laser}'...`);
    const driverOk = await checkAndInstallLidarDriver(laser);
    if (!driverOk) {
      logLine(`[console] ⚠ LiDAR driver setup failed for '${laser}'. Continuing bringup...`);
    }
  }

  // 1-Click Intermediate Step 3b: packages linorobot2_bringup itself launches
  if (!isDockerMode()) {
    await ensureRosPackages(BRINGUP_PACKAGES, "Bringup");
  }

  // 1-Click Intermediate Step 3c: the LiDAR driver itself.
  // bringup does not start it -- it is a separate node in its own slot -- so
  // after a 1-Click SLAM the whole stack came up healthy with no /scan at all,
  // and slam_toolbox sat there producing nothing with no error to explain it.
  if (laser && !isDockerMode() && !isLaserRunning()) {
    logLine("[console] [1-Click] Step 3c: starting the LiDAR driver...");
    await startLaserDriver();
  }

  // 1-Click Intermediate Step 4: Check Nav2 / SLAM packages if launching Nav2/SLAM
  const lowerTitle = (targetTitle || "").toLowerCase();
  if (lowerTitle.includes("nav") || lowerTitle.includes("slam")) {
    await ensureNav2Prerequisites(targetTitle);
  }

  // If Bringup is already running, we are ready -- but only if it is actually
  // publishing. A live process is not a live robot: an orphaned `ros2 launch`
  // wrapper whose children have died still matches BRINGUP_PROC_PATTERN, so
  // bringup_alive_external stays true while nothing publishes. Console then
  // skipped bringup and started SLAM with no odometry and no odom->base_link
  // TF, and nav2 failed 60s later with
  //   "Failed to activate local_costmap because transform from base_link to
  //    odom did not become available before timeout"
  // which says nothing about bringup at all. server.py already probes the
  // graph for exactly this (/api/bringup/health, ready = odom + TF chain), so
  // ask it before trusting the process.
  if (isBringupAlive()) {
    let health = null;
    try {
      health = await fetch("/api/bringup/health?timeout=4").then((r) => r.json());
    } catch (e) {
      health = null;
    }
    if (health && health.ready) {
      logLine("[console] ✓ Robot Bringup is already active. Ready for " + targetTitle + ".");
      return;
    }
    const why = health && health.summary ? health.summary : "health probe failed";
    logLine(`[console] ⚠ A bringup process is running but the robot is not publishing: ${why}`);
    logLine("[console] Restarting bringup rather than launching " + targetTitle + " blind...");
    try {
      await killSlot("bringup");
      await new Promise((r) => setTimeout(r, 3000));
      await refreshStatus();
    } catch (e) {
      logLine(`[console] ⚠ Could not stop the stale bringup: ${e}`);
    }
  }

  // 1-Click Intermediate Step 5: Start Bringup in background and stream logs
  logLine(`[console] [1-Click] Step 5: Automatically launching Robot Bringup in background for ${targetTitle}...`);
  openTerminal("Robot Bringup (1-Click Auto-Started) [streaming]");
  attachBringupStream();

  const cmd = bringupLaunchCommand();
  return new Promise((resolve) => {
    const startBtn = document.getElementById("btn-bringup-start");
    const stopBtn = document.getElementById("btn-bringup-stop");
    if (startBtn) startBtn.disabled = true;
    if (stopBtn) stopBtn.disabled = false;

    runCommand(cmd, {
      slot: "bringup",
      title: `Robot Bringup (${targetTitle})`,
      onDone: (exitCode) => {
        if (startBtn) startBtn.disabled = false;
        if (stopBtn) stopBtn.disabled = true;
        if (state.status) state.status.bringup_busy_console = false;
        refreshStatus();
        if (exitCode !== 0) {
          openTerminal("Robot Bringup [Exited with error]");
          logLine(`[console] ✖ Bringup process exited with error code ${exitCode}. Check output above.`);
        }
      },
    });

    if (state.status) state.status.bringup_busy_console = true;
    const pill = document.getElementById("hdr-bringup-pill");
    if (pill) {
      pill.textContent = "starting...";
      pill.className = "pill pill-starting";
    }

    const startedAt = Date.now();
    const DEADLINE_MS = 40000;
    const poll = async () => {
      let alive = false;
      try {
        const s = await fetch("/api/status").then((r) => r.json());
        state.status = s;
        alive = Boolean(s.bringup_alive_external || s.bringup_busy_console);
      } catch (e) {
        /* keep polling */
      }
      const waited = Date.now() - startedAt;
      if (alive || waited >= DEADLINE_MS) {
        refreshStatus();
        logLine(
          alive
            ? `[console] ✓ Robot Bringup confirmed active after ${(waited / 1000).toFixed(1)}s. Launching ${targetTitle}...`
            : `[console] ⚠ Bringup not confirmed after ${(DEADLINE_MS / 1000)}s -- proceeding with ${targetTitle} anyway. Check logs above.`
        );
        // Settle delay so nodes/agent/TF tree bind before caller launches
        setTimeout(resolve, 2500);
        return;
      }
      setTimeout(poll, 1000);
    };
    setTimeout(poll, 1200);
  });
}


// ---------- ROS 2 auto-install ----------
// Everything else in the 1-Click chain assumes /opt/ros/<distro> exists. On a
// fresh machine it does not, and the chain went straight to `colcon build`,
// which failed with "colcon: command not found" -- a message that says nothing
// about the real problem. Install ROS 2 first, then continue.
async function ensureRos2Installed() {
  if (isDockerMode()) return true;
  await refreshStatus();
  if (state.status && state.status.ros2_installed) return true;

  const distro = getDistro();
  let cmd = null;
  try {
    const res = await fetch(`/api/ros2/install_cmd?distro=${encodeURIComponent(distro)}`);
    if (res.ok) {
      const data = await res.json();
      if (data.installed) return true;
      cmd = data.command;
    }
  } catch (err) {
    console.warn("ROS 2 install command lookup failed:", err);
  }
  if (!cmd) {
    logLine(`[console] ✖ Could not work out how to install ROS 2 ${distro}.`);
    return false;
  }

  openTerminal(`Installing ROS 2 ${distro}`);
  logLine("[console] -------------------------------------------------------------");
  logLine(`[console] ⚠ ROS 2 ${distro} is not installed (/opt/ros/${distro}/setup.bash missing).`);
  logLine("[console] Installing it now -- this takes several minutes on a fresh machine.");
  logLine("[console] -------------------------------------------------------------");
  const ok = await new Promise((resolve) => {
    runCommand(cmd, {
      title: `Install ROS 2 ${distro}`,
      action: `install ROS 2 ${distro}`,
      onDone: (exitCode) => resolve(exitCode === 0),
    });
  });
  await refreshStatus();
  if (!ok) logLine(`[console] ✖ ROS 2 ${distro} install failed. See the output above.`);
  return ok;
}

// ---------- workspace auto-build & lidar driver auto-install ----------
async function checkAndBuildWorkspace() {
  if (isDockerMode()) return true;
  await refreshStatus();
  if (state.status && state.status.workspace_built) {
    return true;
  }
  // colcon and the compiler come from the ROS 2 install, so a bare machine has
  // to get that first. "Run base install" used to go straight to the clone and
  // then die on "colcon: command not found" (exit 127) -- a message that names
  // the missing tool but not the missing distro, so it reads like a broken box.
  // The 1-Click chain already guards this way at its Step 0; the Install tab
  // button reached the same command without it.
  if (state.status && state.status.ros2_installed === false) {
    logLine(`[console] Base install needs ROS 2 ${getDistro()} for colcon -- installing it first...`);
    if (!(await ensureRos2Installed())) {
      logLine("[console] ✖ Cannot run base install: ROS 2 is not installed.");
      return false;
    }
  }

  const workspace = ws();
  openTerminal("Base Install & Workspace Build");
  logLine("[console] -------------------------------------------------------------");
  logLine(`[console] ⚠ Native workspace is not built yet (${workspace}/install/setup.bash missing).`);
  logLine("[console] linorobot2_bringup and base packages must be built before bringup can run.");
  logLine("[console] Automatically running Base Install & colcon build now...");
  logLine("[console] -------------------------------------------------------------");

  let cmd = null;
  try {
    const res = await fetch(`/api/workspace/build_cmd?ws=${encodeURIComponent(workspace)}&distro=${encodeURIComponent(getDistro())}`);
    if (res.ok) {
      const data = await res.json();
      cmd = data.command;
    }
  } catch (_) {}

  if (!cmd) {
    cmd = [
      `mkdir -p ${workspace}/src`,
      `cd ${workspace}/src`,
      gitCloneDistroSnippet("https://github.com/linorobot/linorobot2", "linorobot2"),
      `touch linorobot2/linorobot2_gazebo/COLCON_IGNORE 2>/dev/null || true`,
      `cd ${workspace}`,
      `rosdep update 2>/dev/null || true`,
      `rosdep install --from-paths src --ignore-src -y --skip-keys microxrcedds_agent 2>/dev/null || true`,
      `colcon build --symlink-install`,
    ].join(" && ");
  }

  const success = await new Promise((resolve) => {
    runCommand(envPrefix() + cmd, {
      title: "Base Install & colcon build",
      action: "base install",
      onDone: (exitCode) => {
        if (exitCode === 0) {
          logLine("[console] ✓ Base workspace installed and built successfully!");
          resolve(true);
        } else {
          logLine(`[console] ✖ Base install failed with exit code ${exitCode}. Check output above.`);
          resolve(false);
        }
      }
    });
  });

  if (success) {
    await refreshStatus();
  }
  return success;
}

// ---------- bringup log streaming & lidar driver auto-install ----------
let bringupEventSource = null;

function attachBringupStream() {
  if (bringupEventSource) return;
  openTerminal("Robot Bringup [streaming]");
  try {
    bringupEventSource = new EventSource("/api/bringup/stream");
    bringupEventSource.addEventListener("init", (e) => {
      try {
        const data = JSON.parse(e.data);
        if (data.status === "running") {
          openTerminal(`Robot Bringup [${data.source || 'running'}]`);
        }
      } catch (_) {}
    });
    bringupEventSource.addEventListener("output", (e) => {
      try {
        const data = JSON.parse(e.data);
        if (data.line != null) {
          logLine(`[bringup] ${data.line}`);
        }
      } catch (_) {}
    });
    bringupEventSource.addEventListener("done", (e) => {
      try {
        const data = JSON.parse(e.data);
        logLine(`[bringup] exited with code ${data.exit_code}`);
      } catch (_) {}
      if (bringupEventSource) {
        bringupEventSource.close();
        bringupEventSource = null;
      }
      refreshStatus();
    });
    bringupEventSource.addEventListener("idle", () => {
      if (bringupEventSource) {
        bringupEventSource.close();
        bringupEventSource = null;
      }
    });
    bringupEventSource.onerror = () => {
      if (bringupEventSource) {
        bringupEventSource.close();
        bringupEventSource = null;
      }
    };
  } catch (err) {
    console.error("Error attaching bringup stream:", err);
  }
}

async function checkAndInstallLidarDriver(laserSensor) {
  if (!laserSensor || isDockerMode()) return true;
  try {
    const res = await fetch(`/api/sensors/driver_status?sensor=${encodeURIComponent(laserSensor)}&ws=${encodeURIComponent(ws())}`);
    if (!res.ok) return true;
    const info = await res.json();
    if (!info.installed && info.package && info.install_cmd) {
      openTerminal(`Installing LiDAR Driver (${info.package})`);
      logLine(`[console] -------------------------------------------------------------`);
      logLine(`[console] LiDAR '${laserSensor}' requires ROS 2 package '${info.package}'.`);
      logLine(`[console] Driver package was not found in ROS 2 or workspace ${ws()}.`);
      logLine(`[console] Automatically installing and building driver before bringup...`);
      logLine(`[console] -------------------------------------------------------------`);
      
      const success = await new Promise((resolve) => {
        runCommand(envPrefix() + `cd ${ws()} && ` + info.install_cmd, {
          title: `Install LiDAR Driver: ${info.package}`,
          action: `install driver ${info.package}`,
          onDone: (exitCode) => {
            if (exitCode === 0) {
              logLine(`[console] ✓ LiDAR driver '${info.package}' installed successfully!`);
              resolve(true);
            } else {
              logLine(`[console] ⚠ LiDAR driver install returned exit code ${exitCode}.`);
              resolve(false);
            }
          }
        });
      });
      return success;
    }
  } catch (e) {
    console.warn("Driver pre-flight check failed:", e);
  }
  return true;
}

// ---------- bringup ----------
const btnBringupLogs = document.getElementById("btn-bringup-logs");
if (btnBringupLogs) {
  btnBringupLogs.addEventListener("click", () => attachBringupStream());
}

wireStartStop({
  startBtn: document.getElementById("btn-bringup-start"),
  stopBtn: document.getElementById("btn-bringup-stop"),
  slot: "bringup",
  title: "Bringup",
  buildCommand: async () => {
    if (!isDockerMode() && state.status && state.status.ros2_installed === false) {
      const rosOk = await ensureRos2Installed();
      if (!rosOk) {
        logLine("[console] ✖ Cannot start Bringup: ROS 2 is not installed.");
        throw new Error("ROS 2 install failed");
      }
    }
    if (!isDockerMode() && state.status && state.status.workspace_built === false) {
      const buildOk = await checkAndBuildWorkspace();
      if (!buildOk) {
        logLine("[console] ✖ Cannot start Bringup: workspace build did not succeed.");
        throw new Error("Workspace build failed");
      }
    }
    const laser = document.getElementById("bringup-laser-sensor")?.value || (state.config && state.config.laser_sensor);
    if (laser && !isDockerMode()) {
      await checkAndInstallLidarDriver(laser);
    }
    openTerminal("Robot Bringup [streaming]");
    attachBringupStream();
    return bringupLaunchCommand();
  },
});

// ---------- bringup health (topic + TF level, not just pgrep) ----------
// `bringup_alive_external` in /api/status only says a process exists. This asks
// the ROS graph whether odometry, the IMU, the LiDAR and the TF chain are
// actually live -- the thing SLAM/Nav2 will silently fail on otherwise.
const btnBringupHealth = document.getElementById("btn-bringup-health");
if (btnBringupHealth) {
  btnBringupHealth.addEventListener("click", async () => {
    const summaryEl = document.getElementById("bringup-health-summary");
    const tableEl = document.getElementById("bringup-health-table");
    btnBringupHealth.disabled = true;
    summaryEl.textContent = "Probing the ROS graph (up to ~30 s)…";
    tableEl.innerHTML = "";
    try {
      const h = await fetch("/api/bringup/health?timeout=4").then((r) => r.json());
      const mark = (ok) => (ok ? "🟢" : "🔴");
      const rows = Object.values(h.topics || {}).map((t) => `
        <tr>
          <td>${mark(t.ok)}</td>
          <td><code>${escapeHtml(t.topic)}</code></td>
          <td>${escapeHtml(t.what)}</td>
          <td>${t.hz == null ? (t.advertised ? "no messages" : "not advertised")
                             : t.hz.toFixed(1) + " Hz"}</td>
          <td>&ge; ${t.min_hz} Hz</td>
        </tr>`).join("");
      const tfRows = (h.tf || []).map((l) => `
        <tr>
          <td>${mark(l.ok)}</td>
          <td colspan="2"><code>TF ${escapeHtml(l.parent)} &rarr; ${escapeHtml(l.child)}</code></td>
          <td colspan="2">${escapeHtml(l.ok ? "transform resolves" : l.detail)}</td>
        </tr>`).join("");
      tableEl.innerHTML =
        `<table class="health-table"><tbody>${rows}${tfRows}</tbody></table>`;
      summaryEl.innerHTML =
        `<span style="color: var(--${h.ready ? "accent-ok" : "accent-danger"});">` +
        `${h.ready ? "✓" : "✗"} ${escapeHtml(h.summary)}</span>`;
      logLine(`[console] bringup health: ${h.summary}`);
    } catch (e) {
      summaryEl.textContent = `health check failed: ${e}`;
    } finally {
      btnBringupHealth.disabled = false;
    }
  });
}

// ---------- teleop ----------
wireStartStop({
  startBtn: document.getElementById("btn-teleop-start"),
  stopBtn: document.getElementById("btn-teleop-stop"),
  slot: "main",
  title: "Gamepad teleop",
  needsBringup: true,
  buildCommand: async () => {
    const axisLinear = document.getElementById("joy-axis-linear").value || 1;
    const scaleLinear = document.getElementById("joy-scale-linear").value || 0.5;
    const axisAngular = document.getElementById("joy-axis-angular").value || 0;
    const scaleAngular = document.getElementById("joy-scale-angular").value || 1.0;
    const yamlBody =
      `teleop_twist_joy_node:\n  ros__parameters:\n` +
      `    axis_linear:\n      x: ${axisLinear}\n` +
      `    scale_linear:\n      x: ${scaleLinear}\n` +
      `    axis_angular:\n      yaw: ${axisAngular}\n` +
      `    scale_angular:\n      yaw: ${scaleAngular}\n`;
    const tmpFile = "/tmp/linorobot2_console_joy.yaml";
    // The heredoc terminator must be alone on its own line -- appending
    // " && (...)" right after it on the same line means bash never
    // recognizes it as the terminator and keeps consuming everything after
    // it (including the ros2 run commands) as more heredoc body instead of
    // running them. A real newline before the next statement fixes it.
    const writeYaml = `cat > ${tmpFile} << 'CONSOLE_JOY_EOF'\n${yamlBody}CONSOLE_JOY_EOF`;
    return envPrefix() + writeYaml + "\n" +
      `(ros2 run joy_linux joy_linux_node & ` +
      `ros2 run teleop_twist_joy teleop_node --ros-args --params-file ${tmpFile}; wait)`;
  },
});

// ---------- SLAM / navigation ----------
wireStartStop({
  startBtn: document.getElementById("btn-slam-start"),
  stopBtn: document.getElementById("btn-slam-stop"),
  slot: "main",
  title: "SLAM",
  needsBringup: true,
  // slam.launch.py brings nav2 up alongside slam_toolbox, using
  // linorobot2_navigation's own navigation.yaml. That file is a Jazzy-era
  // layout, so on Lyrical/Rolling controller_server failed to configure
  // ("Failed to get 'primary_controller.plugin' parameter") and
  // lifecycle_manager aborted the whole bringup -- taking SLAM with it. It
  // also meant every parameter tuned in Console was quietly ignored, which is
  // the opposite of what this tab claims. Go through Console's own launcher
  // with slam:=true, exactly as the Navigation button does.
  buildCommand: async () => {
    const launcher = `${state.status?.web_dir || "."}/../launch_nav2.py`;
    const distro = getDistro();
    // Blank means "auto-resolve": launch_nav2.py then tries
    // web/console_nav2_<distro>.yaml, config/nav2_<distro>_<base>.yaml and
    // config/nav2_<distro>.yaml in turn. Naming console_nav2_<distro>.yaml
    // explicitly defeats that -- only the jazzy one is shipped, so on every
    // other distro nav2 was handed a path that does not exist.
    const customParams = document.getElementById("nav-params-file")?.value.trim() || "";
    const paramsArg = customParams ? ` params_file:=${customParams}` : "";
    const depthArg = ` depth_costmap:=${document.getElementById("bringup-depth-sensor")?.value ? "true" : "false"}`;
    return envPrefix() +
      `if [ -f ${launcher} ]; then ` +
      `ros2 launch ${launcher} slam:=true${paramsArg}${depthArg} distro:=${distro} sim:=false; ` +
      `else ros2 launch linorobot2_navigation slam.launch.py; fi`;
  },
});

document.getElementById("btn-map-save").addEventListener("click", () => {
  const name = document.getElementById("map-save-name").value.trim();
  if (!name) return;
  const mapsDir = `${ws()}/src/linorobot2/linorobot2_navigation/maps`;
  // Saving needs SLAM still running -- it is what publishes /map -- so this
  // cannot take the slot SLAM is holding. On "main" it was refused with a 409
  // every time, i.e. the map could never be saved from the UI at all.
  runCommand(
    envPrefix() + `mkdir -p ${mapsDir} && ros2 run nav2_map_server map_saver_cli -f ${mapsDir}/${name}`,
    { slot: "tool", title: `Save map: ${name}`, onDone: refreshMaps }
  );
});

function refreshMaps() {
  fetch("/api/maps").then((r) => r.json()).then((data) => {
    const sel = document.getElementById("nav-map-select");
    sel.innerHTML = "";
    data.maps.forEach((m) => {
      const opt = document.createElement("option");
      opt.value = `${data.maps_dir}/${m}.yaml`;
      opt.textContent = m;
      sel.appendChild(opt);
    });
  });
}
document.getElementById("btn-refresh-maps").addEventListener("click", refreshMaps);
refreshMaps();

wireStartStop({
  startBtn: document.getElementById("btn-nav-start"),
  stopBtn: document.getElementById("btn-nav-stop"),
  slot: "main",
  title: "Navigation",
  needsBringup: true,
  buildCommand: async () => {
    const mapPath = document.getElementById("nav-map-select").value;
    const mapArg = mapPath ? ` map:=${mapPath}` : "";
    const customParams = document.getElementById("nav-params-file").value.trim();
    const launcher = `${state.status?.web_dir || "."}/../launch_nav2.py`;
    const distro = getDistro();
    // Same as SLAM: leave it blank and let launch_nav2.py resolve. Only
    // console_nav2_jazzy.yaml is shipped in web/, so naming
    // console_nav2_<distro>.yaml pointed Lyrical and Rolling at a file that
    // is not there.
    const defaultParams = `${state.status?.web_dir || "."}/console_nav2_${distro}.yaml`;
    const paramsArg = customParams ? ` params_file:=${customParams}` : "";

    // Depth camera -> costmap: the console's launch_nav2.py resolves this and
    // hands nav2 a finished params file (it never mutates the editable YAML and
    // never touches upstream navigation.launch.py). Pass an explicit true/false
    // from the Bringup depth-sensor selection -- only the console launcher
    // understands depth_costmap, so it's omitted from the plain-launch fallback.
    const depthArg = ` depth_costmap:=${document.getElementById("bringup-depth-sensor")?.value ? "true" : "false"}`;
    return envPrefix() +
      `if [ -f ${launcher} ]; then ` +
      `ros2 launch ${launcher}${mapArg}${paramsArg}${depthArg} distro:=${distro} sim:=false; ` +
      // Fallback path: plain nav2_bringup has no auto-resolution of its own,
      // so it does need an explicit file -- and the branch is only taken when
      // that file exists.
      `elif [ -f ${customParams || defaultParams} ]; then ` +
      `ros2 launch nav2_bringup bringup_launch.py${mapArg} params_file:=${customParams || defaultParams} use_sim_time:=false; ` +
      `else ` +
      `ros2 launch linorobot2_navigation navigation.launch.py${mapArg}; ` +
      `fi`;
  },
});

// ---------- RViz via noVNC (headless-friendly) ----------
// Native equivalent of the `kasmvnc` service in linorobot2's own
// docker/docker-compose.yaml: Xvfb gives rviz2 a virtual display to open,
// x11vnc exposes that display over VNC, and websockify (from the `novnc`
// package) fronts it as a plain browser page -- no native VNC client needed,
// and it works the same whether the browser is on the robot computer itself
// or a separate laptop.
const btnVncStart = document.getElementById("btn-vnc-start");
const btnVncStop = document.getElementById("btn-vnc-stop");
// RViz configs shipped in the repo, plus the console's own teleop view. Paths
// are relative to web_dir the same way launch_bringup.py / launch_nav2.py are.
// An empty path means plain `rviz2` with nothing loaded.
const RVIZ_CONFIGS = {
  teleop: "/../rviz/teleop.rviz",
  slam: "/../../../linorobot2_navigation/rviz/linorobot2_slam.rviz",
  navigation: "/../../../linorobot2_navigation/rviz/linorobot2_navigation.rviz",
  description: "/../../../linorobot2_description/rviz/description.rviz",
};

btnVncStart.addEventListener("click", () => {
  const display = document.getElementById("vnc-display").value.trim() || ":99";
  const novncPort = document.getElementById("vnc-novnc-port").value.trim() || "6080";
  const which = document.getElementById("vnc-rviz-config").value;
  const rel = RVIZ_CONFIGS[which];
  // fall back to a bare rviz2 if the config is missing rather than failing to start
  const rvizCmd = rel
    ? `[ -f "${state.status?.web_dir || "."}${rel}" ] && rviz2 -d "${state.status?.web_dir || "."}${rel}" || rviz2`
    : "rviz2";
  const cmd = envPrefix() + [
    "command -v Xvfb >/dev/null 2>&1 || sudo apt-get install -y xvfb",
    "command -v x11vnc >/dev/null 2>&1 || sudo apt-get install -y x11vnc",
    "command -v websockify >/dev/null 2>&1 || sudo apt-get install -y novnc websockify",
    `pkill -f "Xvfb ${display}" 2>/dev/null; sleep 0.3`,
    `(Xvfb ${display} -screen 0 1280x800x24 &) && sleep 1`,
    `(DISPLAY=${display} ${rvizCmd} &) && sleep 1`,
    `(x11vnc -display ${display} -forever -shared -nopw -quiet -rfbport 5900 &) && sleep 1`,
    `websockify --web=/usr/share/novnc ${novncPort} localhost:5900`,
  ].join(" && ");
  btnVncStart.disabled = true;
  btnVncStop.disabled = false;
  const link = document.getElementById("vnc-link");
  link.href = `http://${location.hostname}:${novncPort}/vnc.html`;
  link.style.display = "inline";
  runCommand(cmd, {
    title: "RViz via noVNC",
    onDone: () => {
      btnVncStart.disabled = false;
      btnVncStop.disabled = true;
      link.style.display = "none";
    },
  });
});
btnVncStop.addEventListener("click", () => killSlot("main"));

// ---------- magnetometer calibration ----------
// Needs Bringup already running elsewhere (cmd_vel to spin the base, IMU/mag
// topics to read) -- a bare standalone agent wouldn't be enough, same
// reasoning as teleop/SLAM/navigation above, so this doesn't call
// ensureAgentRunning() either.
document.getElementById("btn-mag-cal").addEventListener("click", async () => {
  if (!confirm("The robot will spin in place for about a minute. Clear the area, then continue?")) return;
  if (isAutoBringupEnabled()) {
    await ensureBringupRunning();
  }
  const resultEl = document.getElementById("mag-cal-result");
  resultEl.textContent = "";
  const cmd = envPrefix() +
    `dpkg -s ros-$ROS_DISTRO-robot-calibration >/dev/null 2>&1 || sudo apt-get install -y ros-$ROS_DISTRO-robot-calibration; ` +
    `ros2 run robot_calibration magnetometer_calibration`;
  runCommand(cmd, {
    title: "Magnetometer calibration",
    onLine: (line) => {
      if (/mag_bias|bias_x|bias_y|bias_z/i.test(line)) {
        resultEl.textContent += line + "\n";
      }
    },
  });
});

// ---------- laser driver (standalone, independent of Bringup/agent) ----------
// Model list + persistent-symlink + baud all come from the sensor registry.
// Two families, per linorobot2_bringup/launch/lasers.launch.py:
// - registry model has NO `product` field  -> delegate to lasers.launch.py
//   `sensor:=<code>`, passing lidar_serial_port:= / lidar_transport:= (those
//   ARE declared launch args). ydlidar/rplidar/xv11.
// - registry model HAS `product`/`bins`/`baud` (ld06/ld19/stl27l) -> run the
//   ldlidar_stl_ros2 node directly with `--ros-args -p`, which also unlocks
//   its UDP-bridge / native-network modes.
const laserModelSel = document.getElementById("laser-driver-model");

function laserModelMeta(code) {
  const hit = laserEntryForModel(code);
  if (!hit) return null;
  const [key, entry] = hit;
  const m = (entry.models || []).find((x) => x.code === code) || {};
  return {
    key, entry, code,
    isLd: Boolean(m.product),
    product: m.product,
    bins: m.bins,
    baud: m.baud || entry.default_baud || "",
    symlink: entry.symlink,
  };
}

// The MCU and the LiDAR sit on two different tty devices, and the MCU's is
// already known -- so the LiDAR's can be guessed instead of left blank:
//
//   agent on /dev/ttyUSB0  -> LiDAR on /dev/ttyUSB1   (both USB-serial bridges;
//                                                      the agent took the first)
//   agent on /dev/ttyACM0  -> LiDAR on /dev/ttyUSB0   (the MCU is native USB CDC,
//                                                      so no ttyUSB is taken yet)
//
// Only when the agent actually holds a serial port: over WiFi it holds none, and
// pairing off a stale agent_device would push the LiDAR one slot too far. This is
// a starting value, not an identity -- ttyUSBn is assignment order and flips on
// replug, so a rig that cares should save a /dev/serial/by-id path instead.
function laserPortPairedWithAgent(c) {
  if ((c.agent_transport || "serial") !== "serial") return "";
  const dev = c.agent_device || "";
  const usb = dev.match(/^\/dev\/ttyUSB(\d+)$/);
  if (usb) return `/dev/ttyUSB${Number(usb[1]) + 1}`;
  if (/^\/dev\/ttyACM\d+$/.test(dev)) return "/dev/ttyUSB0";
  return "";
}

function laserDefaultPort(meta) {
  const c = state.config || {};
  return c.laser_serial_port || laserPortPairedWithAgent(c) || meta.symlink || "";
}

// The laser panel opened on whichever option happened to be first in the list
// (ydlidar) no matter what the robot's saved laser_sensor said, and its port
// and baud were filled in before the config had loaded. "Start laser driver"
// then launched the wrong driver on the wrong device -- silently, because a
// driver that finds nothing on a port looks the same as one that is starting
// up. robot_config.yaml is the source of truth, so follow it once it arrives.
function applyLaserConfigToPanel() {
  if (!laserModelSel || !laserModelSel.options.length) return;
  const c = state.config || {};
  const has = (v) => [...laserModelSel.options].some((o) => o.value === v);
  let code = "";
  if (c.laser_model && has(c.laser_model)) {
    code = c.laser_model;                      // exact model, when recorded
  } else if (c.laser_sensor) {
    // laser_sensor names a driver family (e.g. "ldlidar"); the select holds
    // models (ld06/ld19/stl27l). Take the family's first model as the default.
    const entry = (SENSORS.laser || {})[c.laser_sensor];
    const first = entry && (entry.models || [])[0];
    if (first && has(first.code)) code = first.code;
  }
  if (code) laserModelSel.value = code;
  // The connection route has to be restored before the visibility pass, which
  // reads it to decide which row to show. An imported firmware header sets it:
  // USE_LIDAR_UDP means the scan arrives as datagrams, not on a cable.
  const modeSel = document.getElementById("laser-driver-mode");
  if (modeSel && c.laser_transport &&
      [...modeSel.options].some((o) => o.value === c.laser_transport)) {
    modeSel.value = c.laser_transport;
  }
  const udpField = document.getElementById("laser-driver-udp-port");
  if (udpField && c.laser_udp_port) udpField.value = c.laser_udp_port;
  updateLaserDriverFieldsVisibility();
}

function updateLaserDriverFieldsVisibility() {
  const meta = laserModelMeta(laserModelSel.value);
  if (!meta) return;
  document.getElementById("laser-driver-ld-fields").style.display = meta.isLd ? "block" : "none";
  const hint = document.getElementById("laser-driver-simple-hint");
  if (hint) {
    hint.textContent = meta.isLd
      ? ""
      : `Delegates to lasers.launch.py sensor:=${meta.code}. Serial port below is passed as lidar_serial_port:= (blank = the driver's own /dev symlink).`;
  }
  const portField = document.getElementById("laser-driver-serial-port");
  if (portField) portField.value = laserDefaultPort(meta);
  const baudField = document.getElementById("laser-driver-baud");
  if (baudField && meta.isLd) baudField.value = (state.config || {}).laser_baud || meta.baud;
  updateLaserDriverModeVisibility();
}

function updateLaserDriverModeVisibility() {
  const meta = laserModelMeta(laserModelSel.value);
  const mode = meta && meta.isLd ? document.getElementById("laser-driver-mode").value : "serial";
  document.getElementById("laser-driver-serial-row").style.display = mode === "serial" ? "flex" : "none";
  document.getElementById("laser-driver-udpbridge-row").style.display = mode === "udp_bridge" ? "flex" : "none";
  document.getElementById("laser-driver-netaddr-row").style.display =
    (mode === "udp_server" || mode === "udp_client") ? "flex" : "none";
}

laserModelSel.addEventListener("change", updateLaserDriverFieldsVisibility);
document.getElementById("laser-driver-mode").addEventListener("change", updateLaserDriverModeVisibility);
const btnDetectPorts = document.getElementById("btn-laser-detect-ports");
if (btnDetectPorts) btnDetectPorts.addEventListener("click", refreshSerialPorts);

function persistLaserPort() {
  const port = document.getElementById("laser-driver-serial-port").value.trim();
  const meta = laserModelMeta(laserModelSel.value);
  const baud = meta && meta.isLd ? document.getElementById("laser-driver-baud").value.trim() : "";
  const payload = {};
  // The route is worth saving even when there is no serial port to save with
  // it: on the udp_bridge route the port row is hidden and the field is empty,
  // so returning early on a blank port dropped the route on the floor and a
  // reload came back on "Direct serial" pointed at a device nobody is feeding.
  if (meta && meta.isLd) {
    payload.laser_transport = document.getElementById("laser-driver-mode").value;
    const udpPort = document.getElementById("laser-driver-udp-port").value.trim();
    if (udpPort) payload.laser_udp_port = udpPort;
  }
  // Only record a port the panel actually holds. Before the saved config has
  // been applied this field still shows the registry's first entry
  // (/dev/ydlidar), and writing that back overwrote the real device -- the
  // panel's own defaults ended up saved as if the user had chosen them.
  if (port) {
    payload.laser_serial_port = port;
    payload.laser_baud = baud;
    // Record the model too, so the exact LiDAR survives a reload; the driver
    // family alone cannot say whether this is an LD06 or an LD19.
    payload.laser_model = meta ? meta.code : "";
  }
  if (!Object.keys(payload).length) return;
  fetch("/api/config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  }).then((r) => r.json()).then((c) => { state.config = c; }).catch(() => {});
}

function ldNodeParams(meta, overrides) {
  const base = {
    product_name: meta.product,
    topic_name: "scan",
    frame_id: "laser",
    laser_scan_dir: "true",
    bins: String(meta.bins),
    enable_angle_crop_func: "false",
    angle_crop_min: "135.0",
    angle_crop_max: "225.0",
  };
  Object.assign(base, overrides);
  return Object.entries(base).map(([k, v]) => `-p ${k}:=${v}`).join(" ");
}

function buildLaserDriverCommand() {
  const meta = laserModelMeta(laserModelSel.value);
  const port = document.getElementById("laser-driver-serial-port").value.trim();
  persistLaserPort();

  if (!meta.isLd) {
    let cmd = `ros2 launch linorobot2_bringup lasers.launch.py sensor:=${meta.code}`;
    if (port) cmd += ` lidar_transport:=serial lidar_serial_port:=${port}`;
    return { command: envPrefix() + cmd };
  }

  const mode = document.getElementById("laser-driver-mode").value;
  const baud = document.getElementById("laser-driver-baud").value.trim() || meta.baud;
  const nodeCmd = "ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node";

  if (mode === "serial") {
    const params = ldNodeParams(meta, { comm_mode: "serial", port_name: port || meta.symlink, port_baudrate: baud });
    return { command: envPrefix() + `${nodeCmd} --ros-args ${params}` };
  }

  if (mode === "udp_bridge") {
    // The MCU relays raw LiDAR UART bytes as UDP datagrams to this port (the
    // firmware's own "LiDAR over WiFi UDP" feature -- see LIDAR_SERVER/
    // LIDAR_PORT in config-engine). There is no ROS2-side consumer for a raw
    // byte stream like that, so socat turns it into a normal-looking local
    // serial device (a pty) that ldlidar_stl_ros2_node can just open with
    // comm_mode=serial like any USB-attached unit.
    const udpPort = document.getElementById("laser-driver-udp-port").value.trim() || "8889";
    const bridgePath = document.getElementById("laser-driver-bridge-path").value.trim() || "/dev/lidar_udp_bridge";
    const params = ldNodeParams(meta, { comm_mode: "serial", port_name: bridgePath, port_baudrate: baud });
    const bridgeCmd =
      `command -v socat >/dev/null 2>&1 || sudo apt-get install -y socat; ` +
      `sudo pkill -f "socat.*${bridgePath}" 2>/dev/null; sleep 0.3; ` +
      `(socat -d -d UDP-LISTEN:${udpPort},reuseaddr PTY,link=${bridgePath},raw,echo=0,mode=666 &) && sleep 1.5`;
    return { command: envPrefix() + `${bridgeCmd} && ${nodeCmd} --ros-args ${params}` };
  }

  // udp_server / udp_client: the ldlidar_stl_ros2 driver's own native network
  // modes (talking to a network-attached LiDAR directly) -- unrelated to the
  // firmware's raw-relay feature above, offered for completeness.
  const serverIp = document.getElementById("laser-driver-server-ip").value.trim() || "0.0.0.0";
  const serverPort = document.getElementById("laser-driver-server-port").value.trim() || "8889";
  const params = ldNodeParams(meta, {
    comm_mode: mode, server_ip: serverIp, server_port: serverPort, port_baudrate: baud,
  });
  return { command: envPrefix() + `${nodeCmd} --ros-args ${params}` };
}

const btnLaserStart = document.getElementById("btn-laser-driver-start");
const btnLaserStop = document.getElementById("btn-laser-driver-stop");

function isLaserRunning() {
  return Boolean(state.status && state.status.laser_busy);
}

// Shared by the button and by the 1-Click chain, so both start the driver the
// same way. Resolves once the process has been accepted, not once it exits --
// the driver is long-lived and the caller has to carry on to SLAM.
function startLaserDriver() {
  const { command } = buildLaserDriverCommand();
  btnLaserStart.disabled = true;
  btnLaserStop.disabled = false;
  runCommand(command, {
    // Its own slot: the driver has to keep running while SLAM and Nav2 do, and
    // bringup does not start the LiDAR, so sharing "main" made /scan and SLAM
    // mutually exclusive -- the second one was refused with a 409 that showed
    // up as simply nothing happening.
    slot: "laser",
    title: `Laser driver: ${laserModelSel.value}`,
    onDone: () => {
      btnLaserStart.disabled = false;
      btnLaserStop.disabled = true;
    },
  });
  // Give the driver a moment to open the port and start publishing before the
  // caller launches SLAM on top of it.
  return new Promise((resolve) => setTimeout(async () => {
    await refreshStatus();
    resolve(isLaserRunning());
  }, 4000));
}

btnLaserStart.addEventListener("click", () => { startLaserDriver(); });
btnLaserStop.addEventListener("click", () => killSlot("laser"));

// ---------- LiDAR viewer ----------
let lidarSource = null;
const lidarCanvas = document.getElementById("lidar-canvas");
const lidarCtx = lidarCanvas.getContext("2d");

function drawScan(scan) {
  const w = lidarCanvas.width, h = lidarCanvas.height;
  lidarCtx.clearRect(0, 0, w, h);
  lidarCtx.strokeStyle = "#232b3d";
  lidarCtx.beginPath();
  lidarCtx.arc(w / 2, h / 2, Math.min(w, h) / 2 - 4, 0, Math.PI * 2);
  lidarCtx.stroke();

  const ranges = scan.ranges || [];
  if (!ranges.length) return;
  const maxRange = Math.max(...ranges.filter((r) => isFinite(r) && r > 0), 1);
  const scale = (Math.min(w, h) / 2 - 8) / maxRange;

  lidarCtx.fillStyle = "#6366f1";
  ranges.forEach((r, i) => {
    if (!isFinite(r) || r <= 0) return;
    const angle = scan.angle_min + i * scan.angle_increment;
    const x = w / 2 + r * Math.cos(angle) * scale;
    const y = h / 2 - r * Math.sin(angle) * scale;
    lidarCtx.fillRect(x - 1.5, y - 1.5, 3, 3);
  });
}

document.getElementById("btn-lidar-start").addEventListener("click", () => {
  if (lidarSource) lidarSource.close();
  lidarSource = new EventSource("/api/lidar_stream");
  lidarSource.addEventListener("scan", (ev) => {
    try {
      drawScan(JSON.parse(ev.data));
    } catch {}
  });
  document.getElementById("btn-lidar-start").disabled = true;
  document.getElementById("btn-lidar-stop").disabled = false;
});
document.getElementById("btn-lidar-stop").addEventListener("click", () => {
  if (lidarSource) {
    lidarSource.close();
    lidarSource = null;
  }
  document.getElementById("btn-lidar-start").disabled = false;
  document.getElementById("btn-lidar-stop").disabled = true;
});

// ---------- settings ----------
document.getElementById("btn-save-workspace").addEventListener("click", () => {
  const workspace_path = document.getElementById("cfg-workspace").value.trim();
  fetch("/api/config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ workspace_path }),
  }).then(refreshStatus);
});

document.getElementById("btn-save-agent").addEventListener("click", () => {
  const body = {
    agent_transport: document.getElementById("cfg-agent-transport").value,
    agent_device: document.getElementById("cfg-agent-device").value,
    agent_port: document.getElementById("cfg-agent-port").value,
    agent_baud: document.getElementById("cfg-agent-baud").value,
  };
  fetch("/api/config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  }).then(refreshStatus);
});

// ---------- Nav2 configuration editor (per-distro) ----------
const nav2EditorBox = document.getElementById("nav2-editor-box");
const btnNav2Toggle = document.getElementById("btn-nav2-toggle-editor");
const nav2Textarea = document.getElementById("nav2-config-text");
const nav2EditorDistro = document.getElementById("nav2-editor-distro");
const btnNav2Save = document.getElementById("btn-nav2-save-config");
const btnNav2Reset = document.getElementById("btn-nav2-reset-defaults");
const nav2Status = document.getElementById("nav2-save-status");

async function loadNav2Config(distro) {
  const d = distro || (nav2EditorDistro ? nav2EditorDistro.value : getDistro());
  if (nav2EditorDistro && nav2EditorDistro.value !== d) {
    nav2EditorDistro.value = d;
  }
  try {
    const res = await fetch(`/api/nav2_config?distro=${d}`);
    const data = await res.json();
    if (nav2Textarea && data.config) {
      nav2Textarea.value = data.config;
    }
    const hint = document.getElementById("nav2-depth-costmap-hint");
    if (hint) {
      const on = data.depth_pointcloud_active;
      hint.textContent = on === null || on === undefined
        ? ""
        : `This file lists observation_sources: ${on ? "scan pointcloud" : "scan"}. ` +
          `At launch, launch_nav2.py passes depth_costmap:=<true|false> from the Bringup ` +
          `depth-sensor selection and generates a gated copy if needed — your saved YAML is not modified.`;
    }
  } catch (e) {}
}

if (nav2EditorDistro) {
  nav2EditorDistro.addEventListener("change", () => loadNav2Config(nav2EditorDistro.value));
}

if (btnNav2Toggle) {
  btnNav2Toggle.addEventListener("click", () => {
    if (!nav2EditorBox) return;
    const isHidden = nav2EditorBox.style.display === "none";
    nav2EditorBox.style.display = isHidden ? "block" : "none";
    btnNav2Toggle.textContent = isHidden ? "Hide Nav2 Parameters" : "Edit Nav2 Parameters (YAML)";
    if (isHidden) {
      loadNav2Config(getDistro());
    }
  });
}

if (btnNav2Save) {
  btnNav2Save.addEventListener("click", async () => {
    if (!nav2Textarea) return;
    const d = nav2EditorDistro ? nav2EditorDistro.value : getDistro();
    btnNav2Save.disabled = true;
    if (nav2Status) nav2Status.textContent = `Saving config for ${d}...`;
    try {
      const res = await fetch("/api/nav2_config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ distro: d, config: nav2Textarea.value }),
      });
      const data = await res.json();
      if (nav2Status) {
        nav2Status.textContent = data.status === "ok" ? `✓ Saved to console_nav2_${d}.yaml` : "Error saving";
        setTimeout(() => { if (nav2Status) nav2Status.textContent = ""; }, 4000);
      }
    } catch (e) {
      if (nav2Status) nav2Status.textContent = "Error: " + e.message;
    } finally {
      btnNav2Save.disabled = false;
    }
  });
}

if (btnNav2Reset) {
  btnNav2Reset.addEventListener("click", async () => {
    const d = nav2EditorDistro ? nav2EditorDistro.value : getDistro();
    if (!confirm(`Reset Nav2 configuration for ${d} to default parameters?`)) return;
    btnNav2Reset.disabled = true;
    try {
      const res = await fetch("/api/nav2_config/reset", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ distro: d }),
      });
      const data = await res.json();
      if (data.config && nav2Textarea) {
        nav2Textarea.value = data.config;
        if (nav2Status) {
          nav2Status.textContent = `✓ Reset ${d} to default parameters`;
          setTimeout(() => { if (nav2Status) nav2Status.textContent = ""; }, 4000);
        }
      }
    } catch (e) {
      if (nav2Status) nav2Status.textContent = "Reset failed: " + e.message;
    } finally {
      btnNav2Reset.disabled = false;
    }
  });
}

loadNav2Config();

// ---------- distro & auto-bringup settings sync ----------
const hdrDistroSelect = document.getElementById("hdr-distro-select");
if (hdrDistroSelect) {
  hdrDistroSelect.addEventListener("change", () => {
    const val = hdrDistroSelect.value;
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ ros_distro: val }),
    }).then(refreshStatus);
  });
}

const btnSaveDistro = document.getElementById("btn-save-distro");
if (btnSaveDistro) {
  btnSaveDistro.addEventListener("click", () => {
    const distro = document.getElementById("cfg-ros-distro")?.value || "jazzy";
    const autoBringup = Boolean(document.getElementById("cfg-auto-bringup")?.checked);
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ ros_distro: distro, auto_bringup: autoBringup }),
    }).then(refreshStatus);
  });
}

const bringupAutoToggle = document.getElementById("bringup-auto-toggle");
if (bringupAutoToggle) {
  bringupAutoToggle.addEventListener("change", () => {
    const autoBringup = bringupAutoToggle.checked;
    const cfgAuto = document.getElementById("cfg-auto-bringup");
    if (cfgAuto) cfgAuto.checked = autoBringup;
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ auto_bringup: autoBringup }),
    });
  });
}


// =========================================================
// AI ROBOTICS TUNING & CUSTOM ROBOT BUILDER STUDIO
// =========================================================
function currentRosDistro() {
  return getDistro();
}

let currentAiTuningAnalysis = null;
let currentCustomRobotSpecs = null;

// 1. AI Tuning Prompt & Chips
const aiTunePrompt = document.getElementById("ai-tune-prompt");
const btnAiTuneAsk = document.getElementById("btn-ai-tune-ask");
const aiTuneOutput = document.getElementById("ai-tune-output");
const aiTuneDiag = document.getElementById("ai-tune-diagnosis");
const aiTuneRecs = document.getElementById("ai-tune-recs");
const btnAiTuneApply = document.getElementById("btn-ai-tune-apply");
const aiTuneStatus = document.getElementById("ai-tune-status");

document.querySelectorAll(".btn-chip").forEach((btn) => {
  btn.addEventListener("click", () => {
    if (aiTunePrompt) {
      aiTunePrompt.value = btn.getAttribute("data-prompt") || "";
      if (btnAiTuneAsk) btnAiTuneAsk.click();
    }
  });
});

if (btnAiTuneAsk) {
  btnAiTuneAsk.addEventListener("click", async () => {
    const prompt = (aiTunePrompt?.value || "").trim();
    if (!prompt) return;
    btnAiTuneAsk.disabled = true;
    if (aiTuneStatus) aiTuneStatus.textContent = "Analyzing robotics dynamics...";
    try {
      const distro = currentRosDistro();
      const base = document.getElementById("tune-base-type")?.value || "2wd";
      const res = await fetch("/api/ai/tune", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ prompt, distro, base }),
      });
      const data = await res.json();
      currentAiTuningAnalysis = data;
      if (aiTuneOutput) aiTuneOutput.style.display = "block";
      if (aiTuneDiag) aiTuneDiag.textContent = "🩺 " + (data.diagnosis || "No diagnosis.");
      if (aiTuneRecs) {
        aiTuneRecs.innerHTML = (data.recommendations || []).map((r) => `<li>${escapeHtml(r)}</li>`).join("");
      }
      if (aiTuneStatus) aiTuneStatus.textContent = "Analysis complete.";
    } catch (e) {
      if (aiTuneStatus) aiTuneStatus.textContent = "Error: " + e.message;
    } finally {
      btnAiTuneAsk.disabled = false;
    }
  });
}

if (btnAiTuneApply) {
  btnAiTuneApply.addEventListener("click", async () => {
    if (!currentAiTuningAnalysis) return;
    btnAiTuneApply.disabled = true;
    if (aiTuneStatus) aiTuneStatus.textContent = "Applying patches...";
    try {
      const distro = currentRosDistro();
      const base = currentAiTuningAnalysis.target_base
        || document.getElementById("tune-base-type")?.value || "2wd";
      const res = await fetch("/api/ai/apply", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          nav2_patch: currentAiTuningAnalysis.nav2_patch,
          ekf_patch: currentAiTuningAnalysis.ekf_patch,
          slam_patch: currentAiTuningAnalysis.slam_patch,
          base,
          distro,
        }),
      });
      const data = await res.json();
      if (aiTuneStatus) aiTuneStatus.textContent = "✓ Applied AI recommendations to Nav2, EKF & SLAM!";
      loadNav2Config();
      loadEkfConfig();
      loadSlamConfig();
      setTimeout(() => { if (aiTuneStatus) aiTuneStatus.textContent = ""; }, 5000);
    } catch (e) {
      if (aiTuneStatus) aiTuneStatus.textContent = "Apply failed: " + e.message;
    } finally {
      btnAiTuneApply.disabled = false;
    }
  });
}

// 2. Presets Selector
const btnApplyPreset = document.getElementById("btn-apply-preset");
const tunePresetSelect = document.getElementById("tune-preset-select");
if (btnApplyPreset && tunePresetSelect) {
  btnApplyPreset.addEventListener("click", async () => {
    const preset = tunePresetSelect.value;
    btnApplyPreset.disabled = true;
    try {
      const distro = currentRosDistro();
      const res = await fetch("/api/presets/apply", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ preset, distro }),
      });
      const data = await res.json();
      alert(`Applied preset '${data.label}'! Nav2, EKF, and SLAM configs updated.`);
      loadNav2Config();
      loadEkfConfig();
      loadSlamConfig();
    } catch (e) {
      alert("Failed to apply preset: " + e.message);
    } finally {
      btnApplyPreset.disabled = false;
    }
  });
}

// 3. Interactive Quick Tuning
const btnApplyInteractive = document.getElementById("btn-apply-interactive-tuning");
const interactiveStatus = document.getElementById("tune-interactive-status");
if (btnApplyInteractive) {
  btnApplyInteractive.addEventListener("click", async () => {
    btnApplyInteractive.disabled = true;
    if (interactiveStatus) interactiveStatus.textContent = "Saving tuning...";
    try {
      const distro = currentRosDistro();
      const base = document.getElementById("tune-base-type")?.value || "2wd";
      const max_vel_x = parseFloat(document.getElementById("tune-max-vel-x")?.value || "0.5");
      const max_vel_y = parseFloat(document.getElementById("tune-max-vel-y")?.value || "0.0");
      const max_vel_theta = parseFloat(document.getElementById("tune-max-vel-theta")?.value || "2.5");
      const max_accel_x = parseFloat(document.getElementById("tune-max-accel-x")?.value || "2.5");
      const max_accel_theta = parseFloat(document.getElementById("tune-max-accel-theta")?.value || "3.2");
      const inflation_radius = parseFloat(document.getElementById("tune-inflation-radius")?.value || "0.7");
      const cost_scaling_factor = parseFloat(document.getElementById("tune-cost-scaling")?.value || "3.0");

      const ekf_freq = parseFloat(document.getElementById("tune-ekf-freq")?.value || "50");
      const fuse_vy = Boolean(document.getElementById("tune-fuse-vy")?.checked);
      const fuse_imu_yaw = Boolean(document.getElementById("tune-fuse-imu-yaw")?.checked);

      const slam_res = parseFloat(document.getElementById("tune-slam-res")?.value || "0.05");

      // Patch Nav2
      await fetch("/api/nav2_config/patch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          distro, base, max_vel_x, max_vel_y, max_vel_theta,
          max_accel_x, max_accel_theta, inflation_radius, cost_scaling_factor
        }),
      });

      // Patch EKF
      await fetch("/api/ekf_config/patch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          base, frequency: ekf_freq, fuse_vy, fuse_imu_yaw
        }),
      });

      // Patch SLAM
      await fetch("/api/slam_config/patch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ resolution: slam_res }),
      });

      if (interactiveStatus) interactiveStatus.textContent = "✓ Applied tuning parameters across configs!";
      loadNav2Config();
      loadEkfConfig();
      loadSlamConfig();
      setTimeout(() => { if (interactiveStatus) interactiveStatus.textContent = ""; }, 4000);
    } catch (e) {
      if (interactiveStatus) interactiveStatus.textContent = "Tuning failed: " + e.message;
    } finally {
      btnApplyInteractive.disabled = false;
    }
  });
}

// 4. EKF & SLAM Editors
const ekfTextarea = document.getElementById("ekf-config-text");
const btnEkfSave = document.getElementById("btn-ekf-save-config");
const btnEkfReset = document.getElementById("btn-ekf-reset-defaults");
const ekfStatus = document.getElementById("ekf-save-status");

async function loadEkfConfig() {
  if (!ekfTextarea) return;
  try {
    const base = document.getElementById("tune-base-type")?.value || "2wd";
    const res = await fetch(`/api/ekf_config?base=${base}`);
    const data = await res.json();
    if (data.config) ekfTextarea.value = data.config;
  } catch (e) {}
}

if (btnEkfSave && ekfTextarea) {
  btnEkfSave.addEventListener("click", async () => {
    try {
      const res = await fetch("/api/ekf_config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ config: ekfTextarea.value }),
      });
      if (ekfStatus) ekfStatus.textContent = "✓ Saved EKF configuration";
      setTimeout(() => { if (ekfStatus) ekfStatus.textContent = ""; }, 3000);
    } catch (e) {
      if (ekfStatus) ekfStatus.textContent = "Save failed: " + e.message;
    }
  });
}

if (btnEkfReset && ekfTextarea) {
  btnEkfReset.addEventListener("click", async () => {
    try {
      const base = document.getElementById("tune-base-type")?.value || "2wd";
      const res = await fetch("/api/ekf_config/reset", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ base }),
      });
      const data = await res.json();
      if (data.config) ekfTextarea.value = data.config;
      if (ekfStatus) ekfStatus.textContent = "✓ Reset EKF to default";
      setTimeout(() => { if (ekfStatus) ekfStatus.textContent = ""; }, 3000);
    } catch (e) {
      if (ekfStatus) ekfStatus.textContent = "Reset failed: " + e.message;
    }
  });
}

const slamTextarea = document.getElementById("slam-config-text");
const btnSlamSave = document.getElementById("btn-slam-save-config");
const btnSlamReset = document.getElementById("btn-slam-reset-defaults");
const slamStatus = document.getElementById("slam-save-status");

async function loadSlamConfig() {
  if (!slamTextarea) return;
  try {
    const res = await fetch("/api/slam_config");
    const data = await res.json();
    if (data.config) slamTextarea.value = data.config;
  } catch (e) {}
}

if (btnSlamSave && slamTextarea) {
  btnSlamSave.addEventListener("click", async () => {
    try {
      await fetch("/api/slam_config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ config: slamTextarea.value }),
      });
      if (slamStatus) slamStatus.textContent = "✓ Saved SLAM configuration";
      setTimeout(() => { if (slamStatus) slamStatus.textContent = ""; }, 3000);
    } catch (e) {
      if (slamStatus) slamStatus.textContent = "Save failed: " + e.message;
    }
  });
}

if (btnSlamReset && slamTextarea) {
  btnSlamReset.addEventListener("click", async () => {
    try {
      const res = await fetch("/api/slam_config/reset", { method: "POST" });
      const data = await res.json();
      if (data.config) slamTextarea.value = data.config;
      if (slamStatus) slamStatus.textContent = "✓ Reset SLAM to default";
      setTimeout(() => { if (slamStatus) slamStatus.textContent = ""; }, 3000);
    } catch (e) {
      if (slamStatus) slamStatus.textContent = "Reset failed: " + e.message;
    }
  });
}

// ---------- params export / merge / promote ----------
const paramsOpResult = document.getElementById("params-op-result");
function showParamsResult(obj) {
  if (paramsOpResult) paramsOpResult.textContent =
    typeof obj === "string" ? obj : JSON.stringify(obj, null, 2);
}
async function paramsPost(path, body) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  return res.json();
}
function editorDistro() {
  return (nav2EditorDistro && nav2EditorDistro.value) || getDistro();
}

document.getElementById("btn-params-export")?.addEventListener("click", async () => {
  const dir = document.getElementById("params-export-dir").value.trim();
  if (!dir) { showParamsResult("Enter an export directory first."); return; }
  showParamsResult("Exporting…");
  try {
    const d = await paramsPost("/api/params/export", {
      dest_dir: dir,
      distros: ["jazzy", "lyrical", "rolling"],
      depth_costmap: document.getElementById("bringup-depth-sensor")?.value ? "true" : "false",
    });
    showParamsResult(d);
  } catch (e) { showParamsResult("Export failed: " + e.message); }
});

async function runMerge(dryRun, importText) {
  const body = {
    kind: document.getElementById("params-merge-kind").value,
    distro: editorDistro(),
    dry_run: dryRun,
  };
  if (importText != null) { body.target = "active"; body.source_text = importText; }
  else { body.target = document.getElementById("params-merge-target").value; }
  showParamsResult(dryRun ? "Previewing merge…" : "Merging…");
  try {
    const d = await paramsPost("/api/params/merge", body);
    showParamsResult({ status: d.status, target: d.target_path, report: d.report });
    if (d.status === "merged" && !importText && nav2Textarea) loadNav2Config(editorDistro());
  } catch (e) { showParamsResult("Merge failed: " + e.message); }
}
document.getElementById("btn-params-merge-dry")?.addEventListener("click", () => runMerge(true));
document.getElementById("btn-params-merge")?.addEventListener("click", () => runMerge(false));
document.getElementById("btn-params-merge-import")?.addEventListener("click", () => {
  const t = document.getElementById("params-import-text").value;
  if (!t.trim()) { showParamsResult("Paste a params file first."); return; }
  runMerge(false, t);
});

document.getElementById("btn-params-promote")?.addEventListener("click", async () => {
  showParamsResult("Promoting…");
  try {
    const d = await paramsPost("/api/params/promote", {
      kind: document.getElementById("params-merge-kind").value,
      distro: editorDistro(),
      direction: document.getElementById("params-promote-dir").value,
    });
    showParamsResult(d);
    if (d.status === "promoted" && d.direction.endsWith("_to_active") && nav2Textarea) {
      loadNav2Config(editorDistro());
    }
  } catch (e) { showParamsResult("Promote failed: " + e.message); }
});

loadEkfConfig();
loadSlamConfig();

// 5. AI Custom Robot Builder Studio
const aiRobotPrompt = document.getElementById("ai-robot-prompt");
const btnAiRobotGenerate = document.getElementById("btn-ai-robot-generate");
const aiRobotSpecBox = document.getElementById("ai-robot-spec-box");
const btnAiRobotDeploy = document.getElementById("btn-ai-robot-deploy");
const aiRobotDeployStatus = document.getElementById("ai-robot-deploy-status");

document.querySelectorAll(".btn-robot-chip").forEach((btn) => {
  btn.addEventListener("click", () => {
    if (aiRobotPrompt) {
      aiRobotPrompt.value = btn.getAttribute("data-robot") || "";
      if (btnAiRobotGenerate) btnAiRobotGenerate.click();
    }
  });
});

if (btnAiRobotGenerate) {
  btnAiRobotGenerate.addEventListener("click", async () => {
    const description = (aiRobotPrompt?.value || "").trim();
    if (!description) return;
    btnAiRobotGenerate.disabled = true;
    try {
      const res = await fetch("/api/ai/robot_builder", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ description }),
      });
      const data = await res.json();
      currentCustomRobotSpecs = data;

      if (aiRobotSpecBox) aiRobotSpecBox.style.display = "block";
      const des = data.design || {};
      const specBase = document.getElementById("spec-base");
      const specWheel = document.getElementById("spec-wheel");
      const specTrack = document.getElementById("spec-track");
      const specWheelbase = document.getElementById("spec-wheelbase");
      const specLidar = document.getElementById("spec-lidar");

      if (specBase) specBase.textContent = des.title || (data.base || "").toUpperCase();
      if (specWheel) specWheel.textContent = (des.wheel_diameter_m ? (des.wheel_diameter_m * 1000) + "mm" : "-");
      if (specTrack) specTrack.textContent = (des.track_width_m ? (des.track_width_m * 1000) + "mm" : "-");
      if (specWheelbase) specWheelbase.textContent = (des.wheelbase_m ? (des.wheelbase_m * 1000) + "mm" : "0mm (2WD)");
      if (specLidar) specLidar.textContent = des.laser_name || (data.laser_sensor || "").toUpperCase();

      const wf = document.getElementById("spec-workflow");
      if (wf && data.workflow) {
        wf.innerHTML = data.workflow.map((s) => `<div>${escapeHtml(s)}</div>`).join("");
      }
    } catch (e) {
      alert("Failed to generate robot specs: " + e.message);
    } finally {
      btnAiRobotGenerate.disabled = false;
    }
  });
}

if (btnAiRobotDeploy) {
  btnAiRobotDeploy.addEventListener("click", async () => {
    if (!currentCustomRobotSpecs) return;
    btnAiRobotDeploy.disabled = true;
    if (aiRobotDeployStatus) aiRobotDeployStatus.textContent = "Deploying custom robot architecture...";
    try {
      const distro = currentRosDistro();
      const res = await fetch("/api/ai/deploy_robot", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ specs: currentCustomRobotSpecs, distro }),
      });
      const data = await res.json();
      if (aiRobotDeployStatus) aiRobotDeployStatus.textContent = "✓ " + (data.message || "Custom robot deployed successfully!");
      loadNav2Config();
      loadEkfConfig();
      loadSlamConfig();
      setTimeout(() => { if (aiRobotDeployStatus) aiRobotDeployStatus.textContent = ""; }, 6000);
    } catch (e) {
      if (aiRobotDeployStatus) aiRobotDeployStatus.textContent = "Deployment failed: " + e.message;
    } finally {
      btnAiRobotDeploy.disabled = false;
    }
  });
}

// ============ path / serial-port picker (used by .pick-btn buttons) ============
(function () {
  const overlay = document.getElementById("picker-overlay");
  if (!overlay) return;
  const elTitle = document.getElementById("picker-title");
  const elPath = document.getElementById("picker-path");
  const elCwd = document.getElementById("picker-cwd");
  const elList = document.getElementById("picker-list");
  const elHint = document.getElementById("picker-hint");
  const btnUp = document.getElementById("picker-up");
  const btnUse = document.getElementById("picker-use");
  const btnRefresh = document.getElementById("picker-refresh");
  let ctx = null;

  function close() { overlay.classList.remove("open"); ctx = null; }
  function row(txt) {
    const d = document.createElement("div");
    d.className = "pk-row"; d.textContent = txt; return d;
  }
  function pick(value) {
    if (ctx && ctx.target) {
      ctx.target.value = value;
      ctx.target.dispatchEvent(new Event("input", { bubbles: true }));
      ctx.target.dispatchEvent(new Event("change", { bubbles: true }));
    }
    close();
  }

  overlay.addEventListener("click", (e) => { if (e.target === overlay) close(); });
  document.getElementById("picker-close").onclick = close;
  document.getElementById("picker-cancel").onclick = close;
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && overlay.classList.contains("open")) close();
  });

  async function loadDir(path) {
    elList.replaceChildren(row("Loading…"));
    let data;
    try {
      const q = new URLSearchParams({ path: path || "", only: ctx.only, exts: ctx.exts || "" });
      data = await fetch("/api/list_dir?" + q).then((r) => r.json());
    } catch (e) { elList.replaceChildren(row("Error: " + e.message)); return; }
    ctx.cwd = data.path;
    elCwd.textContent = data.path;
    btnUp.disabled = !data.parent;
    btnUp.onclick = () => loadDir(data.parent);
    const rows = (data.entries || []).map((e) => {
      const r = document.createElement("div");
      r.className = "pk-row";
      r.innerHTML = '<span class="pk-ic">' + (e.is_dir ? "📂" : "📄") + "</span>" + escapeHtml(e.name);
      r.onclick = () => (e.is_dir ? loadDir(e.path) : (ctx.only === "dir" ? null : pick(e.path)));
      r.ondblclick = () => e.is_dir && loadDir(e.path);
      return r;
    });
    elList.replaceChildren(...(rows.length ? rows : [row(data.error ? "(" + data.error + ")" : "(empty)")]));
  }

  async function loadSerial() {
    elList.replaceChildren(row("Scanning…"));
    await refreshSerialPorts();
    const rows = (SERIAL_PORTS || []).map((p) => {
      const r = document.createElement("div");
      r.className = "pk-row";
      const id = [p.vendor, p.model].filter(Boolean).join(" ") || "USB serial";
      r.innerHTML = '<span class="pk-ic">🔌</span>' + escapeHtml(id) +
        (p.usb_id ? ' <span class="pk-sub">' + escapeHtml(p.usb_id) + "</span>" : "") +
        '<span class="pk-sub">→ ' + escapeHtml(p.tty) + "</span>";
      r.onclick = () => pick(p.preferred);
      return r;
    });
    elList.replaceChildren(...(rows.length ? rows : [row("No USB serial devices detected.")]));
  }

  function openPicker(target, kind, exts) {
    const isSerial = kind === "serial";
    ctx = {
      target, kind, exts: exts || "",
      only: kind === "file" ? "file" : kind === "dir" ? "dir" : "any",
      cwd: "",
    };
    elTitle.textContent = isSerial ? "Pick a serial port"
      : kind === "dir" ? "Pick a folder" : "Pick a file";
    elPath.hidden = isSerial;
    btnUse.hidden = kind !== "dir";
    btnUse.onclick = () => pick(ctx.cwd);
    elHint.textContent = isSerial ? "" : "click a folder to open it";
    btnRefresh.onclick = () => (isSerial ? loadSerial() : loadDir(ctx.cwd));
    overlay.classList.add("open");
    if (isSerial) loadSerial();
    else loadDir((target.value || "").trim());
  }

  document.querySelectorAll(".pick-btn[data-target]").forEach((b) => {
    b.addEventListener("click", () => {
      const t = document.getElementById(b.dataset.target);
      if (t) openPicker(t, b.dataset.pick, b.dataset.exts);
    });
  });
})();


// ============================================================================

// ============================================================================
// Robot & Nav2 Configuration Engine (robot_config.yaml - Single File of Truth)
// ============================================================================
function downloadFile(filename, content, type = "text/yaml;charset=utf-8") {
  const blob = new Blob([content], { type });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  setTimeout(() => {
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }, 100);
}

async function loadRobotConfig() {
  try {
    const distro = getDistro();
    const res = await fetch(`/api/robot_config?distro=${distro}`);
    const data = await res.json();
    state.robot_config = data;
    const lino = data.linorobot2 || {};

    const bBase = document.getElementById("bringup-base-type");
    if (bBase && (lino.base || data.base)) bBase.value = lino.base || data.base;

    const bLaser = document.getElementById("bringup-laser-sensor");
    if (bLaser && lino.laser_sensor) bLaser.value = lino.laser_sensor;

    const bDepth = document.getElementById("bringup-depth-sensor");
    if (bDepth && lino.depth_sensor) bDepth.value = lino.depth_sensor;

    const bDev = document.getElementById("bringup-agent-device");
    if (bDev && lino.micro_ros_port) bDev.value = lino.micro_ros_port;

    const bBaud = document.getElementById("bringup-agent-baud");
    if (bBaud && lino.micro_ros_baudrate) bBaud.value = lino.micro_ros_baudrate;

    const instBase = document.getElementById("install-base");
    if (instBase && (lino.base || data.base)) instBase.value = lino.base || data.base;

    const editor = document.getElementById("unified-config-editor");
    if (editor && data.yaml) editor.value = data.yaml;

    const statusEl = document.getElementById("unified-status");
    if (statusEl) statusEl.textContent = `Loaded from ${data.path}`;
  } catch (e) {
    console.warn("Failed to load robot_config.yaml:", e);
  }
}

async function saveRobotConfigFromBringup() {
  const statusEl = document.getElementById("bringup-config-status");
  if (statusEl) statusEl.textContent = "Updating robot_config.yaml...";
  // Fall back to what is already saved, never to a hardcoded default. These
  // controls are populated asynchronously from the sensor registry, so an
  // action fired before that lands used to read them as empty and write the
  // empty string straight over a working robot_config.yaml -- laser_sensor
  // silently became "", the LiDAR port reverted to /dev/ydlidar, and the next
  // bringup came up with no laser at all. The saved value wins over a blank
  // control; only a value the user can actually see may overwrite it.
  const saved = state.config || {};
  const keep = (id, savedVal, fallback) => {
    const v = document.getElementById(id)?.value;
    if (v !== undefined && v !== "") return v;
    if (savedVal !== undefined && savedVal !== "" && savedVal !== null) return savedVal;
    return fallback;
  };
  const payload = {
    base: keep("bringup-base-type", saved.base_type, "2wd"),
    laser_sensor: keep("bringup-laser-sensor", saved.laser_sensor, ""),
    depth_sensor: keep("bringup-depth-sensor", saved.depth_sensor, ""),
    micro_ros_port: keep("bringup-agent-device", saved.agent_device, "/dev/ttyACM0"),
    micro_ros_baudrate: keep("bringup-agent-baud", saved.agent_baud, "1500000"),
    madgwick: document.getElementById("bringup-madgwick-toggle")?.checked ?? true,
  };
  try {
    const res = await fetch("/api/robot_config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (data.status === "saved") {
      if (statusEl) statusEl.innerHTML = `<span style="color: var(--success);">✓ Updated robot_config.yaml</span>`;
      loadRobotConfig();
      logLine(`[console] Updated robot parameters in ${data.path}`);
    } else {
      if (statusEl) statusEl.textContent = data.error || "Update failed";
    }
  } catch (e) {
    if (statusEl) statusEl.textContent = `Error: ${e.message}`;
  }
}

async function saveRobotConfigFromEditor() {
  const editor = document.getElementById("unified-config-editor");
  const statusEl = document.getElementById("unified-status");
  if (!editor || !editor.value.trim()) return;
  if (statusEl) statusEl.textContent = "Saving robot_config.yaml...";
  try {
    const distro = getDistro();
    const res = await fetch("/api/robot_config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ yaml: editor.value, distro }),
    });
    const data = await res.json();
    if (data.status === "saved") {
      if (statusEl) statusEl.innerHTML = `<span style="color: var(--success);">✓ Saved ${data.path}</span>`;
      loadRobotConfig();
      logLine(`[console] robot_config.yaml saved (single source of truth)`);
    } else {
      if (statusEl) statusEl.textContent = data.error || "Save failed";
    }
  } catch (e) {
    if (statusEl) statusEl.textContent = `Error: ${e.message}`;
  }
}

async function exportRobotConfigFile() {
  try {
    const distro = getDistro();
    const res = await fetch(`/api/robot_config?distro=${distro}`);
    const data = await res.json();
    if (data.yaml) {
      downloadFile("robot_config.yaml", data.yaml, "text/yaml;charset=utf-8");
      logLine(`[console] Exported robot_config.yaml`);
    }
  } catch (e) {
    alert(`Export failed: ${e.message}`);
  }
}

// Wire Event Listeners
document.getElementById("btn-save-robot-config-bringup")?.addEventListener("click", saveRobotConfigFromBringup);
document.getElementById("btn-unified-load")?.addEventListener("click", loadRobotConfig);
document.getElementById("btn-unified-save")?.addEventListener("click", saveRobotConfigFromEditor);
document.getElementById("btn-unified-export")?.addEventListener("click", exportRobotConfigFile);
document.getElementById("btn-export-unified")?.addEventListener("click", exportRobotConfigFile);

// Sync kinematics across tabs
document.getElementById("bringup-base-type")?.addEventListener("change", (e) => {
  const inst = document.getElementById("install-base");
  if (inst) inst.value = e.target.value;
});
document.getElementById("install-base")?.addEventListener("change", (e) => {
  const b = document.getElementById("bringup-base-type");
  if (b) b.value = e.target.value;
});

// Auto-load on startup
setTimeout(() => {
  loadRobotConfig();
}, 400);



// ---------- micro-ROS Agent Port Conflict Detection & Lifecycle ----------
async function checkAgentPortStatus(opts = {}) {
  const c = state.config || {};
  const transport = document.getElementById("cfg-agent-transport")?.value || c.agent_transport || "serial";
  const device = document.getElementById("cfg-agent-device")?.value || c.agent_device || "/dev/ttyACM0";
  const port = document.getElementById("cfg-agent-port")?.value || c.agent_port || "8888";
  
  const statusPill = document.getElementById("agent-port-status-pill");
  if (statusPill) {
    statusPill.style.display = "inline-block";
    statusPill.textContent = "Checking...";
    statusPill.className = "pill pill-starting";
  }

  try {
    const res = await fetch("/api/agent/port_check", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        port: device,
        mode: transport,
        udp_port: parseInt(port, 10) || 8888
      }),
    }).then(r => r.json());

    if (statusPill) {
      if (res.in_use) {
        statusPill.textContent = "Port Busy";
        statusPill.className = "pill pill-warn";
      } else {
        statusPill.textContent = "Port Available";
        statusPill.className = "pill pill-ok";
      }
    }

    if (res.in_use && !opts.silent) {
      document.getElementById("port-conflict-summary").textContent = res.summary || "Port is currently in use";
      document.getElementById("port-conflict-details").textContent = res.details || JSON.stringify(res, null, 2);
      document.getElementById("port-modal-overlay").classList.add("open");
    }
    return res;
  } catch (err) {
    if (statusPill) {
      statusPill.textContent = "Check Failed";
      statusPill.className = "pill pill-off";
    }
    return { in_use: false, error: err.message };
  }
}

async function releaseAgentPort() {
  const c = state.config || {};
  const transport = document.getElementById("cfg-agent-transport")?.value || c.agent_transport || "serial";
  const device = document.getElementById("cfg-agent-device")?.value || c.agent_device || "/dev/ttyACM0";
  const port = document.getElementById("cfg-agent-port")?.value || c.agent_port || "8888";

  const btnRel = document.getElementById("btn-release-port-conflict");
  if (btnRel) {
    btnRel.disabled = true;
    btnRel.textContent = "Releasing...";
  }

  try {
    const res = await fetch("/api/agent/port_release", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        port: device,
        mode: transport,
        udp_port: parseInt(port, 10) || 8888
      }),
    }).then(r => r.json());

    document.getElementById("port-modal-overlay").classList.remove("open");
    await checkAgentPortStatus({ silent: true });
    logLine("[console] micro-ROS agent port released successfully.");
  } catch (err) {
    alert("Failed to release port: " + err.message);
  } finally {
    if (btnRel) {
      btnRel.disabled = false;
      btnRel.textContent = "⚡ Release Port & Stop Agent";
    }
  }
}

document.getElementById("btn-agent-port-check")?.addEventListener("click", () => checkAgentPortStatus());
document.getElementById("hdr-btn-agent-check")?.addEventListener("click", () => checkAgentPortStatus());
document.getElementById("btn-close-port-modal")?.addEventListener("click", () => {
  document.getElementById("port-modal-overlay")?.classList.remove("open");
});
document.getElementById("btn-ignore-port-conflict")?.addEventListener("click", () => {
  document.getElementById("port-modal-overlay")?.classList.remove("open");
});
document.getElementById("btn-release-port-conflict")?.addEventListener("click", () => releaseAgentPort());

// =============================================================================
// WORKFLOW SETUP & ROOTLESS CONTAINER CONTROLLER
// =============================================================================

// Automated Rootless Docker Setup Helper

function showToast(message, duration = 3500) {
  let toast = document.getElementById("toast");
  if (!toast) {
    toast = document.createElement("div");
    toast.id = "toast";
    document.body.appendChild(toast);
  }
  toast.textContent = message;
  toast.classList.add("show");
  if (toast._timer) clearTimeout(toast._timer);
  toast._timer = setTimeout(() => {
    toast.classList.remove("show");
  }, duration);
}

async function ensureContainerEngine(engine) {
  if (engine === "native") return { installed: true };
  const targetEngine = (engine === "podman" || engine === "podman_systemd") ? "podman" : "docker";

  try {
    const status = await fetch("/api/docker/status").then(r => r.json());
    if (targetEngine === "podman" && !status.has_podman) {
      showToast("🦭 Podman not found on system. Installing automatically...", 5000);
      const res = await fetch("/api/container/install", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ engine: "podman" })
      }).then(r => r.json());
      if (res.installed) {
        showToast("✅ Podman installed successfully!", 4000);
      } else {
        showToast("⚠️ Podman auto-install failed: " + res.message, 6000);
      }
      return res;
    } else if (targetEngine === "docker") {
      if (!status.has_docker) {
        showToast("🐳 Docker not found on system. Installing Rootless Docker automatically...", 6000);
        const res = await fetch("/api/container/install", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ engine: "docker" })
        }).then(r => r.json());
        if (res.installed) {
          showToast("✅ Docker (Rootless) installed and configured!", 4000);
        } else {
          showToast("⚠️ Docker auto-install failed: " + res.message, 6000);
        }
        return res;
      } else if (!status.is_rootless_docker && status.platform_system === "Linux") {
        await triggerRootlessDockerSetup(false);
      }
    }
  } catch (e) {
    console.warn("Container auto-check error:", e);
  }
}


async function triggerRootlessDockerSetup(isManual = false) {
  const statusText = document.getElementById("rootless-status-text");
  const btnSetup = document.getElementById("btn-setup-rootless-docker");
  if (statusText) statusText.textContent = "⚡ Configuring Rootless Docker daemon...";
  if (btnSetup) {
    btnSetup.disabled = true;
    btnSetup.textContent = "Setting up...";
  }

  try {
    const res = await fetch("/api/docker/setup_rootless", { method: "POST" }).then(r => r.json());
    if (statusText) {
      if (res.success || res.is_rootless) {
        statusText.textContent = `✅ ${res.message || "Rootless Docker active!"}`;
        if (!isManual) showToast("🐳 Rootless Docker was automatically configured for this session.");
      } else {
        statusText.textContent = `⚠️ ${res.message || "Setup completed with warnings. Check logs."}`;
      }
    }
    return res;
  } catch (err) {
    if (statusText) statusText.textContent = `⚠️ Setup error: ${err.message}`;
    return { success: false, error: err.message };
  } finally {
    if (btnSetup) {
      btnSetup.disabled = false;
      btnSetup.textContent = "⚡ Setup Rootless Now";
    }
  }
}

async function checkAndAutoSetupRootlessDocker() {
  try {
    const status = await fetch("/api/docker/status").then(r => r.json());
    if (status.platform_system === "Linux" && status.has_docker && !status.is_rootless_docker) {
      console.log("[Linorobot2 Console] Auto-configuring rootless Docker...");
      await triggerRootlessDockerSetup(false);
    }
  } catch (e) {}
}


async function openRootlessModal() {
  const modal = document.getElementById("modal-rootless-docker");
  if (!modal) return;
  modal.style.display = "flex";
  const statusText = document.getElementById("rootless-status-text");
  if (statusText) statusText.textContent = "Status: Checking local container engine...";
  try {
    const res = await fetch("/api/docker/rootless_info");
    const info = await res.json();
    if (statusText) {
      if (info.is_rootless) {
        statusText.textContent = `✅ Rootless Docker is active for user '${info.user}' (UID ${info.uid})`;
      } else if (info.has_docker) {
        statusText.textContent = `⚠️ Docker is running in standard (rootful) mode. Run setup below to enable rootless daemon.`;
      } else if (info.has_podman) {
        statusText.textContent = `✅ Podman is available (Rootless by default, no daemon needed).`;
      } else {
        statusText.textContent = `ℹ️ Neither Docker nor Podman found. Follow setup below to install.`;
      }
    }
  } catch (e) {
    if (statusText) statusText.textContent = "ℹ️ Container status check complete.";
  }
}

function closeRootlessModal() {
  const modal = document.getElementById("modal-rootless-docker");
  if (modal) modal.style.display = "none";
}

function initWorkflowSetup() {
  const hdrDistro = document.getElementById("hdr-distro-select");
  const cfgDistro = document.getElementById("cfg-ros-distro");
  const hdrMode = document.getElementById("hdr-install-mode");
  const tabMode = document.getElementById("install-mode");
  const hdrAgent = document.getElementById("hdr-agent-engine");
  const cfgAgent = document.getElementById("cfg-agent-engine");
  const btnRootlessHdr = document.getElementById("btn-rootless-guide");
  const btnRootlessSettings = document.getElementById("btn-settings-rootless-guide");
  const btnCloseModal = document.getElementById("btn-close-rootless-modal");
  const btnCloseModalFoot = document.getElementById("btn-close-rootless-modal-foot");
  const btnTestDaemon = document.getElementById("btn-test-rootless-daemon");
  const btnCopyUbuntu = document.getElementById("btn-copy-rootless-ubuntu");
  const btnCopyPodman = document.getElementById("btn-copy-rootless-podman");

  // Restore saved choices from localStorage if available
  const savedMode = localStorage.getItem("linorobot2_install_mode");
  if (savedMode) {
    if (hdrMode) hdrMode.value = savedMode;
    if (tabMode) {
      tabMode.value = savedMode;
      const isNative = savedMode === "native";
      const natCards = document.getElementById("install-native-cards");
      const dkrCard = document.getElementById("install-docker-card");
      if (natCards) natCards.style.display = isNative ? "block" : "none";
      if (dkrCard) dkrCard.style.display = isNative ? "none" : "block";
    }
  }

  const savedAgent = localStorage.getItem("linorobot2_agent_engine");
  if (savedAgent) {
    if (hdrAgent) hdrAgent.value = savedAgent;
    if (cfgAgent) cfgAgent.value = savedAgent;
  }

  // 1. Distro Sync
  if (hdrDistro) {
    hdrDistro.addEventListener("change", () => {
      const val = hdrDistro.value;
      if (cfgDistro) cfgDistro.value = val;
      localStorage.setItem("linorobot2_ros_distro", val);
      fetch("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ros_distro: val }),
      }).then(refreshStatus);
    });
  }

  // 2. Install / Execution Mode Sync
  const onModeChange = (mode) => {
    if (hdrMode) hdrMode.value = mode;
    if (tabMode) tabMode.value = mode;
    const isNative = mode === "native";
    const natCards = document.getElementById("install-native-cards");
    const dkrCard = document.getElementById("install-docker-card");
    if (natCards) natCards.style.display = isNative ? "block" : "none";
    if (dkrCard) dkrCard.style.display = isNative ? "none" : "block";
    localStorage.setItem("linorobot2_install_mode", mode);
    ensureContainerEngine(mode);
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ install_mode: mode }),
    }).then(refreshStatus);
  };

  if (hdrMode) {
    hdrMode.addEventListener("change", () => onModeChange(hdrMode.value));
  }
  if (tabMode) {
    tabMode.addEventListener("change", () => onModeChange(tabMode.value));
  }

  // 3. micro-ROS Agent Engine Sync
  const onAgentEngineChange = (engine) => {
    if (hdrAgent) hdrAgent.value = engine;
    if (cfgAgent) cfgAgent.value = engine;
    const chk = document.getElementById("cfg-agent-use-docker");
    if (chk) chk.checked = (engine !== "native");
    localStorage.setItem("linorobot2_agent_engine", engine);
    ensureContainerEngine(engine);
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ agent_engine: engine }),
    }).then(refreshStatus);
  };

  if (hdrAgent) {
    hdrAgent.addEventListener("change", () => onAgentEngineChange(hdrAgent.value));
  }
  if (cfgAgent) {
    cfgAgent.addEventListener("change", () => onAgentEngineChange(cfgAgent.value));
  }

  // 4. Container Registry Sync
  const hdrReg = document.getElementById("hdr-container-registry");
  const hdrCustomReg = document.getElementById("hdr-custom-registry");
  const cfgReg = document.getElementById("cfg-container-registry");
  const cfgCustomReg = document.getElementById("cfg-custom-registry");

  const syncRegistryState = (val, customVal) => {
    const isCustom = (val === "custom");
    if (hdrReg) hdrReg.value = val;
    if (cfgReg) cfgReg.value = val;
    if (hdrCustomReg) {
      if (customVal !== undefined) hdrCustomReg.value = customVal;
      hdrCustomReg.style.display = isCustom ? "inline-block" : "none";
      if (isCustom) hdrCustomReg.focus();
    }
    if (cfgCustomReg) {
      if (customVal !== undefined) cfgCustomReg.value = customVal;
      cfgCustomReg.style.display = isCustom ? "block" : "none";
    }
  };

  const savedReg = localStorage.getItem("linorobot2_container_registry");
  const savedCustomReg = localStorage.getItem("linorobot2_custom_registry");
  if (savedReg) {
    syncRegistryState(savedReg, savedCustomReg || "");
  }

  const onRegistryChange = (val) => {
    syncRegistryState(val);
    localStorage.setItem("linorobot2_container_registry", val);
    const custom = (hdrCustomReg ? hdrCustomReg.value : (cfgCustomReg ? cfgCustomReg.value : "")).trim();
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ container_registry: val, custom_registry: custom }),
    }).then(refreshStatus);
  };

  const onCustomRegistryInput = (custom) => {
    if (hdrCustomReg) hdrCustomReg.value = custom;
    if (cfgCustomReg) cfgCustomReg.value = custom;
    localStorage.setItem("linorobot2_custom_registry", custom);
    fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ custom_registry: custom }),
    });
  };

  if (hdrReg) hdrReg.addEventListener("change", () => onRegistryChange(hdrReg.value));
  if (cfgReg) cfgReg.addEventListener("change", () => onRegistryChange(cfgReg.value));
  if (hdrCustomReg) hdrCustomReg.addEventListener("input", () => onCustomRegistryInput(hdrCustomReg.value));
  if (cfgCustomReg) cfgCustomReg.addEventListener("input", () => onCustomRegistryInput(cfgCustomReg.value));

  // 4. Modal listeners
  if (btnRootlessHdr) btnRootlessHdr.addEventListener("click", openRootlessModal);
  if (btnRootlessSettings) btnRootlessSettings.addEventListener("click", openRootlessModal);
  if (btnCloseModal) btnCloseModal.addEventListener("click", closeRootlessModal);
  if (btnCloseModalFoot) btnCloseModalFoot.addEventListener("click", closeRootlessModal);
  if (btnTestDaemon) btnTestDaemon.addEventListener("click", openRootlessModal);
  const btnSetupRootless = document.getElementById("btn-setup-rootless-docker");
  if (btnSetupRootless) btnSetupRootless.addEventListener("click", () => triggerRootlessDockerSetup(true));
  checkAndAutoSetupRootlessDocker();

  // Copy buttons
  if (btnCopyUbuntu) {
    btnCopyUbuntu.addEventListener("click", () => {
      const code = document.getElementById("code-rootless-ubuntu")?.textContent || "";
      navigator.clipboard.writeText(code).then(() => {
        btnCopyUbuntu.textContent = "✅ Copied!";
        setTimeout(() => { btnCopyUbuntu.textContent = "📋 Copy Script"; }, 2000);
      });
    });
  }
  if (btnCopyPodman) {
    btnCopyPodman.addEventListener("click", () => {
      const code = document.getElementById("code-rootless-podman")?.textContent || "";
      navigator.clipboard.writeText(code).then(() => {
        btnCopyPodman.textContent = "✅ Copied!";
        setTimeout(() => { btnCopyPodman.textContent = "📋 Copy"; }, 2000);
      });
    });
  }
}

// Call initWorkflowSetup on DOM ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initWorkflowSetup);
} else {
  initWorkflowSetup();
}

// =============================================================================
// AUTOSTART ON BOOT CONTROLLER
// =============================================================================
async function refreshAutostartStatus(opts = {}) {
  const pill = document.getElementById("autostart-status-pill");
  const infoBox = document.getElementById("autostart-info-box");
  const summaryText = document.getElementById("autostart-summary-text");
  const detailsText = document.getElementById("autostart-details-text");

  try {
    const res = await fetch("/api/autostart/status").then(r => r.json());
    if (pill) {
      if (res.active) {
        pill.textContent = "active & running";
        pill.className = "pill pill-ok";
      } else if (res.enabled) {
        pill.textContent = "enabled on boot";
        pill.className = "pill pill-starting";
      } else {
        pill.textContent = "disabled";
        pill.className = "pill pill-off";
      }
    }
    if (opts.showInfo && infoBox && summaryText && detailsText) {
      infoBox.style.display = "block";
      summaryText.textContent = `Service: ${res.service_name} | Enabled: ${res.enabled ? "YES" : "NO"} | Active: ${res.active ? "RUNNING" : "STOPPED"} | Lingering: ${res.lingering ? "ENABLED" : "OFF"}`;
      detailsText.textContent = res.details || "No active process status available.";
    }
    return res;
  } catch (err) {
    if (pill) {
      pill.textContent = "check error";
      pill.className = "pill pill-off";
    }
    return { enabled: false, active: false, error: err.message };
  }
}

async function enableBootAutostart() {
  const btn = document.getElementById("btn-autostart-enable");
  const stack = document.getElementById("autostart-stack-select")?.value || "full_nav2";
  const mapPath = document.getElementById("autostart-map-path")?.value || "";
  const distro = getDistro();
  const mode = document.getElementById("hdr-install-mode")?.value || "native";
  const agentEngine = getAgentEngine();

  if (btn) {
    btn.disabled = true;
    btn.textContent = "Enabling...";
  }

  try {
    const res = await fetch("/api/autostart/enable", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        stack,
        map_path: mapPath,
        distro,
        mode,
        agent_engine: agentEngine
      })
    }).then(r => r.json());

    if (res.enabled) {
      const infoBox = document.getElementById("autostart-info-box");
      const summaryText = document.getElementById("autostart-summary-text");
      const detailsText = document.getElementById("autostart-details-text");
      if (infoBox && summaryText && detailsText) {
        infoBox.style.display = "block";
        summaryText.textContent = `✅ ${res.message}`;
        detailsText.textContent = `Unit: ${res.service_path}\nScript: ${res.script_path}\n\nStack is set to launch on power-on automatically.`;
      }
    }
    await refreshAutostartStatus();
  } catch (err) {
    alert("Failed to enable autostart: " + err.message);
  } finally {
    if (btn) {
      btn.disabled = false;
      btn.textContent = "⚡ Enable Boot Autostart";
    }
  }
}

async function disableBootAutostart() {
  const btn = document.getElementById("btn-autostart-disable");
  if (btn) {
    btn.disabled = true;
    btn.textContent = "Disabling...";
  }

  try {
    const res = await fetch("/api/autostart/disable", {
      method: "POST",
      headers: { "Content-Type": "application/json" }
    }).then(r => r.json());

    const infoBox = document.getElementById("autostart-info-box");
    const summaryText = document.getElementById("autostart-summary-text");
    const detailsText = document.getElementById("autostart-details-text");
    if (infoBox && summaryText && detailsText) {
      infoBox.style.display = "block";
      summaryText.textContent = `🛑 ${res.message}`;
      detailsText.textContent = "Autostart on boot has been removed.";
    }
    await refreshAutostartStatus();
  } catch (err) {
    alert("Failed to disable autostart: " + err.message);
  } finally {
    if (btn) {
      btn.disabled = false;
      btn.textContent = "🛑 Disable Autostart";
    }
  }
}

async function viewBootAutostartLogs() {
  const infoBox = document.getElementById("autostart-info-box");
  const summaryText = document.getElementById("autostart-summary-text");
  const detailsText = document.getElementById("autostart-details-text");

  try {
    const res = await fetch("/api/autostart/logs").then(r => r.json());
    if (infoBox && summaryText && detailsText) {
      infoBox.style.display = "block";
      summaryText.textContent = `📜 Journald Logs (linorobot2-autostart.service)`;
      detailsText.textContent = res.logs || "No logs recorded.";
    }
  } catch (err) {
    alert("Failed to read autostart logs: " + err.message);
  }
}

function initAutostartListeners() {
  const btnEnable = document.getElementById("btn-autostart-enable");
  const btnDisable = document.getElementById("btn-autostart-disable");
  const btnStatus = document.getElementById("btn-autostart-status");
  const btnLogs = document.getElementById("btn-autostart-logs");

  if (btnEnable) btnEnable.addEventListener("click", enableBootAutostart);
  if (btnDisable) btnDisable.addEventListener("click", disableBootAutostart);
  if (btnStatus) btnStatus.addEventListener("click", () => refreshAutostartStatus({ showInfo: true }));
  if (btnLogs) btnLogs.addEventListener("click", viewBootAutostartLogs);

  refreshAutostartStatus();
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initAutostartListeners);
} else {
  initAutostartListeners();
}


// ---------- virtual gamepad ----------
// Drives /cmd_vel straight from the page. The server keeps a single rclpy node
// alive (gamepad_publisher.py) and we feed it target velocities; publishing is
// its job, not ours, because a robot has to be told to keep going -- cmd_vel
// that goes quiet means stop. That also gives us the deadman for free: stop
// sending and the node zeroes the robot on its own.
const vgpPad = document.getElementById("vgp-pad");
if (vgpPad) {
  const vgpPost = (url, body) =>
    fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }).then((r) => r.json());

  const vgpKnob = document.getElementById("vgp-knob");
  const vgpStart = document.getElementById("btn-vgp-start");
  const vgpStop = document.getElementById("btn-vgp-stop");
  const vgpState = document.getElementById("vgp-state");
  const SEND_MS = 100;      // 10 Hz to the server; the node republishes at 20 Hz
  const KNOB_TRAVEL = 0.42; // knob centre stays inside the pad at full deflection

  let vgpRunning = false;
  let vgpTimer = null;
  let vgpStallTimer = null;
  let axes = { x: 0, y: 0 };   // -1..1, y positive = forward
  const keysHeld = new Set();

  const num = (id, fallback) => {
    const v = parseFloat(document.getElementById(id).value);
    return Number.isFinite(v) ? v : fallback;
  };

  function twist() {
    return {
      linear_x: axes.y * num("vgp-max-linear", 0.4),
      linear_y: 0,
      angular_z: -axes.x * num("vgp-max-angular", 1.2),
    };
  }

  function render() {
    document.getElementById("vgp-lx").textContent = twist().linear_x.toFixed(2);
    document.getElementById("vgp-az").textContent = twist().angular_z.toFixed(2);
    vgpKnob.style.left = `${50 + axes.x * KNOB_TRAVEL * 100}%`;
    vgpKnob.style.top = `${50 - axes.y * KNOB_TRAVEL * 100}%`;
  }

  function setAxes(x, y) {
    // clamp into the unit circle so a corner drag isn't faster than a straight one
    const mag = Math.hypot(x, y);
    if (mag > 1) { x /= mag; y /= mag; }
    axes = { x, y };
    render();
  }

  async function tick() {
    if (!vgpRunning) return;
    try {
      const r = await vgpPost("/api/gamepad/cmd", twist());
      if (r && r.running === false) stopGamepad("publisher exited");
    } catch (e) {
      stopGamepad("lost contact with Console");
    }
  }

  // Told to move but not moving: driven into something. Checked on the server,
  // which compares the commanded twist against measured odometry, so it fires
  // for a real robot caught on furniture as readily as for a simulated one
  // against a simulated wall. Two consecutive hits, because a single sample
  // during spin-up is just the robot not having accelerated yet.
  let stallHits = 0;
  async function checkStall() {
    if (!vgpRunning) return;
    try {
      const r = await vgpPost("/api/gamepad/stall", {});
      stallHits = (r && r.stalled) ? stallHits + 1 : 0;
      const el = document.getElementById("vgp-stall");
      if (el) {
        const hit = stallHits >= 2;
        el.style.display = hit ? "block" : "none";
        if (hit) {
          const m = r.measured || {};
          el.textContent = `\u26a0 Robot is not moving \u2014 commanded `
            + `${(r.commanded?.linear_x ?? 0).toFixed(2)} m/s but measuring `
            + `${Math.abs(m.linear_x ?? 0).toFixed(2)} m/s. Something is in the way.`;
        }
      }
    } catch (e) { /* transient */ }
  }

  async function startGamepad() {
    const topic = document.getElementById("vgp-topic").value.trim() || "/cmd_vel";
    vgpState.textContent = "starting...";
    const r = await vgpPost("/api/gamepad/start", { topic });
    if (!r || !r.started) {
      vgpState.textContent = "could not start the publisher (is ROS 2 sourced?)";
      return;
    }
    vgpRunning = true;
    vgpStart.disabled = true;
    vgpStop.disabled = false;
    vgpState.textContent = `publishing ${topic}`;
    vgpTimer = setInterval(tick, SEND_MS);
    vgpStallTimer = setInterval(checkStall, 2000);
    vgpPad.focus();
  }

  async function stopGamepad(reason) {
    vgpRunning = false;
    if (vgpTimer) { clearInterval(vgpTimer); vgpTimer = null; }
    if (vgpStallTimer) { clearInterval(vgpStallTimer); vgpStallTimer = null; }
    stallHits = 0;
    const stallEl = document.getElementById("vgp-stall");
    if (stallEl) stallEl.style.display = "none";
    setAxes(0, 0);
    keysHeld.clear();
    vgpStart.disabled = false;
    vgpStop.disabled = true;
    vgpState.textContent = reason || "stopped";
    try { await vgpPost("/api/gamepad/kill", {}); } catch (e) { /* already gone */ }
  }

  vgpStart.addEventListener("click", startGamepad);
  vgpStop.addEventListener("click", () => stopGamepad());

  // ---- pointer drag ----
  function pointerAxes(ev) {
    const r = vgpPad.getBoundingClientRect();
    const nx = (ev.clientX - (r.left + r.width / 2)) / (r.width / 2);
    const ny = (ev.clientY - (r.top + r.height / 2)) / (r.height / 2);
    setAxes(nx, -ny);
  }
  vgpPad.addEventListener("pointerdown", (ev) => {
    vgpPad.setPointerCapture(ev.pointerId);
    vgpPad.classList.add("vgp-active");
    vgpPad.focus();
    pointerAxes(ev);
  });
  vgpPad.addEventListener("pointermove", (ev) => {
    if (vgpPad.hasPointerCapture(ev.pointerId)) pointerAxes(ev);
  });
  const release = (ev) => {
    if (ev.pointerId !== undefined && vgpPad.hasPointerCapture(ev.pointerId)) {
      vgpPad.releasePointerCapture(ev.pointerId);
    }
    vgpPad.classList.remove("vgp-active");
    setAxes(0, 0);   // spring back to centre: let go and the robot stops
  };
  vgpPad.addEventListener("pointerup", release);
  vgpPad.addEventListener("pointercancel", release);

  // ---- keyboard, only while the pad has focus so it can't hijack form typing ----
  const KEY_AXES = {
    ArrowUp: [0, 1], KeyW: [0, 1],
    ArrowDown: [0, -1], KeyS: [0, -1],
    ArrowLeft: [-1, 0], KeyA: [-1, 0],
    ArrowRight: [1, 0], KeyD: [1, 0],
  };
  function applyKeys() {
    let x = 0, y = 0;
    for (const code of keysHeld) { x += KEY_AXES[code][0]; y += KEY_AXES[code][1]; }
    setAxes(Math.max(-1, Math.min(1, x)), Math.max(-1, Math.min(1, y)));
  }
  vgpPad.addEventListener("keydown", (ev) => {
    if (ev.code === "Space") { ev.preventDefault(); stopGamepad("stopped (space)"); return; }
    if (!KEY_AXES[ev.code]) return;
    ev.preventDefault();
    keysHeld.add(ev.code);
    applyKeys();
  });
  vgpPad.addEventListener("keyup", (ev) => {
    if (!KEY_AXES[ev.code]) return;
    keysHeld.delete(ev.code);
    applyKeys();
  });
  vgpPad.addEventListener("blur", () => { keysHeld.clear(); applyKeys(); });

  // a page unload would otherwise leave the robot driving until the deadman trips
  window.addEventListener("pagehide", () => {
    if (vgpRunning) navigator.sendBeacon("/api/gamepad/kill", "{}");
  });

  render();
}
