const state = {
  snapshot: null,
  taskFilter: "all",
  zoom: 1,
  selectedRobot: null,
  activityView: "events",
  stream: null,
  lastMessageAt: null,
};

const robotColors = ["#25d4e8", "#9c83ff", "#ffbd59", "#46d69a", "#ff647c"];
const activeStatuses = new Set(["대기", "배정", "픽업중", "운송중", "도착"]);
const doneStatuses = new Set(["인계완료", "취소", "실패"]);

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function hasValue(value) {
  return value !== null && value !== undefined && value !== "";
}

function valueOrEmpty(value) {
  return hasValue(value) ? escapeHtml(value) : '<em class="no-data-inline">DB에 내용 없음</em>';
}

function formatTime(value, includeDate = false) {
  if (!value) return "DB에 내용 없음";
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return value;
  return new Intl.DateTimeFormat("ko-KR", {
    month: includeDate ? "2-digit" : undefined,
    day: includeDate ? "2-digit" : undefined,
    hour: "2-digit",
    minute: "2-digit",
    second: includeDate ? "2-digit" : undefined,
    hour12: false,
  }).format(date);
}

function relativeTime(value) {
  if (!value) return "시간 정보 없음";
  const seconds = Math.round((new Date(value).getTime() - Date.now()) / 1000);
  const abs = Math.abs(seconds);
  if (abs < 10) return "방금 전";
  const formatter = new Intl.RelativeTimeFormat("ko", { numeric: "auto" });
  if (abs < 60) return formatter.format(seconds, "second");
  if (abs < 3600) return formatter.format(Math.round(seconds / 60), "minute");
  if (abs < 86400) return formatter.format(Math.round(seconds / 3600), "hour");
  return formatter.format(Math.round(seconds / 86400), "day");
}

function setConnection(status, message) {
  const el = $("#syncState");
  el.dataset.state = status;
  $("#syncText").textContent = message;
}

function showToast(message) {
  const toast = $("#toast");
  toast.textContent = message;
  toast.classList.add("show");
  clearTimeout(showToast.timer);
  showToast.timer = setTimeout(() => toast.classList.remove("show"), 4500);
}

function emptyState(message) {
  return `<div class="empty-state">${escapeHtml(message)}<br><small>현재 DB에 내용이 없습니다.</small></div>`;
}

function updateClock() {
  const now = new Date();
  $("#currentDate").textContent = new Intl.DateTimeFormat("ko-KR", {
    year: "numeric", month: "2-digit", day: "2-digit", weekday: "short",
  }).format(now);
  $("#currentTime").textContent = now.toLocaleTimeString("ko-KR", { hour12: false });
}

function renderSummary(summary = {}) {
  $("#metricRobots").textContent = `${summary.activeRobots ?? 0} / ${summary.totalRobots ?? 0}`;
  $("#metricRobotsSub").textContent = `활성 / 전체`;
  $("#metricTasks").textContent = summary.activeTasks ?? 0;
  $("#metricUrgent").textContent = summary.urgentSpecimens ?? 0;
  $("#metricAttention").textContent = summary.attentionTasks ?? 0;
}

function worldToPercent(robot, map) {
  if (!hasValue(robot.x) || !hasValue(robot.y)) return null;
  const px = (Number(robot.x) - map.originX) / map.resolution;
  const py = map.imageHeight - (Number(robot.y) - map.originY) / map.resolution;
  return {
    left: Math.max(0, Math.min(100, (px / map.imageWidth) * 100)),
    top: Math.max(0, Math.min(100, (py / map.imageHeight) * 100)),
  };
}

function renderMap(robots = [], map) {
  const overlay = $("#mapOverlay");
  if (!robots.length) {
    overlay.innerHTML = emptyState("표시할 로봇 위치가 없습니다.");
    $("#mapLoading").classList.add("hidden");
    return;
  }
  const markers = robots.map((robot, index) => {
    const point = worldToPercent(robot, map);
    if (!point) return "";
    const color = robotColors[index % robotColors.length];
    const selected = state.selectedRobot === robot.robotId ? "highlight" : "";
    const offline = robot.isActive ? "" : "offline";
    return `<button class="robot-marker ${selected} ${offline}" data-robot-id="${robot.robotId}"
      style="left:${point.left}%;top:${point.top}%;--marker:${color}" title="${escapeHtml(robot.name)} · ${escapeHtml(robot.status || "상태 없음")}">
      <span class="marker-ring"></span><span class="marker-core">${robot.robotId}</span>
      <span class="marker-label">${escapeHtml(robot.name)}</span>
    </button>`;
  }).join("");
  overlay.innerHTML = markers || emptyState("위치 좌표가 저장된 로봇이 없습니다.");
  $("#mapLoading").classList.add("hidden");
  $$(".robot-marker").forEach((marker) => marker.addEventListener("click", () => selectRobot(Number(marker.dataset.robotId))));
}

function renderFleet(robots = []) {
  const list = $("#fleetList");
  $("#robotCount").textContent = `${robots.length}대`;
  if (!robots.length) {
    list.innerHTML = emptyState("등록된 로봇이 없습니다.");
    return;
  }
  list.innerHTML = robots.map((robot, index) => {
    const color = robotColors[index % robotColors.length];
    const status = robot.status || (robot.isActive ? "상태 없음" : "비활성");
    const taskText = robot.taskId ? `작업 #${robot.taskId} · ${robot.taskDestination || "목적지 없음"}` : "배정된 작업 없음";
    const coords = hasValue(robot.x) && hasValue(robot.y) ? `X ${Number(robot.x).toFixed(1)} · Y ${Number(robot.y).toFixed(1)}` : "위치 데이터 없음";
    return `<button class="robot-card ${state.selectedRobot === robot.robotId ? "active" : ""}" data-robot-id="${robot.robotId}" style="--robot:${color}">
      <div class="robot-card-head"><div class="robot-name"><span class="robot-index">R${robot.robotId}</span>${escapeHtml(robot.name)}</div><span class="status-chip" data-status="${escapeHtml(status)}">${escapeHtml(status)}</span></div>
      <p class="robot-task"><strong>${robot.taskId ? `T-${robot.taskId}` : "IDLE"}</strong>${escapeHtml(taskText)}</p>
      <div class="robot-meta"><span><svg viewBox="0 0 24 24"><path d="M12 2a7 7 0 0 0-7 7c0 5.25 7 13 7 13s7-7.75 7-13a7 7 0 0 0-7-7Zm0 9.5A2.5 2.5 0 1 1 12 6a2.5 2.5 0 0 1 0 5.5Z"/></svg>${escapeHtml(coords)}</span><span>${escapeHtml(relativeTime(robot.recordedAt))}</span></div>
    </button>`;
  }).join("");
  $$(".robot-card").forEach((card) => card.addEventListener("click", () => selectRobot(Number(card.dataset.robotId))));
}

function selectRobot(robotId) {
  state.selectedRobot = state.selectedRobot === robotId ? null : robotId;
  if (!state.snapshot) return;
  renderFleet(state.snapshot.robots);
  renderMap(state.snapshot.robots, state.snapshot.map);
}

function taskMatchesFilter(task) {
  if (state.taskFilter === "active") return activeStatuses.has(task.status);
  if (state.taskFilter === "done") return doneStatuses.has(task.status);
  return true;
}

function renderTasks(tasks = []) {
  const body = $("#taskTableBody");
  const filtered = tasks.filter(taskMatchesFilter);
  if (!filtered.length) {
    body.innerHTML = `<tr><td colspan="6">${emptyState(tasks.length ? "선택한 조건의 작업이 없습니다." : "등록된 이송 작업이 없습니다.")}</td></tr>`;
    return;
  }
  body.innerHTML = filtered.map((task) => `<tr>
    <td><span class="cell-main">T-${task.taskId}</span><span class="cell-sub">${formatTime(task.createdAt)}</span></td>
    <td><span class="cell-main">${escapeHtml(task.specimenId)}</span><span class="cell-sub">${escapeHtml(task.testType)}</span></td>
    <td><span class="priority p${task.priority}">${task.priority === 1 ? "긴급" : task.priority === 2 ? "우선" : "일반"}</span></td>
    <td><span class="cell-main">${valueOrEmpty(task.robotName)}</span><span class="cell-sub">${hasValue(task.slotNo) ? `슬롯 ${task.slotNo}` : "슬롯 정보 없음"}</span></td>
    <td><span class="task-route">${escapeHtml(task.origin)}<svg viewBox="0 0 24 24"><path d="m14 5 7 7-7 7v-4H3v-6h11V5Z"/></svg>${escapeHtml(task.destination)}</span></td>
    <td><span class="status-chip" data-status="${escapeHtml(task.status)}">${escapeHtml(task.status)}</span>${task.cancelReason ? `<span class="cell-sub">${escapeHtml(task.cancelReason)}</span>` : ""}</td>
  </tr>`).join("");
}

function eventColor(type) {
  if (type.includes("완료") || type.includes("도착")) return "#46d69a";
  if (type.includes("취소") || type.includes("실패") || type.includes("오류")) return "#ff647c";
  if (type.includes("배정")) return "#9c83ff";
  return "#25d4e8";
}

function renderEvents(events = [], statusLogs = []) {
  const timeline = $("#eventTimeline");
  const isLogView = state.activityView === "logs";
  const items = isLogView ? statusLogs : events;
  $("#eventCount").textContent = `${items.length}건`;
  if (isLogView) {
    if (!statusLogs.length) {
      timeline.innerHTML = emptyState("기록된 작업 상태 로그가 없습니다.");
      return;
    }
    timeline.innerHTML = statusLogs.map((log) => {
      const transition = log.fromStatus
        ? `${log.fromStatus} → ${log.toStatus}`
        : `최초 상태 · ${log.toStatus}`;
      return `<div class="event-item" style="--event-color:${eventColor(log.toStatus || "")}">
        <div class="event-head"><strong>작업 T-${log.taskId} · ${escapeHtml(log.toStatus)}</strong><time title="${escapeHtml(formatTime(log.changedAt, true))}">${escapeHtml(relativeTime(log.changedAt))}</time></div>
        <p>${escapeHtml(transition)}</p>
        <div class="event-meta">변경 주체 · ${escapeHtml(log.changedBy || "DB에 내용 없음")}</div>
      </div>`;
    }).join("");
    return;
  }
  if (!events.length) {
    timeline.innerHTML = emptyState("기록된 로봇 이벤트가 없습니다.");
    return;
  }
  timeline.innerHTML = events.map((event) => {
    const detail = event.detail || {};
    const extras = [
      hasValue(detail.task_id) ? `작업 T-${detail.task_id}` : null,
      hasValue(detail.tray_id) ? detail.tray_id : null,
      hasValue(detail.battery_percent) ? `배터리 ${detail.battery_percent}%` : null,
    ].filter(Boolean).join(" · ");
    return `<div class="event-item" style="--event-color:${eventColor(event.eventType)}">
      <div class="event-head"><strong>${escapeHtml(event.robotName)} · ${escapeHtml(event.eventType)}</strong><time title="${escapeHtml(formatTime(event.occurredAt, true))}">${escapeHtml(relativeTime(event.occurredAt))}</time></div>
      <p>${escapeHtml(detail.message || "이벤트 상세 정보가 DB에 없습니다.")}</p>
      ${extras ? `<div class="event-meta">${escapeHtml(extras)}</div>` : ""}
    </div>`;
  }).join("");
}

function renderSpecimens(specimens = []) {
  const grid = $("#specimenGrid");
  $("#specimenCount").textContent = `${specimens.length}건`;
  if (!specimens.length) {
    grid.innerHTML = emptyState("등록된 검체가 없습니다.");
    return;
  }
  grid.innerHTML = specimens.map((item) => `<article class="specimen-card">
    <div class="specimen-head"><span class="specimen-id">${escapeHtml(item.specimenId)}</span><span class="priority p${item.priority}">P${item.priority}</span></div>
    <p class="specimen-test">${escapeHtml(item.testType)}</p>
    <div class="specimen-info"><div><span>환자 ID</span><strong>${valueOrEmpty(item.patientId)}</strong></div><div><span>트레이</span><strong>${valueOrEmpty(item.trayId)}</strong></div><div><span>채취 시각</span><strong>${escapeHtml(formatTime(item.collectedAt))}</strong></div><div><span>작업 상태</span><strong>${valueOrEmpty(item.taskStatus)}</strong></div></div>
  </article>`).join("");
}

function render(snapshot) {
  state.snapshot = snapshot;
  state.lastMessageAt = new Date();
  renderSummary(snapshot.summary);
  renderMap(snapshot.robots || [], snapshot.map);
  renderFleet(snapshot.robots || []);
  renderTasks(snapshot.tasks || []);
  renderEvents(snapshot.events || [], snapshot.statusLogs || []);
  renderSpecimens(snapshot.specimens || []);
  const synced = formatTime(snapshot.serverTime, true);
  $("#mapUpdated").textContent = `DB 위치 갱신 ${synced}`;
  $("#footerSync").textContent = `마지막 동기화 ${synced}`;
  setConnection("live", "DB 실시간 연결");
}

async function loadInitial() {
  try {
    const response = await fetch("/api/dashboard", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch (error) {
    setConnection("error", "DB 연결 오류");
    showToast(`초기 데이터 조회 실패: ${error.message}`);
    $("#mapLoading p").textContent = "DB 연결을 확인해 주세요";
  }
}

function connectStream() {
  if (state.stream) state.stream.close();
  const stream = new EventSource("/api/stream");
  state.stream = stream;
  stream.addEventListener("open", () => setConnection("live", "DB 실시간 연결"));
  stream.addEventListener("dashboard", (event) => {
    try { render(JSON.parse(event.data)); }
    catch { showToast("실시간 데이터 형식을 해석하지 못했습니다."); }
  });
  stream.addEventListener("db-error", (event) => {
    setConnection("error", "DB 조회 오류");
    try { showToast(JSON.parse(event.data).message); } catch { showToast("DB 조회 중 오류가 발생했습니다."); }
  });
  stream.onerror = () => setConnection("connecting", "재연결 중");
}

function setupControls() {
  $$("[data-task-filter]").forEach((button) => button.addEventListener("click", () => {
    state.taskFilter = button.dataset.taskFilter;
    $$("[data-task-filter]").forEach((item) => item.classList.toggle("active", item === button));
    renderTasks(state.snapshot?.tasks || []);
  }));

  $$("[data-activity-view]").forEach((button) => button.addEventListener("click", () => {
    state.activityView = button.dataset.activityView;
    $$("[data-activity-view]").forEach((item) => item.classList.toggle("active", item === button));
    renderEvents(state.snapshot?.events || [], state.snapshot?.statusLogs || []);
  }));

  const setZoom = (value) => {
    state.zoom = Math.max(1, Math.min(2.2, value));
    $("#mapStage").style.transform = `scale(${state.zoom})`;
    $("#zoomReset").textContent = `${Math.round(state.zoom * 100)}%`;
  };
  $("#zoomIn").addEventListener("click", () => setZoom(state.zoom + .2));
  $("#zoomOut").addEventListener("click", () => setZoom(state.zoom - .2));
  $("#zoomReset").addEventListener("click", () => setZoom(1));
}

updateClock();
setInterval(updateClock, 1000);
setupControls();
loadInitial();
connectStream();
