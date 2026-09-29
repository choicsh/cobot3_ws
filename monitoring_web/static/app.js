const state = {
  snapshot: null,
  lanes: null,
  taskFilter: "all",
  zoom: 1,
  selectedRobot: null,
  activityView: "events",
  stream: null,
  lastMessageAt: null,
};

const robotColors = ["#25d4e8", "#9c83ff", "#ffbd59", "#46d69a", "#ff647c"];

// DB v5 task_status ENUM -> 화면 표기
const TASK_LABELS = {
  WAITING: "대기", ASSIGNED: "배정", PICKING_UP: "픽업중", IN_TRANSIT: "운송중",
  ARRIVED: "도착", COMPLETED: "인계완료", CANCELLED: "취소", FAILED: "실패",
};
// robot_state_history.status / Redis robot:{id}:state.status -> 화면 표기
const STAGE_LABELS = {
  IDLE: "대기", PICK_DOCKING: "채취실 도킹", PICKING: "적재", DELIVERING: "운송중",
  PLACE_DOCKING: "분석실 도킹", PLACING: "하역", RETURNING: "복귀", ERROR: "오류",
};
const PLACE_LABELS = { collection: "검체 채취실", analysis: "검체 분석실" };
const PRIORITY_LABELS = { 3: "긴급", 2: "우선", 1: "일반" };   // 3 이 가장 긴급 (ArUco 상)
const ACTIVE_TASKS = new Set(["WAITING", "ASSIGNED", "PICKING_UP", "IN_TRANSIT", "ARRIVED"]);
const DONE_TASKS = new Set(["COMPLETED", "CANCELLED", "FAILED"]);

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

const taskLabel = (status) => TASK_LABELS[status] || status || "상태 없음";
const stageLabel = (status) => STAGE_LABELS[status] || status || "상태 없음";
const placeLabel = (place) => PLACE_LABELS[place] || place || "—";

function formatTime(value, includeDate = false) {
  if (!value) return "DB에 내용 없음";
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return value;
  return new Intl.DateTimeFormat("ko-KR", {
    month: includeDate ? "2-digit" : undefined,
    day: includeDate ? "2-digit" : undefined,
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
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

function renderSummary(summary = {}, robots = []) {
  const alive = robots.filter((robot) => robot.alive).length;
  $("#metricRobots").textContent = `${alive} / ${summary.totalRobots ?? 0}`;
  $("#metricRobotsSub").textContent = "통신 중 / 등록";
  $("#metricTasks").textContent = summary.activeTasks ?? 0;
  $("#metricUrgent").textContent = summary.urgentTrays ?? 0;
  $("#metricAttention").textContent = summary.attentionTasks ?? 0;
}

// ── 지도 ───────────────────────────────────────────────────────
// 지도 픽셀 좌표 (이미지 왼쪽 위 원점). SVG viewBox 와 마커 % 위치가 같은 좌표계를 쓴다.
function toPixel(x, y, map) {
  return [(x - map.originX) / map.resolution, map.imageHeight - (y - map.originY) / map.resolution];
}

function colorOf(robotId) {
  const robots = state.snapshot?.robots || [];
  const index = robots.findIndex((robot) => robot.robotId === robotId);
  return robotColors[(index < 0 ? 0 : index) % robotColors.length];
}

function polyline(points, map) {
  return points.map(([x, y]) => toPixel(x, y, map).map((v) => v.toFixed(1)).join(",")).join(" ");
}

function renderMap(robots = [], map, zones = {}) {
  const overlay = $("#mapOverlay");
  $("#mapViewport").style.aspectRatio = `${map.imageWidth} / ${map.imageHeight}`;
  const lanes = state.lanes?.zones || {};
  const laneSvg = Object.entries(lanes).map(([id, zone]) =>
    `<polyline class="lane ${zone.shared ? "shared" : ""}" points="${polyline(zone.points, map)}"><title>${escapeHtml(id)}${zone.shared ? " · 충돌 구역(문)" : ""}</title></polyline>`).join("");
  const heldSvg = Object.entries(zones).map(([id, owner]) => {
    const zone = lanes[id];
    if (!zone) return "";
    return `<polyline class="zone-held" style="--zone:${colorOf(owner.robotId)}" points="${polyline(zone.points, map)}"><title>${escapeHtml(id)} · ${escapeHtml(owner.robot)} 예약</title></polyline>`;
  }).join("");
  const routeSvg = robots.filter((robot) => (robot.route || []).length > 1 && robot.alive).map((robot) =>
    `<polyline class="route" style="--zone:${colorOf(robot.robotId)}" points="${polyline(robot.route, map)}"></polyline>`).join("");

  const markers = robots.map((robot) => {
    if (!hasValue(robot.x) || !hasValue(robot.y)) return "";
    const [px, py] = toPixel(Number(robot.x), Number(robot.y), map);
    const left = Math.max(0, Math.min(100, (px / map.imageWidth) * 100));
    const top = Math.max(0, Math.min(100, (py / map.imageHeight) * 100));
    const color = colorOf(robot.robotId);
    const selected = state.selectedRobot === robot.robotId ? "highlight" : "";
    const offline = robot.alive ? "" : "offline";
    const heading = -(Number(robot.theta) || 0) * 180 / Math.PI;   // 지도 yaw(반시계) -> 화면 회전(시계)
    return `<button class="robot-marker ${selected} ${offline}" data-robot-id="${robot.robotId}"
      style="left:${left}%;top:${top}%;--marker:${color}" title="${escapeHtml(robot.name)} · ${escapeHtml(stageLabel(robot.status))}">
      <span class="marker-heading" style="transform:rotate(${heading}deg)"></span>
      <span class="marker-core">${robot.robotId}</span>
      <span class="marker-label">${escapeHtml(robot.name)} · ${escapeHtml(stageLabel(robot.status))}</span>
    </button>`;
  }).join("");

  overlay.innerHTML = `<svg class="lane-layer" viewBox="0 0 ${map.imageWidth} ${map.imageHeight}" preserveAspectRatio="none">${laneSvg}${routeSvg}${heldSvg}</svg>${markers}`;
  if (!robots.length) overlay.insertAdjacentHTML("beforeend", emptyState("표시할 로봇 위치가 없습니다."));
  $("#mapLoading").classList.add("hidden");
  $$(".robot-marker").forEach((marker) => marker.addEventListener("click", () => selectRobot(Number(marker.dataset.robotId))));
}

function renderFleet(robots = [], zones = {}) {
  const list = $("#fleetList");
  $("#robotCount").textContent = `${robots.length}대`;
  if (!robots.length) {
    list.innerHTML = emptyState("등록된 로봇이 없습니다.");
    return;
  }
  list.innerHTML = robots.map((robot) => {
    const color = colorOf(robot.robotId);
    const status = robot.alive ? stageLabel(robot.status) : "통신 끊김";
    const held = Object.entries(zones).filter(([, owner]) => owner.robotId === robot.robotId).map(([id]) => id);
    const taskText = robot.taskId ? `작업 T-${robot.taskId}` : "배정된 작업 없음";
    const coords = hasValue(robot.x) && hasValue(robot.y) ? `X ${Number(robot.x).toFixed(2)} · Y ${Number(robot.y).toFixed(2)}` : "위치 데이터 없음";
    const updated = robot.updatedAt ? new Date(robot.updatedAt * 1000) : robot.recordedAt;
    return `<button class="robot-card ${state.selectedRobot === robot.robotId ? "active" : ""}" data-robot-id="${robot.robotId}" style="--robot:${color}">
      <div class="robot-card-head"><div class="robot-name"><span class="robot-index">R${robot.robotId}</span>${escapeHtml(robot.name)}</div><span class="status-chip" data-status="${escapeHtml(status)}">${escapeHtml(status)}</span></div>
      <p class="robot-task"><strong>${robot.taskId ? `T-${robot.taskId}` : "IDLE"}</strong>${escapeHtml(taskText)} · 예약 구역 ${held.length}개</p>
      <div class="robot-meta"><span><svg viewBox="0 0 24 24"><path d="M12 2a7 7 0 0 0-7 7c0 5.25 7 13 7 13s7-7.75 7-13a7 7 0 0 0-7-7Zm0 9.5A2.5 2.5 0 1 1 12 6a2.5 2.5 0 0 1 0 5.5Z"/></svg>${escapeHtml(coords)}</span><span>${escapeHtml(relativeTime(updated))}</span></div>
    </button>`;
  }).join("");
  $$(".robot-card").forEach((card) => card.addEventListener("click", () => selectRobot(Number(card.dataset.robotId))));
}

function selectRobot(robotId) {
  state.selectedRobot = state.selectedRobot === robotId ? null : robotId;
  if (!state.snapshot) return;
  renderFleet(state.snapshot.robots, state.snapshot.zones);
  renderMap(state.snapshot.robots, state.snapshot.map, state.snapshot.zones);
}

// ── 작업 / 트레이 ─────────────────────────────────────────────
function taskMatchesFilter(task) {
  if (state.taskFilter === "active") return ACTIVE_TASKS.has(task.status);
  if (state.taskFilter === "done") return DONE_TASKS.has(task.status);
  return true;
}

function priorityChip(priority) {
  return `<span class="priority p${priority}">${PRIORITY_LABELS[priority] || `P${priority}`}</span>`;
}

function renderTasks(tasks = []) {
  const body = $("#taskTableBody");
  const filtered = tasks.filter(taskMatchesFilter);
  if (!filtered.length) {
    body.innerHTML = `<tr><td colspan="6">${emptyState(tasks.length ? "선택한 조건의 작업이 없습니다." : "등록된 이송 작업이 없습니다.")}</td></tr>`;
    return;
  }
  body.innerHTML = filtered.map((task) => {
    const trays = task.trays || [];
    const top = trays.reduce((best, tray) => Math.max(best, tray.priority || 0), 0);
    const counts = [3, 2, 1].map((p) => trays.filter((tray) => tray.priority === p).length);
    return `<tr>
    <td><span class="cell-main">T-${task.taskId}</span><span class="cell-sub">${formatTime(task.createdAt)}</span></td>
    <td><span class="cell-main">트레이 ${trays.length}개</span><span class="cell-sub">${trays.map((tray) => `S${tray.slot}`).join(" · ") || "—"}</span></td>
    <td>${top ? priorityChip(top) : "—"}<span class="cell-sub">긴급 ${counts[0]} · 우선 ${counts[1]} · 일반 ${counts[2]}</span></td>
    <td><span class="cell-main">${valueOrEmpty(task.robotName)}</span><span class="cell-sub">${task.departedAt ? `출발 ${formatTime(task.departedAt)}` : "출발 전"}</span></td>
    <td><span class="task-route">${escapeHtml(placeLabel(task.origin))}<svg viewBox="0 0 24 24"><path d="m14 5 7 7-7 7v-4H3v-6h11V5Z"/></svg>${escapeHtml(placeLabel(task.destination))}</span><span class="cell-sub">${task.arrivedAt ? `도착 ${formatTime(task.arrivedAt)}` : ""}</span></td>
    <td><span class="status-chip" data-status="${escapeHtml(taskLabel(task.status))}">${escapeHtml(taskLabel(task.status))}</span>${task.cancelReason ? `<span class="cell-sub">${escapeHtml(task.cancelReason)}</span>` : ""}</td>
  </tr>`;
  }).join("");
}

function eventColor(type, detail = {}) {
  if (type === "STAGE" && detail.stage === "ERROR") return "#ff647c";
  if (type === "UNLOADED" || type === "TASK_CREATED") return "#46d69a";
  if (type === "MISSION_RESUME") return "#ffbd59";
  if (type.includes("COMPLETED")) return "#46d69a";
  if (type.includes("CANCELLED") || type.includes("FAILED")) return "#ff647c";
  if (type.includes("ASSIGNED")) return "#9c83ff";
  return "#25d4e8";
}

function eventText(event) {
  const detail = event.detail || {};
  switch (event.eventType) {
    case "STAGE":
      return [`${stageLabel(detail.stage)}`, detail.detail].filter(Boolean).join(" · ");
    case "TASK_CREATED":
      return `트레이 ${(detail.trays || []).length}개 적재 · 긴급도 ${(detail.priority || []).join(", ")}`;
    case "UNLOADED":
      return `하역 ${(detail.unloaded || []).filter(Boolean).length}개 제자리${(detail.warnings || []).length ? ` · 경고 ${detail.warnings.join("; ")}` : ""}`;
    case "MISSION_RESUME":
      return `주행 이어가기 ${detail.attempt}회 (${placeLabel(detail.origin)} → ${placeLabel(detail.destination)})`;
    default:
      return detail.message || JSON.stringify(detail);
  }
}

const EVENT_TITLES = { STAGE: "단계", TASK_CREATED: "작업 생성", UNLOADED: "하역", MISSION_RESUME: "주행 재시도" };

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
        ? `${taskLabel(log.fromStatus)} → ${taskLabel(log.toStatus)}`
        : `최초 상태 · ${taskLabel(log.toStatus)}`;
      return `<div class="event-item" style="--event-color:${eventColor(log.toStatus || "")}">
        <div class="event-head"><strong>작업 T-${log.taskId} · ${escapeHtml(taskLabel(log.toStatus))}</strong><time title="${escapeHtml(formatTime(log.changedAt, true))}">${escapeHtml(relativeTime(log.changedAt))}</time></div>
        <p>${escapeHtml(transition)}</p>
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
    const extras = hasValue(detail.task_id) ? `작업 T-${detail.task_id}` : "";
    return `<div class="event-item" style="--event-color:${eventColor(event.eventType, detail)}">
      <div class="event-head"><strong>${escapeHtml(event.robotName)} · ${escapeHtml(EVENT_TITLES[event.eventType] || event.eventType)}</strong><time title="${escapeHtml(formatTime(event.occurredAt, true))}">${escapeHtml(relativeTime(event.occurredAt))}</time></div>
      <p>${escapeHtml(eventText(event))}</p>
      ${extras ? `<div class="event-meta">${escapeHtml(extras)}</div>` : ""}
    </div>`;
  }).join("");
}

function renderTrays(trays = []) {
  const grid = $("#trayGrid");
  $("#trayCount").textContent = `${trays.length}건`;
  if (!trays.length) {
    grid.innerHTML = emptyState("등록된 트레이가 없습니다.");
    return;
  }
  grid.innerHTML = trays.map((item) => `<article class="specimen-card">
    <div class="specimen-head"><span class="specimen-id">${escapeHtml(item.trayId)}</span>${priorityChip(item.priority)}</div>
    <p class="specimen-test">${escapeHtml(item.testType)}</p>
    <div class="specimen-info"><div><span>작업</span><strong>${item.taskId ? `T-${item.taskId}` : valueOrEmpty(null)}</strong></div><div><span>랙 칸</span><strong>${hasValue(item.slot) ? `S${item.slot}` : valueOrEmpty(null)}</strong></div><div><span>등록 시각</span><strong>${escapeHtml(formatTime(item.createdAt))}</strong></div><div><span>작업 상태</span><strong>${escapeHtml(taskLabel(item.taskStatus))}</strong></div></div>
  </article>`).join("");
}

function render(snapshot) {
  state.snapshot = snapshot;
  state.lastMessageAt = new Date();
  renderSummary(snapshot.summary, snapshot.robots || []);
  renderMap(snapshot.robots || [], snapshot.map, snapshot.zones || {});
  renderFleet(snapshot.robots || [], snapshot.zones || {});
  renderTasks(snapshot.tasks || []);
  renderEvents(snapshot.events || [], snapshot.statusLogs || []);
  renderTrays(snapshot.trays || []);
  const synced = formatTime(snapshot.serverTime, true);
  $("#mapUpdated").textContent = snapshot.redisOk ? `Redis 실시간 위치 · ${synced}` : `Redis 연결 안 됨 — DB 위치 이력 · ${synced}`;
  $("#footerSync").textContent = `마지막 동기화 ${synced}`;
  setConnection(snapshot.redisOk ? "live" : "error", snapshot.redisOk ? "DB · Redis 실시간 연결" : "Redis 연결 오류");
}

async function loadInitial() {
  try {
    const lanes = await fetch("/api/lanes", { cache: "no-store" });
    if (lanes.ok) state.lanes = await lanes.json();
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
    state.zoom = Math.max(1, Math.min(3, value));
    $("#mapStage").style.transform = `scale(${state.zoom})`;
    $("#zoomReset").textContent = `${Math.round(state.zoom * 100)}%`;
  };
  $("#zoomIn").addEventListener("click", () => setZoom(state.zoom + .25));
  $("#zoomOut").addEventListener("click", () => setZoom(state.zoom - .25));
  $("#zoomReset").addEventListener("click", () => setZoom(1));
}

updateClock();
setInterval(updateClock, 1000);
setupControls();
loadInitial();
connectStream();
