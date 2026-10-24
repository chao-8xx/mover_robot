/* All processing happens in the local backend; the browser only displays and exports it. */
"use strict";

const $ = id => document.getElementById(id);
const COLORS = ["#57d5bf", "#74adf5", "#e3bd75", "#cba0e8", "#f590a1", "#8fcd84", "#85cadf", "#d8a283"];
const REASONS = {
  kept: "保留", wheel: "轮区遮挡", invalid: "原始点无效", distance: "距离超限",
  too_few_points: "点数不足", capacity: "聚类数量超限", no_pose: "定位无效",
  invalid_pose: "定位无效", pose_invalid: "定位无效", outside: "场地范围外",
  outside_map: "场地范围外", forbidden: "落入禁区", node: "节点区域",
  no_road: "未命中道路", accepted: "地图已接受", candidate: "等待多圈确认",
  confirmed: "已确认", unknown: "未知", free: "畅通", blocked: "阻塞", road: "道路", updated: "本圈已更新",
  pose_not_valid: "定位无效", no_clusters: "本圈无聚类", not_ready: "等待有效定位",
  invalid_coordinates: "坐标无效", outside_field: "场地范围外", static_forbidden: "静态禁区",
  not_processed: "未处理", map_not_ready: "地图未初始化", invalid_scan_metadata: "扫描编号或时间无效",
  too_many_clusters: "聚类数量过多", stale_scan: "重复或倒序扫描", cluster_rejected: "聚类被拒绝",
  no_affected_road: "未影响道路", duplicate_this_scan: "本圈重复候选", obstacle_capacity: "障碍容量已满"
};
const state = {
  info: null, recording: null, originalFrames: [], frames: [], datasetId: null,
  globalPose: null, editedFrames: false, params: {}, index: 0, frame: null,
  baseline: null, revision: 0, busy: false, pending: false, playing: false,
  playTimer: null, debounceTimer: null, toastTimer: null, selectedAngle: null,
  range: 2000, pointTransform: null, drawPoints: [], importGeneration: 0, paramsValid: true, exporting: false
};
const copy = value => typeof structuredClone === "function" ? structuredClone(value) : JSON.parse(JSON.stringify(value));
const numeric = (value, fallback = 0) => Number.isFinite(Number(value)) ? Number(value) : fallback;
const format = (value, decimals = 0) => Number.isFinite(Number(value)) ? Number(value).toFixed(decimals) : "—";
const reason = value => REASONS[value] || value || "—";
const coord = value => value && Number.isFinite(Number(value.x)) && Number.isFinite(Number(value.y)) ? {x: Number(value.x), y: Number(value.y)} : null;
const colorFor = id => COLORS[Math.abs(numeric(id)) % COLORS.length];

async function api(path, body) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 30000);
  try {
    const response = await fetch(path, {
      method: body === undefined ? "GET" : "POST",
      headers: body === undefined ? {} : {"Content-Type": "application/json"},
      body: body === undefined ? undefined : JSON.stringify(body), signal: controller.signal
    });
    let result;
    try { result = await response.json(); } catch { throw new Error(`本地服务返回了无法读取的内容（${response.status}）。`); }
    if (!response.ok || result.error) throw new Error(typeof result.error === "string" ? result.error : result.message || `处理失败（${response.status}）`);
    return result;
  } catch (error) {
    if (error.name === "AbortError") throw new Error("处理超过 30 秒。请减少记录长度后重试。");
    if (error instanceof TypeError) throw new Error("无法连接本地服务。请保持启动窗口打开，再刷新页面。");
    throw error;
  } finally { clearTimeout(timeout); }
}
function showError(error) { $("error-text").textContent = error.message || String(error); $("error").hidden = false; }
function toast(message) { $("toast").textContent = message; $("toast").hidden = false; clearTimeout(state.toastTimer); state.toastTimer = setTimeout(() => { $("toast").hidden = true; }, 3200); }
function element(tag, className, text) { const node = document.createElement(tag); if (className) node.className = className; if (text !== undefined) node.textContent = text; return node; }

function defaultParams() {
  const defaults = state.info?.defaults || {};
  return copy(defaults.params || defaults);
}
function populateParams() {
  document.querySelectorAll("[data-param]").forEach(input => { input.value = state.params[input.dataset.param] ?? ""; input.setCustomValidity(""); });
  $("mask-enabled").checked = state.params.mask_enabled !== false;
  const tf = state.params.tf || {};
  $("tf-x").value = tf.tx ?? 0; $("tf-y").value = tf.ty ?? 0; $("tf-yaw").value = tf.yaw_deg ?? 0; $("tf-mirror").checked = !!tf.mirror_x;
  const container = $("wheel-ranges"); container.replaceChildren();
  const ranges = state.params.wheel_ranges || [];
  ranges.forEach((range, index) => {
    const row = element("div", "wheel-row"); row.append(element("span", "", `轮区 ${index + 1}`));
    [0, 1].forEach((side, position) => {
      if (position) row.append(element("span", "", "→"));
      const input = element("input"); input.type = "number"; input.min = "0"; input.max = "359"; input.step = "1";
      input.value = range[side]; input.setAttribute("aria-label", `轮区 ${index + 1} ${side ? "终点" : "起点"}角度`);
      input.dataset.wheel = index; input.dataset.side = side; input.addEventListener("input", handleParams);
      row.append(input);
    });
    container.append(row);
  });
}
function handleParams() {
  pause(); $("export-button").disabled = true;
  const inputs = [...document.querySelectorAll("[data-param], [data-wheel], #tf-x, #tf-y, #tf-yaw, #car-width, #safety")];
  if (inputs.some(input => input.value === "" || !Number.isFinite(Number(input.value)) || !input.checkValidity())) {
    state.paramsValid = false; state.revision++; state.pending = false;
    $("processing").textContent = "参数未生效"; $("processing").title = "请填写符合范围和步长的数值。图中仍是上次有效结果。";
    return;
  }
  const next = copy(state.params);
  document.querySelectorAll("[data-param]").forEach(input => { next[input.dataset.param] = Number(input.value); });
  if (next.min_distance >= next.max_distance) {
    state.paramsValid = false; state.revision++; state.pending = false;
    $("processing").textContent = "参数未生效";
    $("processing").title = "最小距离必须小于最大距离。图中仍是上次有效结果。";
    return;
  }
  next.mask_enabled = $("mask-enabled").checked;
  next.tf = {tx: Number($("tf-x").value), ty: Number($("tf-y").value), yaw_deg: Number($("tf-yaw").value), mirror_x: $("tf-mirror").checked};
  next.wheel_ranges = (next.wheel_ranges || []).map(range => [...range]);
  document.querySelectorAll("[data-wheel]").forEach(input => { next.wheel_ranges[Number(input.dataset.wheel)][Number(input.dataset.side)] = Number(input.value); });
  state.params = next; state.paramsValid = true; scheduleReplay(220);
}
function effectiveFrame(index = state.index) {
  const frame = state.frames[index];
  return state.globalPose ? {...frame, pose: copy(state.globalPose)} : frame;
}
function populatePose() {
  const pose = effectiveFrame()?.pose || {};
  $("pose-x").value = numeric(pose.x); $("pose-y").value = numeric(pose.y); $("pose-yaw").value = numeric(pose.yaw_deg); $("pose-valid").checked = pose.valid === true;
  $("pose-origin").textContent = state.globalPose ? "手动定位 · 已应用到整段" : state.editedFrames ? "本圈定位 · 含手动修改" : "记录内定位";
}
function readPose() {
  const inputs = [$("pose-x"), $("pose-y"), $("pose-yaw")];
  if (inputs.some(input => input.value === "" || !Number.isFinite(Number(input.value)))) throw new Error("请填写有效的位置和朝向数值。");
  return {x: Number($("pose-x").value), y: Number($("pose-y").value), yaw_deg: Number($("pose-yaw").value), valid: $("pose-valid").checked};
}
function applyPose(all) {
  if (!state.frames.length) return;
  try {
    const pose = readPose(); pause();
    if (all) state.globalPose = pose;
    else {
      if (state.globalPose) { state.frames = state.frames.map(frame => ({...copy(frame), pose: copy(state.globalPose)})); state.globalPose = null; }
      state.frames[state.index] = {...copy(state.frames[state.index]), pose}; state.editedFrames = true;
    }
    populatePose(); scheduleReplay(); toast(all ? "手动定位已应用到整段记录" : `手动定位已应用到第 ${state.index + 1} 圈`);
  } catch (error) { showError(error); }
}
function renderSource() {
  const source = state.info?.source || {}; const container = $("source-content"); container.replaceChildren();
  container.append(element("div", "source-line", `分支：${source.branch || "未提供"}    提交：${source.commit || "未提供"}`));
  const files = source.files || source.hashes || {};
  if (Array.isArray(files)) files.forEach(file => container.append(element("div", "source-line", typeof file === "string" ? file : `${file.path || file.name || "文件"} · ${file.sha256 || file.hash || ""}`)));
  else Object.entries(files).forEach(([path, hash]) => container.append(element("div", "source-line", `${path} · ${typeof hash === "string" ? hash : hash.sha256 || hash.hash || ""}`)));
  if (source.dirty) container.append(element("div", "source-line warning-text", "算法来源包含本地未提交修改，以显示的文件指纹为准。"));
  if (state.info?.limits) container.append(element("div", "source-line", `载入限制：${typeof state.info.limits === "string" ? state.info.limits : JSON.stringify(state.info.limits)}`));
}
function setRecording(recording, filename) {
  const data = recording.recording || recording;
  if (!Array.isArray(data.frames) || !data.frames.length) throw new Error("记录中没有可回放的完整点云帧。");
  pause(); state.revision++; state.recording = data; state.originalFrames = copy(data.frames); state.frames = copy(data.frames);
  state.datasetId = recording.dataset_id || data.dataset_id || null;
  state.index = Number.isInteger(data.problem_frame_index) && data.problem_frame_index >= 0 && data.problem_frame_index < data.frames.length ? data.problem_frame_index : 0;
  state.frame = null; state.baseline = null;
  state.globalPose = null; state.editedFrames = false; state.selectedAngle = null;
  if (data.params) {
    const defaults = defaultParams(); state.params = {...defaults, ...copy(data.params), tf: {...defaults.tf, ...copy(data.params.tf || {})}}; populateParams();
  }
  if (data.car_width_mm !== undefined) $("car-width").value = data.car_width_mm;
  if (data.safety_mm !== undefined) $("safety").value = data.safety_mm;
  if (data.compare !== undefined) $("compare-toggle").checked = data.compare === true;
  state.paramsValid = true;
  $("recording-name").textContent = filename || data.label || "雷达记录";
  $("recording-meta").textContent = `${data.frames.length} 圈 · ${data.format || "gongxun-lidar-v1"}`;
  $("frame-slider").max = data.frames.length - 1; $("frame-slider").value = state.index; $("frame-slider").disabled = false;
  $("play-button").disabled = false; $("export-button").disabled = true;
  const synthetic = data.synthetic === true;
  $("dataset-banner").className = `notice ${synthetic ? "synthetic" : "real"}`;
  $("banner-title").textContent = synthetic ? "当前为合成演示数据" : "当前为导入记录 · 来源按文件声明";
  $("banner-detail").textContent = synthetic ? "用它检查筛选、聚类与地图流程；不能证明 D6 能检测到真实细杆。" : "回放结论取决于记录完整性、安装标定和定位。实际探测能力仍需现场测量。";
  $("dataset-tag").textContent = synthetic ? "合成演示" : "文件记录";
  $("error").hidden = true; populatePose(); updatePlayback(); drawAll(); scheduleReplay();
}
function updatePlayback() {
  const count = state.frames.length, raw = effectiveFrame() || {};
  $("frame-label").textContent = count ? `第 ${state.index + 1} / ${count} 圈` : "第 — 圈";
  const parts = [];
  if (raw.scan_num !== undefined) parts.push(`扫描号 ${raw.scan_num}`);
  if (raw.tick !== undefined) parts.push(`时间 ${raw.tick} ms`);
  if (raw.label) parts.push(String(raw.label));
  $("frame-detail").textContent = parts.join(" · ") || "逐圈查看处理结果";
  $("frame-slider").value = state.index;
  $("previous-button").disabled = !count || state.index === 0;
  $("next-button").disabled = !count || state.index >= count - 1;
  $("play-button").textContent = state.playing ? "Ⅱ 暂停" : "▶ 播放";
}
function selectFrame(index, manual = true) {
  if (!state.frames.length) return;
  if (manual) pause();
  state.index = Math.max(0, Math.min(state.frames.length - 1, Number(index)));
  state.selectedAngle = null; populatePose(); updatePlayback(); scheduleReplay();
}
function pause() { state.playing = false; clearTimeout(state.playTimer); state.playTimer = null; updatePlayback(); }
function advanceWhenReady() {
  clearTimeout(state.playTimer);
  if (!state.playing) return;
  if (state.index >= state.frames.length - 1) { pause(); toast("已到记录末尾"); return; }
  const delay = 1000 / Number($("play-speed").value);
  state.playTimer = setTimeout(() => { if (state.playing) selectFrame(state.index + 1, false); }, delay);
}
function togglePlay() {
  if (!state.frames.length) return;
  if (!state.paramsValid) { toast("参数尚未生效，请先修正数值"); return; }
  if (state.playing) { pause(); return; }
  state.playing = true;
  if (state.index >= state.frames.length - 1) selectFrame(0, false);
  else if (!state.busy && !state.pending) advanceWhenReady();
  updatePlayback();
}
function scheduleReplay(delay = 0) {
  if (!state.recording) return;
  if (!state.paramsValid) { $("processing").textContent = "参数未生效"; return; }
  state.revision++; state.pending = true; $("export-button").disabled = true;
  clearTimeout(state.debounceTimer); clearTimeout(state.playTimer);
  state.debounceTimer = setTimeout(runReplay, delay);
}
async function runReplay() {
  if (state.busy || !state.pending || !state.frames.length || !state.paramsValid) return;
  state.busy = true; state.pending = false;
  const revision = state.revision, index = state.index;
  $("processing").textContent = "处理中…"; $("processing").title = "";
  const body = {index, params: copy(state.params), car_width_mm: Number($("car-width").value), safety_mm: Number($("safety").value), compare: $("compare-toggle").checked};
  if (state.datasetId && !state.editedFrames) body.dataset_id = state.datasetId;
  else body.frames = state.frames;
  if (state.globalPose) body.pose_override = copy(state.globalPose);
  try {
    const response = await api("/api/replay", body);
    if (revision !== state.revision) return;
    state.frame = response.frame; state.baseline = response.baseline || null;
    if (!state.frame || !Array.isArray(state.frame.points)) throw new Error("回放结果缺少点云，请检查本地服务版本。");
    $("export-button").disabled = state.exporting; $("processing").textContent = "已更新";
    renderResult(); drawAll();
  } catch (error) {
    if (revision === state.revision) { pause(); showError(error); $("processing").textContent = "处理失败"; }
  } finally {
    state.busy = false;
    if (state.pending) runReplay(); else if (revision === state.revision) advanceWhenReady();
  }
}
function resultCounts(result) {
  const points = result?.points || [];
  return {valid: points.filter(p => p.valid === true || p.valid === 1).length, wheel: points.filter(p => p.reason === "wheel" && (p.valid === true || p.valid === 1)).length,
    kept: points.filter(p => p.reason === "kept").length, distance: points.filter(p => p.reason === "distance").length,
    clusters: (result?.clusters || []).length, rejected: (result?.rejected || []).length,
    confirmed: (result?.map_state?.obstacles || []).filter(o => o.state === "confirmed" || o.confirmed === true).length};
}
function mapActive(frame = state.frame) { return !!frame?.map_state?.ready && (frame?.pose || effectiveFrame()?.pose)?.valid === true; }
function renderResult() {
  const frame = state.frame, counts = resultCounts(frame), map = frame.map_state || {}, active = mapActive();
  $("stat-valid").textContent = counts.valid; $("stat-wheel").textContent = counts.wheel;
  $("stat-kept").textContent = counts.kept; $("stat-distance").textContent = `${counts.distance} 点距离超限`;
  $("stat-clusters").textContent = counts.clusters; $("stat-rejected").textContent = `${counts.rejected} 个被拒绝`;
  $("stat-obstacles").textContent = counts.confirmed; $("stat-map").textContent = active ? "经多圈确认" : "等待有效定位";
  $("map-ready").textContent = active ? "定位有效" : "等待有效定位"; $("map-ready").className = `tag ${active ? "success" : "muted"}`;
  $("map-empty").hidden = active; $("map-change").textContent = `地图变化 ${map.change_num ?? "—"}`;
  $("map-reason").textContent = map.reason ? reason(map.reason) : map.updated ? "本圈已更新" : "本圈未更新";
  $("comparison").hidden = !state.baseline;
  if (state.baseline) {
    const base = resultCounts(state.baseline);
    $("comparison-detail").textContent = `保留点 ${base.kept} → ${counts.kept}　聚类 ${base.clusters} → ${counts.clusters}　确认障碍 ${base.confirmed} → ${counts.confirmed}`;
  }
  renderCandidates(); renderMapState(); renderWarnings(); renderInspector();
}
function clusterCenter(cluster) { return coord(cluster.center_point) || coord(cluster.center) || {x: (numeric(cluster.min_x) + numeric(cluster.max_x)) / 2, y: (numeric(cluster.min_y) + numeric(cluster.max_y)) / 2}; }
function clusterBox(cluster) {
  return {min_x: numeric(cluster.min_x, clusterCenter(cluster).x), max_x: numeric(cluster.max_x, clusterCenter(cluster).x), min_y: numeric(cluster.min_y, clusterCenter(cluster).y), max_y: numeric(cluster.max_y, clusterCenter(cluster).y)};
}
// Firmware clustering uses field coordinates. Derive local bounds from its member
// angles for the local plot, keeping all cluster membership decisions unchanged.
function localCluster(cluster, frame = state.frame, system = "body") {
  const angles = new Set(cluster.angles || []);
  const members = (frame?.points || []).filter(point => angles.has(point.angle)).map(point => localCoord(point, system)).filter(Boolean);
  if (!members.length) return cluster;
  const minX = Math.min(...members.map(point => point.x)), maxX = Math.max(...members.map(point => point.x));
  const minY = Math.min(...members.map(point => point.y)), maxY = Math.max(...members.map(point => point.y));
  return {...cluster, center_point: {x: members.reduce((sum, point) => sum + point.x, 0) / members.length, y: members.reduce((sum, point) => sum + point.y, 0) / members.length},
    min_x: minX, max_x: maxX, min_y: minY, max_y: maxY, width: maxX - minX, height: maxY - minY};
}
function cellText(cell) {
  if (!cell) return "—";
  const prefix = cell.type === "node" ? "N" : cell.type === "road" ? "R" : "";
  return `${cell.type ? reason(cell.type) + " " : ""}${cell.id !== null && cell.id !== undefined ? prefix + cell.id + " " : ""}(${cell.x ?? "?"}, ${cell.y ?? "?"})`;
}
function renderCandidates() {
  const tbody = $("candidate-table"), clusters = state.frame?.clusters || [], rejected = state.frame?.rejected || [];
  const candidates = state.frame?.map_state?.candidates || [];
  const rows = [...clusters.map(cluster => ({cluster, rejected: false})), ...rejected.map(cluster => ({cluster, rejected: true}))];
  tbody.replaceChildren(); $("candidate-count").textContent = `${clusters.length} 个有效 / ${rejected.length} 个拒绝`;
  if (!rows.length) { const row = element("tr", "empty-row"), cell = element("td", "empty-cell", "本圈没有形成聚类，可查看点数、连接距离和允许缺点数"); cell.colSpan = 8; row.append(cell); tbody.append(row); return; }
  rows.forEach(({cluster, rejected: rejectedFlag}) => {
    const id = cluster.id ?? cluster.cluster_id;
    const candidate = rejectedFlag ? null : candidates.find(c => c.cluster_id === id || c.id === id);
    const local = localCluster(cluster), center = clusterCenter(local), fieldCenter = clusterCenter(cluster), box = clusterBox(local), row = element("tr");
    const labelCell = element("td"), label = element("span", "cluster-label", id === undefined ? "候选" : rejectedFlag ? `拒 ${id}` : `#${id}`);
    const dot = element("i"); dot.style.background = rejectedFlag ? "#ef7c85" : colorFor(id); label.prepend(dot); labelCell.append(label); row.append(labelCell);
    row.append(element("td", "", cluster.point_count ?? cluster.points?.length ?? "—"));
    row.append(element("td", "", `${format(center.x)}, ${format(center.y)}`));
    row.append(element("td", "cell-label", mapActive() ? `${format(fieldCenter.x)}, ${format(fieldCenter.y)}` : "定位无效"));
    row.append(element("td", "", `${format(local.width ?? box.max_x - box.min_x)} × ${format(local.height ?? box.max_y - box.min_y)}`));
    row.append(element("td", "", `${format(cluster.nearest_distance)} mm`));
    let text, cls;
    if (rejectedFlag) { text = reason(cluster.reason); cls = "reject"; }
    else if (!mapActive()) { text = "聚类有效 · 未映射"; cls = "pending"; }
    else if (candidate?.accepted) { text = reason(candidate.reason || "accepted"); cls = "accept"; }
    else if (candidate) { text = reason(candidate.reason || "no_road"); cls = "reject"; }
    else { text = "未提供映射诊断"; cls = "cell-label"; }
    row.append(element("td", cls, text));
    const roads = candidate?.road_ids || [];
    row.append(element("td", "cell-label", rejectedFlag ? "未提交地图" : !mapActive() ? "定位无效" : `${cellText(candidate?.cell)}${roads.length ? " / " + roads.map(roadLabel).join(", ") : ""}`));
    if (id !== undefined) { row.style.cursor = "pointer"; row.title = "点击聚焦这个聚类中的点"; row.addEventListener("click", () => {
      const point = rejectedFlag ? state.frame.points.find(p => (cluster.angles || []).includes(p.angle)) : state.frame.points.find(p => p.cluster_id === id);
      if (point) { state.selectedAngle = point.angle; renderInspector(); drawPointCloud(); }
    }); }
    tbody.append(row);
  });
}
function roadLabel(id) { return String(id).startsWith("R") ? String(id) : `R${id}`; }
function renderMapState() {
  const map = state.frame?.map_state || {}, roads = map.roads || [], obstacles = map.obstacles || [];
  $("road-list").replaceChildren(); roads.forEach(road => $("road-list").append(element("span", `road-chip ${road.state || "unknown"}`, `${roadLabel(road.id)} ${reason(road.state || "unknown")}`)));
  const container = $("obstacle-list"); container.replaceChildren();
  if (!obstacles.length) { container.append(element("span", "", mapActive() ? "当前没有保存的障碍。拖动到后面的圈，查看确认、保留与清除过程。" : "定位无效时，障碍不会写入场地地图。")); return; }
  obstacles.forEach((obstacle, index) => {
    const row = element("div", "obstacle-row"), box = normalizeBox(obstacle.inflated || obstacle.inflated_box || obstacle.box || obstacle);
    row.append(element("strong", "", `障碍 ${obstacle.id ?? index + 1}`));
    row.append(element("span", obstacle.state === "confirmed" ? "accept" : "pending", reason(obstacle.state || "candidate")));
    if (obstacle.hits !== undefined || obstacle.hit_count !== undefined || obstacle.seen_num !== undefined) row.append(element("span", "", `命中 ${obstacle.hits ?? obstacle.hit_count ?? obstacle.seen_num} 次`));
    if (box) row.append(element("span", "", `膨胀范围 (${format(box.min_x)}, ${format(box.min_y)}) — (${format(box.max_x)}, ${format(box.max_y)})`));
    const roadIds = obstacle.road_ids || []; if (roadIds.length) row.append(element("span", "", roadIds.map(roadLabel).join(" / ")));
    container.append(row);
  });
}
function renderWarnings() {
  const warnings = [...(state.info?.warnings || []), ...(state.frame?.warnings || [])], container = $("warnings"); container.replaceChildren();
  const entries = Array.isArray(warnings) ? warnings : Object.entries(warnings).filter(([, value]) => value).map(([key, value]) => `${key}: ${value}`);
  container.hidden = !entries.length; if (!entries.length) return;
  container.append(element("strong", "", "记录与处理提示"));
  entries.forEach(warning => container.append(element("div", "", typeof warning === "string" ? warning : warning.message || JSON.stringify(warning))));
}
function renderInspector() {
  const point = state.frame?.points?.find(p => p.angle === state.selectedAngle), container = $("point-inspector"); container.replaceChildren();
  $("point-title").textContent = point ? `${point.angle}°${point.cluster_id !== null && point.cluster_id !== undefined && point.cluster_id >= 0 ? ` · 聚类 #${point.cluster_id}` : ""}` : "点击点云中的点";
  const body = coord(point?.body), map = mapActive() ? coord(point?.map) : null;
  const values = [["原始角度", point ? `${format(point.angle, 1)}°` : "—"], ["距离", point ? `${format(point.distance)} mm` : "—"],
    ["筛选结果", point ? reason(point.reason) : "—"], ["车体 (X, Y)", body ? `${format(body.x)}, ${format(body.y)}` : "—"], ["场地 (X, Y)", map ? `${format(map.x)}, ${format(map.y)}` : "—"]];
  values.forEach(([label, value]) => { const item = element("div"); item.append(element("dt", "", label), element("dd", "", value)); container.append(item); });
}

function setupCanvas(canvas) {
  const rect = canvas.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, rect.width), height = Math.max(1, rect.height);
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) { canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr); }
  const context = canvas.getContext("2d"); context.setTransform(dpr, 0, 0, dpr, 0, 0); context.clearRect(0, 0, width, height);
  context.lineCap = "round"; context.lineJoin = "round"; context.font = "10px 'Segoe UI', 'Microsoft YaHei', sans-serif";
  return {context, width, height};
}
function niceStep(value) { const base = 10 ** Math.floor(Math.log10(Math.max(value, 1))); const fraction = value / base; return (fraction <= 1 ? 1 : fraction <= 2 ? 2 : fraction <= 5 ? 5 : 10) * base; }
function drawDot(ctx, x, y, radius, fill) { ctx.beginPath(); ctx.arc(x, y, radius, 0, Math.PI * 2); ctx.fillStyle = fill; ctx.fill(); }
function line(ctx, a, b, stroke, width = 1, dashed = false) { ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.setLineDash(dashed ? [4, 4] : []); ctx.stroke(); ctx.setLineDash([]); }
function drawWorldBox(ctx, transform, box, stroke, fill, dashed = false) {
  if (!box) return;
  const a = transform({x: box.min_x, y: box.max_y}), b = transform({x: box.max_x, y: box.min_y});
  ctx.fillStyle = fill; ctx.strokeStyle = stroke; ctx.lineWidth = 1.1; ctx.setLineDash(dashed ? [4, 4] : []);
  ctx.fillRect(a.x, a.y, Math.max(2, b.x - a.x), Math.max(2, b.y - a.y)); ctx.strokeRect(a.x, a.y, Math.max(2, b.x - a.x), Math.max(2, b.y - a.y)); ctx.setLineDash([]);
}
function localCoord(point, system) {
  const provided = coord(point[system]); if (provided) return provided;
  if (system === "lidar" && numeric(point.distance) > 0) { const angle = numeric(point.angle) * Math.PI / 180; return {x: -numeric(point.distance) * Math.sin(angle), y: numeric(point.distance) * Math.cos(angle)}; }
  return null;
}
function drawPointCloud() {
  const {context: ctx, width, height} = setupCanvas($("point-canvas"));
  const range = state.range, scale = Math.min(width - 56, height - 58) / (2 * range), cx = width / 2, cy = height / 2;
  const transform = point => ({x: cx + point.x * scale, y: cy - point.y * scale}); state.pointTransform = transform; state.drawPoints = [];
  const step = niceStep(range / 3.5), grid = "#203246";
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  for (let value = -Math.floor(range / step) * step; value <= range; value += step) {
    if (value === 0) continue;
    line(ctx, transform({x: value, y: -range}), transform({x: value, y: range}), grid);
    line(ctx, transform({x: -range, y: value}), transform({x: range, y: value}), grid);
    ctx.fillStyle = "#526d84"; const xPos = transform({x: value, y: 0}), yPos = transform({x: 0, y: value});
    ctx.fillText(String(value), xPos.x, cy + 12); ctx.textAlign = "right"; ctx.fillText(String(value), cx - 6, yPos.y); ctx.textAlign = "center";
  }
  for (let radius = step; radius <= range; radius += step) { ctx.beginPath(); ctx.arc(cx, cy, radius * scale, 0, 2 * Math.PI); ctx.strokeStyle = "#1a2c3e"; ctx.stroke(); }
  line(ctx, transform({x: -range, y: 0}), transform({x: range, y: 0}), "#37516a");
  line(ctx, transform({x: 0, y: -range}), transform({x: 0, y: range}), "#37516a");
  const system = $("point-coordinates").value;
  ctx.fillStyle = "#98b1c8"; ctx.textAlign = "right"; ctx.fillText("+X", width - 12, cy - 12); ctx.textAlign = "left"; ctx.fillText("+Y", cx + 10, 14);
  const rawEnabled = $("layer-raw").checked, wheelEnabled = $("layer-wheel").checked, clusterEnabled = $("layer-cluster").checked, rejectedEnabled = $("layer-rejected").checked;
  const points = state.frame?.points || [];
  points.forEach(point => {
    if (!(numeric(point.distance) > 0)) return;
    const local = localCoord(point, system); if (!local) return;
    const pixel = transform(local); if (pixel.x < 0 || pixel.x > width || pixel.y < 0 || pixel.y > height) return;
    let fill = "#4c647c", size = 1.8, visible = rawEnabled;
    if (point.reason === "wheel") { visible = wheelEnabled; fill = "#f0b574"; size = 2.7; }
    else if (point.reason === "kept") { visible = rawEnabled || clusterEnabled; fill = "#79a89e"; size = 2.1; }
    if (clusterEnabled && point.reason === "kept" && point.cluster_id !== null && point.cluster_id !== undefined && point.cluster_id >= 0) { fill = colorFor(point.cluster_id); size = 3.0; visible = true; }
    if (!visible) return;
    drawDot(ctx, pixel.x, pixel.y, size, fill); state.drawPoints.push({point, pixel});
  });
  if (clusterEnabled) {
    (state.baseline?.clusters || []).forEach(cluster => drawWorldBox(ctx, transform, clusterBox(localCluster(cluster, state.baseline, system)), "#8caddeaa", "transparent", true));
    (state.frame?.clusters || []).forEach(cluster => {
      const local = localCluster(cluster, state.frame, system), color = colorFor(cluster.id), box = clusterBox(local), center = transform(clusterCenter(local));
      drawWorldBox(ctx, transform, box, color + "a0", color + "09");
      drawDot(ctx, center.x, center.y, 2, color); ctx.textAlign = "left"; ctx.fillStyle = color; ctx.fillText(`#${cluster.id} · ${cluster.point_count}`, center.x + 7, center.y - 8);
    });
  }
  if (rejectedEnabled) (state.frame?.rejected || []).forEach(cluster => {
    const members = points.filter(point => (cluster.angles || []).includes(point.angle));
    members.forEach(point => { const local = localCoord(point, system); if (!local) return; const pixel = transform(local); ctx.beginPath(); ctx.arc(pixel.x, pixel.y, 5, 0, Math.PI * 2); ctx.strokeStyle = "#ef7c85"; ctx.lineWidth = 1; ctx.stroke(); });
    drawWorldBox(ctx, transform, clusterBox(localCluster(cluster, state.frame, system)), "#ef7c8590", "#ef7c8508", true);
  });
  // The body outline uses the entered width, but its length is deliberately not invented.
  if (system === "body") {
    const halfWidth = numeric($("car-width").value, 450) / 2;
    line(ctx, transform({x: -halfWidth, y: 0}), transform({x: halfWidth, y: 0}), "#9baebf", 2);
    line(ctx, transform({x: 0, y: 0}), transform({x: 0, y: Math.min(halfWidth, 180)}), "#d5e3ee", 2);
  }
  drawDot(ctx, cx, cy, 3.5, "#d5e3ee");
  const selected = state.drawPoints.find(entry => entry.point.angle === state.selectedAngle);
  if (selected) { ctx.beginPath(); ctx.arc(selected.pixel.x, selected.pixel.y, 8, 0, 2 * Math.PI); ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 1.5; ctx.stroke(); }
  $("point-scale").textContent = `视野 ±${format(range)} mm · 网格 ${step} mm`;
  $("point-empty").hidden = !state.frame || points.some(point => numeric(point.distance) > 0);
}
function normalizeBox(value) {
  if (!value) return null;
  if (Array.isArray(value) && value.length === 4) return {min_x: numeric(value[0]), min_y: numeric(value[1]), max_x: numeric(value[2]), max_y: numeric(value[3])};
  const minX = value.min_x ?? value.x_min ?? value.x0 ?? value.left ?? value.x;
  const minY = value.min_y ?? value.y_min ?? value.y0 ?? value.bottom ?? value.y;
  const maxX = value.max_x ?? value.x_max ?? value.x1 ?? value.right ?? (minX !== undefined && value.width !== undefined ? numeric(minX) + numeric(value.width) : undefined);
  const maxY = value.max_y ?? value.y_max ?? value.y1 ?? value.top ?? (minY !== undefined && value.height !== undefined ? numeric(minY) + numeric(value.height) : undefined);
  if ([minX, minY, maxX, maxY].some(item => item === undefined || !Number.isFinite(Number(item)))) return null;
  return {min_x: Number(minX), min_y: Number(minY), max_x: Number(maxX), max_y: Number(maxY)};
}
function mapLayout() { return state.info?.layout || {}; }
function mapNodes(layout) { const nodes = layout.nodes || []; return Array.isArray(nodes) ? nodes : Object.entries(nodes).map(([id, node]) => ({id, ...node})); }
function nodePoint(node) { return coord(node) || coord(node?.point) || coord(node?.center) || (node?.x_mm !== undefined ? {x: Number(node.x_mm), y: Number(node.y_mm)} : null); }
function roadEnds(road, nodes) {
  const ids = road.nodes || road.node_ids || [road.a ?? road.node_a ?? road.from ?? road.start_node ?? road.n1, road.b ?? road.node_b ?? road.to ?? road.end_node ?? road.n2];
  const resolve = value => typeof value === "object" ? nodePoint(value) : nodePoint(nodes.find(node => String(node.id) === String(value)));
  return [nodePoint(road.start) || coord(road.p1) || resolve(ids[0]), nodePoint(road.end) || coord(road.p2) || resolve(ids[1])];
}
function drawMap() {
  const {context: ctx, width, height} = setupCanvas($("map-canvas")), layout = mapLayout();
  const worldWidth = numeric(layout.width ?? layout.width_mm, 6000), worldHeight = numeric(layout.height ?? layout.height_mm, 6000);
  const margin = 34, scale = Math.min((width - 2 * margin) / worldWidth, (height - 2 * margin) / worldHeight);
  const drawWidth = worldWidth * scale, drawHeight = worldHeight * scale, left = (width - drawWidth) / 2, top = (height - drawHeight) / 2;
  const transform = point => ({x: left + point.x * scale, y: top + drawHeight - point.y * scale});
  const boundaries = layout.boundaries;
  const rows = Array.isArray(boundaries) ? boundaries.length - 1 : numeric(layout.rows, 5), cols = Array.isArray(boundaries) ? boundaries.length - 1 : numeric(layout.cols, 5), nodes = mapNodes(layout);
  const xBoundary = index => Array.isArray(boundaries) ? Number(boundaries[index]) : index * worldWidth / cols;
  const yBoundary = index => Array.isArray(boundaries) ? Number(boundaries[index]) : index * worldHeight / rows;
  for (let i = 0; i <= cols; i++) {
    const value = xBoundary(i); line(ctx, transform({x: value, y: 0}), transform({x: value, y: worldHeight}), "#293f52");
    ctx.fillStyle = "#668198"; ctx.textAlign = "center"; ctx.fillText(String(Math.round(value)), transform({x: value, y: 0}).x, top + drawHeight + 16);
  }
  for (let i = 0; i <= rows; i++) {
    const value = yBoundary(i); line(ctx, transform({x: 0, y: value}), transform({x: worldWidth, y: value}), "#293f52");
    ctx.fillStyle = "#668198"; ctx.textAlign = "right"; ctx.fillText(String(Math.round(value)), left - 7, transform({x: 0, y: value}).y + 3);
  }
  const forbidden = layout.forbidden || layout.forbidden_rects || layout.forbidden_rectangles || [];
  forbidden.forEach(rect => {
    const box = normalizeBox(rect); if (!box) return;
    drawWorldBox(ctx, transform, box, "#794c5555", "#3d253044");
    const a = transform({x: box.min_x, y: box.min_y}), b = transform({x: box.max_x, y: box.max_y});
    ctx.save(); ctx.beginPath(); ctx.rect(Math.min(a.x, b.x), Math.min(a.y, b.y), Math.abs(b.x - a.x), Math.abs(b.y - a.y)); ctx.clip();
    for (let d = -drawHeight; d < drawWidth + drawHeight; d += 10) line(ctx, {x: left + d, y: top}, {x: left + d + drawHeight, y: top + drawHeight}, "#5c39464a");
    ctx.restore();
  });
  const roads = layout.roads || [], map = state.frame?.map_state || {}, roadStates = map.roads || [];
  roads.forEach(road => {
    const [start, end] = roadEnds(road, nodes); if (!start || !end) return;
    const status = roadStates.find(value => String(value.id) === String(road.id))?.state || "unknown";
    const color = status === "blocked" ? "#ef7c85" : status === "free" ? "#57d5bf" : "#617b96";
    const roadWidth = Math.max(5, Math.min(16, numeric(road.width ?? layout.road_width, worldWidth / 12) * scale));
    line(ctx, transform(start), transform(end), color + "22", roadWidth);
    line(ctx, transform(start), transform(end), color, 2, status === "unknown");
    const middle = transform({x: (start.x + end.x) / 2, y: (start.y + end.y) / 2});
    ctx.font = "9px 'Segoe UI', sans-serif"; ctx.textAlign = "center"; ctx.fillStyle = "#0d1825"; ctx.fillRect(middle.x - 13, middle.y - 7, 26, 14); ctx.fillStyle = color; ctx.fillText(roadLabel(road.id), middle.x, middle.y + 3);
  });
  nodes.forEach(node => {
    const point = nodePoint(node); if (!point) return; const pixel = transform(point);
    drawDot(ctx, pixel.x, pixel.y, 8.5, "#213a51"); ctx.strokeStyle = "#728ba3"; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(pixel.x, pixel.y, 8.5, 0, 2 * Math.PI); ctx.stroke();
    ctx.textAlign = "center"; ctx.fillStyle = "#bccddd"; ctx.font = "8px 'Segoe UI', sans-serif"; ctx.fillText(String(node.id).startsWith("N") ? node.id : `N${node.id}`, pixel.x, pixel.y + 2.5);
  });
  if (mapActive()) {
    ctx.save(); ctx.beginPath(); ctx.rect(left, top, drawWidth, drawHeight); ctx.clip();
    (map.obstacles || []).forEach(obstacle => {
      const box = normalizeBox(obstacle.inflated || obstacle.inflated_box || obstacle.box || obstacle), confirmed = obstacle.state === "confirmed" || obstacle.confirmed === true;
      drawWorldBox(ctx, transform, box, confirmed ? "#ef7c85" : "#f0b574", confirmed ? "#ef7c8530" : "#f0b57411", !confirmed);
    });
    // Show actual mapped samples separately from inflated obstacle geometry.
    (state.frame?.points || []).filter(point => point.reason === "kept" && point.cluster_id !== null && point.cluster_id !== undefined && point.cluster_id >= 0).forEach(point => {
      const mapped = coord(point.map); if (!mapped || mapped.x < 0 || mapped.x > worldWidth || mapped.y < 0 || mapped.y > worldHeight) return;
      const pixel = transform(mapped); drawDot(ctx, pixel.x, pixel.y, 1.8, colorFor(point.cluster_id));
    });
    (map.candidates || []).forEach(candidate => {
      const cell = candidate.cell;
      if (!cell || cell.x === undefined || cell.y === undefined) return;
      const x = Number(cell.x), y = Number(cell.y); if (x < 0 || y < 0 || x >= cols || y >= rows) return;
      drawWorldBox(ctx, transform, {min_x: xBoundary(x), max_x: xBoundary(x + 1), min_y: yBoundary(y), max_y: yBoundary(y + 1)}, candidate.accepted ? "#57d5bf44" : "#ef7c8544", "transparent", true);
    });
    ctx.restore();
  }
  const pose = effectiveFrame()?.pose;
  if (pose?.valid === true && Number.isFinite(Number(pose.x)) && Number.isFinite(Number(pose.y))) {
    const pixel = transform({x: Number(pose.x), y: Number(pose.y)}), angle = numeric(pose.yaw_deg) * Math.PI / 180;
    // Positive mathematical yaw rotates body +Y toward field -X.
    const tip = {x: pixel.x - Math.sin(angle) * 15, y: pixel.y - Math.cos(angle) * 15};
    drawDot(ctx, pixel.x, pixel.y, 5, "#ffffff"); line(ctx, pixel, tip, "#ffffff", 2);
    ctx.fillStyle = "#dbe6ef"; ctx.font = "9px 'Segoe UI', 'Microsoft YaHei', sans-serif"; ctx.textAlign = "left"; ctx.fillText("车体", pixel.x + 10, pixel.y + 17);
  }
  ctx.textAlign = "right"; ctx.fillStyle = "#9db4c8"; ctx.font = "10px 'Segoe UI', sans-serif"; ctx.fillText("+X →", width - 9, height - 9); ctx.textAlign = "left"; ctx.fillText("+Y ↑", left + 5, top - 12);
}
function drawAll() { drawPointCloud(); drawMap(); }
function fitPointView() {
  const points = state.frame?.points || [], magnitudes = points.filter(point => point.reason === "kept").map(point => localCoord(point, $("point-coordinates").value)).filter(Boolean).map(point => Math.max(Math.abs(point.x), Math.abs(point.y))).sort((a, b) => a - b);
  state.range = magnitudes.length ? Math.max(400, Math.min(20000, magnitudes[Math.floor((magnitudes.length - 1) * .97)] * 1.15)) : 2000; drawPointCloud();
}
async function importFile(file) {
  if (!file) return;
  const maxBytes = numeric(state.info?.limits?.max_bytes, 16 * 1024 * 1024);
  if (file.size > maxBytes) { showError(new Error(`文件超过 ${Math.round(maxBytes / 1024 / 1024)} MB。请截取一段问题记录再导入。`)); return; }
  pause(); const generation = ++state.importGeneration; $("import-button").disabled = true; $("import-button").textContent = "导入中…";
  try {
    const payload = {filename: file.name};
    if (/\.bin$/i.test(file.name)) {
      const bytes = new Uint8Array(await file.arrayBuffer()); let binary = "";
      for (let i = 0; i < bytes.length; i += 32768) binary += String.fromCharCode(...bytes.subarray(i, i + 32768));
      payload.base64 = btoa(binary);
    } else payload.text = await file.text();
    const recording = await api("/api/import", payload);
    if (generation !== state.importGeneration) return;
    setRecording(recording, file.name); toast(`已导入 ${state.frames.length} 圈`);
  } catch (error) { if (generation === state.importGeneration) showError(error); }
  finally { if (generation === state.importGeneration) { $("import-button").disabled = false; $("import-button").textContent = "＋ 导入记录"; } $("file-input").value = ""; }
}
async function exportFrame() {
  if (!state.frame || state.pending || state.busy || state.exporting || !state.paramsValid) return;
  const output = {format: "gongxun-lidar-v1", synthetic: state.recording.synthetic === true, label: `${state.recording.label || $("recording-name").textContent} · 第 ${state.index + 1} 圈`,
    frames: state.frames.slice(0, state.index + 1).map((frame, index) => copy(effectiveFrame(index))), problem_frame_index: state.index,
    params: copy(state.params), car_width_mm: Number($("car-width").value), safety_mm: Number($("safety").value), compare: $("compare-toggle").checked,
    source: copy(state.info?.source || {}), diagnostic: {current: copy(state.frame), baseline: copy(state.baseline), original_index: state.index,
      history_note: "frames 包含从记录开头到问题圈的完整前缀。按原始顺序回放至 problem_frame_index 可重建多圈确认状态；诊断保存了导出时结果。"}};
  state.exporting = true; $("export-button").disabled = true; $("export-button").textContent = "保存中…";
  try {
    const saved = await api("/api/export", {recording: output});
    if (!saved.filename || !saved.url) throw new Error("本地服务未返回问题记录的保存位置。");
    const link = $("export-link"); link.textContent = saved.filename; link.href = saved.url; link.download = saved.filename; link.title = saved.path || saved.filename;
    $("export-result").hidden = false;
    toast("问题记录已保存到工具的 exports 文件夹");
  } catch (error) { showError(error); }
  finally {
    state.exporting = false; $("export-button").textContent = "导出问题帧";
    $("export-button").disabled = !state.frame || state.pending || state.busy || !state.paramsValid;
  }
}
function wireEvents() {
  $("source-toggle").addEventListener("click", () => { $("source-info").hidden = !$("source-info").hidden; });
  $("error-dismiss").addEventListener("click", () => { $("error").hidden = true; });
  $("import-button").addEventListener("click", () => $("file-input").click());
  $("file-input").addEventListener("change", event => importFile(event.target.files[0]));
  $("demo-button").addEventListener("click", async () => {
    const generation = ++state.importGeneration; $("demo-button").disabled = true; pause();
    try { const recording = await api("/api/demo"); if (generation === state.importGeneration) setRecording(recording); }
    catch (error) { showError(error); } finally { $("demo-button").disabled = false; $("import-button").disabled = false; $("import-button").textContent = "＋ 导入记录"; }
  });
  $("export-button").addEventListener("click", exportFrame);
  $("previous-button").addEventListener("click", () => selectFrame(state.index - 1)); $("next-button").addEventListener("click", () => selectFrame(state.index + 1));
  $("frame-slider").addEventListener("input", event => selectFrame(event.target.value)); $("play-button").addEventListener("click", togglePlay);
  $("play-speed").addEventListener("change", () => { if (state.playing && !state.busy) advanceWhenReady(); });
  $("compare-toggle").addEventListener("change", () => { pause(); scheduleReplay(); });
  $("reset-params").addEventListener("click", () => { state.params = defaultParams(); state.paramsValid = true; populateParams(); pause(); handleParams(); toast("已恢复当前源码参数"); });
  document.querySelectorAll("[data-param], #mask-enabled, #tf-x, #tf-y, #tf-yaw, #tf-mirror, #car-width, #safety").forEach(input => input.addEventListener("input", handleParams));
  $("pose-apply-frame").addEventListener("click", () => applyPose(false)); $("pose-apply-all").addEventListener("click", () => applyPose(true));
  $("pose-reset").addEventListener("click", () => { pause(); state.frames = copy(state.originalFrames); state.globalPose = null; state.editedFrames = false; populatePose(); scheduleReplay(); toast("已恢复整段记录内定位"); });
  ["layer-raw", "layer-wheel", "layer-cluster", "layer-rejected", "point-coordinates"].forEach(id => $(id).addEventListener("change", drawPointCloud));
  $("zoom-reset").addEventListener("click", fitPointView);
  $("point-canvas").addEventListener("wheel", event => { event.preventDefault(); state.range = Math.max(100, Math.min(50000, state.range * (event.deltaY > 0 ? 1.18 : 1 / 1.18))); drawPointCloud(); }, {passive: false});
  $("point-canvas").addEventListener("click", event => {
    const rect = event.currentTarget.getBoundingClientRect(), x = event.clientX - rect.left, y = event.clientY - rect.top;
    let best = null, distance = 14 * 14;
    state.drawPoints.forEach(entry => { const d = (entry.pixel.x - x) ** 2 + (entry.pixel.y - y) ** 2; if (d < distance) { best = entry.point; distance = d; } });
    state.selectedAngle = best?.angle ?? null; renderInspector(); drawPointCloud();
  });
  document.addEventListener("keydown", event => {
    if (/INPUT|SELECT|TEXTAREA|BUTTON/.test(event.target.tagName) || event.ctrlKey || event.altKey || event.metaKey) return;
    if (event.code === "Space") { event.preventDefault(); togglePlay(); }
    if (event.code === "ArrowLeft") { event.preventDefault(); selectFrame(state.index - 1); }
    if (event.code === "ArrowRight") { event.preventDefault(); selectFrame(state.index + 1); }
  });
  let dragDepth = 0;
  document.addEventListener("dragenter", event => { if ([...(event.dataTransfer?.types || [])].includes("Files")) { event.preventDefault(); dragDepth++; $("drop-overlay").hidden = false; } });
  document.addEventListener("dragover", event => { if ([...(event.dataTransfer?.types || [])].includes("Files")) event.preventDefault(); });
  document.addEventListener("dragleave", () => { dragDepth = Math.max(0, dragDepth - 1); if (!dragDepth) $("drop-overlay").hidden = true; });
  document.addEventListener("drop", event => { event.preventDefault(); dragDepth = 0; $("drop-overlay").hidden = true; if (event.dataTransfer?.files?.length) importFile(event.dataTransfer.files[0]); });
  const observer = new ResizeObserver(() => requestAnimationFrame(drawAll)); observer.observe($("point-canvas").parentElement); observer.observe($("map-canvas").parentElement);
}
async function init() {
  wireEvents();
  try {
    const [info, demo] = await Promise.all([api("/api/info"), api("/api/demo")]);
    state.info = info; state.params = defaultParams(); populateParams(); renderSource();
    $("connection").textContent = "本地服务已连接"; $("connection").className = "status-pill connected";
    setRecording(demo);
  } catch (error) {
    $("connection").textContent = "本地服务未连接"; $("connection").className = "status-pill failed"; showError(error); drawAll();
  }
}
init();
