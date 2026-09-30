'use strict';

/* ---------------- 状态与示例 ---------------- */

const SAMPLE = {
  channels: [
    { id: 'R', r: 1000, g: 120, b: 40,  power_mw: 3200 },
    { id: 'G', r: 160,  g: 1000, b: 90, power_mw: 2800 },
    { id: 'B', r: 60,   g: 80,  b: 1000, power_mw: 3400 },
    { id: 'W', r: 820,  g: 800, b: 760, power_mw: 6500 },
  ],
  target: { r: 600, g: 420, b: 180 },
  tolerance: 12,
};

// 标定示例样本（针对 SAMPLE 通道，按顺序给出驱动级；实测值由页面公式算得）
const SAMPLE_CALIB_DRIVES = [
  [255, 0,   0,   0  ],
  [0,   255, 0,   0  ],
  [0,   0,   255, 0  ],
  [0,   0,   0,   255],
  [128, 200, 64,  30 ],
];

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => Array.from(document.querySelectorAll(sel));
const clone = (v) => JSON.parse(JSON.stringify(v));

let channels = clone(SAMPLE.channels);
// 最近一次求解结果（通道满档数据以服务端返回为准）
let solution = null;
// 最优驱动级（id -> drive），供“恢复最优”使用
let optimalDrives = null;
let manualMode = false;
// 标定样本：每条 { drives: {id:int}, measured:[r,g,b] }，按 id 存以适应通道增删
let calibSamples = [];
// 最近一次唯一标定结果（{id:{r,g,b}} + 提交时的驱动级，供实测校验/写回）
let calibUnique = null;
// 调光路径：按通道 id 存起始驱动级（0～255），适应通道增删
let pathStart = {};
// 最近一次路径规划结果（服务端原样返回）
let pathResult = null;
// 最近一次单路熄灭容错配方结果（服务端原样返回）
let robustResult = null;

/* ---------------- 工具 ---------------- */

function esc(s) {
  return String(s).replace(/[&<>"']/g, (ch) =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]));
}

function intVal(input, fallback = 0) {
  const v = input.value.trim();
  if (v === '' || !/^[+-]?\d+$/.test(v)) return fallback;
  return parseInt(v, 10);
}

// 最多 3 位小数，去掉多余的 0
function fmt(x) {
  const r = Math.round(x * 1000) / 1000;
  return Number.isInteger(r) ? String(r) : r.toFixed(3).replace(/0+$/, '').replace(/\.$/, '');
}
function fmtSigned(x) { return (x > 0 ? '+' : '') + fmt(x); }

// 四舍五入（小数部分 ≥ 0.5 一律向上，不用 Math.round 的负数/偶舍分歧；此处值非负）
function roundHalfUp(x) { return Math.floor(x + 0.5); }

/** 线性 RGB（0～1000 标度）→ sRGB 显示色。仅显示时裁剪到 0～1。 */
function linearToSrgbHex(rgb) {
  const enc = (c) => {
    let v = c / 1000;
    v = Math.min(1, Math.max(0, v)); // 仅显示裁剪
    v = v <= 0.0031308 ? 12.92 * v : 1.055 * Math.pow(v, 1 / 2.4) - 0.055;
    return Math.round(v * 255);
  };
  const h = (n) => n.toString(16).padStart(2, '0');
  return '#' + rgb.map(enc).map(h).join('');
}

function swatchHtml(rgb, title) {
  return `<span class="mini-swatch" style="background:${linearToSrgbHex(rgb)}" title="${esc(title)}"></span>`;
}

function showBanner(kind, html) {
  const el = $('#error-banner');
  el.className = `banner ${kind}`;
  el.innerHTML = html;
}
function clearBanner() { $('#error-banner').className = 'banner error hidden'; }

// 把 422/500 响应的 detail 格式化为可展示的 HTML
function errDetailHtml(data, fallback) {
  const detail = data.detail;
  if (Array.isArray(detail)) {
    return detail.map((d) => {
      const loc = (d.loc || []).filter((x) => x !== 'body').join('.');
      return `${loc ? loc + '：' : ''}${d.msg}`;
    }).join('<br>');
  }
  return esc(String(detail ?? fallback));
}

/* ---------------- 通道录入表 ---------------- */

function renderChannels() {
  const tb = $('#channel-tbody');
  tb.innerHTML = channels.map((ch, i) => `
    <tr data-i="${i}">
      <td class="id-cell"><input type="text" class="f-id" value="${esc(ch.id)}" maxlength="40"></td>
      <td><input type="number" class="f-r" min="0" max="1000" step="1" value="${ch.r}"></td>
      <td><input type="number" class="f-g" min="0" max="1000" step="1" value="${ch.g}"></td>
      <td><input type="number" class="f-b" min="0" max="1000" step="1" value="${ch.b}"></td>
      <td><input type="number" class="f-p" min="1" max="10000" step="1" value="${ch.power_mw}"></td>
      <td class="full-swatch"></td>
      <td><button type="button" class="btn small danger del-btn" ${channels.length <= 3 ? 'disabled' : ''}>删除</button></td>
    </tr>`).join('');
  refreshChannelSwatches();
  $('#add-channel').disabled = channels.length >= 6;
}

function refreshChannelSwatches() {
  $$('#channel-tbody tr').forEach((tr) => {
    const i = +tr.dataset.i;
    const ch = channels[i];
    const cell = tr.querySelector('.full-swatch');
    cell.innerHTML = swatchHtml([ch.r, ch.g, ch.b],
      `满档线性 RGB(${ch.r}, ${ch.g}, ${ch.b})`);
  });
}

function syncRowState(tr) {
  const i = +tr.dataset.i;
  channels[i] = {
    id: tr.querySelector('.f-id').value.trim(),
    r: intVal(tr.querySelector('.f-r')),
    g: intVal(tr.querySelector('.f-g')),
    b: intVal(tr.querySelector('.f-b')),
    power_mw: intVal(tr.querySelector('.f-p')),
  };
  refreshChannelSwatches();
}

function readForm() {
  $$('#channel-tbody tr').forEach((tr) => syncRowState(tr));
  const target = {
    r: intVal($('#target-r')),
    g: intVal($('#target-g')),
    b: intVal($('#target-b')),
  };
  const tolerance = intVal($('#tolerance'));
  return { target, tolerance };
}

function validateForm({ target, tolerance }) {
  const errs = [];
  if (!channels.every((c) => c.id)) errs.push('每路通道都必须填写 ID。');
  const ids = channels.map((c) => c.id);
  const dup = ids.filter((x, i) => x && ids.indexOf(x) !== i);
  if (dup.length) errs.push(`通道 ID 必须唯一，重复：${[...new Set(dup)].join('、')}。`);
  channels.forEach((c) => {
    const tag = c.id ? `“${c.id}”` : '某通道';
    for (const k of ['r', 'g', 'b']) {
      if (!Number.isInteger(c[k]) || c[k] < 0 || c[k] > 1000)
        errs.push(`通道 ${tag} 的满档 ${k.toUpperCase()} 必须是 0～1000 的整数。`);
    }
    if (c.r === 0 && c.g === 0 && c.b === 0)
      errs.push(`通道 ${tag} 的满档 RGB 不能全为零。`);
    if (!Number.isInteger(c.power_mw) || c.power_mw < 1 || c.power_mw > 10000)
      errs.push(`通道 ${tag} 的满档功率必须是 1～10000 mW 的整数。`);
  });
  for (const k of ['r', 'g', 'b']) {
    if (target[k] < 0 || target[k] > 1000)
      errs.push(`目标 ${k.toUpperCase()} 必须是 0～1000 的整数。`);
  }
  if (tolerance < 0 || tolerance > 20) errs.push('共用容差必须是 0～20 的整数。');
  return [...new Set(errs)];
}

/* ---------------- 目标色块（实时） ---------------- */

function refreshTargetSwatch() {
  const { target } = readForm();
  const rgb = [target.r, target.g, target.b];
  const hex = linearToSrgbHex(rgb);
  $('#target-swatch').style.background = hex;
  $('#target-swatch').title = `目标线性 RGB(${target.r}, ${target.g}, ${target.b})`;
  $('#target-hex').textContent = hex.toUpperCase();
  if (solution) {
    $('#target-swatch2').style.background = hex;
  }
}

/* ---------------- 求解 ---------------- */

async function onSolve() {
  clearBanner();
  const form = readForm();
  const errs = validateForm(form);
  if (errs.length) {
    showBanner('error', '<strong>录入有误：</strong><br>' + errs.map(esc).join('<br>'));
    return;
  }
  const btn = $('#solve-btn');
  btn.disabled = true;
  btn.textContent = '求解中…';
  try {
    const resp = await fetch('/api/solve', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        channels: channels.map(({ id, r, g, b, power_mw }) => ({ id, r, g, b, power_mw })),
        target_r: form.target.r,
        target_g: form.target.g,
        target_b: form.target.b,
        tolerance: form.tolerance,
      }),
    });
    const data = await resp.json();
    if (!resp.ok) {
      showBanner('error', '<strong>请求被拒绝：</strong><br>' +
        errDetailHtml(data, '求解失败'));
      return;
    }
    renderSolution(data);
  } catch (e) {
    showBanner('error', `请求失败：${esc(e.message)}。请确认后端服务已启动。`);
  } finally {
    btn.disabled = false;
    btn.textContent = '求解最省功率配方';
  }
}

/* ---------------- 结果渲染与手动预览 ---------------- */

function currentDrives() {
  const drives = {};
  $$('#drive-tbody tr').forEach((tr) => {
    drives[tr.dataset.id] = intVal(tr.querySelector('.f-drive'), 0);
  });
  return drives;
}

function computePreview(drives) {
  const s = [0, 0, 0];
  let powerNum = 0;
  // 与 #drive-tbody 中的行顺序一一对应（渲染顺序与 solution.channels 相同）
  const rows = solution.channels.map((ch) => {
    const d = Math.max(0, Math.min(255, drives[ch.id] ?? 0));
    const full = [ch.r_full, ch.g_full, ch.b_full];
    const contrib = full.map((v) => (v * d) / 255);
    for (let c = 0; c < 3; c++) s[c] += full[c] * d; // 整数分子
    powerNum += ch.power_full_mw * d;
    return { ch, d, contrib };
  });
  const actual = s.map((v) => v / 255);
  return { rows, s, actual, powerNum };
}
function renderSolution(data) {
  solution = data;
  if (!data.feasible) {
    $('#result-panel').hidden = true;
    optimalDrives = null;
    showBanner('info',
      `<strong>⛔ 无可行配方</strong><br>${esc(data.message)}<br>` +
      `<span style="color:var(--muted)">未返回任何近似配方。</span>`);
    return;
  }
  clearBanner();
  $('#result-panel').hidden = false;
  optimalDrives = {};
  data.channels.forEach((c) => { optimalDrives[c.id] = c.drive; });
  manualMode = false;

  $('#drive-tbody').innerHTML = data.channels.map((c) => `
    <tr data-id="${esc(c.id)}">
      <td><strong>${esc(c.id)}</strong></td>
      <td><input type="number" class="f-drive" min="0" max="255" step="1" value="${c.drive}"></td>
      <td class="full-cell"></td>
      <td class="actual-cell"></td>
      <td class="power-cell"></td>
    </tr>`).join('');

  $('#tiebreak-hint').textContent =
    `字典序 tie-break（按 ID 字符串升序）：${data.sorted_ids.map(esc).join(' ＜ ')}`;
  refreshTargetSwatch();
  refreshPreview();
}

function refreshPreview() {
  if (!solution || !solution.feasible) return;
  const drives = currentDrives();
  const { rows, s, actual, powerNum } = computePreview(drives);
  const target = solution.target;
  const tol = solution.tolerance;

  // 驱动表：满档色 / 实际贡献色 / 功率贡献
  const trs = $$('#drive-tbody tr');
  rows.forEach(({ ch, d, contrib }, i) => {
    const tr = trs[i];
    const full = [ch.r_full, ch.g_full, ch.b_full];
    tr.querySelector('.full-cell').innerHTML =
      swatchHtml(full, `${ch.id} 满档 RGB(${full.join(', ')})`);
    tr.querySelector('.actual-cell').innerHTML =
      swatchHtml(contrib, `${ch.id} 驱动 ${d}/255 的线性贡献 RGB(${contrib.map(fmt).join(', ')})`);
    tr.querySelector('.power-cell').textContent =
      fmt((ch.power_full_mw * d) / 255);
  });

  // 实际色块
  const hex = linearToSrgbHex(actual);
  $('#actual-swatch').style.background = hex;
  $('#actual-hex').textContent = hex.toUpperCase();
  const clipped = actual.some((v) => v < 0 || v > 1000);
  $('#actual-swatch').title =
    `实际线性 RGB(${actual.map(fmt).join(', ')})` + (clipped ? '（超出 0～1000，显示已裁剪，模型不裁剪）' : '');

  // 分量对照表
  let allOk = true;
  const names = ['R', 'G', 'B'];
  $$('#component-table tbody tr').forEach((tr) => {
    const c = +tr.dataset.c;
    const lo = target[c] - tol, hi = target[c] + tol;
    const err = actual[c] - target[c];
    const ok = lo <= actual[c] && actual[c] <= hi && 0 <= actual[c] && actual[c] <= 1000;
    if (!ok) allOk = false;
    tr.querySelector('.lo').textContent = Math.max(0, lo);
    tr.querySelector('.tgt').textContent = target[c];
    tr.querySelector('.hi').textContent = Math.min(1000, hi);
    tr.querySelector('.act').innerHTML =
      `<span title="精确值 ${s[c]}/255">${fmt(actual[c])}</span>`;
    tr.querySelector('.err').textContent = fmtSigned(err);
    const j = tr.querySelector('.judge');
    j.textContent = ok ? '✔ 在区间内' : '✘ 超差';
    j.className = 'judge ' + (ok ? 'ok' : 'bad');
  });

  // 功率
  $('#total-power').textContent = fmt(powerNum / 255);
  $('#power-frac').textContent = `精确值 ${powerNum} / 255 mW`;

  // 状态标记
  const tag = $('#power-opt-tag');
  const isOptimal =
    !manualMode &&
    Object.entries(drives).every(([id, d]) => d === optimalDrives[id]);
  if (isOptimal) {
    tag.className = 'tag ok';
    tag.textContent = '最优（最小功率 + 字典序）';
  } else {
    tag.className = 'tag warn';
    tag.textContent = allOk ? '手动预览（仍满足约束）' : '手动预览（已超差，仅供参考）';
  }
  $('#restore-opt').hidden = isOptimal;
  $('#manual-hint').hidden = isOptimal;
}

function restoreOptimal() {
  if (!optimalDrives) return;
  $$('#drive-tbody tr').forEach((tr) => {
    tr.querySelector('.f-drive').value = optimalDrives[tr.dataset.id];
  });
  manualMode = false;
  refreshPreview();
}

/* ---------------- 满档读数标定 ---------------- */

function defaultCalibSamples() {
  // 依据当前 SAMPLE 通道的满档值生成无噪声示例样本
  return SAMPLE_CALIB_DRIVES.map((ds) => {
    const drives = {};
    SAMPLE.channels.forEach((ch, i) => { drives[ch.id] = ds[i] ?? 0; });
    return { drives, measured: predictedFromChannels(SAMPLE.channels, ds) };
  });
}

// 按 Σ(满档读数×驱动级)/255 四舍五入计算实测线性 RGB
function predictedFromChannels(chs, driveList) {
  const s = [0, 0, 0];
  chs.forEach((ch, i) => {
    const d = driveList[i] ?? 0;
    s[0] += ch.r * d; s[1] += ch.g * d; s[2] += ch.b * d;
  });
  return s.map((v) => roundHalfUp(v / 255));
}

// 通道增删/重命名后，把样本对齐到当前通道 id 集合
function reconcileCalibSamples() {
  const ids = new Set(channels.map((c) => c.id));
  calibSamples.forEach((smp) => {
    Object.keys(smp.drives).forEach((id) => {
      if (!ids.has(id)) delete smp.drives[id];
    });
    channels.forEach((c) => {
      if (smp.drives[c.id] === undefined) smp.drives[c.id] = 0;
    });
  });
}

function emptySample() {
  const drives = {};
  channels.forEach((c) => { drives[c.id] = 0; });
  return { drives, measured: [0, 0, 0] };
}

function renderCalibTable() {
  reconcileCalibSamples();
  const head = $('#sample-head-row');
  head.innerHTML =
    '<th>样本</th>' +
    channels.map((c) => `<th class="calib-drive-col">驱动 ${esc(c.id)}</th>`).join('') +
    '<th>实测 R</th><th>实测 G</th><th>实测 B</th><th></th>';

  const tb = $('#sample-tbody');
  tb.innerHTML = calibSamples.map((smp, si) => `
    <tr data-s="${si}">
      <td class="calib-idx">#${si + 1}</td>
      ${channels.map((c) => `
        <td><input type="number" class="cd-drive" data-id="${esc(c.id)}"
             min="0" max="255" step="1" value="${smp.drives[c.id] ?? 0}"></td>`).join('')}
      <td><input type="number" class="cd-m" data-c="0" min="0" max="6000" step="1" value="${smp.measured[0]}"></td>
      <td><input type="number" class="cd-m" data-c="1" min="0" max="6000" step="1" value="${smp.measured[1]}"></td>
      <td><input type="number" class="cd-m" data-c="2" min="0" max="6000" step="1" value="${smp.measured[2]}"></td>
      <td><button type="button" class="btn small danger sample-del" ${calibSamples.length <= 3 ? 'disabled' : ''}>删除</button></td>
    </tr>`).join('');
  $('#add-sample').disabled = calibSamples.length >= 12;
}

function syncCalibRow(tr) {
  const si = +tr.dataset.s;
  const smp = calibSamples[si];
  tr.querySelectorAll('.cd-drive').forEach((inp) => {
    smp.drives[inp.dataset.id] = intVal(inp, 0);
  });
  tr.querySelectorAll('.cd-m').forEach((inp) => {
    smp.measured[+inp.dataset.c] = intVal(inp, 0);
  });
}

function readCalibForm() {
  $$('#sample-tbody tr').forEach((tr) => syncCalibRow(tr));
  return calibSamples;
}

function validateCalibForm() {
  const errs = [];
  if (!channels.every((c) => c.id)) errs.push('每路通道都必须填写 ID，才能标定。');
  const ids = channels.map((c) => c.id);
  if (new Set(ids).size !== ids.length) errs.push('通道 ID 必须唯一，才能标定。');
  if (![3, 4, 5, 6].includes(channels.length))
    errs.push(`当前通道数为 ${channels.length}，标定仅支持 3～6 路。`);
  if (![3, 4, 5, 6, 7, 8, 9, 10, 11, 12].includes(calibSamples.length))
    errs.push(`标定样本数为 ${calibSamples.length}，必须录入 3～12 组。`);
  calibSamples.forEach((smp, si) => {
    channels.forEach((c) => {
      const d = smp.drives[c.id];
      if (!Number.isInteger(d) || d < 0 || d > 255)
        errs.push(`样本 #${si + 1} 通道 “${c.id}” 的驱动级必须是 0～255 的整数。`);
    });
    smp.measured.forEach((m, c) => {
      if (!Number.isInteger(m) || m < 0 || m > 6000)
        errs.push(`样本 #${si + 1} 的实测 ${'RGB'[c]} 必须是 0～6000 的整数。`);
    });
  });
  return [...new Set(errs)];
}

function clearCalibResult() {
  calibUnique = null;
  $('#calib-result').hidden = true;
  $('#calib-status').className = 'calib-status';
  $('#calib-status').textContent = '';
  $('#calib-unique-block').hidden = true;
}

async function onCalibrate() {
  clearBanner();
  const samples = readCalibForm();
  const errs = validateCalibForm();
  if (errs.length) {
    clearCalibResult();
    showBanner('error', '<strong>样本录入有误，已拒绝提交：</strong><br>' + errs.map(esc).join('<br>'));
    return;
  }
  const btn = $('#calib-btn');
  btn.disabled = true;
  btn.textContent = '反求中…';
  try {
    const orderedIds = channels.map((c) => c.id);
    const resp = await fetch('/api/calibrate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        channels: orderedIds.map((id) => ({ id })),
        samples: samples.map((smp) => ({
          drives: orderedIds.map((id) => smp.drives[id]),
          measured: smp.measured.slice(),
        })),
      }),
    });
    const data = await resp.json();
    if (!resp.ok) {
      clearCalibResult();
      showBanner('error', '<strong>请求被拒绝：</strong><br>' +
        errDetailHtml(data, '标定失败'));
      return;
    }
    renderCalibResult(data, samples);
  } catch (e) {
    clearCalibResult();
    showBanner('error', `请求失败：${esc(e.message)}。请确认后端服务已启动。`);
  } finally {
    btn.disabled = false;
    btn.textContent = '反求满档读数';
  }
}

function renderCalibResult(data, submittedSamples) {
  const box = $('#calib-result');
  const status = $('#calib-status');
  box.hidden = false;
  const ICON = {
    unique: '✔',
    contradictory: '⛔',
    non_unique: '⚠',
    unknown: '⏳',
  };
  status.className = `calib-status s-${data.status}`;
  status.innerHTML =
    `<strong>${ICON[data.status] || ''} ${
      { unique: '标定唯一', contradictory: '样本矛盾',
        non_unique: '标定不唯一', unknown: '搜索未完成' }[data.status]
    }</strong> ${esc(data.message)}`;

  if (data.status !== 'unique') {
    // 非唯一结论：不显示任何参数，也不允许写回
    $('#calib-unique-block').hidden = true;
    calibUnique = null;
    return;
  }

  calibUnique = {
    readings: {},
    samples: clone(submittedSamples),
  };
  data.channels.forEach((c) => { calibUnique.readings[c.id] = { r: c.r, g: c.g, b: c.b }; });

  const curPower = {};
  channels.forEach((c) => { curPower[c.id] = c.power_mw; });
  $('#calib-tbody').innerHTML = data.channels.map((c) => {
    const rgb = [c.r, c.g, c.b];
    return `
      <tr data-id="${esc(c.id)}">
        <td><strong>${esc(c.id)}</strong></td>
        <td>${c.r}</td>
        <td>${c.g}</td>
        <td>${c.b}</td>
        <td>${swatchHtml(rgb, `${c.id} 反求满档 RGB(${c.r}, ${c.g}, ${c.b})`)}</td>
        <td>${curPower[c.id] ?? '—'}</td>
      </tr>`;
  }).join('');

  // 逐样本残差总表：用反求读数重算预测实测，与录入实测对照
  const ids = channels.map((c) => c.id);
  const rowsHtml = submittedSamples.map((smp, si) => {
    const s = [0, 0, 0];
    ids.forEach((id) => {
      const rd = calibUnique.readings[id];
      const d = smp.drives[id] ?? 0;
      s[0] += rd.r * d; s[1] += rd.g * d; s[2] += rd.b * d;
    });
    const pred = s.map((v) => roundHalfUp(v / 255));
    const diffs = pred.map((p, c) => p - smp.measured[c]);
    const ok = diffs.every((x) => x === 0);
    return `<tr>
      <td>#${si + 1}</td>
      <td>${pred.join(', ')}</td>
      <td>${smp.measured.join(', ')}</td>
      <td class="${ok ? 'judge ok' : 'judge bad'}">${ok ? '✔ 一致' : '✘ ' + diffs.join(', ')}</td>
    </tr>`;
  }).join('');
  $('#calib-residual-table tbody').innerHTML = rowsHtml;

  const missing = ids.filter((id) => !calibUnique.readings[id]);
  const meta = $('#writeback-meta');
  const wb = $('#writeback-btn');
  if (missing.length) {
    wb.disabled = true;
    meta.textContent = `当前通道与标定通道不一致（缺少：${missing.join('、')}），请恢复标定所用通道后再写回。`;
  } else {
    wb.disabled = false;
    meta.textContent = '写回只更新各路满档 R/G/B，满档功率保持当前值不变。';
  }
  $('#calib-unique-block').hidden = false;
}

function onWriteback() {
  if (!calibUnique) return;
  $$('#channel-tbody tr').forEach((tr) => syncRowState(tr));
  let applied = 0;
  channels.forEach((c) => {
    const rd = calibUnique.readings[c.id];
    if (rd) {
      c.r = rd.r; c.g = rd.g; c.b = rd.b;
      applied += 1;
    }
  });
  renderChannels();
  // 通道满档值已变，旧配方结果失效
  solution = null;
  optimalDrives = null;
  $('#result-panel').hidden = true;
  clearRobustResult();
  renderPathStartTable();
  clearPathResult();
  showBanner('info',
    `<strong>✔ 已写回 ${applied} 路满档 RGB</strong>，各路满档功率保持原值。` +
    `可直接在 ② 设置目标色后点击“求解最省功率配方”。`);
}

/* ---------------- 单路熄灭容错配方 ---------------- */

function clearRobustResult() {
  robustResult = null;
  $('#robust-result').hidden = true;
  $('#robust-detail').hidden = true;
  const st = $('#robust-status');
  st.className = 'calib-status';
  st.textContent = '';
}

function validateRobustForm({ target, tolerance }, adj) {
  const errs = validateForm({ target, tolerance });
  if (!Number.isInteger(adj) || adj < 1 || adj > 255)
    errs.push('故障后每路最大调整级差必须是 1～255 的整数。');
  return [...new Set(errs)];
}

async function onSolveRobust() {
  clearBanner();
  const form = readForm();
  const adj = intVal($('#robust-delta'));
  const errs = validateRobustForm(form, adj);
  if (errs.length) {
    clearRobustResult();
    showBanner('error', '<strong>容错配方参数有误，已拒绝提交：</strong><br>' +
      errs.map(esc).join('<br>'));
    return;
  }
  const btn = $('#robust-solve-btn');
  btn.disabled = true;
  btn.textContent = '规划中…';
  try {
    const resp = await fetch('/api/robust/solve', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        channels: channels.map(({ id, r, g, b, power_mw }) => ({ id, r, g, b, power_mw })),
        target_r: form.target.r,
        target_g: form.target.g,
        target_b: form.target.b,
        tolerance: form.tolerance,
        max_adjust: adj,
      }),
    });
    const data = await resp.json();
    if (!resp.ok) {
      clearRobustResult();
      showBanner('error', '<strong>请求被拒绝：</strong><br>' +
        errDetailHtml(data, '容错配方规划失败'));
      return;
    }
    renderRobustResult(data);
  } catch (e) {
    clearRobustResult();
    showBanner('error', `请求失败：${esc(e.message)}。请确认后端服务已启动。`);
  } finally {
    btn.disabled = false;
    btn.textContent = '规划容错配方';
  }
}

// 某场景的分量对照表行（正常/备用通用）
function robustCompRows(actual, target, tol) {
  const names = ['R', 'G', 'B'];
  const ccls = ['r', 'g', 'b'];
  return names.map((nm, c) => {
    const lo = target[c] - tol, hi = target[c] + tol;
    const err = actual[c] - target[c];
    const ok = lo <= actual[c] && actual[c] <= hi && 0 <= actual[c] && actual[c] <= 1000;
    return `<tr>
      <td class="cname ${ccls[c]}">${nm}</td>
      <td>${Math.max(0, lo)}</td>
      <td>${target[c]}</td>
      <td>${Math.min(1000, hi)}</td>
      <td>${fmt(actual[c])}</td>
      <td>${fmtSigned(err)}</td>
      <td class="judge ${ok ? 'ok' : 'bad'}">${ok ? '✔' : '✘'}</td>
    </tr>`;
  }).join('');
}

function renderRobustResult(data) {
  robustResult = data;
  $('#robust-result').hidden = false;
  const status = $('#robust-status');
  const MAP = {
    optimal: ['s-unique', '✔ 完整容错方案'],
    infeasible: ['s-contradictory', '⛔ 无完整容错方案'],
    incomplete: ['s-unknown', '⏳ 搜索未完成'],
  };
  const [cls, label] = MAP[data.status] || ['s-unknown', data.status];
  status.className = `calib-status ${cls}`;
  status.innerHTML = `<strong>${label}</strong> ${esc(data.message)}`;

  if (data.status !== 'optimal') {
    // 无完整方案 / 搜索未完成：不展示任何正常或备用驱动级
    $('#robust-detail').hidden = true;
    return;
  }
  $('#robust-detail').hidden = false;
  const tol = data.tolerance;
  const target = data.target;
  const ids = data.channel_ids;
  const normal = data.normal;
  const chById = Object.fromEntries(channels.map((c) => [c.id, c]));

  // 目标 / 正常实际色块
  const tHex = linearToSrgbHex(target);
  $('#robust-target-swatch').style.background = tHex;
  $('#robust-target-swatch').title =
    `目标线性 RGB(${target.join(', ')})`;
  const nHex = linearToSrgbHex(normal.actual_rgb);
  $('#robust-normal-swatch').style.background = nHex;
  $('#robust-normal-hex').textContent = nHex.toUpperCase();

  // 正常配方表
  $('#robust-normal-table tbody').innerHTML = ids.map((id, i) => {
    const ch = chById[id];
    const d = normal.drives[i];
    const full = [ch.r, ch.g, ch.b];
    const contrib = full.map((v) => (v * d) / 255);
    return `<tr>
      <td><strong>${esc(id)}</strong></td>
      <td class="drive-cell">${d}</td>
      <td>${swatchHtml(full, `${id} 满档 RGB(${full.join(', ')})`)}</td>
      <td>${swatchHtml(contrib, `${id} 驱动 ${d}/255 的线性贡献 RGB(${contrib.map(fmt).join(', ')})`)}</td>
      <td>${fmt((ch.power_mw * d) / 255)}</td>
    </tr>`;
  }).join('');
  $('#robust-normal-power').textContent = fmt(normal.power_mw);
  $('#robust-normal-power-frac').textContent =
    `精确值 ${normal.power_num} / 255 mW`;
  $('#robust-tiebreak-hint').textContent =
    `字典序 tie-break（按 ID 字符串升序）：${data.sorted_ids.map(esc).join(' ＜ ')}`;

  // 逐路熄灭备用配方
  $('#robust-backups').innerHTML = data.backups.map((bk, fi) => {
    const failedId = bk.failed_id;
    const bHex = linearToSrgbHex(bk.actual_rgb);
    const driveRows = ids.map((id, i) => {
      const nd = normal.drives[i];
      const bd = bk.drives[i];
      const delta = bk.deltas[i];
      const isFailed = id === failedId;
      const dCls = 'drive-cell' + (isFailed ? ' zero' : '');
      const deltaTxt = delta === 0
        ? '<span class="delta">±0</span>'
        : `<span class="delta delta-ok">(${delta > 0 ? '+' : ''}${delta})</span>`;
      return `<tr>
        <td><strong>${esc(id)}</strong>${isFailed ? ' <span class="tag warn">熄灭</span>' : ''}</td>
        <td class="drive-cell">${nd}</td>
        <td class="${dCls}">${bd}</td>
        <td>${deltaTxt}</td>
      </tr>`;
    }).join('');
    return `<div class="robust-backup-card">
      <div class="robust-backup-head">
        <div class="swatch" style="width:42px;height:42px;background:${bHex}"
             title="备用实际线性 RGB(${bk.actual_rgb.map(fmt).join(', ')})"></div>
        <div>
          <div class="robust-backup-title">通道 <span class="failed">${esc(failedId)}</span> 熄灭时的备用配方</div>
          <div class="robust-backup-meta">
            实际 RGB(${bk.actual_rgb.map(fmt).join(', ')})，
            备用功率 <strong>${fmt(bk.power_mw)}</strong> mW
            （${bk.power_num}/255），各路调整 ≤ ${data.max_adjust}
          </div>
        </div>
      </div>
      <div class="robust-backup-grid">
        <div class="table-scroll">
          <table class="grid robust-backup-drives">
            <thead><tr><th>ID</th><th>正常驱动</th><th>备用驱动</th><th>Δ</th></tr></thead>
            <tbody>${driveRows}</tbody>
          </table>
        </div>
        <div class="table-scroll">
          <table class="grid robust-comp">
            <thead><tr><th>分量</th><th>下限</th><th>目标</th><th>上限</th>
                       <th>实际</th><th>误差</th><th>判定</th></tr></thead>
            <tbody>${robustCompRows(bk.actual_rgb, target, tol)}</tbody>
          </table>
        </div>
      </div>
    </div>`;
  }).join('');
}

/* ---------------- 调光路径规划 ---------------- */

// 通道增删/重命名后，把起始驱动级对齐到当前通道 id 集合
function reconcilePathStart() {
  const ids = new Set(channels.map((c) => c.id));
  Object.keys(pathStart).forEach((id) => {
    if (!ids.has(id)) delete pathStart[id];
  });
  channels.forEach((c) => {
    if (pathStart[c.id] === undefined) pathStart[c.id] = 0;
  });
}

function renderPathStartTable() {
  reconcilePathStart();
  $('#path-start-tbody').innerHTML = channels.map((c) => `
    <tr data-id="${esc(c.id)}">
      <td><strong>${esc(c.id)}</strong></td>
      <td>${swatchHtml([c.r, c.g, c.b], `${c.id} 满档 RGB(${c.r}, ${c.g}, ${c.b})`)}</td>
      <td><input type="number" class="f-pstart" min="0" max="255" step="1"
           value="${pathStart[c.id] ?? 0}"></td>
    </tr>`).join('');
  refreshPathSwatches();
}

// 起点实际色：与后端相同的未裁剪公式 Σ(满档读数×起始驱动级)/255
function startActualRgb() {
  const s = [0, 0, 0];
  channels.forEach((c) => {
    const d = Math.max(0, Math.min(255, pathStart[c.id] ?? 0));
    s[0] += c.r * d; s[1] += c.g * d; s[2] += c.b * d;
  });
  return s.map((v) => v / 255);
}

function refreshPathSwatches() {
  const start = startActualRgb();
  const shex = linearToSrgbHex(start);
  const ssw = $('#path-start-swatch');
  ssw.style.background = shex;
  ssw.title = `起点实际线性 RGB(${start.map(fmt).join(', ')})` +
    (start.some((v) => v > 1000) ? '（超出 0～1000，显示已裁剪，模型不裁剪）' : '');
  $('#path-start-hex').textContent = shex.toUpperCase();
  const tgt = [intVal($('#path-target-r')), intVal($('#path-target-g')), intVal($('#path-target-b'))];
  const thex = linearToSrgbHex(tgt);
  $('#path-target-swatch').style.background = thex;
  $('#path-target-swatch').title = `终点线性 RGB(${tgt.join(', ')})`;
  $('#path-target-hex').textContent = thex.toUpperCase();
  $('#path-tol-echo').textContent = `±${intVal($('#tolerance'))}`;
}

function readPathForm() {
  $$('#path-start-tbody tr').forEach((tr) => {
    pathStart[tr.dataset.id] = intVal(tr.querySelector('.f-pstart'), 0);
  });
  return {
    target: {
      r: intVal($('#path-target-r')),
      g: intVal($('#path-target-g')),
      b: intVal($('#path-target-b')),
    },
    steps: intVal($('#path-steps')),
    delta: intVal($('#path-delta')),
    tolerance: intVal($('#tolerance')),
  };
}

function validatePathForm(f) {
  const errs = [];
  if (!channels.every((c) => c.id)) errs.push('每路通道都必须填写 ID，才能规划路径。');
  const ids = channels.map((c) => c.id);
  if (new Set(ids).size !== ids.length) errs.push('通道 ID 必须唯一，才能规划路径。');
  channels.forEach((c) => {
    const d = pathStart[c.id];
    if (!Number.isInteger(d) || d < 0 || d > 255)
      errs.push(`通道 “${c.id}” 的起始驱动级必须是 0～255 的整数。`);
  });
  for (const k of ['r', 'g', 'b']) {
    if (!Number.isInteger(f.target[k]) || f.target[k] < 0 || f.target[k] > 1000)
      errs.push(`终点 ${k.toUpperCase()} 必须是 0～1000 的整数。`);
  }
  if (!Number.isInteger(f.steps) || f.steps < 2 || f.steps > 5)
    errs.push('等时步骤数必须是 2～5 的整数。');
  if (!Number.isInteger(f.delta) || f.delta < 1 || f.delta > 255)
    errs.push('每步驱动变化上限必须是 1～255 的整数。');
  if (f.tolerance < 0 || f.tolerance > 20)
    errs.push('② 的共用容差必须是 0～20 的整数。');
  return [...new Set(errs)];
}

function clearPathResult() {
  pathResult = null;
  $('#path-result').hidden = true;
  $('#path-detail').hidden = true;
  const st = $('#path-status');
  st.className = 'calib-status';
  st.textContent = '';
}

async function onSolvePath() {
  clearBanner();
  readForm(); // 先同步通道表与 ② 的容差
  const f = readPathForm();
  const errs = validatePathForm(f);
  if (errs.length) {
    clearPathResult();
    showBanner('error', '<strong>路径参数有误，已拒绝提交：</strong><br>' +
      errs.map(esc).join('<br>'));
    return;
  }
  const btn = $('#path-solve-btn');
  btn.disabled = true;
  btn.textContent = '规划中…';
  try {
    const resp = await fetch('/api/path/solve', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        channels: channels.map(({ id, r, g, b, power_mw }) => ({ id, r, g, b, power_mw })),
        start_drives: Object.fromEntries(channels.map((c) => [c.id, pathStart[c.id] ?? 0])),
        target_r: f.target.r,
        target_g: f.target.g,
        target_b: f.target.b,
        steps: f.steps,
        max_step_delta: f.delta,
        tolerance: f.tolerance,
      }),
    });
    const data = await resp.json();
    if (!resp.ok) {
      clearPathResult();
      showBanner('error', '<strong>请求被拒绝：</strong><br>' +
        errDetailHtml(data, '路径规划失败'));
      return;
    }
    renderPathResult(data);
  } catch (e) {
    clearPathResult();
    showBanner('error', `请求失败：${esc(e.message)}。请确认后端服务已启动。`);
  } finally {
    btn.disabled = false;
    btn.textContent = '规划最小功率调光路径';
  }
}

function renderPathResult(data) {
  pathResult = data;
  $('#path-result').hidden = false;
  const status = $('#path-status');
  const MAP = {
    optimal: ['s-unique', '✔ 最优完整路径'],
    infeasible: ['s-contradictory', '⛔ 无完整路径'],
    incomplete: ['s-unknown', '⏳ 搜索未完成'],
  };
  const [cls, label] = MAP[data.status] || ['s-unknown', data.status];
  status.className = `calib-status ${cls}`;
  status.innerHTML = `<strong>${label}</strong> ${esc(data.message)}`;

  if (data.status !== 'optimal') {
    // 无完整路径 / 搜索未完成：不展示任何步骤
    $('#path-detail').hidden = true;
    return;
  }
  $('#path-detail').hidden = false;
  const tol = data.tolerance;
  const K = data.steps.length;

  // ---- 时间线：起点（第 0 步）→ 第 1 步 → … → 第 K 步 ----
  const nodeHtml = (title, actual, ref, errs, power) => {
    const lines = [
      `<div class="path-node-meta">实际 (${actual.map(fmt).join(', ')})</div>`,
    ];
    if (ref) lines.push(`<div class="path-node-meta">参考 (${ref.map(fmt).join(', ')})</div>`);
    if (errs) lines.push(`<div class="path-node-meta">误差 (${errs.map(fmtSigned).join(', ')})</div>`);
    if (power != null) lines.push(`<div class="path-node-meta">功率 ${fmt(power)} mW</div>`);
    return `<div class="path-node">
      <div class="path-node-title">${esc(title)}</div>
      <div class="swatch" style="background:${linearToSrgbHex(actual)}"
           title="实际线性 RGB(${actual.map(fmt).join(', ')})"></div>
      ${lines.join('')}
    </div>`;
  };
  const nodes = [nodeHtml('起点（第 0 步）', data.start_actual_rgb, null, null, null)];
  data.steps.forEach((st) => {
    nodes.push(nodeHtml(
      st.step === K ? `第 ${st.step} 步（终点）` : `第 ${st.step} 步`,
      st.actual_rgb, st.ref_rgb, st.errors, st.power_mw));
  });
  $('#path-timeline').innerHTML = nodes.join('<div class="swatch-arrow">→</div>');

  // ---- 驱动级矩阵：行 = 通道，列 = 时间 ----
  $('#path-drive-head').innerHTML =
    '<th>ID</th><th>起点</th>' +
    data.steps.map((st) => `<th>第 ${st.step} 步</th>`).join('');
  $('#path-drive-tbody').innerHTML = data.channel_ids.map((id, i) => {
    const cells = [`<td><strong>${esc(id)}</strong></td>`];
    let prev = data.start_drives[i];
    cells.push(`<td>${prev}</td>`);
    data.steps.forEach((st) => {
      const d = st.drives[i];
      const dd = d - prev;
      const bad = Math.abs(dd) > data.max_step_delta;
      cells.push(`<td>${d} <span class="delta${bad ? ' bad' : ''}">` +
        `(Δ${dd >= 0 ? '+' : ''}${dd})</span></td>`);
      prev = d;
    });
    return `<tr>${cells.join('')}</tr>`;
  }).join('');

  // ---- 分量对照：每步 3 行（R/G/B）----
  $('#path-comp-head').innerHTML =
    '<th>步骤</th><th>分量</th><th>下限</th><th>参考</th><th>上限</th>' +
    '<th>实际</th><th>误差(实际−参考)</th><th>判定</th>';
  const names = ['R', 'G', 'B'];
  const ccls = ['r', 'g', 'b'];
  const rows = [];
  data.steps.forEach((st) => {
    for (let c = 0; c < 3; c++) {
      const lo = st.ref_rgb[c] - tol;
      const hi = st.ref_rgb[c] + tol;
      const ok = st.in_range[c];
      rows.push(`<tr>
        ${c === 0 ? `<td rowspan="3">第 ${st.step} 步</td>` : ''}
        <td class="cname ${ccls[c]}">${names[c]}</td>
        <td>${fmt(Math.max(0, lo))}</td>
        <td>${fmt(st.ref_rgb[c])}</td>
        <td>${fmt(Math.min(1000, hi))}</td>
        <td>${fmt(st.actual_rgb[c])}</td>
        <td>${fmtSigned(st.errors[c])}</td>
        <td class="judge ${ok ? 'ok' : 'bad'}">${ok ? '✔ 在区间内' : '✘ 超差'}</td>
      </tr>`);
    }
  });
  $('#path-comp-tbody').innerHTML = rows.join('');

  // ---- 各步功率之和 ----
  $('#path-total-power').textContent = fmt(data.total_power_mw);
  $('#path-power-frac').textContent =
    `精确值 ${data.total_power_num} / 255 mW（${K} 步合计）`;
}

/* ---------------- 事件绑定 ---------------- */

function bindEvents() {
  $('#add-channel').addEventListener('click', () => {
    if (channels.length >= 6) return;
    let n = channels.length + 1;
    let id = `LED${n}`;
    const used = new Set(channels.map((c) => c.id));
    while (used.has(id)) { n += 1; id = `LED${n}`; }
    channels.push({ id, r: 500, g: 500, b: 500, power_mw: 3000 });
    reconcileCalibSamples();
    renderChannels();
    renderCalibTable();
    clearCalibResult();
    clearRobustResult();
    renderPathStartTable();
    clearPathResult();
  });

  $('#reset-sample').addEventListener('click', () => {
    channels = clone(SAMPLE.channels);
    $('#target-r').value = SAMPLE.target.r;
    $('#target-g').value = SAMPLE.target.g;
    $('#target-b').value = SAMPLE.target.b;
    $('#tolerance').value = SAMPLE.tolerance;
    calibSamples = defaultCalibSamples();
    // 路径参数一并恢复默认（起点全 0、终点同 ② 目标色、3 步、上限 40）
    pathStart = {};
    $('#path-target-r').value = SAMPLE.target.r;
    $('#path-target-g').value = SAMPLE.target.g;
    $('#path-target-b').value = SAMPLE.target.b;
    $('#path-steps').value = 3;
    $('#path-delta').value = 40;
    renderChannels();
    renderCalibTable();
    clearCalibResult();
    clearRobustResult();
    renderPathStartTable();
    clearPathResult();
    refreshTargetSwatch();
    clearBanner();
  });

  $('#channel-tbody').addEventListener('input', (e) => {
    const tr = e.target.closest('tr');
    if (tr) syncRowState(tr);
    clearRobustResult();
    renderPathStartTable();
    clearPathResult();
  });
  $('#channel-tbody').addEventListener('click', (e) => {
    if (!e.target.classList.contains('del-btn')) return;
    const tr = e.target.closest('tr');
    channels.splice(+tr.dataset.i, 1);
    reconcileCalibSamples();
    renderChannels();
    renderCalibTable();
    clearCalibResult();
    clearRobustResult();
    renderPathStartTable();
    clearPathResult();
  });

  ['#target-r', '#target-g', '#target-b', '#tolerance'].forEach((sel) =>
    $(sel).addEventListener('input', () => {
      refreshTargetSwatch();
      // 容错配方沿用 ② 的目标与容差：变更后旧容错结果失效
      clearRobustResult();
    }));
  // 容差被路径规划沿用：变更后清除旧路径结果
  $('#tolerance').addEventListener('input', () => {
    refreshPathSwatches();
    clearPathResult();
  });

  $('#solve-btn').addEventListener('click', onSolve);

  $('#drive-tbody').addEventListener('input', (e) => {
    if (e.target.classList.contains('f-drive')) {
      manualMode = true;
      refreshPreview();
    }
  });
  $('#restore-opt').addEventListener('click', restoreOptimal);

  // ---- 标定 ----
  $('#add-sample').addEventListener('click', () => {
    if (calibSamples.length >= 12) return;
    calibSamples.push(emptySample());
    renderCalibTable();
    clearCalibResult();
  });

  $('#gen-samples').addEventListener('click', () => {
    // 用当前通道表中的满档 RGB 合成“每路单独满档点亮”的示例样本（功率不参与）
    const errs = [];
    channels.forEach((c) => {
      for (const k of ['r', 'g', 'b']) {
        if (!Number.isInteger(c[k]) || c[k] < 0 || c[k] > 1000)
          errs.push(`通道 “${c.id}” 的满档 ${k.toUpperCase()} 非法（须 0～1000 整数）。`);
      }
      if (c.r === 0 && c.g === 0 && c.b === 0)
        errs.push(`通道 “${c.id}” 满档 RGB 全为零，无法生成示例样本。`);
    });
    if (errs.length) {
      showBanner('error', '<strong>当前通道表不能生成示例样本：</strong><br>' +
        [...new Set(errs)].map(esc).join('<br>'));
      return;
    }
    clearBanner();
    calibSamples = channels.map((_, i) => {
      const drives = channels.map(() => 0);
      drives[i] = 255;
      const drivesById = {};
      channels.forEach((c, j) => { drivesById[c.id] = drives[j]; });
      return { drives: drivesById, measured: predictedFromChannels(channels, drives) };
    });
    renderCalibTable();
    clearCalibResult();
  });

  $('#sample-tbody').addEventListener('input', (e) => {
    const tr = e.target.closest('tr');
    if (tr) syncCalibRow(tr);
    clearCalibResult();
  });
  $('#sample-tbody').addEventListener('click', (e) => {
    if (!e.target.classList.contains('sample-del')) return;
    const tr = e.target.closest('tr');
    calibSamples.splice(+tr.dataset.s, 1);
    renderCalibTable();
    clearCalibResult();
  });

  $('#calib-btn').addEventListener('click', onCalibrate);
  $('#writeback-btn').addEventListener('click', onWriteback);

  // ---- 单路熄灭容错 ----
  $('#robust-solve-btn').addEventListener('click', onSolveRobust);
  $('#robust-delta').addEventListener('input', clearRobustResult);

  // ---- 调光路径 ----
  $('#path-start-tbody').addEventListener('input', (e) => {
    if (!e.target.classList.contains('f-pstart')) return;
    const tr = e.target.closest('tr');
    if (tr) pathStart[tr.dataset.id] = intVal(e.target, 0);
    refreshPathSwatches();
    clearPathResult();
  });
  ['#path-target-r', '#path-target-g', '#path-target-b',
   '#path-steps', '#path-delta'].forEach((sel) =>
    $(sel).addEventListener('input', () => {
      refreshPathSwatches();
      clearPathResult();
    }));
  $('#path-use-target').addEventListener('click', () => {
    $('#path-target-r').value = intVal($('#target-r'));
    $('#path-target-g').value = intVal($('#target-g'));
    $('#path-target-b').value = intVal($('#target-b'));
    refreshPathSwatches();
    clearPathResult();
  });
  $('#path-solve-btn').addEventListener('click', onSolvePath);
}

/* ---------------- 启动 ---------------- */

calibSamples = defaultCalibSamples();
renderChannels();
renderCalibTable();
renderPathStartTable();
refreshTargetSwatch();
bindEvents();
