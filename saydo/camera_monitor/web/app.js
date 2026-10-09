/**
 * 按元素编号取页面元素。
 * @param {string} id 元素编号。
 * @return {!Element} 对应的元素。
 */
const $ = id => document.getElementById(id);

/**
 * 创建一路相机在页面上的状态。
 * @param {boolean} present 服务是否提供这一路。
 * @return {{active: boolean, startAt: number, present: boolean,
 *     wanted: boolean, resume: boolean}} 初始状态：active 表示画面流已
 *     连接；wanted 表示用户想看这一路；resume 表示相机被采集程序借走，
 *     归还后要自动重连。
 */
function cameraState(present) {
  return {
    active: false, startAt: 0, present, wanted: false, resume: false
  };
}

// 2 臂手上的相机只有服务配置了才提供，先按“没有”处理，等状态返回再显示。
const cameras = {
  d435i: cameraState(true),
  d405: cameraState(true),
  d435i_arm2: cameraState(false),
};
const labels = {
  idle: '空闲',
  starting: '启动中',
  streaming: '实时采集中',
  stopping: '正在释放',
  error: '采集异常',
};
let errors = 0;

/**
 * 收起一路画面，恢复成“未预览”的样子。
 * @param {string} id 相机的键。
 * @param {string=} title 占位区显示的文字。
 */
function clearFeed(id, title = '预览已释放') {
  cameras[id].active = false;
  $(id + '-feed').removeAttribute('src');
  $(id + '-feed').style.display = 'none';
  $(id + '-placeholder').style.display = 'block';
  $(id + '-title').textContent = title;
  $(id + '-start').disabled = false;
}

/**
 * 开始预览一路相机。
 * @param {string} id 相机的键。
 */
function start(id) {
  if (cameras[id].active || !cameras[id].present) {
    return;
  }
  cameras[id].active = true;
  cameras[id].wanted = true;
  cameras[id].resume = false;
  cameras[id].startAt = Date.now();
  $(id + '-error').textContent = '';
  $(id + '-start').disabled = true;
  $(id + '-stop').disabled = false;
  $(id + '-title').textContent = '正在连接相机…';
  $(id + '-feed').src = '/stream/' + id + '?t=' + Date.now();
}

/**
 * 由用户释放预览；不再自动重连。
 * @param {string=} id 相机的键；不给时释放全部。
 * @return {!Promise<void>} 请求结束后完成。
 */
async function stop(id) {
  const ids = id ? [id] : Object.keys(cameras);
  ids.forEach(key => {
    cameras[key].wanted = false;
    cameras[key].resume = false;
    clearFeed(key);
    $(key + '-stop').disabled = true;
  });
  try {
    const r = await fetch(
        id ? '/api/cameras/' + id + '/pause' : '/api/pause', {method: 'POST'}
    );
    if (!r.ok) {
      throw Error(r.status);
    }
    await status();
  } catch (e) {
    $('service-error').textContent = '释放请求未确认，请检查服务连接。';
  }
}

for (const id of Object.keys(cameras)) {
  $(id + '-start').onclick = () => start(id);
  $(id + '-stop').onclick = () => stop(id);
  $(id + '-feed').onerror = () => clearFeed(id, '预览已结束，可重新连接');
  $(id + '-snapshot').onclick = e => {
    if (!cameras[id].active) {
      e.preventDefault();
      $(id + '-error').textContent = '请先开始此机位的预览。';
    }
  };
}
$('start-all').onclick = () => Object.keys(cameras).forEach(start);
$('stop-all').onclick = () => stop();

/**
 * 相机被采集程序借走时收起画面，归还后自动重连。
 *
 * 只在“借走再归还”这一种情况下自动重连。别的原因断开（出错、被别处
 * 释放）仍由用户决定是否重新开始，避免页面和其他程序抢相机。
 * @param {string} id 相机的键。
 * @param {!Object} s 服务返回的这一路状态。
 */
function followHold(id, s) {
  const camera = cameras[id];
  if (s.held_s > 0) {
    if (camera.wanted) {
      camera.resume = true;
    }
    if (camera.active) {
      clearFeed(id);
    }
    $(id + '-title').textContent =
        '相机已借给采集程序，结束后自动恢复预览';
    $(id + '-start').disabled = true;
    return;
  }
  if (camera.resume && s.state === 'idle') {
    start(id);
  }
}

/**
 * 读取服务状态并刷新页面。
 * @return {!Promise<void>} 刷新结束后完成。
 */
async function status() {
  try {
    const r = await fetch('/api/status', {cache: 'no-store'});
    if (!r.ok) {
      throw Error(r.status);
    }
    const data = await r.json();
    errors = 0;
    $('service-error').textContent = '';
    for (const id of Object.keys(cameras)) {
      cameras[id].present = id in data.cameras;
      $(id + '-card').hidden = !cameras[id].present;
    }
    for (const [id, s] of Object.entries(data.cameras)) {
      if (!cameras[id]) {
        continue;
      }
      $(id + '-state').textContent = labels[s.state] || s.state;
      $(id + '-dot').className = 'dot' + (s.state === 'streaming' ? ' on' : '');
      $(id + '-resolution').textContent = s.width + ' × ' + s.height;
      $(id + '-fps').textContent =
          (s.fps || '—') + ' / ' + s.target_fps + ' fps';
      $(id + '-frames').textContent =
          s.clients + ' 个连接 · ' + s.sequence + ' 帧';
      $(id + '-device').textContent =
          s.camera + ' · 序列号 ' + (s.camera_serial || '待确认');
      $(id + '-source').textContent =
          s.source === 'tracker' ?
          '共享 8089 原始画面' :
          s.source === 'device' ?
          '本服务直接采集' + (s.usb ? ' · USB ' + s.usb : '') :
          '等待采集';
      $(id + '-updated').textContent =
          s.frame_at ?
          '最近图像 ' +
          new Date(s.frame_at * 1000).toLocaleTimeString() +
          ' · ' +
          s.frame_age_ms +
          ' ms 前' :
          '尚未收到图像';
      $(id + '-error').textContent = s.error || '';
      $(id + '-stop').disabled = ![
        'starting', 'streaming', 'stopping'
      ].includes(s.state);
      if (cameras[id].active && s.state === 'streaming') {
        $(id + '-feed').style.display = 'block';
        $(id + '-placeholder').style.display = 'none';
      }
      if (
          s.state === 'error' ||
          (
              cameras[id].active &&
              s.state === 'idle' &&
              Date.now() - cameras[id].startAt > 3000
          )
      ) {
        clearFeed(
            id, s.state === 'error' ? '暂时无法读取此机位' : '预览已释放'
        );
      }
      followHold(id, s);
    }
  } catch (e) {
    if (++errors >= 2) {
      $(
          'service-error'
      ).textContent = '无法连接相机监控服务，请检查网络或刷新页面。';
    }
  }
}

status();
setInterval(status, 1000);
