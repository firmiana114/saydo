const $ = id => document.getElementById(id);
const cameras = {
  d435i: {active: false, startAt: 0}, d405: {active: false, startAt: 0}
};
const labels = {
  idle: '空闲',
  starting: '启动中',
  streaming: '实时采集中',
  stopping: '正在释放',
  error: '采集异常',
};
let errors = 0;

function clearFeed(id, title = '预览已释放') {
  cameras[id].active = false;
  $(id + '-feed').removeAttribute('src');
  $(id + '-feed').style.display = 'none';
  $(id + '-placeholder').style.display = 'block';
  $(id + '-title').textContent = title;
  $(id + '-start').disabled = false;
}

function start(id) {
  if (cameras[id].active) {
    return;
  }
  cameras[id].active = true;
  cameras[id].startAt = Date.now();
  $(id + '-error').textContent = '';
  $(id + '-start').disabled = true;
  $(id + '-stop').disabled = false;
  $(id + '-title').textContent = '正在连接相机…';
  $(id + '-feed').src = '/stream/' + id + '?t=' + Date.now();
}

async function stop(id) {
  const ids = id ? [id] : Object.keys(cameras);
  ids.forEach(key => {
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

async function status() {
  try {
    const r = await fetch('/api/status', {cache: 'no-store'});
    if (!r.ok) {
      throw Error(r.status);
    }
    const data = await r.json();
    errors = 0;
    $('service-error').textContent = '';
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
