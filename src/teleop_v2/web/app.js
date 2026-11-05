/* Robot-independent browser input and visualization for KineWeave.
 * The backend is the sole command authority. UI packets expire without a fresh
 * heartbeat; display playback never sends recorded commands to any robot.
 */
'use strict';
const $ = id => document.getElementById(id);
const token = new URLSearchParams(location.hash.slice(1)).get('token') || '';
const source = `browser-${crypto.randomUUID ? crypto.randomUUID() : Date.now()}`;
let sequence = 0, state = null, profile = null, held = false, playback = null;
let xrSession = null, xrFrameData = null, xrSampleTime = 0, sending = false, pollBusy = false;
let selectedJoint = null, jointTarget = 0, lastCamera = 0;
const keys = new Set();
const cameraURLs = new Map();

function message(text, error = false) { $('message').textContent = text; $('message').className = error ? 'error' : ''; }
async function api(path, body) {
  const response = await fetch(path, {method: body === undefined ? 'GET' : 'POST',
    headers: {'X-Teleop-Token': token, 'Content-Type': 'application/json'},
    ...(body === undefined ? {} : {body: JSON.stringify(body)})});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || response.statusText);
  return data;
}
async function action(name, extra = {}) {
  try { await api('/api/action', {action: name, ...extra}); message(`${name} ✓`); }
  catch (error) { message(error.message, true); }
}

function initializeProfile(p) {
  profile = p;
  $('robotName').textContent = p.display_name;
  $('robotDetail').textContent = `${p.id} · ${p.joints.length} coordinates`;
  $('task').replaceChildren();
  for (const name of Object.keys(p.chains)) { const option = new Option(name, name); $('task').add(option); }
  $('jointSelect').replaceChildren(); $('joints').replaceChildren();
  const groups = state?.control_groups || p.metadata.default_control_groups || [];
  for (const joint of p.joints) {
    const row = document.createElement('div'); row.className = 'joint';
    const line = document.createElement('div'); line.className = 'joint-label';
    const name = document.createElement('span'); name.textContent = joint.name; name.title = `${joint.group} · ${joint.unit}`;
    const value = document.createElement('code'); value.dataset.joint = joint.name;
    line.append(name, value); const bar = document.createElement('div'); bar.className = 'bar';
    const fill = document.createElement('div'); fill.dataset.bar = joint.name; bar.append(fill); row.append(line, bar); $('joints').append(row);
    if (!groups.length || groups.includes(joint.group)) $('jointSelect').add(new Option(joint.name, joint.name));
  }
  chooseJoint();
}
function chooseJoint() {
  if (!profile) return;
  selectedJoint = profile.joints.find(j => j.name === $('jointSelect').value);
  if (!selectedJoint) return;
  const index = profile.joints.indexOf(selectedJoint);
  jointTarget = state?.state[index] ?? selectedJoint.initial;
  $('jointTarget').min = selectedJoint.lower; $('jointTarget').max = selectedJoint.upper;
  $('jointTarget').value = jointTarget; updateJointLabel();
}
function updateJointLabel() { $('jointTargetValue').textContent = `${Number(jointTarget).toFixed(4)} ${selectedJoint?.unit || ''}`; }

async function poll() {
  if (pollBusy) return; pollBusy = true;
  try {
    state = await api('/api/state');
    if (!profile || profile.id !== state.profile.id) initializeProfile(state.profile);
    $('connection').textContent = 'Connected'; $('connection').className = 'pill active';
    $('hardwareNote').textContent = state.hardware ? (state.writable ? 'Hardware writable' : 'Hardware read-only') : 'Simulation';
    $('hardwareNote').className = `notice ${state.hardware ? 'hardware' : ''}`;
    $('mode').textContent = state.mode.toUpperCase();
    $('mode').className = `pill ${state.mode === 'active' ? 'active' : ['fault','estop'].includes(state.mode) ? 'fault' : ''}`;
    $('dimensions').textContent = profile.joints.length;
    $('latency').textContent = state.input_age_ms === null ? '—' : `${Math.round(state.input_age_ms)} ms`;
    $('cycles').textContent = state.sequence.toLocaleString();
    $('recordStatus').textContent = state.recording ? '● REC' : 'IDLE';
    $('record').textContent = state.recording ? 'Stop recording' : 'Start recording';
    $('episodePath').textContent = state.episode || 'HDF5 · JSONL · JPEG + timestamps';
    $('arm').disabled = Boolean(playback) || !state.writable || Boolean(state.fault) || state.mode === 'estop';
    $('events').replaceChildren(...state.events.slice(-8).reverse().map(event => {
      const row = document.createElement('div'); row.textContent = `${new Date(event.wall_time_ns/1e6).toLocaleTimeString()} · ${event.kind}`; return row;
    }));
    if (state.fault) message(state.fault, true);
    else if (Object.keys(state.camera_errors || {}).length) message(JSON.stringify(state.camera_errors), true);
    renderState();
    if (performance.now()-lastCamera > 500) { lastCamera = performance.now(); await refreshCameras(); }
  } catch (error) {
    $('connection').textContent = 'Offline'; $('connection').className = 'pill fault';
    message(error.message || 'Connection failed', true);
  } finally { pollBusy = false; }
}

function renderState() {
  const display = playback ? playback.frames[Number($('timeline').value)] : state;
  if (!display) return;
  const p = playback ? playback.profile : profile;
  if (!playback || p.id === profile.id) {
    for (let i = 0; i < p.joints.length; i++) {
      const joint = p.joints[i], q = display.state[i];
      const label = [...document.querySelectorAll('[data-joint]')].find(el => el.dataset.joint === joint.name);
      const bar = [...document.querySelectorAll('[data-bar]')].find(el => el.dataset.bar === joint.name);
      if (label) label.textContent = `${q.toFixed(3)} ${joint.unit}`;
      if (bar) bar.style.width = `${100*Math.min(1, Math.max(0, (q-joint.lower)/(joint.upper-joint.lower)))}%`;
    }
  }
  draw(display.points);
}

/* Draw a local FK skeleton without a mesh download or a third-party CDN.
 */
let yaw = 0.8, pitch = 0.35, zoom = 1, dragging = null;
function draw(chains = {}) {
  const canvas = $('robotView'), dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w*dpr || canvas.height !== h*dpr) {canvas.width=w*dpr; canvas.height=h*dpr;}
  const ctx = canvas.getContext('2d'); ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,w,h);
  const all = Object.values(chains).flat();
  const centerZ = all.length ? all.reduce((a,p)=>a+p[2],0)/all.length : 0.5;
  const project = p => {const x=p[0],y=p[1],z=p[2]-centerZ; const a=Math.cos(yaw)*x-Math.sin(yaw)*y,b=Math.sin(yaw)*x+Math.cos(yaw)*y; return [w/2+a*190*zoom,h*.5-(Math.cos(pitch)*z-Math.sin(pitch)*b)*190*zoom];};
  function line(a,b,color,width=1) {const p=project(a),q=project(b);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke();}
  for(let i=-6;i<=6;i++){line([i*.15,-.9,0],[i*.15,.9,0],'#243244');line([-.9,i*.15,0],[.9,i*.15,0],'#243244');}
  line([0,0,0],[.22,0,0],'#da797f',2);line([0,0,0],[0,.22,0],'#58dfbc',2);line([0,0,0],[0,0,.22],'#75a8ff',2);
  const colors=['#58dfbc','#75a8ff','#d7a6ff']; let index=0;
  for(const [name,points] of Object.entries(chains)) {
    const color=colors[index++%colors.length];
    for(let i=1;i<points.length;i++)line(points[i-1],points[i],color,5);
    for(let i=0;i<points.length;i++){const p=project(points[i]);ctx.beginPath();ctx.arc(...p,i===points.length-1?6:4,0,Math.PI*2);ctx.fillStyle=i===points.length-1?'#ecf5ff':'#172331';ctx.fill();ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();}
    if(points.length){const p=project(points.at(-1));ctx.font='11px Segoe UI';ctx.fillStyle=color;ctx.fillText(name,p[0]+12,p[1]-10);}
  }
}
$('robotView').addEventListener('pointerdown',e=>{dragging=[e.clientX,e.clientY];e.target.setPointerCapture(e.pointerId);});
$('robotView').addEventListener('pointermove',e=>{if(!dragging)return;yaw+=(e.clientX-dragging[0])*.008;pitch=Math.max(-1.1,Math.min(1.1,pitch+(e.clientY-dragging[1])*.008));dragging=[e.clientX,e.clientY];renderState();});
$('robotView').addEventListener('pointerup',()=>dragging=null);
$('robotView').addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.4,Math.min(3,zoom*Math.exp(-e.deltaY*.001)));renderState();},{passive:false});

function currentFrame() {
  const frame = {source_id:source,sequence:++sequence,enabled:false,sent_wall_ns:Date.now()*1e6,
    frame_id:'robot_base',twists:{},grasp:{},raw:{device:'browser'}};
  if (!profile || playback) return frame;
  const mode = $('inputMode').value;
  if (mode === 'xr') {
    // Heartbeats must not rejuvenate a frozen tracking sample; only XR frames refresh it.
    if (xrFrameData && performance.now()-xrSampleTime < 150) Object.assign(frame, xrFrameData);
    else {frame.enabled=false;frame.valid=false;}
    return frame;
  }
  const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : [];
  const pad = pads[0], deadzone = x => Math.abs(x || 0)<.12 ? 0 : x;
  $('deviceInfo').textContent = pad ? pad.id : 'Keyboard input';
  frame.enabled = !document.hidden && document.hasFocus() &&
    (held || keys.has('Space') || Boolean(pad?.buttons[4]?.pressed));
  const task = $('task').value, speed=Number($('speed').value);
  if (mode === 'joint' && selectedJoint) frame.joints = {[selectedJoint.name]:Number(jointTarget)};
  else if (task) {
    const k = (positive,negative) => Number(keys.has(positive))-Number(keys.has(negative));
    frame.twists[task] = [speed*(k('ArrowUp','ArrowDown')-deadzone(pad?.axes[1])),
      speed*(k('ArrowLeft','ArrowRight')-deadzone(pad?.axes[0])),speed*(k('KeyW','KeyS')-deadzone(pad?.axes[3])),
      .4*k('KeyI','KeyK'),.4*k('KeyU','KeyO'),.4*(k('KeyJ','KeyL')-deadzone(pad?.axes[2]))];
  }
  if (profile.effectors[task] && mode !== 'joint') frame.grasp[task] = pad?.buttons[7]?.value ?? Number($('grasp').value);
  frame.raw = {device:pad?'gamepad':'keyboard',axes:pad?[...pad.axes]:[],buttons:pad?pad.buttons.map(b=>b.value):[],keys:[...keys]};
  return frame;
}
async function sendInput() {
  if (sending || !profile || playback) return; sending = true;
  try {await api('/api/input',currentFrame());} catch(error) {message(error.message,true);} finally {sending=false;}
}
window.addEventListener('keydown',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;if(['Space','ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.code))e.preventDefault();keys.add(e.code);});
window.addEventListener('keyup',e=>keys.delete(e.code));
window.addEventListener('blur',()=>{keys.clear();held=false;});
document.addEventListener('visibilitychange',()=>{if(document.hidden){keys.clear();held=false;xrFrameData=null;}});
$('deadman').addEventListener('pointerdown',e=>{held=true;e.target.setPointerCapture(e.pointerId);$('deadman').classList.add('pressed');});
for(const name of ['pointerup','pointercancel','lostpointercapture'])$('deadman').addEventListener(name,()=>{held=false;$('deadman').classList.remove('pressed');});
$('arm').onclick=async()=>{await sendInput();await action('arm',{source_id:source});};
for(const [id,name] of [['pause','pause'],['reset','reset'],['estop','estop'],['calibrate','calibrate']])$(id).onclick=()=>action(name);
$('record').onclick=()=>action(state?.recording?'record_stop':'record_start',{task:$('taskText').value});
$('inputMode').onchange=()=>{held=false;keys.clear();$('jointControl').hidden=$('inputMode').value!=='joint';chooseJoint();action('calibrate');};
$('task').onchange=()=>action('calibrate');
$('jointSelect').onchange=()=>{chooseJoint();action('calibrate');};
$('jointTarget').oninput=()=>{jointTarget=Number($('jointTarget').value);updateJointLabel();};
$('speed').oninput=()=>$('speedValue').textContent=`${Number($('speed').value).toFixed(2)} m/s`;
$('grasp').oninput=()=>$('graspValue').textContent=`${Math.round(Number($('grasp').value)*100)}%`;

async function refreshEpisodes() {try{const data=await api('/api/episodes');$('episodes').replaceChildren(...data.episodes.map(id=>new Option(id,id)));}catch(e){message(e.message,true);}}
$('refreshEpisodes').onclick=refreshEpisodes;
$('loadEpisode').onclick=async()=>{try{await action('disarm');playback=await api(`/api/episode?id=${encodeURIComponent($('episodes').value)}`);$('timeline').max=Math.max(0,playback.frames.length-1);$('timeline').value=0;$('timeline').disabled=false;$('playbackInfo').textContent=`${playback.profile.id} · ${playback.sample_count} samples · display stride ${playback.stride}`;$('cameras').replaceChildren();renderState();refreshCameras();}catch(e){message(e.message,true);}};
$('timeline').oninput=()=>{renderState();refreshCameras();};
$('live').onclick=()=>{playback=null;$('timeline').disabled=true;$('cameras').replaceChildren();renderState();refreshCameras();};
let cameraGeneration=0;
async function refreshCameras(){
  const generation=++cameraGeneration, replay=playback;
  const names=replay?[...new Set(replay.cameras.map(frame=>frame.camera))]:state.cameras;
  for(const name of names){try{
    let endpoint=`/api/camera/${encodeURIComponent(name)}`;
    if(replay){
      const time=replay.frames[Number($('timeline').value)]?.wall_time_ns;
      const frames=replay.cameras.filter(frame=>frame.camera===name);
      if(!frames.length||time===undefined)continue;
      const selected=frames.reduce((best,frame)=>Math.abs(frame.timestamp_ns-time)<Math.abs(best.timestamp_ns-time)?frame:best);
      endpoint=`/api/episode-camera?id=${encodeURIComponent(replay.id)}&camera=${encodeURIComponent(name)}&frame=${selected.frame_index}`;
    }
    const response=await fetch(endpoint,{headers:{'X-Teleop-Token':token}});if(!response.ok)continue;
    const blob=await response.blob();if(generation!==cameraGeneration)return;
    let image=document.getElementById(`camera-${name}`);
    if(!image){const label=document.createElement('label');label.textContent=`${replay?'Replay':'Live'} · ${name}`;image=document.createElement('img');image.id=`camera-${name}`;label.append(image);$('cameras').append(label);}
    const url=URL.createObjectURL(blob);image.src=url;
    if(cameraURLs.has(name))URL.revokeObjectURL(cameraURLs.get(name));cameraURLs.set(name,url);
  }catch(_error){/* Retry display on the next poll. */}}
}

/* WebXR comes from the device browser. Full hand joints are recorded as raw
 * data; current hand control uses pinch/grasp synergy, not independent-finger IK.
 */
async function startXR(mode) {
  try {
    if (!navigator.xr) throw new Error('WebXR unavailable; remote devices require trusted HTTPS');
    if (!await navigator.xr.isSessionSupported(mode)) throw new Error(`${mode} unsupported`);
    xrSession=await navigator.xr.requestSession(mode,{optionalFeatures:['local-floor','hand-tracking','dom-overlay'],domOverlay:{root:$('xrOverlay')}});
    $('inputMode').value='xr';$('jointControl').hidden=true;$('xrOverlay').classList.add('xr-active');
    await action('calibrate');
    const canvas=document.createElement('canvas'),gl=canvas.getContext('webgl',{xrCompatible:true,alpha:true});
    if(!gl)throw new Error('WebGL unavailable');
    await gl.makeXRCompatible();xrSession.updateRenderState({baseLayer:new XRWebGLLayer(xrSession,gl)});
    let space;try{space=await xrSession.requestReferenceSpace('local-floor');}catch(_e){space=await xrSession.requestReferenceSpace('local');}
    space.addEventListener('reset',()=>{xrFrameData=null;action('calibrate');});
    xrSession.addEventListener('end',()=>{xrSession=null;xrFrameData=null;$('xrOverlay').classList.remove('xr-active');action('pause');});
    const poseObject=pose=>({position:[pose.transform.position.x,pose.transform.position.y,pose.transform.position.z],quaternion:[pose.transform.orientation.x,pose.transform.orientation.y,pose.transform.orientation.z,pose.transform.orientation.w]});
    function frameLoop(_time,frame){
      if(!xrSession)return;
      const packet={frame_id:'webxr',poses:{},grasp:{},raw:{device:'webxr',hands:{},head:null},enabled:false,valid:true};
      const viewer=frame.getViewerPose(space);if(viewer)packet.raw.head=poseObject(viewer);
      for(const input of xrSession.inputSources){const side=input.handedness;if(!['left','right'].includes(side)||!profile.chains[side])continue;let pose=null,enable=false,grasp=0;
        if(input.hand){const wrist=input.hand.get('wrist');pose=wrist?frame.getJointPose(wrist,space):null;const joints={};for(const [name,jointSpace]of input.hand){const p=frame.getJointPose(jointSpace,space);if(p)joints[name]=poseObject(p);}packet.raw.hands[side]=joints;const a=joints['thumb-tip']?.position,b=joints['index-finger-tip']?.position;if(a&&b){const distance=Math.hypot(...a.map((v,i)=>v-b[i]));enable=distance<.025;grasp=Math.max(0,Math.min(1,(.06-distance)/.04));}}
        else{pose=input.gripSpace?frame.getPose(input.gripSpace,space):null;enable=Boolean(input.gamepad?.buttons[1]?.pressed);grasp=input.gamepad?.buttons[0]?.value||0;}
        if(pose&&enable&&!pose.emulatedPosition){packet.poses[side]=poseObject(pose);if(profile.effectors[side])packet.grasp[side]=grasp;packet.enabled=true;}
      }
      packet.sent_wall_ns=Date.now()*1e6;xrSampleTime=performance.now();xrFrameData=packet;sendInput();
      const layer=xrSession.renderState.baseLayer;gl.bindFramebuffer(gl.FRAMEBUFFER,layer.framebuffer);gl.clearColor(.02,.06,.08,mode==='immersive-ar'?0:1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
      xrSession.requestAnimationFrame(frameLoop);
    }
    xrSession.requestAnimationFrame(frameLoop);
  }catch(error){message(error.message,true);if(xrSession)xrSession.end();}
}
$('vr').onclick=()=>startXR('immersive-vr');$('ar').onclick=()=>startXR('immersive-ar');$('exitXR').onclick=()=>xrSession?.end();
setInterval(poll,150);setInterval(sendInput,50);poll();refreshEpisodes();
