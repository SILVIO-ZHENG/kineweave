/* Input regressions without a physical browser/device.
 * Run the actual app.js in an isolated JS VM; no robot SDK or network is opened.
 */
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
const elements = new Map();
let focused = true, pads = [], now = 1000;
function element(id) {
  if (!elements.has(id)) elements.set(id, {id, value:'', textContent:'', className:'', hidden:false,
    classList:{add(){},remove(){}},addEventListener(){},replaceChildren(){},append(){},add(){}});
  return elements.get(id);
}
const context = vm.createContext({console, URLSearchParams, Date, Math, Set, Map, JSON,
  crypto:{randomUUID:()=> 'test'},location:{hash:'#token=test'},
  performance:{now:()=>now},navigator:{getGamepads:()=>pads},
  window:{addEventListener(){}},document:{hidden:false,hasFocus:()=>focused,
    getElementById:element,addEventListener(){},querySelectorAll:()=>[]},
  setInterval(){},fetch(){throw new Error('Network is forbidden in this input test');}});
let source = fs.readFileSync(path.join(__dirname,'../src/teleop_v2/web/app.js'),'utf8');
source = source.replace('setInterval(poll,150);setInterval(sendInput,50);poll();refreshEpisodes();','');
vm.runInContext(source, context);
vm.runInContext(`profile={joints:[],chains:{right:{}},effectors:{right:{}}};`,context);
element('inputMode').value='cartesian';element('task').value='right';
element('speed').value='.08';element('grasp').value='.8';
const frame=()=>JSON.parse(vm.runInContext('JSON.stringify(currentFrame())',context));
pads=[{id:'test-pad',axes:[0,0,0,0],buttons:Array.from({length:8},()=>({value:0,pressed:false}))}];
pads[0].buttons[4].pressed=true;
assert.equal(frame().enabled,true);
assert.equal(frame().grasp.right,0,'Released trigger must remain exactly zero');
focused=false;
assert.equal(frame().enabled,false,'Unfocused gamepad must not enable motion');
focused=true;context.document.hidden=true;
assert.equal(frame().enabled,false,'Hidden pages must not enable gamepad motion');
context.document.hidden=false;
element('inputMode').value='xr';
vm.runInContext('xrFrameData={enabled:true,valid:true,poses:{right:{}},sent_wall_ns:123};xrSampleTime=900;',context);
assert.equal(frame().enabled,true);
assert.equal(frame().sent_wall_ns,123,'Heartbeats must retain original XR capture time');
now=1100;
assert.equal(frame().enabled,false,'Frozen XR tracking must expire independently of HTTP heartbeat');
assert.equal(frame().valid,false);
console.log('Browser input regressions passed: trigger zero, focus, visibility, XR freshness.');
