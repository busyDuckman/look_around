// lookaround testbed A viewer: a capsule mannequin driven by poses streamed from the
// Python reference implementation (testbed/animation/server.py).
//
// Everything lookaround-related lives in `world`, a group rotated so that its local frame is
// lookaround's mount frame M: x forward, y left, z up (three.js itself is y-up).

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const DEG = 180 / Math.PI;
const query = new URLSearchParams(location.search);
const wsPort = query.get('ws') || '8620';
let initial = { preset: query.get('preset'), path: query.get('path') };  // optional deep link

// ---- scene -------------------------------------------------------------------------------

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1216);
scene.fog = new THREE.Fog(0x0f1216, 9, 20);

const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.05, 100);
camera.position.set(2.7, 1.9, 2.5);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 1.0, 0);
controls.enableDamping = true;
controls.update();

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1d22, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 2.2);
sun.position.set(3, 6, 2);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -4, right: 4, top: 4, bottom: -4, near: 0.5, far: 20 });
scene.add(sun);

const ground = new THREE.Mesh(
  new THREE.CircleGeometry(12, 64),
  new THREE.MeshStandardMaterial({ color: 0x161a20, roughness: 1 }),
);
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);
const grid = new THREE.GridHelper(12, 24, 0x2c333d, 0x20262e);
grid.position.y = 0.001;
scene.add(grid);

const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;   // world's local frame = lookaround M (z up)
scene.add(world);

// +x marker on the floor: the mount frame's forward direction
const fwd = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.002), 0.6, 0x3d7bd9, 0.12, 0.06);
world.add(fwd);

// ---- materials and shape helpers -------------------------------------------------------------

const mat = {
  body: new THREE.MeshStandardMaterial({ color: 0xc9ced6, roughness: 0.55, metalness: 0.05 }),
  joint: new THREE.MeshStandardMaterial({ color: 0x5d6673, roughness: 0.6 }),
  dark: new THREE.MeshStandardMaterial({ color: 0x2b3038, roughness: 0.8 }),
  chair: new THREE.MeshStandardMaterial({ color: 0x3a4250, roughness: 0.7 }),
  sclera: new THREE.MeshStandardMaterial({ color: 0xf4f4f0, roughness: 0.3 }),
  iris: new THREE.MeshStandardMaterial({ color: 0x2f6fae, roughness: 0.3 }),
  pupil: new THREE.MeshStandardMaterial({ color: 0x050505, roughness: 0.2 }),
  ball: new THREE.MeshStandardMaterial({ color: 0xff8a3d, roughness: 0.35, emissive: 0xff5a10, emissiveIntensity: 0.55 }),
};

function mesh(geo, material, parent, pos = [0, 0, 0]) {
  const m = new THREE.Mesh(geo, material);
  m.position.set(...pos);
  m.castShadow = true;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}

// Capsule between two points (in the parent's frame).
function limb(parent, a, b, r, material = mat.body) {
  const va = new THREE.Vector3(...a), vb = new THREE.Vector3(...b);
  const len = va.distanceTo(vb);
  const m = mesh(new THREE.CapsuleGeometry(r, Math.max(len, 1e-4), 6, 16), material, parent);
  m.position.copy(va).add(vb).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), vb.clone().sub(va).normalize());
  return m;
}

function box(parent, size, pos, material = mat.chair) {
  return mesh(new THREE.BoxGeometry(...size), material, parent, pos);
}

// ---- mannequin built from the settings document ------------------------------------------------

let body = null;   // { root, groups: {base, torso, head}, eyes: {left, right}, eyePoint, settings }
const EYE_R = 0.022;   // drawn eyeball radius (m)

function offsetOf(part) {
  const o = (part && part.offset) || {};
  return o.xyz || [0, 0, 0];
}

function buildBody(settings) {
  if (body) world.remove(body.root);
  const root = new THREE.Group();
  world.add(root);
  const groups = {};
  let parent = root;

  const torsoZ = settings.torso ? offsetOf(settings.torso)[2] : 0.9;
  const headZ = settings.head ? offsetOf(settings.head)[2] : 0.6;
  const seated = torsoZ < 0.7;
  const swivel = seated && !!settings.base;

  // base: legs (standing) or chair + legs (seated); turns with the base yaw
  const base = new THREE.Group();
  base.position.set(...offsetOf(settings.base));
  parent.add(base);
  if (settings.base) groups.base = base;
  parent = base;
  if (!seated) {
    for (const y of [0.1, -0.1]) {
      limb(base, [0, y, 0.08], [0, y, torsoZ - 0.05], 0.06);
      box(base, [0.24, 0.09, 0.06], [0.06, y, 0.03], mat.dark);
    }
  } else {
    const seatTop = torsoZ - 0.08;
    box(base, [0.46, 0.46, 0.05], [0.05, 0, seatTop - 0.025]);
    box(base, [0.05, 0.44, 0.5], [-0.2, 0, seatTop + 0.25]);
    if (swivel) {
      limb(base, [0, 0, 0.06], [0, 0, seatTop - 0.05], 0.025, mat.joint);
      for (let i = 0; i < 5; i++) {
        const a = (i / 5) * Math.PI * 2;
        limb(base, [0, 0, 0.05], [0.3 * Math.cos(a), 0.3 * Math.sin(a), 0.04], 0.018, mat.joint);
      }
    } else {
      for (const [x, y] of [[0.24, 0.2], [0.24, -0.2], [-0.18, 0.2], [-0.18, -0.2]]) {
        limb(base, [x, y, 0.02], [x, y, seatTop - 0.05], 0.015, mat.joint);
      }
    }
    for (const y of [0.1, -0.1]) {
      limb(base, [0, y, torsoZ - 0.02], [0.42, y, torsoZ - 0.02], 0.065);
      limb(base, [0.42, y, torsoZ - 0.02], [0.44, y, 0.08], 0.055);
      box(base, [0.24, 0.09, 0.06], [0.5, y, 0.03], mat.dark);
    }
  }

  // torso: pelvis to shoulders, arms, neck
  if (settings.torso) {
    const torso = new THREE.Group();
    torso.position.set(...offsetOf(settings.torso));
    parent.add(torso);
    groups.torso = torso;
    parent = torso;
    const top = headZ - 0.12;
    const chest = limb(torso, [0, 0, 0.1], [0, 0, top - 0.1], 0.12);
    chest.scale.set(0.75, 1, 1.45);   // capsule axis is local y after the quaternion; widen sideways
    limb(torso, [0, 0.19, top - 0.04], [0, -0.19, top - 0.04], 0.06);
    for (const y of [0.22, -0.22]) {
      limb(torso, [0, y, top - 0.05], [0.03, y * 1.08, top - 0.35], 0.045);
      limb(torso, [0.03, y * 1.08, top - 0.35], seated ? [0.3, y * 0.9, top - 0.45] : [0.05, y * 1.08, top - 0.62], 0.04);
    }
    limb(torso, [0, 0, top - 0.04], [0, 0, headZ + 0.01], 0.045, mat.joint);
  }

  // head: rotates about the neck pivot; face points +x
  const head = new THREE.Group();
  head.position.set(...offsetOf(settings.head));
  parent.add(head);
  groups.head = head;
  const skull = mesh(new THREE.SphereGeometry(1, 40, 28), mat.body, head, [0, 0, 0.1]);
  skull.scale.set(0.085, 0.075, 0.11);
  const nose = mesh(new THREE.ConeGeometry(0.014, 0.035, 12), mat.body, head, [0.09, 0, 0.075]);
  nose.rotation.z = -Math.PI / 2;
  for (const y of [0.074, -0.074]) {
    const ear = mesh(new THREE.SphereGeometry(1, 16, 12), mat.body, head, [-0.005, y, 0.1]);
    ear.scale.set(0.02, 0.008, 0.03);
  }

  // eyes: one group per eye, rotated by eye yaw/pitch
  const eyePoint = new THREE.Group();
  eyePoint.position.set(...offsetOf(settings.eyes || { offset: { xyz: [0.08, 0, 0.115] } }));
  head.add(eyePoint);
  const ipd = (settings.eyes && settings.eyes.ipd) || 0.063;
  const eyes = {};
  for (const [side, y] of [['left', ipd / 2], ['right', -ipd / 2]]) {
    const g = new THREE.Group();
    g.position.set(0, y, 0);
    eyePoint.add(g);
    // Drawn larger than life (real eyeball radius is about 0.012 m) so eye movement reads clearly.
    mesh(new THREE.SphereGeometry(EYE_R, 32, 20), mat.sclera, g);
    const iris = mesh(new THREE.CylinderGeometry(EYE_R * 0.5, EYE_R * 0.5, EYE_R * 0.16, 32), mat.iris, g, [EYE_R * 0.94, 0, 0]);
    iris.rotation.z = Math.PI / 2;
    const pupil = mesh(new THREE.CylinderGeometry(EYE_R * 0.24, EYE_R * 0.24, EYE_R * 0.18, 20), mat.pupil, g, [EYE_R * 0.97, 0, 0]);
    pupil.rotation.z = Math.PI / 2;
    eyes[side] = g;
  }

  body = { root, groups, eyes, eyePoint, settings };
}

function setYawPitch(obj, yaw = 0, pitch = 0) {
  obj.rotation.set(0, -pitch, yaw, 'ZYX');   // Rz(yaw) Ry(-pitch), spec 3.4
}

function applyPose(pose) {
  if (!body) return;
  const g = body.groups;
  if (g.base) setYawPitch(g.base, pose['base.yaw']);
  if (g.torso) setYawPitch(g.torso, pose['torso.yaw']);
  if (g.head) setYawPitch(g.head, pose['head.yaw'], pose['head.pitch']);
  const bino = body.settings.eyes && body.settings.eyes.binocular;
  for (const side of ['left', 'right']) {
    const p = bino ? `eyes.${side}.` : 'eyes.';
    setYawPitch(body.eyes[side], pose[p + 'yaw'] || 0, pose[p + 'pitch'] || 0);
  }
}

// ---- ball, gaze line, dragging ---------------------------------------------------------------

const ball = mesh(new THREE.SphereGeometry(0.06, 32, 20), mat.ball, world, [1.5, 0, 1.3]);
const ballShadowHint = new THREE.Mesh(
  new THREE.RingGeometry(0.05, 0.065, 32),
  new THREE.MeshBasicMaterial({ color: 0xff8a3d, transparent: true, opacity: 0.35 }),
);
world.add(ballShadowHint);

const lineGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const gazeLine = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0xffc94d, transparent: true, opacity: 0.55 }));
gazeLine.frustumCulled = false;
scene.add(gazeLine);

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const dragPlane = new THREE.Plane();
let dragging = false;

function setPointer(ev) {
  pointer.set((ev.clientX / window.innerWidth) * 2 - 1, -(ev.clientY / window.innerHeight) * 2 + 1);
  raycaster.setFromCamera(pointer, camera);
}

function makeDragPlane(vertical) {
  const p = ball.getWorldPosition(new THREE.Vector3());
  if (vertical) {
    const n = camera.getWorldDirection(new THREE.Vector3()).setY(0).normalize();
    dragPlane.setFromNormalAndCoplanarPoint(n, p);
  } else {
    dragPlane.setFromNormalAndCoplanarPoint(new THREE.Vector3(0, 1, 0), p);
  }
}

renderer.domElement.addEventListener('pointerdown', (ev) => {
  setPointer(ev);
  if (ball.visible && raycaster.intersectObject(ball).length) {
    dragging = true;
    controls.enabled = false;
    makeDragPlane(ev.shiftKey);
    renderer.domElement.setPointerCapture(ev.pointerId);
  }
});

renderer.domElement.addEventListener('pointermove', (ev) => {
  if (!dragging) return;
  setPointer(ev);
  const hit = raycaster.ray.intersectPlane(dragPlane, new THREE.Vector3());
  if (!hit) return;
  const local = world.worldToLocal(hit);
  local.z = Math.max(0.05, Math.min(3.0, local.z));
  ball.position.copy(local);
  send({ type: 'target', pos: [local.x, local.y, local.z] });
});

window.addEventListener('keydown', (ev) => { if (dragging && ev.key === 'Shift') makeDragPlane(true); });
window.addEventListener('keyup', (ev) => { if (dragging && ev.key === 'Shift') makeDragPlane(false); });

renderer.domElement.addEventListener('pointerup', (ev) => {
  if (!dragging) return;
  dragging = false;
  controls.enabled = true;
  renderer.domElement.releasePointerCapture(ev.pointerId);
});

// ---- UI --------------------------------------------------------------------------------------

const ui = {
  preset: document.getElementById('preset'),
  path: document.getElementById('path'),
  speed: document.getElementById('speed'),
  rays: document.getElementById('rays'),
  pause: document.getElementById('pause'),
  readout: document.getElementById('readout'),
  status: document.getElementById('status'),
};

function fillSelect(sel, options, value) {
  sel.replaceChildren(...options.map((o) => new Option(o.replaceAll('_', ' '), o)));
  sel.value = value;
}

ui.preset.addEventListener('change', () => send({ type: 'preset', name: ui.preset.value }));
ui.path.addEventListener('change', () => send({ type: 'path', name: ui.path.value }));
ui.speed.addEventListener('input', () => send({ type: 'speed', value: Number(ui.speed.value) }));
let paused = false;
ui.pause.addEventListener('click', () => send({ type: 'pause', value: !paused }));

function fmt(rad) { return `${(rad * DEG).toFixed(1)}°`; }

function renderReadout(frame) {
  const rows = Object.entries(frame.pose).map(([k, v]) => `<tr><td class="k">${k}</td><td>${fmt(v)}</td></tr>`);
  const d = frame.diagnostics;
  rows.push(`<tr class="sep"><td class="k">gaze error</td><td>${fmt(d.gaze_error)}</td></tr>`);
  rows.push(`<tr><td class="k">reachable</td><td>${d.reachable ? 'yes' : 'no'}</td></tr>`);
  if (d.base_state) rows.push(`<tr><td class="k">base</td><td>${d.base_state}</td></tr>`);
  if (d.vergence != null) rows.push(`<tr><td class="k">vergence</td><td>${fmt(d.vergence)}</td></tr>`);
  ui.readout.innerHTML = rows.join('');
}

// ---- connection ------------------------------------------------------------------------------

let ws = null;
let latest = null;

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

function connect() {
  ws = new WebSocket(`ws://${location.hostname}:${wsPort}`);
  ws.onopen = () => { ui.status.textContent = 'connected to Python reference'; ui.status.className = 'ok'; };
  ws.onclose = () => {
    ui.status.textContent = 'disconnected; retrying…';
    ui.status.className = 'bad';
    setTimeout(connect, 1000);
  };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'config') {
      if (initial) {  // apply ?preset=&path= once, after the first config
        if (initial.preset && initial.preset !== msg.preset) send({ type: 'preset', name: initial.preset });
        if (initial.path && initial.path !== msg.path) send({ type: 'path', name: initial.path });
        initial = null;
      }
      fillSelect(ui.preset, msg.presets, msg.preset);
      fillSelect(ui.path, msg.paths, msg.path);
      ui.speed.value = msg.speed;
      paused = msg.paused;
      ui.pause.textContent = paused ? 'Resume' : 'Pause';
      if (!body || body.settings.name !== msg.settings.name) buildBody(msg.settings);
    } else if (msg.type === 'frame') {
      latest = msg;
    } else if (msg.type === 'error') {
      console.warn('server:', msg.message);
    }
  };
}
connect();

// ---- render loop -------------------------------------------------------------------------------

let lastReadout = 0;
const eyeWorld = new THREE.Vector3();
const ballWorld = new THREE.Vector3();

function animate(now) {
  requestAnimationFrame(animate);
  if (latest) {
    applyPose(latest.pose);
    const t = latest.target;
    ball.visible = ballShadowHint.visible = !!t;
    if (t && !dragging) ball.position.set(t[0], t[1], t[2]);
    ballShadowHint.position.set(ball.position.x, ball.position.y, 0.003);
    if (now - lastReadout > 100) { renderReadout(latest); lastReadout = now; }
  }
  gazeLine.visible = ui.rays.checked && ball.visible && !!body;
  if (gazeLine.visible) {
    body.eyePoint.getWorldPosition(eyeWorld);
    ball.getWorldPosition(ballWorld);
    lineGeo.setFromPoints([eyeWorld, ballWorld]);
  }
  controls.update();
  renderer.render(scene, camera);
}
requestAnimationFrame(animate);

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
