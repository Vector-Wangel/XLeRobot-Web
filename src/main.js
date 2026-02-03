
import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { setupGUI, downloadDefaultRobot, loadSceneFromURL, loadModularScene, drawTendonsAndFlex, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import { keyboardController } from './utils/KeyboardControl.js';
import   load_mujoco        from '../node_modules/mujoco-js/dist/mujoco_wasm.js';
import { getSceneManager } from './utils/SceneManager.js';

// ===== 新增：后处理相关 =====
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');

// Default modular scene settings
const initialEnvironment = 'tabletop';
const initialRobot = 'xlerobot';

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    // Model and data will be initialized in init() via modular loading
    this.model = null;
    this.data = null;

    // Define Random State Variables
    this.params = {
      scene: null,
      environment: initialEnvironment,
      robot: initialRobot,
      paused: false,
      help: false,
      ctrlnoiserate: 0.0,
      ctrlnoisestd: 0.0,
      keyframeNumber: 0
    };
    this.mujoco_time = 0.0;
    this.bodies  = {}, this.lights = {};
    this.tmpVec  = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.container = document.createElement( 'div' );
    this.container.style.cssText = 'position: relative; width: 100%; height: 100%; z-index: 1;';
    document.body.appendChild( this.container );

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(0.5, 1.7, -3);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

    this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 * 3.14 );
    this.ambientLight.name = 'AmbientLight';
    this.scene.add( this.ambientLight );

    this.spotlight = new THREE.SpotLight();
    this.spotlight.angle = 1.11;
    this.spotlight.distance = 10000;
    this.spotlight.penumbra = 0.5;
    this.spotlight.castShadow = true; // default false
    this.spotlight.intensity = this.spotlight.intensity * 3.14 * 10.0;
    this.spotlight.shadow.mapSize.width = 1024; // default
    this.spotlight.shadow.mapSize.height = 1024; // default
    this.spotlight.shadow.camera.near = 0.1; // default
    this.spotlight.shadow.camera.far = 100; // default
    this.spotlight.position.set(0, 3, 3);
    const targetObject = new THREE.Object3D();
    this.scene.add(targetObject);
    this.spotlight.target = targetObject;
    targetObject.position.set(0, 1, 0);
    this.scene.add( this.spotlight );

    this.renderer = new THREE.WebGLRenderer( {
      antialias: true,
      alpha: true,
      premultipliedAlpha: false,
      preserveDrawingBuffer: true
    } );
    this.renderer.setPixelRatio(1.0);////window.devicePixelRatio );
    this.renderer.setSize( window.innerWidth, window.innerHeight );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    THREE.ColorManagement.enabled = false;
    this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    //this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    //this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    //this.renderer.toneMappingExposure = 2.0;
    this.renderer.useLegacyLights = true;

    this.renderer.setAnimationLoop( this.render.bind(this) );

    // Position canvas absolutely so it can layer above GS iframe
    // Will be modified when GS is enabled to use blend mode
    this.renderer.domElement.style.cssText = `
      position: absolute;
      top: 0;
      left: 0;
      z-index: 1;
      background: transparent;
    `;
    this.renderer.domElement.id = 'mujoco-canvas';
    this.container.appendChild( this.renderer.domElement );

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // Initialize the Drag State Manager.
    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);

    // ===== 新增：Toon 后处理 =====
    this.setupToonRendering();
  }

  // ===== 新增：Toon 渲染设置方法 =====
  setupToonRendering() {
    // 创建支持 alpha 的渲染目标
    const renderTarget = new THREE.WebGLRenderTarget(
      window.innerWidth,
      window.innerHeight,
      {
        format: THREE.RGBAFormat,
        type: THREE.HalfFloatType,
        stencilBuffer: false,
      }
    );

    // 创建后处理管线（使用支持 alpha 的渲染目标）
    this.composer = new EffectComposer(this.renderer, renderTarget);

    // 基础渲染 Pass
    const renderPass = new RenderPass(this.scene, this.camera);
    renderPass.clearAlpha = 0;  // 透明背景
    this.composer.addPass(renderPass);

    // 描边 Shader
    const OutlineShader = {
      uniforms: {
        'tDiffuse': { value: null },
        'resolution': { value: new THREE.Vector2(window.innerWidth, window.innerHeight) },
        'outlineColor': { value: new THREE.Color(0x000000) },
        'outlineThickness': { value: 1.0 }
      },
      vertexShader: `
        varying vec2 vUv;
        void main() {
          vUv = uv;
          gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }
      `,
      fragmentShader: `
        uniform sampler2D tDiffuse;
        uniform vec2 resolution;
        uniform vec3 outlineColor;
        uniform float outlineThickness;
        varying vec2 vUv;

        void main() {
          vec2 texel = vec2(outlineThickness) / resolution;

          // 采样周围像素
          vec4 center = texture2D(tDiffuse, vUv);
          vec4 left   = texture2D(tDiffuse, vUv - vec2(texel.x, 0.0));
          vec4 right  = texture2D(tDiffuse, vUv + vec2(texel.x, 0.0));
          vec4 top    = texture2D(tDiffuse, vUv + vec2(0.0, texel.y));
          vec4 bottom = texture2D(tDiffuse, vUv - vec2(0.0, texel.y));

          // 简单边缘检测（基于颜色差异）
          float edge = 0.0;
          edge += length(center.rgb - left.rgb);
          edge += length(center.rgb - right.rgb);
          edge += length(center.rgb - top.rgb);
          edge += length(center.rgb - bottom.rgb);
          edge = edge / 4.0;

          // 阈值化
          float outline = smoothstep(0.05, 0.15, edge);

          // 混合描边
          vec3 finalColor = mix(center.rgb, outlineColor, outline);
          gl_FragColor = vec4(finalColor, center.a);
        }
      `
    };

    this.outlinePass = new ShaderPass(OutlineShader);
    this.composer.addPass(this.outlinePass);

    // 输出 Pass
    this.composer.addPass(new OutputPass());
  }

  async init() {
    // Download only the default robot on startup (lazy loading)
    // Other robots will be downloaded on-demand when selected
    await downloadDefaultRobot(mujoco, 'xlerobot');

    // Initialize scene manager
    this.sceneManager = getSceneManager(mujoco);

    // Load initial modular scene (environment + robot + objects)
    await loadModularScene(this, this.params.environment, this.params.robot);

    this.gui = new GUI();
    setupGUI(this);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize( window.innerWidth, window.innerHeight );

    // ===== 新增：更新后处理尺寸 =====
    this.composer.setSize(window.innerWidth, window.innerHeight);
    this.outlinePass.uniforms.resolution.value.set(window.innerWidth, window.innerHeight);
  }

  render(timeMS) {
    this.controls.update();

    // Skip physics if model not yet loaded
    if (!this.model || !this.data) {
      this.renderer.render(this.scene, this.camera);
      return;
    }

    if (!this.params["paused"]) {
      let timestep = this.model.opt.timestep;
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS; }
      while (this.mujoco_time < timeMS) {

        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate  = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.data.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones.
        for (let i = 0; i < this.data.qfrc_applied.length; i++) { this.data.qfrc_applied[i] = 0.0; }
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition  (this.data.xpos , b, this.bodies[b].position);
              getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update(); // Update the world-space force origin
          let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          mujoco.mj_applyFT(this.model, this.data, [force.x, force.y, force.z], [0, 0, 0], [point.x, point.y, point.z], bodyID, this.data.qfrc_applied);

          // TODO: Apply pose perturbations (mocap bodies only).
        }

        // Update keyboard controls
        keyboardController.update();

        mujoco.mj_step(this.model, this.data);

        this.mujoco_time += timestep * 1000.0;
      }

    } else if (this.params["paused"]) {
      this.dragStateManager.update(); // Update the world-space force origin
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        getPosition  (this.data.xpos , b, this.tmpVec , false); // Get raw coordinate from MuJoCo
        getQuaternion(this.data.xquat, b, this.tmpQuat, false); // Get raw coordinate from MuJoCo

        let offset = toMujocoPos(this.dragStateManager.currentWorld.clone()
          .sub(this.dragStateManager.worldHit).multiplyScalar(0.3));
        if (this.model.body_mocapid[b] >= 0) {
          // Set the root body's mocap position...
          console.log("Trying to move mocap body", b);
          let addr = this.model.body_mocapid[b] * 3;
          let pos  = this.data.mocap_pos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;
        } else {
          // Set the root body's position directly...
          let root = this.model.body_rootid[b];
          let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
          let pos  = this.data.qpos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;
        }
      }

      mujoco.mj_forward(this.model, this.data);
    }

    // Update body transforms.
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition  (this.data.xpos , b, this.bodies[b].position);
        getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Update light transforms.
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.data.light_xpos, l, this.lights[l].position);
        getPosition(this.data.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Draw Tendons and Flex verts
    drawTendonsAndFlex(this.mujocoRoot, this.model, this.data);

    // Render the scene
    if (this.gsController && this.gsController.enabled) {
      // When 3DGS is enabled, bypass post-processing and render directly
      // SplatMesh is in the same scene, no camera sync needed
      // Z-buffer automatically handles depth occlusion between GS and MuJoCo meshes
      this.renderer.setClearColor(0x000000, 0);
      this.renderer.clear();
      this.renderer.render(this.scene, this.camera);
    } else {
      // Render with toon post-processing
      this.composer.render();
    }
  }
}

let demo = new MuJoCoDemo();
await demo.init();

// ============================================================================
// Gaussian Splatting Environment Controller (Same-Scene SplatMesh)
// ============================================================================

class GaussianSplatController {
  constructor(container, scene) {
    this.container = container;
    this.scene = scene;
    this.splatMesh = null;
    this.enabled = false;
    this.loading = false;
    this.savedBackground = null;
    this.savedFog = null;
    this.SparkModule = null;
  }

  async enable(spzUrl = './assets/scene.spz') {
    if (this.enabled || this.loading) return;
    this.loading = true;

    // Convert relative URL to absolute URL
    const absoluteSpzUrl = new URL(spzUrl, window.location.href).href;

    try {
      console.log('Loading 3DGS with same-scene SplatMesh...');
      console.log('Loading splat from:', absoluteSpzUrl);

      // Dynamically import Spark module (cached after first load)
      // Uses local npm package via importmap to ensure same Three.js instance
      if (!this.SparkModule) {
        console.log('Importing Spark module...');
        this.SparkModule = await import('@sparkjsdev/spark');
        console.log('Spark module loaded');
      }

      const { SplatMesh } = this.SparkModule;

      // Create SplatMesh and add directly to the scene
      // SplatMesh extends THREE.Object3D, so it integrates with the scene graph
      // Z-buffer handles depth occlusion automatically with opaque geometry
      this.splatMesh = new SplatMesh({ url: absoluteSpzUrl });
      this.splatMesh.name = 'GaussianSplatMesh';

      // Add to scene - Z-buffer will handle occlusion with MuJoCo meshes
      this.scene.add(this.splatMesh);
      console.log('SplatMesh added to scene');

      // Save and clear scene background for transparent compositing
      this.savedBackground = this.scene.background;
      this.savedFog = this.scene.fog;
      this.scene.background = null;
      this.scene.fog = null;

      // Hide floor/ground meshes to show GS environment
      this.hiddenMeshes = [];
      this.scene.traverse((obj) => {
        if (obj.isMesh && obj.name && (obj.name.toLowerCase().includes('floor') || obj.name.toLowerCase().includes('ground') || obj.name.toLowerCase().includes('plane'))) {
          obj.visible = false;
          this.hiddenMeshes.push(obj);
          console.log('Hidden mesh for GS:', obj.name);
        }
      });

      this.enabled = true;
      console.log('3D Gaussian Splatting environment enabled (same-scene)');
    } catch (err) {
      console.error('Failed to load 3DGS:', err);
      this.loading = false;
      throw err;
    }

    this.loading = false;
  }

  disable() {
    if (!this.enabled) return;

    // Remove SplatMesh from scene
    if (this.splatMesh) {
      this.scene.remove(this.splatMesh);
      // Dispose of SplatMesh resources if available
      if (this.splatMesh.dispose) {
        this.splatMesh.dispose();
      }
      this.splatMesh = null;
    }

    // Restore scene background
    if (this.savedBackground !== null) {
      this.scene.background = this.savedBackground;
      this.savedBackground = null;
    }
    if (this.savedFog !== null) {
      this.scene.fog = this.savedFog;
      this.savedFog = null;
    }

    // Restore hidden meshes
    if (this.hiddenMeshes) {
      this.hiddenMeshes.forEach(mesh => {
        mesh.visible = true;
      });
      this.hiddenMeshes = null;
    }

    this.enabled = false;
    console.log('3D Gaussian Splatting environment disabled');
  }

  async toggle(spzUrl) {
    if (this.enabled) {
      this.disable();
    } else {
      await this.enable(spzUrl);
    }
    return this.enabled;
  }

  isLoading() {
    return this.loading;
  }
}

// Create controller and UI button
const gsController = new GaussianSplatController(demo.container, demo.scene);
demo.gsController = gsController;  // Attach to demo for render loop access

// GitHub button
const githubBtn = document.createElement('button');
githubBtn.textContent = 'GitHub';
githubBtn.style.cssText = `
  position: fixed;
  bottom: 20px;
  right: 200px;
  padding: 12px 20px;
  font-size: 14px;
  font-weight: 600;
  color: white;
  background: linear-gradient(135deg, #333 0%, #111 100%);
  border: none;
  border-radius: 12px;
  cursor: pointer;
  z-index: 1000;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
  transition: transform 0.1s ease, box-shadow 0.1s ease;
`;

githubBtn.onmouseenter = () => {
  githubBtn.style.transform = 'scale(1.05)';
  githubBtn.style.boxShadow = '0 6px 16px rgba(0, 0, 0, 0.4)';
};

githubBtn.onmouseleave = () => {
  githubBtn.style.transform = 'scale(1)';
  githubBtn.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.3)';
};

githubBtn.onclick = () => {
  window.open('https://github.com/Vector-Wangel/MuJoCo-GS-Web', '_blank');
};

document.body.appendChild(githubBtn);

// 3DGS toggle button
const gsToggleBtn = document.createElement('button');
gsToggleBtn.textContent = 'Enable 3DGS';
gsToggleBtn.style.cssText = `
  position: fixed;
  bottom: 20px;
  right: 20px;
  padding: 12px 20px;
  font-size: 14px;
  font-weight: 600;
  color: white;
  background: linear-gradient(135deg, #3b82f6 0%, #1d4ed8 100%);
  border: none;
  border-radius: 12px;
  cursor: pointer;
  z-index: 1000;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
  transition: transform 0.1s ease, box-shadow 0.1s ease;
`;

gsToggleBtn.onmouseenter = () => {
  gsToggleBtn.style.transform = 'scale(1.05)';
  gsToggleBtn.style.boxShadow = '0 6px 16px rgba(0, 0, 0, 0.4)';
};

gsToggleBtn.onmouseleave = () => {
  gsToggleBtn.style.transform = 'scale(1)';
  gsToggleBtn.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.3)';
};

gsToggleBtn.onclick = async () => {
  gsToggleBtn.disabled = true;
  gsToggleBtn.style.cursor = 'wait';
  gsToggleBtn.textContent = '⏳ Loading...';

  try {
    const enabled = await gsController.toggle('./assets/environments/tabletop/scene.spz');

    gsToggleBtn.textContent = enabled ? 'Disable 3DGS' : 'Enable 3DGS';
    gsToggleBtn.style.background = enabled
      ? 'linear-gradient(135deg, #10b981 0%, #059669 100%)'
      : 'linear-gradient(135deg, #3b82f6 0%, #1d4ed8 100%)';
  } catch (err) {
    console.error('Toggle failed:', err);
    gsToggleBtn.textContent = 'Load Failed - Retry';
    gsToggleBtn.style.background = 'linear-gradient(135deg, #ef4444 0%, #dc2626 100%)';
    // Reset after 2 seconds
    setTimeout(() => {
      gsToggleBtn.textContent = 'Enable 3DGS';
      gsToggleBtn.style.background = 'linear-gradient(135deg, #3b82f6 0%, #1d4ed8 100%)';
    }, 2000);
  }

  gsToggleBtn.disabled = false;
  gsToggleBtn.style.cursor = 'pointer';
};

document.body.appendChild(gsToggleBtn);