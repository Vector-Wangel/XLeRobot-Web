/**
 * Scene Manager
 *
 * Manages loading of modular scenes using MuJoCo's include mechanism.
 * Scene files use <include file="robot.xml"/> to include robot definitions.
 */

import { RobotLoader, SceneConfigManager } from './RobotLoader.js';

export class SceneManager {
  /**
   * Robot configurations - centralized robot metadata
   * All robots are in assets/robots/{robotDir}/
   */
  static ROBOT_CONFIGS = {
    'xlerobot': {
      xmlPath: './assets/robots/xlerobot/xlerobot.xml',
      objectsPath: './assets/robots/xlerobot/objects.xml',
      robotDir: 'xlerobot',
      meshDir: 'assets',
      description: 'XLeRobot Dual-Arm Mobile Robot'
    },
    'SO101': {
      xmlPath: './assets/robots/xlerobot/SO101.xml',
      objectsPath: './assets/robots/xlerobot/objects_SO101.xml',
      robotDir: 'xlerobot',  // shares assets with xlerobot
      meshDir: 'assets',
      description: 'SO101 Single Arm'
    },
    'panda': {
      xmlPath: './assets/robots/panda/panda.xml',
      objectsPath: './assets/robots/panda/objects.xml',
      robotDir: 'panda',
      meshDir: 'assets',
      description: 'Franka Emika Panda'
    },
    'humanoid': {
      xmlPath: './assets/robots/humanoid/humanoid.xml',
      objectsPath: null,  // humanoid has no objects
      robotDir: 'humanoid',
      meshDir: null,  // humanoid uses primitive shapes, no assets
      description: 'DeepMind Humanoid'
    }
  };

  /**
   * Environment configurations
   * All environments are in assets/environments/{envName}/
   */
  static ENV_CONFIGS = {
    'tabletop': {
      xmlPath: './assets/environments/tabletop/scene.xml',
      spzPath: './assets/environments/tabletop/scene.spz',
      description: 'Tabletop manipulation environment'
    },
    'basic': {
      xmlPath: './assets/environments/basic.xml',
      spzPath: null,
      description: 'Basic environment with light and floor only'
    }
  };

  constructor(mujoco) {
    this.mujoco = mujoco;
    this.robotLoader = new RobotLoader(mujoco);
    this.configManager = new SceneConfigManager();
    this.currentEnv = null;
    this.currentRobot = null;
    this.scenePath = null;
    this.customSpzData = null;  // Store custom SPZ data from user upload
  }

  /**
   * Load a scene using MuJoCo's include mechanism.
   * Creates a scene XML that uses <include> to bring in robot and objects.
   *
   * @param {string} envName - Environment name (e.g., 'tabletop')
   * @param {string} robotName - Robot name (e.g., 'xlerobot', 'SO101')
   * @returns {Promise<string>} - Path to the scene XML in virtual filesystem
   */
  async loadModularScene(envName, robotName) {
    console.log(`Loading modular scene: env=${envName}, robot=${robotName}`);

    try {
      const robotConfig = SceneManager.ROBOT_CONFIGS[robotName];
      if (!robotConfig) {
        throw new Error(`Unknown robot: ${robotName}`);
      }

      const robotDir = robotConfig.robotDir;
      const vfsSceneDir = `/working/scenes/${robotName}`;

      // 1. Ensure scene directory exists in VFS
      this._ensureDir('/working/scenes');
      this._ensureDir(vfsSceneDir);
      if (robotConfig.meshDir) {
        this._ensureDir(`${vfsSceneDir}/assets`);
      }

      // 2. Copy robot XML to scene directory (so include can find it)
      const robotXmlResponse = await fetch(robotConfig.xmlPath);
      if (!robotXmlResponse.ok) {
        throw new Error(`Robot XML not found: ${robotConfig.xmlPath}`);
      }
      let robotXml = await robotXmlResponse.text();

      // Fix meshdir to be relative to scene directory
      if (robotConfig.meshDir) {
        robotXml = robotXml.replace(
          /meshdir="[^"]*"/g,
          `meshdir="./assets/"`
        );
      }
      this._writeToFS(`${vfsSceneDir}/${robotName}.xml`, robotXml);

      // 3. Copy objects XML if exists
      let hasObjects = false;
      if (robotConfig.objectsPath) {
        try {
          const objectsResponse = await fetch(robotConfig.objectsPath);
          if (objectsResponse.ok) {
            const objectsXml = await objectsResponse.text();
            this._writeToFS(`${vfsSceneDir}/objects.xml`, objectsXml);
            hasObjects = true;
          }
        } catch (e) {
          console.log('No objects file for robot:', robotName);
        }
      }

      // 4. Copy asset files from /working/robots/{robotDir}/assets/ to scene directory
      // (assets were already downloaded by downloadExampleScenesFolder)
      if (robotConfig.meshDir) {
        const srcMeshDir = `/working/robots/${robotDir}/assets`;
        const dstMeshDir = `${vfsSceneDir}/assets`;

        try {
          const meshFiles = this.mujoco.FS.readdir(srcMeshDir);
          console.log(`Copying ${meshFiles.length - 2} mesh files to ${dstMeshDir}`);
          for (const file of meshFiles) {
            if (file === '.' || file === '..') continue;
            const srcPath = `${srcMeshDir}/${file}`;
            const dstPath = `${dstMeshDir}/${file}`;
            try {
              const content = this.mujoco.FS.readFile(srcPath);
              this.mujoco.FS.writeFile(dstPath, content);
            } catch (e) {
              console.error(`Failed to copy mesh ${file}:`, e);
            }
          }
        } catch (e) {
          console.error('Failed to read mesh directory:', e);
        }
      }

      // 5. Load environment XML template
      const envConfig = SceneManager.ENV_CONFIGS[envName];
      if (!envConfig) {
        throw new Error(`Unknown environment: ${envName}`);
      }

      const envResponse = await fetch(envConfig.xmlPath);
      if (!envResponse.ok) {
        throw new Error(`Environment XML not found: ${envConfig.xmlPath}`);
      }
      const envXml = await envResponse.text();

      // 6. Create scene XML with include
      const sceneName = `${envName}_${robotName}`;
      const sceneXml = this._createSceneXml(envXml, robotName, hasObjects, sceneName);

      // Debug: output the scene XML
      console.log('=== SCENE XML WITH INCLUDE ===');
      console.log(sceneXml);
      console.log('=== END SCENE XML ===');

      // 7. Write scene XML to VFS
      const sceneXmlPath = `${vfsSceneDir}/scene.xml`;
      this._writeToFS(sceneXmlPath, sceneXml);

      this.currentEnv = envName;
      this.currentRobot = robotName;
      this.scenePath = `scenes/${robotName}/scene.xml`;

      console.log(`Scene loaded: ${this.scenePath}`);
      return this.scenePath;

    } catch (error) {
      console.error('Scene loading failed:', error);
      throw error;
    }
  }

  /**
   * Create scene XML by inserting include statements into environment XML
   */
  _createSceneXml(envXml, robotName, hasObjects, sceneName) {
    // Parse environment XML
    const parser = new DOMParser();
    const doc = parser.parseFromString(envXml, 'text/xml');

    if (doc.querySelector('parsererror')) {
      throw new Error('Failed to parse environment XML');
    }

    // Update model name
    const mujoco = doc.documentElement;
    mujoco.setAttribute('model', sceneName);

    // Create include element for robot
    const robotInclude = doc.createElement('include');
    robotInclude.setAttribute('file', `${robotName}.xml`);

    // Insert include at the beginning (after mujoco tag)
    mujoco.insertBefore(robotInclude, mujoco.firstChild);

    // Add objects include if exists
    if (hasObjects) {
      const objectsInclude = doc.createElement('include');
      objectsInclude.setAttribute('file', 'objects.xml');
      // Insert after robot include
      mujoco.insertBefore(objectsInclude, robotInclude.nextSibling);
    }

    // Serialize back to string
    const serializer = new XMLSerializer();
    let result = serializer.serializeToString(doc);

    // Clean up XML
    result = result.replace(/(<\?xml[^?]*\?>)/g, '');
    result = result.replace(/\s+xmlns="[^"]*"/g, '');
    result = result.replace(/\s+xmlns:[a-z]+="[^"]*"/g, '');
    result = '<?xml version="1.0" encoding="UTF-8"?>\n' + result.trim();

    return result;
  }

  /**
   * Write content to MuJoCo virtual filesystem
   */
  _writeToFS(path, content) {
    // Try to remove existing file
    try {
      this.mujoco.FS.unlink(path);
    } catch (e) {
      // File doesn't exist, that's fine
    }

    // Write new file
    if (typeof content === 'string') {
      this.mujoco.FS.writeFile(path, content);
    } else {
      this.mujoco.FS.writeFile(path, content);
    }
    console.log(`Written to VFS: ${path}`);
  }

  /**
   * Ensure directory exists in virtual filesystem
   */
  _ensureDir(path) {
    try {
      if (!this.mujoco.FS.analyzePath(path).exists) {
        this.mujoco.FS.mkdir(path);
      }
    } catch (e) {
      // Directory might already exist
    }
  }

  /**
   * Load user-uploaded robot files
   * @param {FileList} files - Files from input[webkitdirectory]
   * @param {string} envName - Environment to load robot into
   * @returns {Promise<string>} - Path to scene XML
   */
  async loadUploadedRobot(files, envName = 'tabletop') {
    // Parse uploaded files
    const { robotXml, objectsXml, meshFiles, robotName } =
      await this.robotLoader.loadUploadedRobot(files);

    const vfsSceneDir = `/working/scenes/uploaded_${robotName}`;

    // Create directories
    this._ensureDir('/working/scenes');
    this._ensureDir(vfsSceneDir);
    this._ensureDir(`${vfsSceneDir}/assets`);

    // Write asset files
    for (const [name, buffer] of meshFiles) {
      const meshPath = `${vfsSceneDir}/assets/${name}`;
      this.mujoco.FS.writeFile(meshPath, new Uint8Array(buffer));
    }

    // Fix meshdir and write robot XML
    let fixedRobotXml = robotXml.replace(
      /meshdir="[^"]*"/g,
      `meshdir="./assets/"`
    );
    this._writeToFS(`${vfsSceneDir}/robot.xml`, fixedRobotXml);

    // Write objects XML if exists
    const hasObjects = !!objectsXml;
    if (hasObjects) {
      this._writeToFS(`${vfsSceneDir}/objects.xml`, objectsXml);
    }

    // Load environment XML
    const envConfig = SceneManager.ENV_CONFIGS[envName];
    const envResponse = await fetch(envConfig.xmlPath);
    const envXml = await envResponse.text();

    // Create scene XML with include
    const sceneName = `${envName}_uploaded_${robotName}`;
    const sceneXml = this._createSceneXml(envXml, 'robot', hasObjects, sceneName);

    // Write scene XML
    this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);

    this.currentEnv = envName;
    this.currentRobot = `uploaded_${robotName}`;
    this.scenePath = `scenes/uploaded_${robotName}/scene.xml`;

    return this.scenePath;
  }

  /**
   * Get current scene info
   */
  getCurrentSceneInfo() {
    return {
      environment: this.currentEnv,
      robot: this.currentRobot,
      scenePath: this.scenePath
    };
  }

  /**
   * Get 3DGS path for current environment
   * @returns {string|null}
   */
  getSpzPath() {
    if (!this.currentEnv) return null;
    const envConfig = SceneManager.ENV_CONFIGS[this.currentEnv];
    return envConfig ? envConfig.spzPath : null;
  }

  /**
   * Get custom SPZ data if available
   * @returns {ArrayBuffer|null}
   */
  getCustomSpzData() {
    return this.customSpzData;
  }

  /**
   * Check if using custom SPZ
   * @returns {boolean}
   */
  hasCustomSpz() {
    return this.customSpzData !== null;
  }

  /**
   * Clear custom SPZ data
   */
  clearCustomSpz() {
    this.customSpzData = null;
  }

  /**
   * Load a custom SPZ file with basic environment
   * @param {File} spzFile - The SPZ file to load
   * @param {string} robotName - Robot to use (default: none)
   * @returns {Promise<string>} - Path to scene XML
   */
  async loadCustomSpz(spzFile, robotName = null) {
    console.log(`Loading custom SPZ: ${spzFile.name}`);

    // Store the SPZ data
    this.customSpzData = await spzFile.arrayBuffer();

    // Use basic environment
    const envName = 'basic';
    const vfsSceneDir = `/working/scenes/custom_spz`;

    // Create directories
    this._ensureDir('/working/scenes');
    this._ensureDir(vfsSceneDir);

    // Load basic environment XML
    const envConfig = SceneManager.ENV_CONFIGS[envName];
    const envResponse = await fetch(envConfig.xmlPath);
    const envXml = await envResponse.text();

    if (robotName && SceneManager.ROBOT_CONFIGS[robotName]) {
      // Load with robot
      const robotConfig = SceneManager.ROBOT_CONFIGS[robotName];
      const robotDir = robotConfig.robotDir;

      if (robotConfig.meshDir) {
        this._ensureDir(`${vfsSceneDir}/assets`);
      }

      // Copy robot XML
      const robotXmlResponse = await fetch(robotConfig.xmlPath);
      let robotXml = await robotXmlResponse.text();

      if (robotConfig.meshDir) {
        robotXml = robotXml.replace(/meshdir="[^"]*"/g, `meshdir="./assets/"`);
      }
      this._writeToFS(`${vfsSceneDir}/${robotName}.xml`, robotXml);

      // Copy objects if exists
      let hasObjects = false;
      if (robotConfig.objectsPath) {
        try {
          const objectsResponse = await fetch(robotConfig.objectsPath);
          if (objectsResponse.ok) {
            const objectsXml = await objectsResponse.text();
            this._writeToFS(`${vfsSceneDir}/objects.xml`, objectsXml);
            hasObjects = true;
          }
        } catch (e) {
          console.log('No objects file for robot:', robotName);
        }
      }

      // Copy mesh files
      if (robotConfig.meshDir) {
        const srcMeshDir = `/working/robots/${robotDir}/assets`;
        const dstMeshDir = `${vfsSceneDir}/assets`;

        try {
          const meshFiles = this.mujoco.FS.readdir(srcMeshDir);
          for (const file of meshFiles) {
            if (file === '.' || file === '..') continue;
            try {
              const content = this.mujoco.FS.readFile(`${srcMeshDir}/${file}`);
              this.mujoco.FS.writeFile(`${dstMeshDir}/${file}`, content);
            } catch (e) {
              // Ignore copy errors
            }
          }
        } catch (e) {
          console.error('Failed to read mesh directory:', e);
        }
      }

      // Create scene XML with robot include
      const sceneName = `custom_spz_${robotName}`;
      const sceneXml = this._createSceneXml(envXml, robotName, hasObjects, sceneName);
      this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);

      this.currentRobot = robotName;
    } else {
      // No robot, just basic environment
      this._writeToFS(`${vfsSceneDir}/scene.xml`, envXml);
      this.currentRobot = null;
    }

    this.currentEnv = 'custom_spz';
    this.scenePath = `scenes/custom_spz/scene.xml`;

    console.log(`Custom SPZ scene loaded: ${this.scenePath}`);
    return this.scenePath;
  }

  /**
   * List available environments
   */
  listEnvironments() {
    return Object.keys(SceneManager.ENV_CONFIGS);
  }

  /**
   * List available robots
   */
  listRobots() {
    return Object.keys(SceneManager.ROBOT_CONFIGS);
  }
}

// Singleton instance
let sceneManagerInstance = null;

export function getSceneManager(mujoco) {
  if (!sceneManagerInstance && mujoco) {
    sceneManagerInstance = new SceneManager(mujoco);
  }
  return sceneManagerInstance;
}
