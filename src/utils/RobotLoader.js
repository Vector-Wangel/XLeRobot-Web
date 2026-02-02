/**
 * Robot Loader Utility
 *
 * Handles loading robot files from:
 * 1. Predefined robot folders in assets/robots/
 * 2. User-uploaded robot folders (zip or directory)
 *
 * Manages writing files to MuJoCo's virtual filesystem.
 */

import { MujocoXmlMerger } from './XmlMerger.js';

export class RobotLoader {
  constructor(mujoco) {
    this.mujoco = mujoco;
    this.loadedRobots = new Map(); // Track loaded robots
  }

  /**
   * Load a predefined robot from assets/robots/
   * @param {string} robotName - Robot folder name (e.g., 'xlerobot', 'SO101', 'panda')
   * @param {string} basePath - Base path to robots folder
   * @returns {Promise<{robotXml: string, objectsXml: string|null}>}
   */
  async loadPredefinedRobot(robotName, basePath = './assets/robots') {
    const robotDir = `${basePath}/${robotName}`;

    // Try to load main robot XML (try common names)
    const xmlNames = [`${robotName}.xml`, 'robot.xml', 'main.xml'];
    let robotXml = null;

    for (const name of xmlNames) {
      try {
        const response = await fetch(`${robotDir}/${name}`);
        if (response.ok) {
          robotXml = await response.text();
          break;
        }
      } catch (e) {
        continue;
      }
    }

    if (!robotXml) {
      throw new Error(`Could not find robot XML in ${robotDir}`);
    }

    // Try to load objects XML (optional)
    let objectsXml = null;
    try {
      const response = await fetch(`${robotDir}/objects.xml`);
      if (response.ok) {
        objectsXml = await response.text();
      }
    } catch (e) {
      // Objects file is optional
    }

    return { robotXml, objectsXml, robotDir };
  }

  /**
   * Load robot from user-uploaded files (FileList from input[webkitdirectory])
   * @param {FileList|File[]} files - Uploaded files
   * @returns {Promise<{robotXml: string, objectsXml: string|null, meshFiles: Map}>}
   */
  async loadUploadedRobot(files) {
    const meshFiles = new Map();
    let robotXml = null;
    let objectsXml = null;
    let robotName = null;

    for (const file of files) {
      const path = file.webkitRelativePath || file.name;
      const pathParts = path.split('/');

      // Extract robot folder name from first directory
      if (pathParts.length > 1 && !robotName) {
        robotName = pathParts[0];
      }

      if (path.endsWith('.xml')) {
        const content = await file.text();
        const fileName = path.split('/').pop().toLowerCase();

        if (fileName.includes('object')) {
          objectsXml = content;
        } else if (!robotXml) {
          // First non-object XML is the main robot XML
          robotXml = content;
        }
      } else if (path.includes('meshes/') || path.includes('mesh/')) {
        // Store mesh files
        const buffer = await file.arrayBuffer();
        const meshName = path.split(/meshes?\//).pop();
        meshFiles.set(meshName, buffer);
      }
    }

    if (!robotXml) {
      throw new Error('No robot XML file found in uploaded folder');
    }

    // Default robot name if not detected from folder structure
    if (!robotName) {
      robotName = 'user_robot';
    }

    return { robotXml, objectsXml, meshFiles, robotName };
  }

  /**
   * Write robot files to MuJoCo virtual filesystem
   * @param {string} robotName - Robot identifier
   * @param {string} robotXml - Robot XML content
   * @param {Map} meshFiles - Map of mesh filename to ArrayBuffer
   * @returns {string} - Path to robot XML in virtual filesystem
   */
  writeToMujocoFS(robotName, robotXml, meshFiles = new Map()) {
    const robotDir = `/robots/${robotName}`;
    const meshDir = `${robotDir}/meshes`;

    // Create directories (ignore errors if they exist)
    try {
      this.mujoco.FS_mkdir('/robots');
    } catch (e) { /* directory may exist */ }

    try {
      this.mujoco.FS_mkdir(robotDir);
    } catch (e) { /* directory may exist */ }

    try {
      this.mujoco.FS_mkdir(meshDir);
    } catch (e) { /* directory may exist */ }

    // Fix mesh paths in XML
    const fixedXml = MujocoXmlMerger.fixMeshPaths(robotXml, robotDir);

    // Write robot XML
    const xmlPath = `${robotDir}/robot.xml`;
    try {
      this.mujoco.FS_unlink(xmlPath);
    } catch (e) { /* file may not exist */ }
    this.mujoco.FS_createDataFile(robotDir, 'robot.xml', fixedXml, true, true);

    // Write mesh files
    for (const [name, buffer] of meshFiles) {
      const meshPath = `${meshDir}/${name}`;
      try {
        this.mujoco.FS_unlink(meshPath);
      } catch (e) { /* file may not exist */ }
      this.mujoco.FS_createDataFile(meshDir, name, new Uint8Array(buffer), true, true);
    }

    this.loadedRobots.set(robotName, {
      xmlPath,
      meshDir,
      timestamp: Date.now()
    });

    return xmlPath;
  }

  /**
   * Load and merge environment + robot + objects
   * @param {string} envXml - Environment XML content
   * @param {string} robotXml - Robot XML content
   * @param {string} objectsXml - Objects XML content (optional)
   * @param {string} sceneName - Name for the merged scene
   * @returns {string} - Path to merged scene XML in virtual filesystem
   */
  createMergedScene(envXml, robotXml, objectsXml = null, sceneName = 'merged_scene') {
    // Merge XMLs
    let mergedXml = MujocoXmlMerger.merge(envXml, robotXml, objectsXml);

    // Set model name
    mergedXml = MujocoXmlMerger.setModelName(mergedXml, sceneName);

    // Write to virtual filesystem
    const scenePath = `/${sceneName}.xml`;
    try {
      this.mujoco.FS_unlink(scenePath);
    } catch (e) { /* file may not exist */ }
    this.mujoco.FS_createDataFile('/', `${sceneName}.xml`, mergedXml, true, true);

    return scenePath;
  }

  /**
   * Clean up uploaded robot files from virtual filesystem
   * @param {string} robotName - Robot identifier to remove
   */
  cleanupRobot(robotName) {
    const robotInfo = this.loadedRobots.get(robotName);
    if (!robotInfo) return;

    try {
      // Note: Full cleanup would require listing and removing all files
      // For simplicity, we just track that it's been "cleaned"
      this.loadedRobots.delete(robotName);
    } catch (e) {
      console.warn(`Failed to cleanup robot ${robotName}:`, e);
    }
  }

  /**
   * Get list of loaded robots
   * @returns {string[]} - Array of robot names
   */
  getLoadedRobots() {
    return Array.from(this.loadedRobots.keys());
  }
}

/**
 * Scene Configuration Manager
 *
 * Manages the mapping between environments, robots, and their configurations.
 */
export class SceneConfigManager {
  constructor() {
    // Environment definitions
    this.environments = {
      'tabletop': {
        envPath: './assets/environments/tabletop/scene.xml',
        spzPath: './assets/environments/tabletop/scene.spz',
        description: 'Tabletop manipulation environment'
      }
    };

    // Robot definitions with their controllers
    this.robots = {
      'xlerobot': {
        robotPath: './assets/robots/xlerobot/xlerobot.xml',
        objectsPath: './assets/robots/xlerobot/objects.xml',
        controllerType: 'XLeRobotController',
        description: 'XLeRobot Dual-Arm Mobile Robot'
      },
      'SO101': {
        robotPath: './assets/robots/SO101/SO101.xml',
        objectsPath: './assets/robots/SO101/objects.xml',
        controllerType: 'SO101Controller',
        description: 'SO101 Single Arm'
      },
      'panda': {
        robotPath: './assets/robots/panda/panda.xml',
        objectsPath: null,
        controllerType: 'PandaController',
        description: 'Franka Emika Panda'
      }
    };
  }

  getEnvironment(envName) {
    return this.environments[envName];
  }

  getRobot(robotName) {
    return this.robots[robotName];
  }

  listEnvironments() {
    return Object.keys(this.environments);
  }

  listRobots() {
    return Object.keys(this.robots);
  }

  /**
   * Register a user-uploaded robot
   */
  registerUploadedRobot(robotName, config) {
    this.robots[robotName] = {
      ...config,
      isUserUploaded: true
    };
  }
}
