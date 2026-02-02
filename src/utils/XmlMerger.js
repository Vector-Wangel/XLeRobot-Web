/**
 * MuJoCo XML Merger Utility
 *
 * Merges environment, robot, and objects XML files at runtime.
 * This allows decoupling of environments (with 3DGS) from robot definitions.
 */

export class MujocoXmlMerger {
  /**
   * Merge environment, robot, and optional objects XML
   * @param {string} envXml - Environment XML (lighting, collision boxes, floor)
   * @param {string} robotXml - Robot XML (robot body, actuators, sensors)
   * @param {string} objectsXml - Optional manipulable objects XML (freejoint bodies)
   * @returns {string} - Merged XML string
   */
  static merge(envXml, robotXml, objectsXml = null) {
    const parser = new DOMParser();
    const serializer = new XMLSerializer();

    const envDoc = parser.parseFromString(envXml, 'text/xml');
    const robotDoc = parser.parseFromString(robotXml, 'text/xml');

    // Check for parse errors
    if (envDoc.querySelector('parsererror')) {
      throw new Error('Failed to parse environment XML');
    }
    if (robotDoc.querySelector('parsererror')) {
      throw new Error('Failed to parse robot XML');
    }

    // 1. Merge <compiler> - robot's compiler takes precedence for meshdir
    this._mergeCompiler(envDoc, robotDoc);

    // 2. Merge <option> nodes
    this._mergeNodes(envDoc, robotDoc, 'option');

    // 3. Merge <asset> nodes (materials, textures, meshes)
    this._mergeNodes(envDoc, robotDoc, 'asset');

    // 4. Merge <default> nodes (default classes)
    this._mergeNodes(envDoc, robotDoc, 'default');

    // 5. Merge <worldbody> - insert robot bodies into environment
    this._mergeWorldbody(envDoc, robotDoc);

    // 6. Copy robot-specific sections (contact, tendon, actuator, sensor, keyframe)
    this._copyNodes(envDoc, robotDoc, ['contact', 'tendon', 'actuator', 'sensor', 'keyframe']);

    // 7. If objects XML provided, merge it too
    if (objectsXml) {
      const objDoc = parser.parseFromString(objectsXml, 'text/xml');
      if (objDoc.querySelector('parsererror')) {
        throw new Error('Failed to parse objects XML');
      }
      this._mergeNodes(envDoc, objDoc, 'asset');
      this._mergeWorldbody(envDoc, objDoc);
    }

    // Serialize back to string
    let result = serializer.serializeToString(envDoc);

    // Clean up XML declaration if duplicated
    result = result.replace(/(<\?xml[^?]*\?>)/g, '');
    result = '<?xml version="1.0" encoding="UTF-8"?>\n' + result.trim();

    return result;
  }

  /**
   * Merge compiler settings - robot's meshdir is critical
   */
  static _mergeCompiler(targetDoc, sourceDoc) {
    const sourceCompiler = sourceDoc.querySelector('compiler');
    if (!sourceCompiler) return;

    let targetCompiler = targetDoc.querySelector('compiler');
    if (!targetCompiler) {
      targetCompiler = targetDoc.createElement('compiler');
      // Insert after mujoco opening tag
      const mujoco = targetDoc.documentElement;
      mujoco.insertBefore(targetCompiler, mujoco.firstChild);
    }

    // Copy all attributes from source compiler
    for (const attr of sourceCompiler.attributes) {
      targetCompiler.setAttribute(attr.name, attr.value);
    }
  }

  /**
   * Merge child elements of a specific node type
   */
  static _mergeNodes(targetDoc, sourceDoc, nodeName) {
    const sourceNode = sourceDoc.querySelector(nodeName);
    if (!sourceNode) return;

    let targetNode = targetDoc.querySelector(nodeName);
    if (!targetNode) {
      targetNode = targetDoc.createElement(nodeName);
      targetDoc.documentElement.appendChild(targetNode);
    }

    // Copy attributes from source node
    for (const attr of sourceNode.attributes) {
      if (!targetNode.hasAttribute(attr.name)) {
        targetNode.setAttribute(attr.name, attr.value);
      }
    }

    // Copy all child elements
    for (const child of sourceNode.children) {
      targetNode.appendChild(child.cloneNode(true));
    }
  }

  /**
   * Merge worldbody contents - insert source bodies into target worldbody
   */
  static _mergeWorldbody(targetDoc, sourceDoc) {
    const sourceWb = sourceDoc.querySelector('worldbody');
    const targetWb = targetDoc.querySelector('worldbody');
    if (!sourceWb || !targetWb) return;

    // Copy all child elements (bodies, geoms, lights, etc.)
    for (const child of sourceWb.children) {
      targetWb.appendChild(child.cloneNode(true));
    }
  }

  /**
   * Copy entire nodes from source to target (for contact, tendon, actuator, etc.)
   */
  static _copyNodes(targetDoc, sourceDoc, nodeNames) {
    for (const name of nodeNames) {
      const sourceNode = sourceDoc.querySelector(name);
      if (!sourceNode) continue;

      let targetNode = targetDoc.querySelector(name);
      if (targetNode) {
        // Merge children into existing node
        for (const child of sourceNode.children) {
          targetNode.appendChild(child.cloneNode(true));
        }
      } else {
        // Copy entire node
        targetDoc.documentElement.appendChild(sourceNode.cloneNode(true));
      }
    }
  }

  /**
   * Fix mesh paths in XML string
   * Converts relative meshdir to absolute path
   * @param {string} xml - XML string
   * @param {string} robotDir - Robot directory path (e.g., '/working/xlerobot')
   * @param {string} meshDirName - Mesh directory name (default: 'meshes', panda uses 'assets')
   * @returns {string} - XML with fixed paths
   */
  static fixMeshPaths(xml, robotDir, meshDirName = null) {
    // Normalize path separators
    const normalizedDir = robotDir.replace(/\\/g, '/');

    // Check if XML has meshdir at all
    const meshdirMatch = xml.match(/meshdir="([^"]+)"/);
    if (!meshdirMatch) {
      // No meshdir in XML (e.g., humanoid uses primitives), return as-is
      return xml;
    }

    // If meshDirName not specified, detect from XML
    if (!meshDirName) {
      const currentMeshdir = meshdirMatch[1].replace(/^\.\//, '').replace(/\/$/, '');
      meshDirName = currentMeshdir;
    }

    // Replace any meshdir with absolute path
    return xml.replace(
      /meshdir="[^"]*"/g,
      `meshdir="${normalizedDir}/${meshDirName}/"`
    );
  }

  /**
   * Update model name in merged XML
   * @param {string} xml - XML string
   * @param {string} modelName - New model name
   * @returns {string} - XML with updated model name
   */
  static setModelName(xml, modelName) {
    return xml.replace(
      /<mujoco\s+model="[^"]*"/,
      `<mujoco model="${modelName}"`
    );
  }
}
