// src/commands/new.ts
import { Command } from "commander";
import fs from "fs";
import path from "path";
import { execSync } from "child_process";

// Run shell command safely
function runCmd(cmd: string): string | null {
  try {
    return execSync(cmd, { stdio: ["pipe", "pipe", "ignore"] })
      .toString()
      .trim();
  } catch {
    return null;
  }
}

// Detect ROS version
function detectROS(): "ros2" | "ros1" | "none" {
  const ros2 = runCmd("ros2 --version");
  if (ros2) return "ros2";
  const ros1 = runCmd("rosversion -d");
  if (ros1) return "ros1";
  return "none";
}

// Check dependencies list
function checkDependencies() {
  console.log("🔍 Checking dependencies...");

  const ros = detectROS();
  if (ros === "none") {
    console.error("❌ ROS not detected. Please install ROS 2 (recommended).");
    process.exit(1);
  }

  console.log(`✅ Detected ${ros.toUpperCase()}`);

  const deps = [
    { cmd: "python3 --version", name: "Python 3" },
    { cmd: "cmake --version", name: "CMake" },
    { cmd: "colcon --version", name: "colcon build" },
  ];

  let allDepsOk = true;
  deps.forEach((dep) => {
    if (!runCmd(dep.cmd)) {
      console.error(`❌ Missing dependency: ${dep.name}`);
      allDepsOk = false;
    }
  });

  if (!allDepsOk) {
    console.error(
      "\n⚠️ One or more dependencies are missing. Please install them before creating a GENESYS project."
    );
    process.exit(1);
  }

  console.log("✅ All dependencies OK!\n");
}

// Create base project structure
function createStructure(projectRoot: string, name: string) {
  const dirs = [
    "app",
    "src",
    "python",
    "interfaces",
    "config",
    "launch",
    "sim",
    "packages",
    "tests",
    "scripts",
    "tools",
  ];

  fs.mkdirSync(projectRoot, { recursive: true });
  dirs.forEach((dir) => {
    fs.mkdirSync(path.join(projectRoot, dir), { recursive: true });
  });

  // README
  fs.writeFileSync(
    path.join(projectRoot, "README.md"),
    `# ${name}

A **GENESYS** robotics project.

## Structure
- \`app/\`: high-level logic/controllers
- \`src/\`: C++ ROS 2 nodes
- \`python/\`: Python ROS 2 nodes
- \`interfaces/\`: shared msgs, srvs, actions
- \`config/\`: YAML configs
- \`launch/\`: ROS 2 launch files
- \`sim/\`: URDF + Gazebo worlds
- \`packages/\`: installable modules
- \`tests/\`: testing
- \`scripts/\`: helper scripts
- \`tools/\`: dev utilities

Generated with **GENESYS CLI** 🚀
`
  );

  // Robot config
  fs.writeFileSync(
    path.join(projectRoot, "config", "robot.yaml"),
    `# Robot configuration
name: ${name}
sensors: {}
motors: {}
`
  );

  // Launch stub
  fs.writeFileSync(
    path.join(projectRoot, "launch", `${name}_launch.py`),
    `from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([])
`
  );

  // package.xml
  fs.writeFileSync(
    path.join(projectRoot, "package.xml"),
    `<package format="3">
  <name>${name}</name>
  <version>0.1.0</version>
  <description>GENESYS robotics project</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
`
  );

  // Workspace-level CMakeLists.txt (so colcon has an entry)
  fs.writeFileSync(
    path.join(projectRoot, "src", "CMakeLists.txt"),
    `cmake_minimum_required(VERSION 3.8)
project(${name}_workspace)

# Add subdirectories for nodes as they are created
`
  );

  // Drop COLCON_IGNORE into non-package dirs
  const ignoreDirs = [
    "app",
    "python",
    "interfaces",
    "config",
    "launch",
    "sim",
    "packages",
    "tests",
    "scripts",
    "tools",
  ];
  ignoreDirs.forEach((dir) => {
    fs.writeFileSync(path.join(projectRoot, dir, "COLCON_IGNORE"), "");
  });
}

export default function newProject(program: Command) {
  program
    .command("new <name>")
    .description("Create a new GENESYS robotics project")
    .action((name) => {
      checkDependencies();

      const projectRoot = path.resolve(process.cwd(), name);

      if (fs.existsSync(projectRoot)) {
        console.error(`❌ Project folder '${name}' already exists.`);
        process.exit(1);
      }

      createStructure(projectRoot, name);

      console.log(`✅ New GENESYS project '${name}' created at ${projectRoot}`);
      console.log(`👉 Next: cd ${name} && genesys make:node MyNode`);
    });
}
