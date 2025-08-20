import { Command } from "commander";
import fs from "fs";
import path from "path";

export default function newProject(program: Command) {
  program
    .command("new <name>")
    .description("Create a new GENESYS robotics project")
    .action((name) => {
      const projectRoot = path.resolve(process.cwd(), name);

      if (fs.existsSync(projectRoot)) {
        console.error(`❌ Project folder '${name}' already exists.`);
        process.exit(1);
      }

      // Base structure
      const dirs = [
        "app",          // high-level logic/controllers
        "src",          // C++ ROS 2 nodes
        "python",       // Python ROS 2 nodes
        "interfaces",   // shared msgs/srvs/actions
        "config",       // YAML configs
        "launch",       // launch files
        "sim",          // URDF, Gazebo worlds
        "packages",     // installable modules
        "tests",        // unit/integration tests
        "scripts",      // helper scripts
        "tools",        // dev utilities
        ".devcontainer" // VS Code Dev Container config
      ];

      fs.mkdirSync(projectRoot, { recursive: true });
      dirs.forEach((dir) => {
        fs.mkdirSync(path.join(projectRoot, dir), { recursive: true });
      });

      // --- README ---
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
- \`.devcontainer/\`: VS Code Dev Container setup

Generated with **GENESYS CLI** 🚀
`
      );

      // --- Robot config ---
      fs.writeFileSync(
        path.join(projectRoot, "config", "robot.yaml"),
        `# Robot configuration
name: ${name}
sensors: {}
motors: {}
`
      );

      // --- Launch stub ---
      fs.writeFileSync(
        path.join(projectRoot, "launch", `${name}_launch.py`),
        `from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([])
`
      );

      // --- ROS package.xml ---
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

      // --- Devcontainer config ---
      fs.writeFileSync(
        path.join(projectRoot, ".devcontainer", "devcontainer.json"),
        `{
  "name": "${name}-dev",
  "dockerFile": "../Dockerfile",
  "context": "..",
  "workspaceFolder": "/workspace",
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-iot.vscode-ros",
        "ms-python.python",
        "ms-vscode.cpptools",
        "ms-azuretools.vscode-docker"
      ]
    }
  },
  "remoteUser": "root"
}
`
      );

      // --- Dockerfile ---
      fs.writeFileSync(
        path.join(projectRoot, "Dockerfile"),
        `# GENESYS Robotics Project Dockerfile
FROM ros:humble

# Install common tools
RUN apt-get update && apt-get install -y \\
    python3-pip \\
    build-essential \\
    ros-humble-ros-base \\
    && rm -rf /var/lib/apt/lists/*

# Workspace setup
WORKDIR /workspace
COPY . /workspace

# Install Python deps if requirements.txt exists
RUN if [ -f requirements.txt ]; then pip3 install -r requirements.txt; fi

# Build ROS 2 workspace
RUN . /opt/ros/humble/setup.sh && colcon build || true

CMD ["/bin/bash"]
`
      );

      // --- docker-compose.yml ---
      fs.writeFileSync(
        path.join(projectRoot, "docker-compose.yml"),
        `version: "3.9"
services:
  app:
    build: .
    container_name: ${name}_container
    volumes:
      - .:/workspace
    working_dir: /workspace
    tty: true
    stdin_open: true
`
      );

      console.log(`✅ New GENESYS project '${name}' created at ${projectRoot}`);
      console.log(`👉 Next: cd ${name} && genesys make:node MyNode --lang python`);
    });
}
