# 🤖 GENESYS CLI

**GENESYS CLI** is a developer-first tool to scaffold, build, and launch ROS 2 robotics projects seamlessly. 🚀 Focus on building intelligent robots while the CLI handles project structure, nodes, launch files, Docker, and VS Code devcontainers.

---

## Features

- ⚡ **Project Scaffolding:** Create new ROS 2 projects with structured directories for Python, C++, launch files, configs, and more.
- 🐍 **Python & C++ Nodes:** Interactive node creation with auto-registration.
- 🚀 **Automatic Launch Files:** Launch any node instantly.
- 🐳 **Docker Support:** Preconfigured Dockerfile and docker-compose for cross-platform builds.
- 🖥️ **VS Code Devcontainer:** Ready-to-use development container.
- 🔧 **Modular & Extendable:** Add custom packages and tools easily.

---

## Installation

```bash
git clone https://github.com/yourusername/genesys-cli.git
cd genesys-cli
npm install -g
````

> Requires Node.js and Docker installed.

---

## Usage

### Create a new project

```bash
genesys new MyRobot
```

Generates the full project structure with configs, launch files, and a README.

### Create a new node

```bash
genesys make:node MyNode
```

Interactive prompt to select Python or C++ and auto-registers the node.

### Launch all nodes

```bash
genesys launch
```

Launches all registered nodes in your project.

---

## Project Structure

```
/app            # High-level logic/controllers
/src            # C++ ROS 2 nodes
/python         # Python ROS 2 nodes
/interfaces     # Custom msgs, services, actions
/config         # YAML configs
/launch         # ROS 2 launch files
/sim            # URDF + Gazebo worlds
/packages       # Installable modules
/tests          # Unit/integration tests
/scripts        # Helper scripts
/tools          # Dev utilities
.devcontainer   # VS Code devcontainer
Dockerfile      # Project Dockerfile
docker-compose.yml
```

---

## Contributing

Contributions are welcome! Please open issues or submit pull requests for bug fixes, new features, or improvements.

---

## License

MIT License © 2025 \ Chukwudulue Isaac


