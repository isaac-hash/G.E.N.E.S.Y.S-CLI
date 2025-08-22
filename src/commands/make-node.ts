// src/commands/make-node.ts
import { Command } from "commander";
import inquirer from "inquirer";
import fs from "fs";
import path from "path";
import chalk from "chalk";

export default function makeNode(program: Command) {
  program
    .command("make:node <name>")
    .description("Generate a new ROS 2 package with a node (Python or C++) and register it")
    .action(async (name: string) => {
      const { lang } = await inquirer.prompt([
        {
          type: "list",
          name: "lang",
          message: "Which language do you want to use?",
          choices: ["Python", "C++"],
        },
      ]);

      const baseDir = process.cwd();
      const launchDir = path.join(baseDir, "launch");
      if (!fs.existsSync(launchDir)) fs.mkdirSync(launchDir, { recursive: true });

      // -------------------------------------------------
      // PYTHON PACKAGE
      // -------------------------------------------------
      if (lang === "Python") {
        const pkgDir = path.join(baseDir, "python", name);
        const moduleDir = path.join(pkgDir, name);

        fs.mkdirSync(moduleDir, { recursive: true });

        // __init__.py for Python package
        fs.writeFileSync(path.join(moduleDir, "__init__.py"), "");

        // Node implementation
        const nodePath = path.join(moduleDir, "main.py");
        const nodeContent = `import rclpy
from rclpy.node import Node

class ${name}(Node):
    def __init__(self):
        super().__init__('${name.toLowerCase()}')
        self.get_logger().info("${name} node has started")

def main(args=None):
    rclpy.init(args=args)
    node = ${name}()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
`;
        fs.writeFileSync(nodePath, nodeContent);

        // setup.py
        const setupPath = path.join(pkgDir, "setup.py");
        const setupContent = `from setuptools import setup

package_name = '${name}'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='ROS 2 Python node package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = ${name}.main:main',
        ],
    },
)
`;
        fs.writeFileSync(setupPath, setupContent);

        // package.xml
        const pkgXmlPath = path.join(pkgDir, "package.xml");
        const pkgXmlContent = `<package format="3">
  <name>${name}</name>
  <version>0.1.0</version>
  <description>Python node package for ROS 2</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
</package>
`;
        fs.writeFileSync(pkgXmlPath, pkgXmlContent);

        // resource/<pkg>
        const resDir = path.join(pkgDir, "resource");
        fs.mkdirSync(resDir, { recursive: true });
        fs.writeFileSync(path.join(resDir, name), "");

        // Launch file
        const launchFile = path.join(launchDir, `${name}_launch.py`);
        const launchContent = `from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='${name}',
            executable='main',
            name='${name.toLowerCase()}',
            output='screen'
        )
    ])
`;
        fs.writeFileSync(launchFile, launchContent);

        registerInGlobalLaunch(launchDir, name);

        console.log(chalk.greenBright("✅ Python package created at: ") + chalk.cyan(pkgDir));
        console.log(chalk.magenta(`👉 Build with: colcon build --packages-select ${name}`));
        console.log(chalk.magenta(`👉 Run with: ros2 launch ${name} ${name}_launch.py`));
      }

      // -------------------------------------------------
      // C++ PACKAGE
      // -------------------------------------------------
      if (lang === "C++") {
        const pkgDir = path.join(baseDir, "src", name);
        fs.mkdirSync(pkgDir, { recursive: true });

        // Node implementation
        const nodePath = path.join(pkgDir, "main.cpp");
        const nodeContent = `#include "rclcpp/rclcpp.hpp"

class ${name} : public rclcpp::Node {
public:
    ${name}() : Node("${name.toLowerCase()}") {
        RCLCPP_INFO(this->get_logger(), "${name} node has started");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<${name}>());
    rclcpp::shutdown();
    return 0;
}
`;
        fs.writeFileSync(nodePath, nodeContent);

        // CMakeLists.txt
        const cmakePath = path.join(pkgDir, "CMakeLists.txt");
        const cmakeContent = `cmake_minimum_required(VERSION 3.8)
project(${name})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(main main.cpp)
ament_target_dependencies(main rclcpp)

install(TARGETS
  main
  DESTINATION lib/\${PROJECT_NAME}
)

ament_package()
`;
        fs.writeFileSync(cmakePath, cmakeContent);

        // package.xml
        const pkgXmlPath = path.join(pkgDir, "package.xml");
        const pkgXmlContent = `<package format="3">
  <name>${name}</name>
  <version>0.1.0</version>
  <description>C++ node package for ROS 2</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
</package>
`;
        fs.writeFileSync(pkgXmlPath, pkgXmlContent);

        // Launch file
        const launchFile = path.join(launchDir, `${name}_launch.py`);
        const launchContent = `from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='${name}',
            executable='main',
            name='${name.toLowerCase()}',
            output='screen'
        )
    ])
`;
        fs.writeFileSync(launchFile, launchContent);

        registerInGlobalLaunch(launchDir, name);

        console.log(chalk.greenBright("✅ C++ package created at: ") + chalk.cyan(pkgDir));
        console.log(chalk.magenta(`👉 Build with: colcon build --packages-select ${name}`));
        console.log(chalk.magenta(`👉 Run with: ros2 launch ${name} ${name}_launch.py`));
      }
    });
}

/**
 * Auto-registers new launch files inside a global genesys_launch.py
 */
function registerInGlobalLaunch(launchDir: string, nodeName: string) {
  const globalLaunch = path.join(launchDir, "genesys_launch.py");
  let globalContent = "";

  if (!fs.existsSync(globalLaunch)) {
    globalContent = `from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()
    return ld
`;
  } else {
    globalContent = fs.readFileSync(globalLaunch, "utf8");
  }

  if (!globalContent.includes(`${nodeName}_launch.py`)) {
    const includeStmt = `    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), "${nodeName}_launch.py"))
    ))\n    `;
    globalContent = globalContent.replace("return ld", includeStmt + "return ld");
    fs.writeFileSync(globalLaunch, globalContent);
    console.log(chalk.yellowBright(`📦 Registered node '${nodeName}' in genesys_launch.py`));
  }
}
