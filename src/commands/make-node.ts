// src/commands/make-node.ts
import { Command } from "commander";
import inquirer from "inquirer";
import fs from "fs";
import path from "path";
import chalk from "chalk";

export default function makeNode(program: Command) {
  program
    .command("make:node <name>")
    .description("Generate a new robotics node (Python or C++) and register it")
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

      // ---------- PYTHON ----------
      if (lang === "Python") {
        const dir = path.join(baseDir, "python", name);
        if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });

        const filePath = path.join(dir, "main.py");
        const content = `import rclpy
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
        fs.writeFileSync(filePath, content);

        // ---------- Launch file ----------
        const launchDir = path.join(baseDir, "launch");
        if (!fs.existsSync(launchDir)) fs.mkdirSync(launchDir, { recursive: true });
        const launchFile = path.join(launchDir, `${name}_launch.py`);
        const launchContent = `from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python',
            executable='${name}',
            name='${name.toLowerCase()}',
            output='screen'
        )
    ])
`;
        fs.writeFileSync(launchFile, launchContent);

        // ---------- Global genesys_launch.py ----------
        const globalLaunch = path.join(launchDir, "genesys_launch.py");
        let globalContent = "";
        if (fs.existsSync(globalLaunch)) {
          globalContent = fs.readFileSync(globalLaunch, "utf8");
        } else {
          globalContent = "from launch import LaunchDescription\n\n";
          globalContent += "def generate_launch_description():\n";
          globalContent += "    ld = LaunchDescription()\n";
          globalContent += "    return ld\n";
        }
        if (!globalContent.includes(`${name}_launch`)) {
          // Insert import and append node
          globalContent = `from .${name}_launch import generate_launch_description as ${name}_launch\n` +
            globalContent.replace(
              "return ld",
              `ld.add_action(${name}_launch())\n    return ld`
            );
          fs.writeFileSync(globalLaunch, globalContent);
          console.log(chalk.yellowBright(`📦 Registered Python node '${name}' in genesys_launch.py`));
        }

        console.log(chalk.greenBright("✅ Python node created at: ") + chalk.cyan(filePath));
        console.log(chalk.greenBright("📄 Launch file created at: ") + chalk.cyan(launchFile));
      }

      // ---------- C++ ----------
      if (lang === "C++") {
        const dir = path.join(baseDir, "src", name);
        if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });

        const filePath = path.join(dir, "main.cpp");
        const cmakePath = path.join(dir, "CMakeLists.txt");

        const content = `#include "rclcpp/rclcpp.hpp"

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
        const cmakeContent = `cmake_minimum_required(VERSION 3.8)
project(${name})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${name} main.cpp)
ament_target_dependencies(${name} rclcpp)

install(TARGETS
  ${name}
  DESTINATION lib/${name}
)

ament_package()
`;
        fs.writeFileSync(filePath, content);
        fs.writeFileSync(cmakePath, cmakeContent);

        // auto-register in root CMakeLists.txt
        const rootCmake = path.join(baseDir, "src", "CMakeLists.txt");
        if (fs.existsSync(rootCmake)) {
          let rootContent = fs.readFileSync(rootCmake, "utf8");
          if (!rootContent.includes(`add_subdirectory(${name})`)) {
            rootContent += `\nadd_subdirectory(${name})\n`;
            fs.writeFileSync(rootCmake, rootContent);
            console.log(chalk.yellowBright(`📦 Registered C++ node '${name}' in root CMakeLists.txt`));
          }
        }

        // ---------- Launch file ----------
        const launchDir = path.join(baseDir, "launch");
        if (!fs.existsSync(launchDir)) fs.mkdirSync(launchDir, { recursive: true });
        const launchFile = path.join(launchDir, `${name}_launch.py`);
        const launchContent = `from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='${name}',
            executable='${name}',
            name='${name.toLowerCase()}',
            output='screen'
        )
    ])
`;
        fs.writeFileSync(launchFile, launchContent);

        console.log(chalk.greenBright("✅ C++ node created at: ") + chalk.cyan(filePath));
        console.log(chalk.greenBright("📄 Launch file created at: ") + chalk.cyan(launchFile));
      }
    });
}
