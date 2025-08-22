#!/usr/bin/env node
import { Command } from "commander";
import { spawn } from "child_process";
import path from "path";
import fs from "fs";
import chalk from "chalk";

/**
 * Original buildWorkspace function
 */
export async function buildWorkspace(args: { release?: boolean }) {
  const projectRoot = process.cwd();
  const srcDir = path.join(projectRoot, "src");
  const pythonDir = path.join(projectRoot, "python");

  if (!fs.existsSync(srcDir) && !fs.existsSync(pythonDir)) {
    console.error(chalk.red("❌ No src/ or python/ directories found. Are you in a Genesys workspace?"));
    process.exit(1);
  }

  const rosDistro = process.env.ROS_DISTRO || "humble";
  if (!process.env.ROS_DISTRO) {
    console.warn(chalk.yellow(`⚠️  ROS_DISTRO not set, defaulting to '${rosDistro}'`));
  }

  console.log(chalk.cyan(`\n🔧 Building Genesys workspace with ROS 2 '${rosDistro}'...\n`));

  const rosSetup = `/opt/ros/${rosDistro}/setup.bash`;
  let buildCmd = `source ${rosSetup} && colcon build --symlink-install --event-handlers console_direct+`;

  if (args.release) {
    console.log(chalk.green("⚡ Building in RELEASE mode (optimized)..."));
    buildCmd += " --cmake-args -DCMAKE_BUILD_TYPE=Release";
  } else {
    console.log(chalk.green("🐛 Building in DEBUG mode (default)..."));
  }

  const colcon = spawn("bash", ["-c", buildCmd], { stdio: "inherit", cwd: projectRoot });

  colcon.on("close", (code) => {
    if (code === 0) {
      console.log(chalk.green.bold("\n✅ Build completed successfully!\n"));
      console.log(chalk.yellow("💡 Tip: Run `source install/setup.bash` before running nodes.\n"));
    } else {
      console.error(chalk.red("\n❌ Build failed. See errors above.\n"));
      process.exit(code ?? 1);
    }
  });
}

/**
 * Wrap in Commander command
 */
export const buildCommand = new Command("build")
  .description("Build the Genesys workspace")
  .option("-r, --release", "Build in release mode")
  .action(async (options: { release?: boolean }) => {
    await buildWorkspace({ release: options.release });
  });
