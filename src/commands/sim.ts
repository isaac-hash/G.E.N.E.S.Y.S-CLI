// src/commands/sim.ts
import { Command } from "commander";
import chalk from "chalk";
import path from "path";
import fs from "fs";
import { listLaunchFiles, normalizeLaunchFile, runLaunchFile } from "../utils/launchUtils.js";

export const simCommand = new Command("sim")
  .description("Launch a simulation environment (Gazebo) in the Genesys workspace")
  .argument("[file]", "Simulation launch file name (without suffix) or 'all' for default sim")
  .option("-l, --list", "List available simulation launch files")
  .option("-w, --world <world>", "Specify a Gazebo world file to launch")
  .option("-a, --args <args...>", "Pass arbitrary ROS 2 launch arguments")
  .action(
    (
      file: string | undefined,
      options: { list?: boolean; world?: string; args?: string[] }
    ) => {
      const workspaceRoot = process.cwd();
      const launchDir = path.join(workspaceRoot, "launch");

      if (!fs.existsSync(launchDir)) {
        console.error(chalk.red("❌ No 'launch/' directory found. Did you scaffold any nodes?"));
        process.exit(1);
      }

      // List mode
      if (options.list) {
        const files = listLaunchFiles(launchDir);
        if (files.length === 0) {
          console.log(chalk.yellow("⚠️ No launch files found."));
          return;
        }
        console.log(chalk.cyan("\n📂 Available simulation launch files:\n"));
        files.forEach((f) => console.log("  • " + chalk.green(f)));
        console.log();
        return;
      }

      // Determine launch file
      let launchFile = "genesys_launch.py"; // default global launch
      if (file && file.toLowerCase() !== "all") {
        launchFile = normalizeLaunchFile(file);
      }

      // Prepare launch arguments
      const launchArgs: string[] = [];
      if (options.world) {
        console.log(chalk.yellow(`🌎 Launching with world: ${options.world}`));
        launchArgs.push(`world:=${options.world}`);
      }
      if (options.args) {
        launchArgs.push(...options.args);
      }

      // Run the launch file with optional args
      runLaunchFile(workspaceRoot, launchFile, launchArgs);
    }
  );
