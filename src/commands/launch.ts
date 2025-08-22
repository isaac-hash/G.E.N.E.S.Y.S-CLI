// src/commands/launch.ts
import { Command } from "commander";
import chalk from "chalk";
import path from "path";
import fs from "fs";
import { listLaunchFiles, runLaunchFile, normalizeLaunchFile } from "../utils/launchUtils.js";

export const launchCommand = new Command("launch")
  .description("Manage and run ROS 2 launch files in the Genesys workspace")
  .argument("[file]", "Launch file name (no need to include suffix) or 'all' to run global genesys_launch.py")
  .option("-l, --list", "List available launch files")
  .option("-a, --args <args...>", "Pass arbitrary ROS 2 launch arguments (e.g., key:=value)")
  .action((file: string | undefined, options?: { list?: boolean; args?: string[] }) => {
    const workspaceRoot = process.cwd();
    const launchDir = path.join(workspaceRoot, "launch");

    if (!fs.existsSync(launchDir)) {
      console.error(chalk.red("❌ No 'launch/' directory found. Did you scaffold any nodes?"));
      process.exit(1);
    }

    const files = listLaunchFiles(launchDir);

    // List mode
    if (options?.list) {
      if (files.length === 0) {
        console.log(chalk.yellow("⚠️ No launch files found yet."));
        return;
      }
      console.log(chalk.cyan("\n📂 Available launch files:\n"));
      files.forEach((f) => console.log("  • " + chalk.green(f)));
      console.log();
      return;
    }

    // Determine launch file
    let launchFile = "genesys_launch.py"; // default global launch
    if (file && file.toLowerCase() !== "all") {
      launchFile = normalizeLaunchFile(file);
    }

    // Run the launch file with optional args
    const launchArgs = options?.args || [];
    runLaunchFile(workspaceRoot, launchFile, launchArgs);
  });
