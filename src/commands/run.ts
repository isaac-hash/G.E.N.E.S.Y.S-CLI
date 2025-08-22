// src/commands/run.ts
import { Command } from "commander";
import chalk from "chalk";
import path from "path";
import fs from "fs";
import { normalizeLaunchFile, runLaunchFile, listLaunchFiles } from "../utils/launchUtils.js";

export const runCommand = new Command("run")
  .description("Run a specific node or all nodes in the Genesys workspace")
  .argument("[nodeName]", "Name of the node to run, or 'all' for all nodes")
  .option("-l, --list", "List available launch files")
  .option("-a, --args <args...>", "Pass arbitrary ROS 2 launch arguments (e.g., key:=value)")
  .action(
    (
      nodeName: string | undefined,
      options: { list?: boolean; args?: string[] }
    ) => {
      const workspaceRoot = process.cwd();
      const launchDir = path.join(workspaceRoot, "launch");

      if (!fs.existsSync(launchDir)) {
        console.error(
          chalk.red("❌ No 'launch/' directory found. Did you scaffold any nodes?")
        );
        process.exit(1);
      }

      // List available launch files
      if (options.list) {
        const files = listLaunchFiles(launchDir);
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
      if (nodeName && nodeName.toLowerCase() !== "all") {
        launchFile = normalizeLaunchFile(nodeName);
      }

      // Run the launch file with optional args
      const launchArgs = options.args || [];
      runLaunchFile(workspaceRoot, launchFile, launchArgs);
    }
  );
