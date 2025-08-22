import { Command } from "commander";
import { spawn } from "child_process";
import path from "path";
import fs from "fs";
import chalk from "chalk";

export const recordCommand = new Command("record")
  .description("Record ROS 2 topics to a bag file (records all topics by default)")
  .argument("[topics...]", "List of ROS topics to record")
  .option("-o, --output <file>", "Output bag file name", "recording.bag")
  .option("-a, --args <args...>", "Additional ros2 bag record arguments")
  .action((topics: string[] | undefined, options?: { output?: string; args?: string[] }) => {
    const workspaceRoot = process.cwd();
    const srcDir = path.join(workspaceRoot, "src");
    const pythonDir = path.join(workspaceRoot, "python");

    // Validate workspace
    if (!fs.existsSync(srcDir) && !fs.existsSync(pythonDir)) {
      console.error(
        chalk.red("❌ No src/ or python/ directories found. Are you in a Genesys workspace?")
      );
      process.exit(1);
    }

    // Record all topics by default if none specified
    const topicsToRecord = topics && topics.length > 0 ? topics : ["-a"];

    const rosDistro = process.env.ROS_DISTRO || "humble";
    if (!process.env.ROS_DISTRO) {
      console.warn(chalk.yellow(`⚠️ ROS_DISTRO not set, defaulting to '${rosDistro}'`));
    }

    console.log(
      chalk.cyan(
        `🛑 Recording topics: ${
          topicsToRecord.includes("-a") ? "ALL TOPICS" : topicsToRecord.join(", ")
        }`
      )
    );
    console.log(chalk.yellow(`Output bag file: ${options?.output}`));

    const rosSetup = `/opt/ros/${rosDistro}/setup.bash`;
    const extraArgs = options?.args ? options.args.join(" ") : "";
    const cmd = `source ${rosSetup} && ros2 bag record -o ${options?.output} ${topicsToRecord.join(
      " "
    )} ${extraArgs}`;

    const proc = spawn("bash", ["-c", cmd], { stdio: "inherit" });

    proc.on("close", (code) => {
      console.log(chalk.green(`✅ Recording stopped with exit code ${code}`));
    });

    // Forward SIGINT to child process
    process.on("SIGINT", () => {
      proc.kill("SIGINT");
    });
  });
