import { Command } from "commander";
import { spawn } from "child_process";
import fs from "fs";
import path from "path";
import chalk from "chalk";

export const replayCommand = new Command("replay")
  .description("Replay a ROS 2 bag file")
  .argument("[file]", "Bag file to replay (defaults to the latest .bag in workspace)")
  .option("-a, --args <args...>", "Additional ros2 bag play arguments")
  .action((file: string | undefined, options?: { args?: string[] }) => {
    const workspaceDir = process.cwd();

    let bagFile: string;

    if (file) {
      bagFile = path.resolve(workspaceDir, file);
      if (!fs.existsSync(bagFile)) {
        console.error(chalk.red(`❌ Bag file not found: ${bagFile}`));
        process.exit(1);
      }
    } else {
      // Find latest .bag file in workspace
      const bagFiles = fs.readdirSync(workspaceDir)
        .filter((f) => f.endsWith(".bag"))
        .map((f) => path.join(workspaceDir, f));

      if (bagFiles.length === 0) {
        console.error(chalk.red("❌ No .bag files found in the workspace."));
        process.exit(1);
      }

      // Sort by modified time descending
      bagFiles.sort((a, b) => fs.statSync(b).mtime.getTime() - fs.statSync(a).mtime.getTime());
      bagFile = bagFiles[0];
      console.log(chalk.yellow(`⚡ No bag file specified, defaulting to latest: ${path.basename(bagFile)}`));
    }

    const rosDistro = process.env.ROS_DISTRO || "humble";
    if (!process.env.ROS_DISTRO) {
      console.warn(chalk.yellow(`⚠️ ROS_DISTRO not set, defaulting to '${rosDistro}'`));
    }

    const rosSetup = `/opt/ros/${rosDistro}/setup.bash`;
    const extraArgs = options?.args ? options.args.join(" ") : "";

    console.log(chalk.cyan(`▶️ Replaying bag file: ${bagFile}`));
    if (extraArgs) console.log(chalk.yellow(`📄 With extra arguments: ${extraArgs}`));

    const cmd = `source ${rosSetup} && ros2 bag play ${bagFile} ${extraArgs}`;
    const proc = spawn("bash", ["-c", cmd], { stdio: "inherit" });

    proc.on("close", (code) => {
      console.log(chalk.green(`✅ Replay finished with exit code ${code}`));
    });

    // Forward SIGINT to child process
    process.on("SIGINT", () => {
      proc.kill("SIGINT");
    });
  });
