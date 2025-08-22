// src/utils/launchUtils.ts
import { spawn } from "child_process";
import path from "path";
import fs from "fs";
import chalk from "chalk";

/* ----------------------------- Normalizers ----------------------------- */

/**
 * Normalize a launch file input.
 * Accepts: "nav", "nav_launch", "nav_launch.py"
 * Returns: "nav_launch.py"
 */
export function normalizeLaunchFile(input: string): string {
  let file = input.trim();

  if (!file.endsWith(".py") && !file.endsWith(".xml")) {
    if (!file.endsWith("_launch")) file += "_launch";
    file += ".py";
  }

  return file;
}

/* ----------------------------- Launch utils ----------------------------- */

/**
 * List all launch files in the workspace.
 */
export function listLaunchFiles(workspaceRoot: string): string[] {
  const launchDir = path.join(workspaceRoot, "launch");
  if (!fs.existsSync(launchDir)) return [];

  return fs
    .readdirSync(launchDir)
    .filter(
      (f) =>
        f.endsWith("_launch.py") ||
        f.endsWith(".launch.py") ||
        f.endsWith(".launch.xml") ||
        f === "genesys_launch.py"
    );
}

/**
 * Run a launch file from the workspace with optional ROS 2 launch arguments.
 *
 * @param workspaceRoot - Path to the workspace root
 * @param fileName - Launch file name (absolute or relative to launch/)
 * @param args - Optional array of launch arguments, e.g., ["world:=my_world.world"]
 * @param rosDistro - ROS 2 distro to use (defaults to process.env.ROS_DISTRO or 'humble')
 */
export function runLaunchFile(
  workspaceRoot: string,
  fileName: string,
  args: string[] = [],
  rosDistro: string = process.env.ROS_DISTRO || "humble"
) {
  const launchDir = path.join(workspaceRoot, "launch");
  const filePath = path.isAbsolute(fileName)
    ? fileName
    : path.join(launchDir, fileName);

  if (!fs.existsSync(filePath)) {
    console.error(chalk.red(`❌ Launch file not found: ${fileName}`));
    process.exit(1);
  }

  const rosSetup = `/opt/ros/${rosDistro}/setup.bash`;
  const argsString = args.join(" "); // convert args array to string

  console.log(
    chalk.cyan(
      `🚀 Launching: ${chalk.green(filePath)} with ROS 2 '${rosDistro}'` +
        (argsString ? ` and args: ${chalk.yellow(argsString)}` : "")
    )
  );

  const proc = spawn(
    "bash",
    ["-c", `source ${rosSetup} && ros2 launch ${filePath} ${argsString}`],
    { stdio: "inherit" }
  );

  proc.on("close", (code) => {
    if (code === 0) console.log(chalk.green("✅ Launch completed successfully"));
    else console.error(chalk.red(`❌ Launch exited with code ${code}`));
  });

  return proc;
}


/* ----------------------------- Node utils ----------------------------- */

/**
 * Spawn ros2 run process for a package executable.
 */
export function runNode(
  packageName: string,
  executable: string,
  rosDistro: string = process.env.ROS_DISTRO || "humble"
) {
  const rosSetup = `/opt/ros/${rosDistro}/setup.bash`;

  console.log(
    chalk.cyan(`▶️ Running node: ${packageName}/${executable} with ROS 2 '${rosDistro}'`)
  );

  const proc = spawn("bash", ["-c", `source ${rosSetup} && ros2 run ${packageName} ${executable}`], {
    stdio: "inherit",
  });

  proc.on("close", (code) => {
    if (code === 0) console.log(chalk.green("✅ Node exited cleanly"));
    else console.error(chalk.red(`❌ Node exited with code ${code}`));
  });

  return proc;
}

/**
 * List available executables in packages under src/ and python/.
 */
export function listExecutables(workspaceRoot: string): Record<string, string[]> {
  const pkgDirs = [path.join(workspaceRoot, "src"), path.join(workspaceRoot, "python")];
  const executables: Record<string, string[]> = {};

  pkgDirs.forEach((pkgDir) => {
    if (!fs.existsSync(pkgDir)) return;

    for (const pkg of fs.readdirSync(pkgDir)) {
      const pkgPath = path.join(pkgDir, pkg);

      // C++ package
      if (fs.existsSync(path.join(pkgPath, "package.xml"))) {
        const srcPath = path.join(pkgPath, "src");
        if (fs.existsSync(srcPath)) {
          const exeFiles = fs.readdirSync(srcPath).filter((f) => f.endsWith(".cpp"));
          if (exeFiles.length > 0) {
            executables[pkg] = exeFiles.map((f) => f.replace(".cpp", ""));
          }
        }
      }

      // Python package
      else if (
        fs.existsSync(path.join(pkgPath, "setup.py")) ||
        fs.existsSync(path.join(pkgPath, "setup.cfg"))
      ) {
        const scriptsPath = path.join(pkgPath, "scripts");
        if (fs.existsSync(scriptsPath)) {
          const pyFiles = fs.readdirSync(scriptsPath).filter((f) => f.endsWith(".py"));
          if (pyFiles.length > 0) {
            executables[pkg] = pyFiles.map((f) => f.replace(".py", ""));
          }
        }
      }
    }
  });

  return executables;
}
