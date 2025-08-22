import { Command } from "commander";
import { spawn } from "child_process";
import chalk from "chalk";
import path from "path";
import fs from "fs";

export const testCommand = new Command("test")
  .description("Run unit or integration tests for Python and C++ packages")
  .option("-a, --args <args...>", "Additional pytest arguments")
  .action(async (options?: { args?: string[] }) => {
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

    console.log(chalk.cyan("🧪 Running tests in Genesys workspace..."));

    const testPromises: Promise<number>[] = [];

    // ----------- Top-Level Tests -----------
    const topLevelTestDir = path.join(workspaceRoot, "tests");
    if (fs.existsSync(topLevelTestDir)) {
      const pytestArgs = options?.args ? options.args.join(" ") : "";
      const cmd = `pytest ${pytestArgs}`;
      console.log(chalk.cyan("🐍 Running top-level Python tests in /tests"));
      testPromises.push(
        new Promise((resolve) => {
          const proc = spawn("bash", ["-c", cmd], { cwd: topLevelTestDir, stdio: "inherit" });
          proc.on("close", (code) => resolve(code ?? 1));
        })
      );
    }

    // ----------- Python Packages -----------
    if (fs.existsSync(pythonDir)) {
      const pythonPkgs = fs.readdirSync(pythonDir).filter((pkg) => {
        const pkgPath = path.join(pythonDir, pkg);
        return fs.existsSync(pkgPath) && fs.existsSync(path.join(pkgPath, "setup.py"));
      });

      for (const pkg of pythonPkgs) {
        const pkgPath = path.join(pythonDir, pkg);
        const testDir = path.join(pkgPath, "tests");

        if (!fs.existsSync(testDir)) {
          console.log(chalk.yellow(`⚠️ No tests/ directory for Python package: ${pkg}`));
          continue;
        }

        const pytestArgs = options?.args ? options.args.join(" ") : "";
        const cmd = `pytest ${pytestArgs}`;

        console.log(chalk.cyan(`🐍 Running Python tests for package: ${pkg}`));
        testPromises.push(
          new Promise((resolve) => {
            const proc = spawn("bash", ["-c", cmd], { cwd: testDir, stdio: "inherit" });
            proc.on("close", (code) => resolve(code ?? 1));
          })
        );
      }
    }

    // ----------- C++ Packages -----------
    if (fs.existsSync(srcDir)) {
      const cppPkgs = fs.readdirSync(srcDir).filter((pkg) => {
        const pkgPath = path.join(srcDir, pkg);
        return fs.existsSync(pkgPath) && fs.existsSync(path.join(pkgPath, "package.xml"));
      });

      for (const pkg of cppPkgs) {
        const pkgPath = path.join(srcDir, pkg);
        const testDir = path.join(pkgPath, "tests");

        if (!fs.existsSync(testDir)) {
          console.log(chalk.yellow(`⚠️ No tests/ directory for C++ package: ${pkg}`));
          continue;
        }

        console.log(chalk.cyan(`💻 Running C++ tests for package: ${pkg}`));
        const cmd = `colcon test --packages-select ${pkg}`;
        testPromises.push(
          new Promise((resolve) => {
            const proc = spawn("bash", ["-c", cmd], { cwd: workspaceRoot, stdio: "inherit" });
            proc.on("close", (code) => resolve(code ?? 1));
          })
        );
      }
    }

    if (testPromises.length === 0) {
      console.log(chalk.yellow("⚠️ No tests found in workspace."));
      process.exit(0);
    }

    // Wait for all tests
    const results = await Promise.all(testPromises);
    const failed = results.filter((code) => code !== 0).length;

    if (failed === 0) {
      console.log(chalk.green("✅ All tests passed!"));
      process.exit(0);
    } else {
      console.error(chalk.red(`❌ ${failed} test(s) failed.`));
      process.exit(1);
    }
  });
