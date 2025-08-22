import { Command } from "commander";
import chalk from "chalk";
import path from "path";
import fs from "fs";

export const packageCommand = new Command("package")
  .description("Scaffold a new Genesys package")
  .argument("<name>", "Package name")
  .option("-l, --language <lang>", "Language: cpp or python", "cpp")
  .action((name: string, options: { language?: string }) => {
    const workspaceRoot = process.cwd();
    const pkgDir = path.join(workspaceRoot, options.language === "python" ? "python" : "src", name);

    if (fs.existsSync(pkgDir)) {
      console.error(chalk.red(`❌ Package ${name} already exists.`));
      process.exit(1);
    }

    fs.mkdirSync(pkgDir, { recursive: true });

    console.log(chalk.green(`✅ Created ${options.language} package: ${name}`));
    // Could add basic structure scaffolding: src/scripts, package.xml/setup.py
  });
