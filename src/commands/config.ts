import { Command } from "commander";
import chalk from "chalk";
import path from "path";
import fs from "fs";

export const configCommand = new Command("config")
  .description("Manage workspace configuration")
  .argument("[key]", "Config key to view or edit")
  .argument("[value]", "Value to set for the key (optional)")
  .action((key?: string, value?: string) => {
    const configFile = path.join(process.cwd(), "config", "config.json");

    if (!fs.existsSync(configFile)) {
      fs.mkdirSync(path.dirname(configFile), { recursive: true });
      fs.writeFileSync(configFile, JSON.stringify({}, null, 2));
    }

    const config = JSON.parse(fs.readFileSync(configFile, "utf-8"));

    if (!key) {
      console.log(chalk.cyan("Current configuration:"), config);
      return;
    }

    if (value === undefined) {
      console.log(chalk.cyan(`${key}: ${config[key] ?? "undefined"}`));
    } else {
      config[key] = value;
      fs.writeFileSync(configFile, JSON.stringify(config, null, 2));
      console.log(chalk.green(`✅ Set ${key} = ${value}`));
    }
  });
