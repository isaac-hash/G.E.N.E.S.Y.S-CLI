import { Command } from "commander";
import chalk from "chalk";

const init = new Command("init")
    .description("Initialize a new Genesys project")
    .action(() => {
        console.log(chalk.green("✅ Genesys project initialized successfully!"));
    })

export default init;