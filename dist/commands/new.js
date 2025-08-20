import { Command } from "commander";
import chalk from "chalk";
const newProject = new Command("new")
    .description("Create a new Genesys project")
    .argument("<name>", "Name of the new project")
    .action((name) => {
    console.log(chalk.blue(`Creating a new Genesys project: ${name}...`));
    // Here you would add the logic to create a new project
    // console.log(chalk.green(`✅ Project ${name} created successfully!`));
});
export default newProject;
