#!/usr/bin/env node
import { Command } from "commander";
import process from "process";
import makeNode from "./commands/make-node.js";
import newProject from "./commands/new.js";
import { launchCommand } from "./commands/launch.js";
import { runCommand } from "./commands/run.js";
import { buildCommand } from "./commands/build.js";
import { simCommand } from "./commands/sim.js";
import { testCommand } from "./commands/test.js";
import { configCommand } from "./commands/config.js";
import { recordCommand } from "./commands/record.js";
import { replayCommand } from "./commands/replay.js";
import { packageCommand } from "./commands/package.js";

const program = new Command();

program
  .name("genesys-cli")
  .description("G.E.N.E.S.Y.S CLI")
  .version("1.0.0");

// Basic commands
program
  .command("make:node <name>")
  .description("Generate a new Genesys node")
  .action(makeNode);

program
  .command("new <name>")
  .description("Create a new Genesys project")
  .action(newProject);

// Add modular commands
program.addCommand(launchCommand);
program.addCommand(runCommand);
program.addCommand(buildCommand);
program.addCommand(simCommand);
program.addCommand(testCommand);
program.addCommand(configCommand);
program.addCommand(recordCommand);
program.addCommand(replayCommand);
program.addCommand(packageCommand);

// Parse CLI arguments
program.parse(process.argv);
