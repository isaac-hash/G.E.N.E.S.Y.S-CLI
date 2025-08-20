#!/usr/bin/env node
console.log("GENESYS CLI is working!");
import { Command } from 'commander';
import process from 'process';
import newProject from './commands/new.js';
const program = new Command();
program
    .name('genesys-cli')
    .description('G.E.N.E.S.Y.S CLI')
    .version('1.0.0');
program.addCommand(newProject);
// program.addCommand(makeNodeCommand);
program.parse(process.argv);
