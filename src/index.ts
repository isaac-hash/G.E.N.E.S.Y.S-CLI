#!/usr/bin/env node
console.log("GENESYS CLI is working!");

import { Command } from 'commander'
import makeNode from './commands/make-node.js'
import process from 'process';
import newProject from './commands/new.js';

const program = new Command();

program
    .name('genesys-cli')
    .description('G.E.N.E.S.Y.S CLI')
    .version('1.0.0')

program
    .command('make:node <name>')
    .description('Generate a new Genesys node')
    .action(makeNode);

program
    .command('new <name>')
    .description('Create a new Genesys project')
    .action(newProject);

program.parse(process.argv);
