#!/bin/bash
PACKAGE_DIR=$(rospack find engix_robot)
cd $PACKAGE_DIR/nodejs
node main.js $@
