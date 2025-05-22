#!/bin/bash

# Check for argument
if [ -z "$1" ]; then
  echo "Usage: $0 [xxx]"
  exit 1
fi

DEMO_FOLDER="$1"

# Check if the folder exists
if [ ! -d "$DEMO_FOLDER" ]; then
  echo "Error: Folder '$DEMO_FOLDER' does not exist."
  exit 1
fi

cd "$DEMO_FOLDER" || exit 1

# Check and run build.sh
if [ -x "./build.sh" ]; then
  echo "Running build.sh in $DEMO_FOLDER..."
  ./build.sh
else
  echo "Error: build.sh not found or not executable."
  exit 1
fi

# Check and run run.sh
if [ -x "./run.sh" ]; then
  echo "Running run.sh in $DEMO_FOLDER..."
  ./run.sh
else
  echo "Error: run.sh not found or not executable."
  exit 1
fi
