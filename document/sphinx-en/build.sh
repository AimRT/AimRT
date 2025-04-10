#!/bin/bash

DIR=${1:-"./"}
SINGLE_VERSION=false

# Parse arguments
for arg in "$@"
do
    if [ "$arg" == "--single-version" ]; then
        SINGLE_VERSION=true
    elif [ "$arg" != "--single-version" ] && [ -z "$DIR" ]; then
        DIR=$arg
    fi
done

mkdir -p $DIR/_build
mkdir -p $DIR/_static
mkdir -p $DIR/_templates

if command -v sphinx-build >/dev/null 2>&1; then
  if [ ! -d "$DIR/_build/html" ]; then
    rm -rf "$DIR/_build/html"
  fi

  mkdir -p $DIR/_build/html
  cp $DIR/_static/html/index.html $DIR/_build/html/index.html

  if [ "$SINGLE_VERSION" = true ] || ! { [ -d ".git" ] || git rev-parse --git-dir > /dev/null 2>&1; }; then
    # Build with sphinx-build for single version or non-git repos
    [ "$SINGLE_VERSION" = true ] && echo "Building single version documentation..." || echo "Warning: Not a Git repository. Falling back to sphinx-build..."
    sphinx-autobuild -b html $DIR $DIR/_build/html --host 0.0.0.0
  else
    # Build with sphinx-multiversion for git repos (multi-version)
    echo "Using sphinx-multiversion to build multi-version documentation..."
    sphinx-multiversion $DIR $DIR/_build/html
  fi
fi