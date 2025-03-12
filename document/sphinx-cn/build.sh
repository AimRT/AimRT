#!/bin/bash

DIR=${1:-"./"}

mkdir -p $DIR/_build
mkdir -p $DIR/_static
mkdir -p $DIR/_templates

if command -v sphinx-build >/dev/null 2>&1; then
  if [ ! -d "$DIR/_build/html" ]; then
    rm -rf "$DIR/_build/html"
  fi

  cp $DIR/_static/html/index.html $DIR/_build/html/index.html

  if [ -d ".git" ] || git rev-parse --git-dir > /dev/null 2>&1; then
    echo "Using sphinx-multiversion to build multi-version documentation..."

    sphinx-multiversion $DIR $DIR/_build/html
  else
    echo "Warning: Not a Git repository. Falling back to sphinx-build..."
    sphinx-build -b html $DIR $DIR/_build/html
  fi
fi