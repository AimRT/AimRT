#!/bin/bash

DIR=${1:-"./"}

mkdir -p $DIR/_build
mkdir -p $DIR/_static
mkdir -p $DIR/_templates

if command -v sphinx-build >/dev/null 2>&1; then
  if [ ! -d "$DIR/_build/html" ]; then
    rm -rf "$DIR/_build/html"
  fi
  sphinx-build -b html $DIR $DIR/_build/html
fi