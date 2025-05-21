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
    sphinx-build -b html $DIR $DIR/_build/html
  else
    # Build with sphinx-multiversion for git repos (multi-version)
    echo "Using sphinx-multiversion to build multi-version documentation..."
    sphinx-multiversion $DIR $DIR/_build/html

    # Get current version from conf.py
    CURRENT_VERSION=$(python3 -c "import sys; sys.path.append('$DIR'); from conf import html_context; print(html_context['current_version'])")
    if [ -n "$CURRENT_VERSION" ]; then
      echo "Creating latest symlink to $CURRENT_VERSION..."
      cd $DIR/_build/html
      rm -f latest
      ln -sf $CURRENT_VERSION latest
      cd - > /dev/null
    else
      echo "Warning: Could not find current_version in conf.py"
      echo "Available versions:"
      ls -l $DIR/_build/html
    fi
  fi
fi