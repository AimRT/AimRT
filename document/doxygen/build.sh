#!/bin/bash

DIR=${1:-"./"}

if command -v doxygen >/dev/null 2>&1; then
  if [ -d "$DIR/html" ]; then
    rm -rf "$DIR/html"
  fi
  doxygen "$DIR/Doxyfile" > "$DIR/doxygen.log" 2>&1
fi
