#!/bin/bash

clang_format_version=$(clang-format --version 2>&1)

if [[ $clang_format_version == *"version 15"* ]]; then
    echo "use clang-format-15"
else
    echo "error, can not find clang-format-15"
    exit 1
fi

# clang-format, version v15 is required
find ./src -regex '.*\.cc\|.*\.h\|.*\.proto' -and -not -regex '.*\.pb\.cc\|.*\.pb\.h' | xargs clang-format -i --style=file
echo "clang-format done"

# cmake-format, apt install cmake-format
{ find . -maxdepth 1 -name "CMakeLists.txt"; find ./src -name "CMakeLists.txt"; } | xargs cmake-format -c ./.cmake-format.py -i
{ find ./cmake -name "*.cmake"; find ./src -name "*.cmake"; } | xargs cmake-format -c ./.cmake-format.py -i
echo "cmake-format done"

# autopep8, apt install python3-autopep8
{ find . -maxdepth 1 -name "*.py" ! -name "*pb2.py" -print; find ./src -name "*.py" ! -name "*pb2.py" -print; } | xargs autopep8 -i --global-config ./.pycodestyle
echo "python format done"
