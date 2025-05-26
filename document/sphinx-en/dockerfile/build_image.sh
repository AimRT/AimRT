#!/bin/bash

html_dir=../_build/html
html_pkg_dir=./html.tar.gz

if [ -f "$html_pkg_dir" ]; then
    rm "$html_pkg_dir"
fi

tar -zcvf ${html_pkg_dir} -C ${html_dir} .

docker build -t aimrt-doc-web-en .
