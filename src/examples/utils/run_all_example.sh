#!/bin/bash

# -n <int> : number of iterations (default: 10)
# -s <string>  : fault test log output directory (default: "build/test_log")
# -t <string1> <string2> ...  : test tags, with logic is "AND" (default: "all")
# -i <string1> <string2> ... : ignore test tags, with logic is "OR" (default: None)
# -p : print test log to console (default: False), we don't suggest to use this option with n > 1 

export PYTHONPATH=$(dirname "$(pwd)"):$PYTHONPATH

python3 ./run_all_example.py -n 20 -s "./test_log" -t "python" 

