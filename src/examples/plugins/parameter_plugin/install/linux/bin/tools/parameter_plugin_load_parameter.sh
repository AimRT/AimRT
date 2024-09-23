#!/bin/bash

data='{
	"module_parameter_map": {
		"ParameterModule": {
			"value": {
				"key-a": "val-111",
				"key-b": "val-222",
				"key-c": "val-333",
				"key-d": "val-444",
				"key-e": "val-555",
				"key-f": "val-666",
				"key-g": "val-777"
			}
		}
	}
}'


curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.parameter_plugin.ParameterService/Load' \
    -d "$data"
