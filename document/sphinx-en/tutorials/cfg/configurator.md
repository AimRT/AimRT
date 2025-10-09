# aimrt.configurator

## Configuration Overview

The `aimrt.configurator` configuration item is used to determine certain details of the module configuration functionality. The detailed configuration items are explained below:

| Node            | Type          | Optional | Default Value | Purpose |
| ----            | ----          | ----     | ----          | ----    |
| temp_cfg_path   | string        | Optional | "./cfg/tmp"   | Storage path for generated temporary module configuration files |

Usage instructions for the `aimrt.configurator` node are as follows:
- `temp_cfg_path` is used to configure the storage path for generated temporary module configuration files.
  - A directory-style path needs to be configured. If the configured directory does not exist, the AimRT framework will create it. If creation fails, an exception will be thrown.
  - If the user has not configured this item for a certain module, but the AimRT framework configuration file contains a root node named after that module, the framework will generate a temporary configuration file for that module under the path configured by `temp_cfg_path`, copying the configuration for that module from the AimRT framework configuration file into this temporary configuration file.

## Usage Example

Below is a simple example:

```yaml
aimrt:
  configurator:
    temp_cfg_path: ./cfg/tmp
```
