# aimrt.configurator

## Configuration Overview

The `aimrt.configurator` configuration item is used to determine some details of the module configuration functionality. The detailed configuration items are described below:

| Node            | Type          | Optional | Default Value | Purpose |
| ----            | ----          | ----     | ----          | ----    |
| temp_cfg_path   | string        | Optional | "./cfg/tmp"   | Storage path for generated temporary module configuration files |

Usage instructions for the `aimrt.configurator` node:
- `temp_cfg_path` is used to configure the storage path for generated temporary module configuration files.
  - Requires a directory-style path configuration. If the configured directory does not exist, the AimRT framework will create one. If creation fails, an exception will be thrown.
  - If this item is not configured for a module by the user, but a root node named after the module exists in the AimRT framework configuration file, the framework will generate a temporary configuration file for that module under the path configured by `temp_cfg_path`, copying the module's configuration from the AimRT framework configuration file to this temporary configuration file.

## Usage Example

Here is a simple example:
```yaml
aimrt:
  configurator:
    temp_cfg_path: ./cfg/tmp
```