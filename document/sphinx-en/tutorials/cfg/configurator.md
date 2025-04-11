

# aimrt.configurator

## Configuration Overview

The `aimrt.configurator` configuration item determines detailed settings for module configuration functionality. The specific configuration items are described below:

| Node            | Type          | Optional | Default Value | Description |
| ----            | ----          | ----     | ----          | ----        |
| temp_cfg_path   | string        | Yes      | "./cfg/tmp"   | Storage path for generated temporary module configuration files |

## Usage Instructions

Usage instructions for the `aimrt.configurator` node:
- `temp_cfg_path` configures the storage path for generated temporary module configuration files.
  - Requires a directory-style path configuration. If the configured directory doesn't exist, the AimRT framework will attempt to create it. Creation failure will throw an exception.
  - If a user doesn't configure this item for a module, but the AimRT framework configuration file contains a root node named after that module, the framework will generate a temporary configuration file under the `temp_cfg_path` directory. This file will contain a copy of the module's configuration from the AimRT framework configuration file.

## Usage Example

Here's a simple example:
```yaml
aimrt:
  configurator:
    temp_cfg_path: ./cfg/tmp
```