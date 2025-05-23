# aimrt.plugin

## Configuration Overview

The `aimrt.plugin` configuration item is used to configure plugins. The detailed configuration items are described below:

| Node                    | Type          | Optional | Default Value | Purpose |
| ----                    | ----          | ----     | ----          | ----    |
| plugins                 | array         | Optional | []            | Configuration for each plugin |
| plugins[i].name         | string        | Required | ""            | Plugin name |
| plugins[i].path         | string        | Optional | ""            | Plugin path. Not required for hardcoded registered plugins |
| plugins[i].options      | map           | Optional | -             | Initialization configuration passed to the plugin, specific content is introduced in each plugin's documentation |

Notes for using `aimrt.plugin`:
- `plugins` is an array used to configure each plugin.
  - `plugins[i].name` is used to configure the plugin name. Duplicate plugin names are not allowed.
  - If `plugins[i].path` is configured, the AimRT framework will load the corresponding plugin dynamic library file from this path. If users hardcode plugin registration in App mode, this item does not need to be configured.
  - `plugins[i].options` are initialization parameters passed by AimRT to the plugin. The configuration format is defined by each plugin, please refer to the corresponding plugin's documentation.

## Usage Example

Here is a simple example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: xxx_plugin
        path: ./libxxx.so
        options:
          xxx_key: xxx_val
          yyy_key: yyy_val
```