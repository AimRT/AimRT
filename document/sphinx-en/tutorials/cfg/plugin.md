

# aimrt.plugin

## Configuration Overview

The `aimrt.plugin` configuration item is used to configure plugins. Detailed configuration item descriptions are as follows:

| Node                   | Type          | Optional | Default | Purpose |
| ----                   | ----          | ----     | ----    | ----    |
| plugins                | array         | Optional | []      | Configurations for each plugin |
| plugins[i].name        | string        | Required | ""      | Plugin name |
| plugins[i].path         | string        | Optional | ""      | Plugin path. Not required for hard-coded registered plugins |
| plugins[i].options     | map           | Optional | -       | Initialization parameters passed to the plugin, specific content introduced in each plugin's documentation |
Important considerations for `aimrt.plugin`:
- `plugins` is an array used to configure various plugins.
  - `plugins[i].name` configures the plugin name. Duplicate plugin names are prohibited.
  - When `plugins[i].path` is configured, the AimRT framework will load the corresponding plugin dynamic library from this path. Not required if plugins are hard-coded using App mode registration.
  - `plugins[i].options` contains initialization parameters passed by AimRT to plugins. The configuration format is defined by each plugin, please refer to corresponding plugin documentation.

## Usage Example

Here is a simple configuration example:
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