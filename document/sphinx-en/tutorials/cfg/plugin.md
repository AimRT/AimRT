# aimrt.plugin

## Configuration Overview

The `aimrt.plugin` configuration item is used to configure plugins. The detailed configuration items are described below:

| Node                    | Type          | Optional | Default | Purpose |
| ----                    | ----          | ----     | ----    | ----    |
| plugins                 | array         | Optional | []      | Configuration for each plugin |
| plugins[i].name         | string        | Required | ""      | Plugin name |
| plugins[i].path         | string        | Optional | ""      | Plugin path. Not required for hard-coded registered plugins |
| plugins[i].options      | map           | Optional | -       | Initialization configuration passed to the plugin, specific content is described in each plugin chapter |

Notes for using `aimrt.plugin`:
- `plugins` is an array used to configure each plugin.
  - `plugins[i].name` is used to configure the plugin name. Duplicate plugin names are not allowed.
  - If `plugins[i].path` is configured, the AimRT framework will load the corresponding plugin dynamic library file from that path. If the user registers the plugin via hard-coding in App mode, this item does not need to be configured.
  - `plugins[i].options` is the initialization parameter passed by AimRT to the plugin. The format of this configuration is defined by each plugin; please refer to the documentation of the corresponding plugin.

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
