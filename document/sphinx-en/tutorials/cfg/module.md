# aimrt.module

## Configuration Overview

The `aimrt.module` configuration item is mainly used to configure module loading information and special configurations for various other components. This is an optional configuration item, with detailed configuration descriptions as follows:

| Node                      | Type          | Optional | Default | Purpose |
| ----                      | ----          | ----     | ----    | ---- |
| pkgs                     | array         | Optional | []      | Configuration for Pkg dynamic libraries to be loaded |
| pkgs[i].path             | string        | Required | ""      | Path of the Pkg dynamic library to be loaded |
| pkgs[i].enable_modules   | string array  | Optional | []      | Names of modules to be loaded in this dynamic library, cannot be used simultaneously with disable_modules option |
| pkgs[i].disable_modules  | string array  | Optional | []      | Names of modules to be disabled in this dynamic library, cannot be used simultaneously with enable_modules option |
| modules                  | array         | Optional | []      | Detailed module configuration |
| modules[i].name          | string        | Required | ""      | Module name |
| modules[i].enable        | bool          | Optional | True    | Whether to enable |
| modules[i].log_lvl       | string        | Optional | ${aimrt.log.default_module_lvl} | Module log level |
| modules[i].cfg_file_path | string        | Optional | ""      | Custom module configuration file path |

When using, please note that under the `aimrt.module` node:
- `pkg` is an array used for Pkg dynamic libraries to be loaded.
  - `pkgs[i].path` is used to configure the path of the Pkg dynamic library to be loaded. Duplicate Pkg paths are not allowed. If the Pkg file does not exist, the AimRT process will throw an exception.
  - `pkgs[i].enable_modules` and `pkgs[i].disable_modules` are used to configure modules to be loaded/disabled, with the following logic:
    - If neither `enable_modules` nor `disable_modules` is configured, all modules will be loaded;
    - If only `enable_modules` is configured, all modules in `enable_modules` will be loaded;
    - If only `disable_modules` is configured, all modules except those in `disable_modules` will be loaded;
    - If both `enable_modules` and `disable_modules` are configured, all modules in `enable_modules` will be loaded, the `disable_modules` option will be ignored, and a warning will be issued during initialization;
- `modules` is an array used to configure individual modules.
  - `modules[i].name` represents the module name. Duplicate module names are not allowed.
  - `modules[i].log_lvl` is used to configure the module log level.
    - If this item is not configured, the default value is the value configured in the `aimrt.log.default_module_lvl` node.
    - For configurable log levels, please refer to the [aimrt.log](./log.md) documentation.
  - `modules[i].cfg_file_path` is used to configure the custom module configuration file path. This configuration relates to the result returned by the `config_file_path` method of the `configurator` component in the Module interface, with the following rules:
    - If the user configures this item, the `config_file_path` method of the `configurator` component will return the string content configured here;
    - If the user does not configure this item and there is no root node named after this module in the AimRT framework configuration file, the `config_file_path` method of the `configurator` component will return an empty string.
    - If the user does not configure this item but there is a root node named after this module in the AimRT framework configuration file, the `config_file_path` method of the `configurator` component will return a temporary configuration file path. This temporary configuration file will contain the content under the module name node in the AimRT framework configuration file.

## Usage Example

Here is a simple example:

```yaml
aimrt:
  module:
    pkgs:
      - path: /path/to/libxxx_pkg.so
        enable_modules: [FooModule, BarModule]
    modules:
      - name: FooModule
        enable: True
        log_lvl: INFO
        cfg_file_path: /path/to/foo_module_cfg.yaml
      - name: BarModule
        enable: True
        log_lvl: WARN

BarModule:
  foo_key: foo_val
  bar_key: bar_val
```
