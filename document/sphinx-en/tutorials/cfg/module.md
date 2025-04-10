

# aimrt.module

## Configuration Overview

The `aimrt.module` configuration item is primarily used to configure module loading information and special configurations for various components. This is an optional configuration item with detailed specifications as follows:

| Node                     | Type          | Optional | Default | Description |
| ----                     | ----          | ----     | ----    | ----        |
| pkgs                     | array         | Yes      | []      | Configuration for loading Pkg dynamic libraries |
| pkgs[i].path             | string        | Required | ""      | Path to the dynamic library of the Pkg to load |
| pkgs[i].enable_modules   | string array  | Yes      | []      | Module names to load from this dynamic library. Cannot be used with disable_modules |
| pkgs[i].disable_modules  | string array  | Yes      | []      | Module names to exclude from this dynamic library. Cannot be used with enable_modules |
| modules                  | array         | Yes      | []      | Detailed module configurations |
| modules[i].name          | string        | Required | ""      | Module name |
| modules[i].enable        | bool          | Yes      | True    | Whether to enable the module |
| modules[i].log_lvl       | string        | Yes      | ${aimrt.log.default_module_lvl} | Module log level |
| modules[i].cfg_file_path | string        | Yes      | ""      | Custom module configuration file path |

Usage notes for the `aimrt.module` node:
- `pkgs` is an array for loading Pkg dynamic libraries.
  - `pkgs[i].path` configures the path to load Pkg dynamic libraries. Duplicate Pkg paths are not allowed. AimRT will throw exceptions if Pkg files are missing.
  - `pkgs[i].enable_modules` and `pkgs[i].disable_modules` control module loading/exclusion with following logic:
    - Load all modules if neither enable_modules nor disable_modules are configured;
    - If only enable_modules is configured, load all modules listed in enable_modules;
    - If only disable_modules is configured, load all modules except those in disable_modules;
    - If both are configured, enable_modules takes priority (disable_modules is ignored) with initialization warnings;
- `modules` array configures individual modules.
  - `modules[i].name` specifies module names. Duplicate names are prohibited.
  - `modules[i].log_lvl` sets module log level.
    - Defaults to `${aimrt.log.default_module_lvl}` if unconfigured.
    - For valid log levels, refer to [aimrt.log](./log.md) documentation.
  - `modules[i].cfg_file_path` configures custom module configuration file path, affecting the `config_file_path` method in Module interface's `configurator` component:
    - Returns configured string when set;
    - Returns empty string if unconfigured and no matching root node exists in AimRT config;
    - Returns temporary config file path containing module node contents from AimRT config when unconfigured but matching root node exists.

## Usage Example

Below is a simple example:
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