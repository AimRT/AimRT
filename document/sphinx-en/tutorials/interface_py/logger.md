# Logger

## Related Links

Reference example:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}


## Interface Overview

The `aimrt_py` package provides the following interfaces for logging:
- `trace(logger, msg)`
- `debug(logger, msg)`
- `info(logger, msg)`
- `warn(logger, msg)`
- `error(logger, msg)`
- `fatal(logger, msg)`

The first parameter of these interfaces is a `LoggerRef` handle type, and the second parameter is the log message string. Modules can obtain the `LoggerRef` handle by calling the `GetLogger()` interface of the `CoreRef` handle.


## Usage Example

Module developers can directly refer to the following example to use the log handle assigned to the module for logging:
```python
import aimrt_py

class HelloWorldModule(aimrt_py.ModuleBase):
    def Initialize(self, core):
        # Get log handle
        logger = core.GetLogger()

        # Print log
        aimrt_py.trace(logger, "This is a test trace log")
        aimrt_py.debug(logger, "This is a test debug log")
        aimrt_py.info(logger, "This is a test info log")
        aimrt_py.warn(logger, "This is a test warn log")
        aimrt_py.error(logger, "This is a test error log")
        aimrt_py.fatal(logger, "This is a test fatal log")

        return True
```