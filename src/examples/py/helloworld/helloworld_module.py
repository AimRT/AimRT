# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import aimrt_py
import yaml
import datetime


class HelloWorldModule(aimrt_py.ModuleBase):
    def __init__(self):
        super().__init__()
        self.core = aimrt_py.CoreRef()
        self.logger = aimrt_py.LoggerRef()
        self.work_executor = aimrt_py.ExecutorRef()

    def Info(self):
        info = aimrt_py.ModuleInfo()
        info.name = "HelloWorldModule"
        return info

    def Initialize(self, core):
        assert (isinstance(core, aimrt_py.CoreRef))

        self.core = core
        self.logger = self.core.GetLogger()

        # log
        aimrt_py.info(self.logger, "Module initialize")

        try:
            # configure
            module_cfg_file_path = self.core.GetConfigurator().GetConfigFilePath()
            with open(module_cfg_file_path, 'r') as file:
                data = yaml.safe_load(file)
                aimrt_py.info(self.logger, str(data))

            # executor
            self.work_executor = self.core.GetExecutorManager().GetExecutor("work_thread_pool")
            if (not self.work_executor or not self.work_executor.SupportTimerSchedule()):
                aimrt_py.error(self.logger, "Get executor 'work_thread_pool' failed.")
                return False

        except Exception as e:
            aimrt_py.error(self.logger, "Initialize failed. {}".format(e))
            return False

        return True

    def Start(self):
        aimrt_py.info(self.logger, "Module start")

        try:
            # executor
            def test_task():
                aimrt_py.info(self.logger, "run test task.")

            self.work_executor.Execute(test_task)
            self.work_executor.ExecuteAfter(datetime.timedelta(seconds=1), test_task)
            self.work_executor.ExecuteAt(datetime.datetime.now() + datetime.timedelta(seconds=2), test_task)

        except Exception as e:
            aimrt_py.error(self.logger, "Initialize failed. {}".format(e))
            return False

        return True

    def Shutdown(self):
        aimrt_py.info(self.logger, "Module shutdown")
