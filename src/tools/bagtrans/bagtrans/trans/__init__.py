# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.


class TransBase:
    def __init__(self, output_dir: str):
        super(TransBase, self).__init__()
        self.output_dir_ = output_dir

    def trans(self):
        raise NotImplementedError()
