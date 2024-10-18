# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.


class CommandBase:
    def __init__(self):
        super(CommandBase, self).__init__()

    def add_arguments(self, parser, cmd_name):
        pass

    def main(self, *, args=None):
        raise NotImplementedError()
