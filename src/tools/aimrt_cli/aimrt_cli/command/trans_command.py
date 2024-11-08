# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from aimrt_cli.command import CommandBase
from aimrt_cli.trans.rosbag_trans import AimrtbagToRos2


class TransCommand(CommandBase):
    def __init__(self):
        super().__init__()
        self.parser_ = None

    def add_arguments(self, parser, cmd_name):
        if cmd_name == "trans":
            self.parser_ = parser
            parser.add_argument(
                "-s",
                "--src_dir",
                nargs='+',
                help="aimrtbag source directories (support multiple directories).")
            parser.add_argument("-o", "--output_dir", help="directory you want to output your files.")

    def main(self, *, args=None):
        if args is None:
            self.parser_.print_help()
            return 0
        trans = AimrtbagToRos2(args.src_dir, args.output_dir)
        trans.trans()
