# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from aimrt_cli.command import CommandBase
from aimrt_cli.generator.project_generator import ProjectGenerator
from aimrt_cli.trans.rosbag_trans import AimrtbagToRos2


class GenCommand(CommandBase):
    def __init__(self):
        super().__init__()
        self.parser_ = None

    def add_arguments(self, parser, cmd_name):
        if cmd_name == "gen":
            self.parser_ = parser
            parser.add_argument("-p", "--project_cfg", help="path of the configuration yaml file.")
            parser.add_argument("-o", "--output_dir", help="directory you want to output your files.")

    def main(self, *, args=None):
        if args is None:
            self.parser_.print_help()
            return 0

        generator = ProjectGenerator(cfg_path=args.project_cfg, output_dir=args.output_dir)
        generator.generate()

        trans = AimrtbagToRos2(args.src_dir, args.output_dir)
        trans.trans()
