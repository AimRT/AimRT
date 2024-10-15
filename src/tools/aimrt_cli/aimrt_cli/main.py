# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse


from aimrt_cli.command.gen_command import GenCommand
from aimrt_cli.command.trans_command import TransCommand

def main(description=None):
    if description is None:
        description = 'This is the application generation tool for AimRT.'

    parser = argparse.ArgumentParser(description=description)
    subparsers = parser.add_subparsers(dest='command', required=True)

    # 为 "gen" 命令创建子解析器
    gen_parser = subparsers.add_parser('gen', help='generate the configured project')
    gen_command = GenCommand()
    gen_command.add_arguments(gen_parser, "gen")

    # 为 "trans" 命令创建子解析器
    trans_parser = subparsers.add_parser('trans', help='translate the aimrtbag to ros2bag')
    trans_command = TransCommand()
    trans_command.add_arguments(trans_parser, "trans")

    args = parser.parse_args()
    if args.command == "gen":
        gen_command.main(args=args)
    elif args.command == "trans":
        trans_command.main(args=args)
    else:
        parser.print_help()
        return 0


if __name__ == '__main__':
    main()
