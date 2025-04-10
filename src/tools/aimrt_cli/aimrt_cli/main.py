# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse

from aimrt_cli.command.gen_command import GenCommand


def main(description=None):
    if description is None:
        description = 'This is the application generation tool for AimRT.'

    parser = argparse.ArgumentParser(description=description)
    subparsers = parser.add_subparsers(dest='command')

    # gen sub command
    gen_parser = subparsers.add_parser('gen', help='Generate the configured project')
    gen_parser.add_argument("-p", "--project_cfg", help="path of the configuration yaml file")
    gen_parser.add_argument("-o", "--output_dir", help="directory you want to output your files")

    args = parser.parse_args()

    if args.command == 'gen':
        generator = GenCommand()
        generator.main(args=args)
    else:
        parser.print_help()
        return 0


if __name__ == '__main__':
    main()
