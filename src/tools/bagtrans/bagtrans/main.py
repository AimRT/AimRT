# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse

from bagtrans.command.trans_command import TransCommand

def main(description=None):
    if description is None:
        description = 'This is the bagtrans tool.'

    parser = argparse.ArgumentParser(description=description)
    subparsers = parser.add_subparsers(dest='command', required=True)

    # Create a subparser for the "trans" command
    trans_parser = subparsers.add_parser('trans', help='translate the aimrtbag to ros2bag')
    trans_command = TransCommand()
    trans_command.add_arguments(trans_parser, "trans")

    args = parser.parse_args()
    if args.command == "trans":
        trans_command.main(args=args)
    else:
        parser.print_help()
        return 0


if __name__ == '__main__':
    main()
