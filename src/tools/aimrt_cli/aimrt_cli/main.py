# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse


from aimrt_cli.command.gen_command import GenCommand


def main(description=None):
    if description is None:
        description = 'This is the application generation tool for AimRT.'

    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "generate",
        help="generate the configured project, the configuration yaml files is required.",
        type=str
    )
    # will support more console parameters later.
    gen_command = GenCommand()
    gen_command.add_arguments(parser, "gen")

    args = parser.parse_args()
    if args.generate == "gen":
        gen_command.main(args=args)
    else:
        parser.print_help()
        return 0


if __name__ == '__main__':
    main()
