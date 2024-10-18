# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.
import os
import platform


chmod_shell_name = [
    'linux_start_shell',
    'format_shell',
    'build_sh',
    'test_sh',
]


class TransBase:
    def __init__(self, output_dir: str):
        super(TransBase, self).__init__()
        self.output_dir_ = output_dir

    def trans(self):
        raise NotImplementedError()

    def output(self, template_info, name, output_stream):
        if isinstance(template_info['output_path'], str):
            output_path = template_info['output_path']
        else:
            output_path = template_info['output_path'].render(name=name)

        if platform.system().lower() == "linux":
            output_dir = self.output_dir_.replace('~', str(os.path.expanduser("~")))
            output_path = os.path.join(output_dir, output_path)
        else:
            output_path = os.path.join(self.output_dir_, output_path)

        if not os.path.exists(output_path):
            print("creating folder:", output_path)
            os.makedirs(output_path)

        if isinstance(template_info['output_file'], str):
            # output file name is not Template.
            output_file = template_info['output_file']
        else:
            output_file = template_info['output_file'].render(name=name)
        output_file_path = os.path.join(output_path, output_file)
        if os.path.exists(output_file_path):
            raise Exception("The file is already exist in" + output_file_path)
        else:
            with open(output_file_path, 'w') as f:
                print("creating file:", output_file_path)
                f.write(output_stream)

        if platform.system().lower() == "linux" and template_info['template_name'] in chmod_shell_name:
            os.chmod(output_file_path, 0o777)
