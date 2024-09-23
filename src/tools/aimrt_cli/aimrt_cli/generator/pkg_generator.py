# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from dataclasses import dataclass
from jinja2 import Template
from warnings import warn as print_warnings

from aimrt_cli.generator import GeneratorBase

pkg_template_infos = [
    {
        'template_name': 'pkg_main',
        'template_package': 'templates.pkg',
        'template_file': 'pkg_main.cc.jinja2',
        'output_path': Template("./src/pkg/{{ name }}"),
        'output_file': 'pkg_main.cc',
    },
    {
        'template_name': 'pkg_cmakelists',
        'template_package': 'templates.pkg',
        'template_file': 'pkg_CMakeLists.txt.jinja2',
        'output_path': Template("./src/pkg/{{ name }}"),
        'output_file': 'CMakeLists.txt',
    },
]


@dataclass
class PkgRelatedModule:
    name: str
    class_name: str
    namespace: str = None


@dataclass
class PkgExpandInfo:
    pkg_name: str
    local_modules: list
    remote_modules: list
    options: dict = None


class PkgGenerator(GeneratorBase):
    def __init__(self, pkg_infos, project_name, output_dir=None):
        super().__init__(output_dir)
        self.pkg_infos_ = pkg_infos
        self.project_name_ = project_name
        self.expand_infos_ = []

    def parse(self, configured_modules=list):
        if not configured_modules:
            raise Exception("Can not find any modules, you should at least configure one!")

        build_mode_tags = {}
        for pkg_info in self.pkg_infos_:
            pkg_name = pkg_info['name']
            pkg_modules = pkg_info['modules']
            # options = pkg_info['options']
            if 'build_mode_tag' in pkg_info.keys() and pkg_info['build_mode_tag'] is not None:
                build_mode_tags[pkg_name] = pkg_info['build_mode_tag']
            else:
                build_mode_tags[pkg_name] = []

            local_modules = []
            remote_modules = []
            used_modules = []
            for pkg_module in pkg_modules:
                module_name = pkg_module['name']
                if 'namespace' in pkg_module.keys():
                    module_namespace = pkg_module['namespace']
                else:
                    module_namespace = "local"
                if module_name not in configured_modules:
                    print_warnings("Module name: " + module_name + " is not configured in configuration file, Please "
                                                                   "make sure it has already installed.", UserWarning)

                split_names = module_name.split('_')
                class_name = ''
                for name in split_names:
                    class_name += name.capitalize()

                if module_name not in used_modules:
                    used_modules.append(module_name)
                else:
                    raise Exception(module_name + " is duplicated in pkg modules. Please Check it!!!")

                if module_namespace == 'local':
                    local_modules.append(PkgRelatedModule(module_name, class_name))
                else:
                    remote_modules.append(PkgRelatedModule(module_name, class_name, module_namespace))

            self.expand_infos_.append(PkgExpandInfo(pkg_name, local_modules, remote_modules))

        return build_mode_tags, self.expand_infos_

    def generate(self):
        print("Package Files Generating...")
        for expand_info in self.expand_infos_:
            self.generate_templates(expand_info)

    def generate_templates(self, expand_info):
        for pkg_template_info in pkg_template_infos:
            template = self.get_template_environment(pkg_template_info)

            output_stream = template.render(
                project_name=self.project_name_,
                pkg=expand_info,
            )

            self.output(pkg_template_info, expand_info.pkg_name, output_stream)
