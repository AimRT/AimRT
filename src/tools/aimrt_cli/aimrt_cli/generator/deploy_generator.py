# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from dataclasses import dataclass
from jinja2 import Template
from warnings import warn as print_warnings

from aimrt_cli.generator import GeneratorBase

deploy_template_infos = [
    {
        'template_name': 'linux_cfg_yaml',
        'template_package': 'templates.install',
        'template_file': 'linux_cfg_template.yaml.jinja2',
        'output_path': "./src/install/linux/bin/cfg",
        'output_file': Template("{{ name }}_cfg.yaml"),
    },
    {
        'template_name': 'linux_start_shell',
        'template_package': 'templates.install',
        'template_file': 'start_template.sh.jinja2',
        'output_path': "./src/install/linux/bin",
        'output_file': Template("start_{{ name }}.sh"),
    },
    {
        'template_name': 'win_cfg_yaml',
        'template_package': 'templates.install',
        'template_file': 'win_cfg_template.yaml.jinja2',
        'output_path': "./src/install/win/bin/cfg",
        'output_file': Template("{{ name }}_cfg.yaml"),
    },
    {
        'template_name': 'win_start_shell',
        'template_package': 'templates.install',
        'template_file': 'start_template.bat.jinja2',
        'output_path': "./src/install/win/bin",
        'output_file': Template("start_{{ name }}.bat"),
    },
]


@dataclass
class DeployedPackages:
    name: str
    deployed_modules: list
    disable_modules: list


@dataclass
class DeployExpandInfo:
    name: str
    pkgs: list


def check_pkg_build_mode(pkg, deploy_build_modes, pkg_build_modes):
    if deploy_build_modes is None:
        return

    for pkg_build_mode in pkg_build_modes[pkg.pkg_name]:
        if pkg_build_mode is not None and pkg_build_mode not in deploy_build_modes:
            raise Exception("Your pkg build mode: " + pkg_build_mode + " is not fit with deployment. Please Check it.")


class DeployGenerator(GeneratorBase):
    def __init__(self, *, deploy_infos, project_name, output_dir=None):
        super().__init__(output_dir)
        self.deploy_infos_ = deploy_infos
        self.project_name_ = project_name
        self.expand_infos_ = []

    def parse(self, pkg_cfgs=None, pkg_build_modes=None):
        for deploy_info in self.deploy_infos_:
            deploy_mode_name = deploy_info['name']
            deploy_build_modes = None
            if 'build_mode_tag' in deploy_info.keys():
                deploy_build_modes = deploy_info['build_mode_tag']
            # will not generate the deploy_mode without deploy_ins
            if 'deploy_ins' not in deploy_info.keys() or deploy_info['deploy_ins'] is None:
                continue

            for deploy_ins in deploy_info['deploy_ins']:
                deploy_ins_name = deploy_ins['name']
                # will not generate the deployment without pkgs configured.
                if 'pkgs' not in deploy_ins.keys() or deploy_ins['pkgs'] is None:
                    continue

                deploy_name = deploy_mode_name + '_' + deploy_ins_name
                deploy_pkgs = []
                already_used_modules = []
                pkg_names = []
                for pkg_cfg in pkg_cfgs:
                    pkg_names.append(pkg_cfg.pkg_name)

                for pkg in deploy_ins['pkgs']:
                    pkg_name = pkg['name']
                    relate_modules = []
                    pkg_disable_modules = []
                    if 'options' in pkg.keys() and 'disable_modules' in pkg['options'].keys():
                        pkg_disable_modules = pkg['options']['disable_modules']

                    if pkg_name not in pkg_names:
                        print_warnings("Pkg name: " + pkg_name +
                                       " is not the customized pkg in configuration file, Please make sure it has "
                                       "been already fetched!")
                    else:
                        for pkg_cfg in pkg_cfgs:
                            if pkg_name == pkg_cfg.pkg_name:
                                check_pkg_build_mode(pkg_cfg, deploy_build_modes, pkg_build_modes)
                                pkg_modules = pkg_cfg.local_modules + pkg_cfg.remote_modules
                                for related_module in pkg_modules:
                                    if related_module.class_name not in pkg_disable_modules:
                                        if related_module.name not in already_used_modules:
                                            relate_modules.append(related_module.class_name)
                                            already_used_modules.append(related_module.name)
                                        else:
                                            raise Exception(related_module.name + "is duplicated in more than one "
                                                                                  "pkgs, Please disable it necessary.")
                                break

                    deploy_pkgs.append(DeployedPackages(name=pkg_name, deployed_modules=relate_modules,
                                                        disable_modules=pkg_disable_modules))
                self.expand_infos_.append(DeployExpandInfo(name=deploy_name, pkgs=deploy_pkgs))

    def generate(self):
        print("Deploy Files Generating...")
        for expand_info in self.expand_infos_:
            for deploy_template_info in deploy_template_infos:
                template = self.get_template_environment(deploy_template_info)

                output_stream = template.render(deploy=expand_info)

                self.output(deploy_template_info, expand_info.name, output_stream)
