# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import os
import yaml
import subprocess

from aimrt_cli.generator import GeneratorBase
from aimrt_cli.generator.workspace_generator import WorkspaceGenerator
from aimrt_cli.generator.module_generator import ModuleGenerator
from aimrt_cli.generator.pkg_generator import PkgGenerator
from aimrt_cli.generator.protocol_generator import ProtoGenerator
from aimrt_cli.generator.deploy_generator import DeployGenerator


def check_format(cfg):
    required_tags = ['base_info']
    for required_tag in required_tags:
        if required_tag not in cfg.keys():
            raise Exception(
                "yaml configuration file is illegal, You should add required tag: " + required_tag)


def check_duplicated_modules(module_list):
    new_list = []
    for module in module_list:
        if module not in new_list:
            new_list.append(module)
        else:
            raise Exception("Module " + module +
                            " is duplicated, please change its name.")


class ProjectGenerator(GeneratorBase):
    def __init__(self, *, cfg_path, output_dir=None):
        super().__init__()
        self.path_ = cfg_path
        self.output_dir_ = output_dir

    def parse(self):
        pass

    def generate(self):
        root_path = os.getcwd().replace('\\', '/')
        yaml_path = os.path.join(root_path, self.path_)

        with open(yaml_path, 'r', encoding='utf-8') as cfg_file:
            root_cfg = yaml.load(cfg_file, Loader=yaml.FullLoader)
            check_format(root_cfg)

            if root_cfg['base_info'] is None:
                raise Exception("yaml configuration file is illegal, it's base_info is NULL, can not create AimRT "
                                "Project!")
            else:
                if 'depends_std_modules' in root_cfg.keys():
                    depends_std_modules = root_cfg['depends_std_modules']
                else:
                    depends_std_modules = {}
                workspace_generator = WorkspaceGenerator(
                    base_info=root_cfg['base_info'],
                    deploy_modes=root_cfg['deploy_modes'],
                    depends_std_modules=depends_std_modules,
                    output_dir=self.output_dir_,
                )
            project_name = workspace_generator.get_project_name()
            output_dir = workspace_generator.get_output_dir()

            if root_cfg['modules'] is None:
                raise Exception(
                    "yaml configuration file is illegal, you should add at least one module!")
            else:
                module_generator = ModuleGenerator(module_infos=root_cfg['modules'],
                                                   project_name=project_name,
                                                   output_dir=output_dir)

            if root_cfg['pkgs'] is None:
                raise Exception(
                    "yaml configuration file is illegal, you should add at least one pkg!")
            else:
                pkg_generator = PkgGenerator(pkg_infos=root_cfg['pkgs'],
                                             project_name=project_name,
                                             output_dir=output_dir)

            if root_cfg['deploy_modes'] is None:
                raise Exception(
                    "yaml configuration file is illegal, you should configure at least one deploy mode!")
            else:
                deploy_generator = DeployGenerator(deploy_infos=root_cfg['deploy_modes'],
                                                   project_name=project_name,
                                                   output_dir=output_dir)

            proto_build_modes = {}
            proto_generator = None
            if 'protocols' in root_cfg.keys() and root_cfg['protocols'] is not None:
                proto_generator = ProtoGenerator(proto_infos=root_cfg['protocols'],
                                                 project_name=project_name,
                                                 output_dir=output_dir)
                proto_build_modes = proto_generator.parse()

            module_build_modes, customize_module_list = module_generator.parse()
            customize_module_list.extend(
                workspace_generator.get_std_module_depends())
            check_duplicated_modules(customize_module_list)

            pkg_build_modes, pkgs_relationships = pkg_generator.parse(
                customize_module_list)

            deploy_generator.parse(pkgs_relationships, pkg_build_modes)

            build_mode = {
                'protocol': proto_build_modes,
                'module': module_build_modes,
                'pkg': pkg_build_modes
            }
            workspace_generator.parse(build_modes=build_mode)

            # generate codes, the generation sequence can not be changed.
            if proto_generator is not None:
                proto_generator.generate()
            module_generator.generate()
            pkg_generator.generate()
            workspace_generator.generate()
            deploy_generator.generate()

            # run the format shell
            os.chdir(output_dir)
            if os.path.exists('format.sh'):
                subprocess.run('sh format.sh', shell=True, check=True)
