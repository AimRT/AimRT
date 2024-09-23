# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from dataclasses import dataclass
from jinja2 import Template

from aimrt_cli.generator import GeneratorBase

module_template_infos = [
    {
        'template_name': 'cpp_src',
        'template_package': 'templates.module',
        'template_file': 'helloworld_template.cc.jinja2',
        'output_path': Template("./src/module/{{ name }}"),
        'output_file': Template("{{ name }}.cc"),
    },
    {
        'template_name': 'cpp_header',
        'template_package': 'templates.module',
        'template_file': 'helloworld_template.h.jinja2',
        'output_path': Template("./src/module/{{ name }}"),
        'output_file': Template("{{ name }}.h"),
    },
    {
        'template_name': 'module_cmakelists',
        'template_package': 'templates.module',
        'template_file': 'module_CMakeLists.txt.jinja2',
        'output_path': Template("./src/module/{{ name }}"),
        'output_file': 'CMakeLists.txt',
    },
]


@dataclass
class ModuleExpandInfo:
    module_name: str
    class_name: str
    options: dict = None


class ModuleGenerator(GeneratorBase):
    def __init__(self, module_infos, project_name, output_dir=None):
        super().__init__(output_dir)
        self.module_infos_ = module_infos
        self.project_name_ = project_name
        self.expand_infos_ = []

    def parse(self):
        module_compile_tags = {}
        customize_modules = []
        for module_info in self.module_infos_:
            module_name = module_info['name']
            if module_name not in customize_modules:
                customize_modules.append(module_name)
            else:
                raise Exception(module_name + " is duplicated in Module cfg, Please fix it!!!")

            if 'build_mode_tag' in module_info.keys() and module_info['build_mode_tag'] is not None:
                module_compile_tags[module_name] = module_info['build_mode_tag']
            else:
                module_compile_tags[module_name] = []
            # module_options = module_info['options']
            split_names = module_name.split('_')
            class_name = ''
            for name in split_names:
                class_name += name.capitalize()

            expand_info = ModuleExpandInfo(
                module_name,
                class_name,
                # module_options,
            )
            self.expand_infos_.append(expand_info)

        return module_compile_tags, customize_modules

    def generate(self):
        print("Module Files Generating...")
        for expand_info in self.expand_infos_:
            self.generate_templates(expand_info.module_name, expand_info.class_name)

    def generate_templates(self, module_name, class_name):
        for module_template_info in module_template_infos:
            template = self.get_template_environment(module_template_info)

            output_stream = template.render(
                project_name=self.project_name_,
                module_name=module_name,
                class_name=class_name
            )

            self.output(module_template_info, module_name, output_stream)
