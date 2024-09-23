# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from dataclasses import dataclass
from jinja2 import Template

from aimrt_cli.generator import GeneratorBase

proto_template_dist = {
    'protobuf':
        [
            {
                'template_name': 'proto_file',
                'template_package': 'templates.protocols',
                'template_file': 'template.proto.jinja2',
                'output_path': Template("./src/protocols/{{ name }}"),
                'output_file': Template("{{ name }}.proto"),
            },
        ],
    'ros2':
        [
            {
                'template_name': 'ros2_package_xml',
                'template_package': 'templates.protocols',
                'template_file': 'ros2_package.xml.jinja2',
                'output_path': Template("./src/protocols/{{ name }}"),
                'output_file': "package.xml",
            },
        ]
}

static_proto_template_dist = {
    'protobuf':
        [
            {
                'template_name': 'proto_cmakelists',
                'template_package': 'templates.protocols',
                'template_file': 'proto_CMakeLists.txt.jinja2',
                'output_path': Template("./src/protocols/{{ name }}"),
                'output_file': 'CMakeLists.txt',
            },
        ],
    'ros2':
        [
            {
                'template_name': 'ros2_msg',
                'template_package': 'templates.protocols',
                'template_file': 'ros2_template.msg.jinja2',
                'output_path': Template("./src/protocols/{{ name }}/msg"),
                'output_file': "RosChangeMe.msg",
            },
            {
                'template_name': 'ros2_cmakelists',
                'template_package': 'templates.protocols',
                'template_file': 'ros2_CMakeLists.txt.jinja2',
                'output_path': Template("./src/protocols/{{ name }}"),
                'output_file': "CMakeLists.txt",
            },
        ],
}


@dataclass
class ProtoExpandInfo:
    name: str
    type: str
    options: dict = None


class ProtoGenerator(GeneratorBase):
    def __init__(self, proto_infos, project_name, output_dir):
        super().__init__(output_dir)
        self.proto_infos_ = proto_infos
        self.project_name_ = project_name
        self.expand_infos_ = []

    def parse(self):
        build_mode_tags = {}
        for proto_info in self.proto_infos_:
            proto_name = proto_info['name']
            proto_type = proto_info['type']
            # proto_options = proto_info['options']
            if 'build_mode_tag' in proto_info.keys() and proto_info['build_mode_tag'] is not None:
                build_mode_tags[proto_name] = proto_info['build_mode_tag']
            else:
                build_mode_tags[proto_name] = []

            expand_info = ProtoExpandInfo(name=proto_name, type=proto_type)
            self.expand_infos_.append(expand_info)

        return build_mode_tags

    def generate(self):
        print("Generating protocol files...")
        for expand_info in self.expand_infos_:
            self.generate_templates(expand_info,
                                    proto_template_dist[expand_info.type], static_proto_template_dist[expand_info.type])

    def generate_templates(self, expand_info, template_infos, static_template_infos):
        for proto_template_info in template_infos:
            template = self.get_template_environment(proto_template_info)
            if expand_info.type == 'protobuf':
                output_stream = template.render(project_name=self.project_name_, proto=expand_info)
            else:
                output_stream = template.render(proto=expand_info)
            self.output(proto_template_info, expand_info.name, output_stream)

        for static_proto_template_info in static_template_infos:
            template = self.get_template_environment(static_proto_template_info)
            output_stream = template.render()
            self.output(static_proto_template_info, expand_info.name, output_stream)
