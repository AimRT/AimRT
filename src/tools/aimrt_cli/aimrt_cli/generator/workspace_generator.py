# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from dataclasses import dataclass
from jinja2 import Template

from aimrt_cli.generator import GeneratorBase


@dataclass
class BuildModeExpandInfo:
    build_option: str
    items: list


@dataclass
class DefinitionExpandInfo:
    name: str
    status: str


@dataclass
class CustomizeBuildOptionsExpandInfo:
    name: str
    description: str
    status: str


@dataclass
class DependsStdModuleExpandInfo:
    name: str
    std_depend_name: str
    git_version: str
    git_repository: str
    import_options: dict = None


@dataclass
class WorkspaceExpandInfo:
    depends_std_modules: list
    aimrt_import_options: list
    definitions: list
    build_options: list
    protocols_build_modes: list
    modules_build_modes: list
    pkgs_build_modes: list


aimrt_core_options = [
    "AIMRT_BUILD_TESTS",
    "AIMRT_BUILD_EXAMPLES",
    "AIMRT_BUILD_PROTOCOLS",
    "AIMRT_BUILD_DOCUMENT",
    "AIMRT_BUILD_RUNTIME",
    "AIMRT_BUILD_CLI_TOOLS",
    "AIMRT_USE_FMT_LIB",
    "AIMRT_BUILD_WITH_PROTOBUF",
    "AIMRT_USE_LOCAL_PROTOC_COMPILER",
    "AIMRT_USE_PROTOC_PYTHON_PLUGIN",
    "AIMRT_BUILD_WITH_ROS2",
    "AIMRT_BUILD_NET_PLUGIN",
    "AIMRT_BUILD_ROS2_PLUGIN",
    "AIMRT_BUILD_MQTT_PLUGIN",
    "AIMRT_BUILD_ZENOH_PLUGIN",
    "AIMRT_BUILD_ICEORYX_PLUGIN",
    "AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN",
    "AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN",
    "AIMRT_BUILD_PARAMETER_PLUGIN",
    "AIMRT_BUILD_LOG_CONTROL_PLUGIN",
    "AIMRT_BUILD_SERVICE_INTROSPECTION_PLUGIN"
    "AIMRT_BUILD_TOPIC_LOGGER_PLUGIN",
    "AIMRT_BUILD_OPENTELEMETRY_PLUGIN",
]

workspace_template_infos = [
    {
        "template_name": "src_cmakelists",
        "template_package": "templates.workspace",
        "template_file": "src_CMakeLists.txt.jinja2",
        "output_path": "./src",
        "output_file": "CMakeLists.txt",
    },
    {
        "template_name": "ws_cmakelists",
        "template_package": "templates.workspace",
        "template_file": "ws_CMakeLists.txt.jinja2",
        "output_path": "./",
        "output_file": "CMakeLists.txt",
    },
    {
        "template_name": "build_sh",
        "template_package": "templates.workspace",
        "template_file": "template.sh.jinja2",
        "output_path": "./",
        "output_file": "build.sh",
    },
    {
        "template_name": "test_sh",
        "template_package": "templates.workspace",
        "template_file": "template.sh.jinja2",
        "output_path": "./",
        "output_file": "test.sh",
    },
    {
        "template_name": "get_depends_cmake",
        "template_package": "templates.workspace.cmake",
        "template_file": "get_depends_template.cmake.jinja2",
        "output_path": "./cmake",
        "output_file": Template("Get{{ name }}.cmake"),
    },
    {
        "template_name": "get_aimrt_cmake",
        "template_package": "templates.workspace.cmake",
        "template_file": "GetAimRT.cmake.jinja2",
        "output_path": "./cmake",
        "output_file": "GetAimRT.cmake",
    },
]

static_ws_template_infos = [
    {
        "template_name": "get_gtest_cmake",
        "template_package": "templates.workspace.cmake",
        "template_file": "GetGTest.cmake.jinja2",
        "output_path": "./cmake",
        "output_file": "GetGTest.cmake",
    },
    {
        "template_name": "get_namespace_tool_cmake",
        "template_package": "templates.workspace.cmake",
        "template_file": "NamespaceTool.cmake.jinja2",
        "output_path": "./cmake",
        "output_file": "NamespaceTool.cmake",
    },
    {
        "template_name": "clang_format",
        "template_package": "templates.workspace",
        "template_file": "ws_clang_format.jinja2",
        "output_path": "./",
        "output_file": ".clang-format",
    },
    {
        "template_name": "cmake_format",
        "template_package": "templates.workspace",
        "template_file": "ws_cmake_format.py.jinja2",
        "output_path": "./",
        "output_file": ".cmake-format.py",
    },
    {
        "template_name": "format_shell",
        "template_package": "templates.workspace",
        "template_file": "ws_format.sh.jinja2",
        "output_path": "./",
        "output_file": "format.sh",
    },
    {
        "template_name": "gitignore",
        "template_package": "templates.workspace",
        "template_file": "ws_gitignore.jinja2",
        "output_path": "./",
        "output_file": ".gitignore",
    },
    {
        "template_name": "pycodestyle",
        "template_package": "templates.workspace",
        "template_file": "ws_pycodestyle.jinja2",
        "output_path": "./",
        "output_file": ".pycodestyle",
    },
    {
        "template_name": "readme",
        "template_package": "templates.workspace",
        "template_file": "ws_readme.jinja2",
        "output_path": "./",
        "output_file": "README.md",
    },
]


class WorkspaceGenerator(GeneratorBase):
    def __init__(self, *, base_info, depends_std_modules, deploy_modes, output_dir=None):
        super().__init__()
        self.base_info_ = base_info
        self.depends_std_modules_ = depends_std_modules
        self.module_depends_ = []
        if depends_std_modules:
            for depend_module in depends_std_modules:
                self.module_depends_.append(depend_module["name"].replace("-", "_"))
        self.deploy_modes_ = deploy_modes
        if "project_name" not in self.base_info_.keys() and self.base_info_["project_name"] is None:
            raise Exception("You need to designate your project name in your yaml file!")
        else:
            self.project_name_ = self.base_info_["project_name"].replace("-", "_")
        self.build_mode_tags_ = self.base_info_["build_mode_tags"]
        self.aimrt_import_options_ = self.base_info_["aimrt_import_options"]
        self.test_marco_ = self.project_name_.upper() + "_BUILD_TESTS"
        self.expand_info_ = None
        if output_dir is None:
            self.output_dir_ = "./" + self.project_name_
        else:
            self.output_dir_ = output_dir

    def parse(self, *, build_modes=None):
        depends_std_modules, depend_import_options = self.parse_depends_std_modules()
        aimrt_import_options, definitions = self.parse_import_options(depend_import_options)
        self.expand_info_ = WorkspaceExpandInfo(
            depends_std_modules=depends_std_modules,
            aimrt_import_options=aimrt_import_options,
            definitions=definitions,
            build_options=self.parse_customize_build_options(),
            protocols_build_modes=self.parse_build_mode(build_modes["protocol"]),
            modules_build_modes=self.parse_build_mode(build_modes["module"]),
            pkgs_build_modes=self.parse_build_mode(build_modes["pkg"]),
        )

    def parse_import_options(self, depend_import_options):
        # parse the aimrt import options.
        import_options = []
        definitions = []
        if self.aimrt_import_options_:
            for aimrt_import_option, status in self.aimrt_import_options_.items():
                if aimrt_import_option in aimrt_core_options:
                    import_options.append(DefinitionExpandInfo(name=aimrt_import_option, status=status))
                else:
                    raise Exception(
                        "The aimrt import option: " + aimrt_import_option + " is not defined in aimrt, Please Check it!"
                    )
        definitions.extend(depend_import_options)
        # default OFF for build.sh
        definitions.append(DefinitionExpandInfo(name=self.test_marco_, status="OFF"))
        return import_options, definitions

    def parse_customize_build_options(self):
        # parse the build options.
        customize_build_options = []
        if self.build_mode_tags_:
            for build_mode_tag in self.build_mode_tags_:
                option_name = self.project_name_.upper() + "_" + build_mode_tag.upper()
                option_description = "Build " + build_mode_tag.lower()
                option_status = "OFF"
                customize_build_options.append(
                    CustomizeBuildOptionsExpandInfo(
                        name=option_name, description=option_description, status=option_status
                    )
                )
        return customize_build_options

    @staticmethod
    def check_depend_name(git_repository):
        split_names = git_repository.split("/")
        name = split_names[-1].replace(".git", "")
        return name

    def parse_depends_std_modules(self):
        # parse depends std modules.
        depends_std_modules = []
        import_options = []
        if self.depends_std_modules_:
            for depends_std_module in self.depends_std_modules_:
                depend_module_name = self.check_depend_name(depends_std_module["git_repository"])
                split_names = depend_module_name.split("-")
                std_depend_name = ""
                for name in split_names:
                    std_depend_name += name.capitalize()

                depends_std_modules.append(
                    DependsStdModuleExpandInfo(
                        name=depend_module_name,
                        std_depend_name=std_depend_name,
                        git_version=depends_std_module["git_tag"],
                        git_repository=depends_std_module["git_repository"],
                    )
                )
                if "import_options" in depends_std_module.keys() and depends_std_module["import_options"]:
                    for option, status in depends_std_module["import_options"].items():
                        import_options.append(DefinitionExpandInfo(name=option, status=status))
        return depends_std_modules, import_options

    def parse_build_mode(self, build_mode):
        # parse the build mode
        build_mode_expand_infos = []
        if build_mode:
            build_mode_dict = {"global": []}
            for name, build_mode_tags in build_mode.items():
                if build_mode_tags:
                    for build_mode_tag in build_mode_tags:
                        build_option = self.project_name_.upper() + "_" + build_mode_tag.upper()
                        if build_mode_tag in self.build_mode_tags_:
                            if build_option not in build_mode_dict.keys():
                                build_mode_dict[build_option] = []
                            build_mode_dict[build_option].append(name)
                        else:
                            raise Exception("Build mode tag: " + build_mode_tag + " Not found, Please check it!")
                else:
                    build_mode_dict["global"].append(name)

            for option, names in build_mode_dict.items():
                build_mode_expand_infos.append(BuildModeExpandInfo(build_option=option, items=names))
        return build_mode_expand_infos

    def generate(self):
        print("Workspace Files Generating...")
        for workspace_template_info in workspace_template_infos:
            template = self.get_template_environment(workspace_template_info)

            if workspace_template_info["template_name"] == "get_depends_cmake":
                for depends_std_module in self.expand_info_.depends_std_modules:
                    output_stream = template.render(depends_std_module=depends_std_module)
                    self.output(workspace_template_info, depends_std_module.std_depend_name, output_stream)
                continue

            if workspace_template_info["template_name"] == "test_sh":
                for definition in self.expand_info_.definitions:
                    if definition.name == self.test_marco_:
                        definition.status = "ON"

            output_stream = template.render(workspace=self.expand_info_, project_name=self.project_name_)
            self.output(workspace_template_info, "", output_stream)

        for static_ws_template_info in static_ws_template_infos:
            template = self.get_template_environment(static_ws_template_info)
            output_stream = template.render()
            self.output(static_ws_template_info, "", output_stream)

    def get_project_name(self):
        return self.project_name_

    def get_std_module_depends(self):
        return self.module_depends_

    def get_output_dir(self):
        return self.output_dir_
