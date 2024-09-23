# ----------------------------------
# Options affecting listfile parsing
# ----------------------------------
with section("parse"):

    # Specify structure for custom cmake functions
    additional_commands = additional_commands = {
        'FetchContent_Declare': {
            'flags': ['OVERRIDE_FIND_PACKAGE'],
            'kwargs': {
                'URL': '*',
                'URL_HASH': '*',
                'SOURCE_DIR': '*',
                'DOWNLOAD_EXTRACT_TIMESTAMP': '*',
                'GIT_REPOSITORY ': '*',
                'GIT_TAG ': '*'
            }
        },
        'add_gtest_target': {
            'flags': [],
            'kwargs': {
                'TEST_TARGET': '*',
                'TEST_SRC': '*',
                'INC_DIR': '*'
            }
        },
        'add_protobuf_gencode_target_for_proto_path': {
            'flags': [],
            'kwargs': {
                'TARGET_NAME': '*',
                'PROTO_PATH': '*',
                'GENCODE_PATH': '*',
                'DEP_PROTO_TARGETS': '*',
                'OPTIONS': '*'
            }
        },
        'add_protobuf_gencode_target_for_one_proto_file': {
            'flags': [],
            'kwargs': {
                'TARGET_NAME': '*',
                'PROTO_FILE': '*',
                'GENCODE_PATH': '*',
                'DEP_PROTO_TARGETS': '*',
                'OPTIONS': '*'
            }
        },
        'add_protobuf_aimrt_rpc_gencode_target_for_proto_files': {
            'flags': [],
            'kwargs': {
                'TARGET_NAME': '*',
                'PROTO_FILES': '*',
                'GENCODE_PATH': '*',
                'DEP_PROTO_TARGETS': '*',
                'OPTIONS': '*'
            }
        },
        'add_ros2_aimrt_rpc_gencode_target_for_one_file': {
            'flags': [],
            'kwargs': {
                'PACKAGE_NAME': '*',
                'PROTO_FILE': '*',
                'GENCODE_PATH': '*',
                'DEP_PROTO_TARGETS': '*',
                'OPTIONS': '*'
            }
        }
    }

    # Override configurations per-command where available
    override_spec = {}

    # Specify variable tags.
    vartags = []

    # Specify property tags.
    proptags = []

# -----------------------------
# Options effecting formatting.
# -----------------------------
with section("format"):

    # How wide to allow formatted cmake files
    line_width = 180

    # How many spaces to tab for indent
    tab_size = 2

    # If an argument group contains more than this many sub-groups (parg or kwarg
    # groups) then force it to a vertical layout.
    max_subgroups_hwrap = 2

    # If a positional argument group contains more than this many arguments, then
    # force it to a vertical layout.
    max_pargs_hwrap = 6

    # If a cmdline positional group consumes more than this many lines without
    # nesting, then invalidate the layout (and nest)
    max_rows_cmdline = 2

    # If true, separate flow control names from their parentheses with a space
    separate_ctrl_name_with_space = False

    # If true, separate function names from parentheses with a space
    separate_fn_name_with_space = False

    # If a statement is wrapped to more than one line, than dangle the closing
    # parenthesis on its own line.
    dangle_parens = False

    # A list of command names which should always be wrapped
    always_wrap = ['target_link_libraries/PUBLIC/PargGroupNode[0]',
                   'target_link_libraries/PRIVATE/PargGroupNode[0]',
                   'target_link_libraries/INTERFACE/PargGroupNode[0]',
                   'target_include_directories/PUBLIC/PargGroupNode[0]',
                   'target_include_directories/PRIVATE/PargGroupNode[0]',
                   'target_include_directories/INTERFACE/PargGroupNode[0]',
                   'add_custom_target/DEPENDS/PargGroupNode[0]',
                   'add_protobuf_gencode_target_for_proto_path/DEP_PROTO_TARGETS/PargGroupNode[0]',
                   'add_protobuf_gencode_target_for_one_proto_file/DEP_PROTO_TARGETS/PargGroupNode[0]',
                   'add_protobuf_aimrt_rpc_gencode_target_for_proto_files/DEP_PROTO_TARGETS/PargGroupNode[0]',
                   'add_ros2_aimrt_rpc_gencode_target_for_one_file/DEP_PROTO_TARGETS/PargGroupNode[0]']

# ------------------------------------------------
# Options affecting comment reflow and formatting.
# ------------------------------------------------
with section("markup"):

    # What character to use for bulleted lists
    bullet_char = '*'

    # What character to use as punctuation after numerals in an enumerated list
    enum_char = '.'

    # If comment markup is enabled, don't reflow the first comment block in each
    # listfile. Use this to preserve formatting of your copyright/license
    # statements.
    first_comment_is_literal = False

    # If comment markup is enabled, don't reflow any comment block which matches
    # this (regex) pattern. Default is `None` (disabled).
    literal_comment_pattern = None

    # Regular expression to match preformat fences in comments default=
    # ``r'^\s*([`~]{3}[`~]*)(.*)$'``
    fence_pattern = '^\\s*([`~]{3}[`~]*)(.*)$'

    # Regular expression to match rulers in comments default=
    # ``r'^\s*[^\w\s]{3}.*[^\w\s]{3}$'``
    ruler_pattern = '^\\s*[^\\w\\s]{3}.*[^\\w\\s]{3}$'

    # If a comment line matches starts with this pattern then it is explicitly a
    # trailing comment for the preceeding argument. Default is '#<'
    explicit_trailing_pattern = '#<'

    # If a comment line starts with at least this many consecutive hash
    # characters, then don't lstrip() them off. This allows for lazy hash rulers
    # where the first hash char is not separated by space
    hashruler_min_length = 10

    # If true, then insert a space between the first hash char and remaining hash
    # chars in a hash ruler, and normalize its length to fill the column
    canonicalize_hashrulers = True

    # enable comment markup parsing and reflow
    enable_markup = False
