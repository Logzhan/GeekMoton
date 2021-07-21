set(srcs
	"hello_world_main.c"
    "geek_shell.c"
    "geek_shell_cmd_list.c"
    "geek_shell_companion.c"
    "geek_shell_ext.c")

set(include_dirs "include")




idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}")

