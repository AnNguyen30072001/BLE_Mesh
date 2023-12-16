set(srcs "main.c"
        "board.c"
	"DHT_c.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
