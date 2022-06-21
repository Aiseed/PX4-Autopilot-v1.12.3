# set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__rw_uart
    MAIN rw_uart
    COMPILE_FLAGS
    SRCS
        rw_uart.c
    DEPENDS
    	# px4_config
    	# px4_work_queue
    	# platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
