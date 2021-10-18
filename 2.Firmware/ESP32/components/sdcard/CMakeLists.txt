set(sdcard_srcs "src/sdcard.c")

idf_component_register(SRCS "${sdcard_srcs}"
                       INCLUDE_DIRS "include"
					   REQUIRES fatfs)
