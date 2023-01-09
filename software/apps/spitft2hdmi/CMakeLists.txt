# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

add_executable(spitft2hdmi
	main.c
)

pico_generate_pio_header(spitft2hdmi ${CMAKE_CURRENT_LIST_DIR}/fourwire.pio)

target_compile_options(spitft2hdmi PRIVATE -Wall)

target_compile_definitions(spitft2hdmi PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	)

target_link_libraries(spitft2hdmi
	pico_stdlib
	pico_multicore
	pico_util
	libdvi
)

# create map/bin/hex file etc.
pico_add_extra_outputs(spitft2hdmi)
