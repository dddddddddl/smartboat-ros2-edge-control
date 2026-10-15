SET (LIBITEKONUSB_SRC iTekon-usb.c rbuf.c sm4.c)

ADD_LIBRARY (iTekon_usb STATIC ${LIBITEKONUSB_SRC})

SET_TARGET_PROPERTIES (iTekon_usb PROPERTIES OUTPUT_NAME "iTekon-usb")

SET_TARGET_PROPERTIES (iTekon_usb PROPERTIES CLEAN_DIRECT_OUTPUT 1)

ADD_DEFINITIONS("-fPIC")

target_link_libraries(iTekon_usb usb-1.0 pthread )

