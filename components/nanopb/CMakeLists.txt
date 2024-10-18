set(NANOPB_INCLUDE_DIR
        nanopb)

set(NANOPB_CPP_INCLUDE_DIR
        nanopb_cpp)

set(GEN_PROTOBUF_INCLUDE_DIR
        protobuf)

set(NANOPB_SRCS
        nanopb/pb_common.c nanopb/pb_decode.c nanopb/pb_encode.c)

set(GENERATED_PROTOBUF_SRCS
        protobuf/lane.pb.c
        protobuf/ble.pb.c
)

idf_component_register(
        SRCS ${NANOPB_SRCS} ${GENERATED_PROTOBUF_SRCS}
        INCLUDE_DIRS ${NANOPB_INCLUDE_DIR} ${NANOPB_CPP_INCLUDE_DIR} ${GEN_PROTOBUF_INCLUDE_DIR}
)
