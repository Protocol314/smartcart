#ifndef SMARTCART_UWB_PARSER_BRIDGE_H
#define SMARTCART_UWB_PARSER_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#define SC_UWB_API __declspec(dllexport)
#else
#define SC_UWB_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ABI 경계에는 고정 폭 정수만 사용한다. 순서와 부호를 변경하면 버전을 올린다. */
typedef struct {
    uint32_t sequence;
    uint32_t tag_address;
    int32_t angle_deg;
    uint32_t distance_cm;
    uint32_t user_command;
    int32_t first_path_power;
    int32_t rx_level;
    int32_t acceleration_x;
    int32_t acceleration_y;
    int32_t acceleration_z;
} sc_uwb_frame;

SC_UWB_API uint32_t sc_uwb_abi_version(void);
SC_UWB_API uint32_t sc_uwb_frame_size(void);
SC_UWB_API void *sc_uwb_create(void);
SC_UWB_API void sc_uwb_destroy(void *handle);
SC_UWB_API void sc_uwb_reset(void *handle);
/* 0: 추가 바이트 대기, 1: 정상 프레임, 2: 불량 프레임/인수 오류. */
SC_UWB_API uint32_t sc_uwb_feed(void *handle, uint8_t byte, sc_uwb_frame *frame);
SC_UWB_API uint32_t sc_uwb_accepted(void *handle);
SC_UWB_API uint32_t sc_uwb_rejected(void *handle);
/* 원본 bu04_pdoa_error_t의 값을 반환하며 소유 포인터 이외의 사용은 허용하지 않는다. */
SC_UWB_API uint32_t sc_uwb_last_error(void *handle);

#ifdef __cplusplus
}
#endif

#endif
