#include "uwb_parser_bridge.h"

#include <stdlib.h>

#include "bu04_pdoa.h"

_Static_assert(sizeof(sc_uwb_frame) == 40U, "UWB ABI 크기 불일치");
_Static_assert(offsetof(sc_uwb_frame, acceleration_z) == 36U, "UWB ABI 배치 불일치");

uint32_t sc_uwb_abi_version(void)
{
    return 1U;
}

uint32_t sc_uwb_frame_size(void)
{
    return (uint32_t)sizeof(sc_uwb_frame);
}

void *sc_uwb_create(void)
{
    bu04_pdoa_stream_t *stream = calloc(1U, sizeof(*stream));
    if (stream != NULL) {
        bu04_pdoa_stream_init(stream);
    }
    return stream;
}

void sc_uwb_destroy(void *handle)
{
    free(handle);
}

void sc_uwb_reset(void *handle)
{
    bu04_pdoa_stream_init(handle);
}

uint32_t sc_uwb_feed(void *handle, uint8_t byte, sc_uwb_frame *frame)
{
    if (handle == NULL || frame == NULL) {
        return 2U;
    }
    bu04_pdoa_frame_t decoded;
    const bu04_pdoa_feed_result_t result = bu04_pdoa_stream_feed(handle, byte, &decoded);
    if (result == BU04_PDOA_FEED_FRAME) {
        frame->sequence = decoded.sequence;
        frame->tag_address = decoded.tag_address;
        frame->angle_deg = decoded.angle_deg;
        frame->distance_cm = decoded.distance_cm;
        frame->user_command = decoded.user_command;
        frame->first_path_power = decoded.first_path_power;
        frame->rx_level = decoded.rx_level;
        frame->acceleration_x = decoded.acceleration_x;
        frame->acceleration_y = decoded.acceleration_y;
        frame->acceleration_z = decoded.acceleration_z;
        return 1U;
    }
    return result == BU04_PDOA_FEED_BAD_FRAME ? 2U : 0U;
}

uint32_t sc_uwb_accepted(void *handle)
{
    return handle == NULL ? 0U : ((bu04_pdoa_stream_t *)handle)->accepted_frames;
}

uint32_t sc_uwb_rejected(void *handle)
{
    return handle == NULL ? 0U : ((bu04_pdoa_stream_t *)handle)->rejected_frames;
}

uint32_t sc_uwb_last_error(void *handle)
{
    return handle == NULL ? (uint32_t)BU04_PDOA_ERROR_ARGUMENT :
                           (uint32_t)((bu04_pdoa_stream_t *)handle)->last_error;
}
