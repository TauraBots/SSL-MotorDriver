#include "quadmd_protocol.h"

#include <math.h>

// ============================================================
// LITTLE ENDIAN WRITE
// ============================================================

static void write_u16(
    uint8_t *buffer,
    size_t *index,
    uint16_t value)
{
    buffer[(*index)++] =
        (uint8_t)(value & 0xFFU);

    buffer[(*index)++] =
        (uint8_t)((value >> 8) & 0xFFU);
}

// ============================================================

static void write_i16(
    uint8_t *buffer,
    size_t *index,
    int16_t value)
{
    write_u16(
        buffer,
        index,
        (uint16_t)value
    );
}

// ============================================================

static void write_u32(
    uint8_t *buffer,
    size_t *index,
    uint32_t value)
{
    buffer[(*index)++] =
        (uint8_t)(value & 0xFFU);

    buffer[(*index)++] =
        (uint8_t)((value >> 8) & 0xFFU);

    buffer[(*index)++] =
        (uint8_t)((value >> 16) & 0xFFU);

    buffer[(*index)++] =
        (uint8_t)((value >> 24) & 0xFFU);
}

// ============================================================
// LITTLE ENDIAN READ
// ============================================================

static uint16_t read_u16(
    const uint8_t *p)
{
    return
        ((uint16_t)p[0]) |
        ((uint16_t)p[1] << 8);
}

// ============================================================

static int16_t read_i16(
    const uint8_t *p)
{
    return (int16_t)read_u16(p);
}

// ============================================================

static uint32_t read_u32(
    const uint8_t *p)
{
    return
        ((uint32_t)p[0]) |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);
}

// ============================================================
// SATURAÇÃO
// ============================================================

static int16_t saturate_i16(
    float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)lroundf(value);
}

// ============================================================
// CRC16 CCITT FALSE
// ============================================================

uint16_t quadmd_crc16(
    const uint8_t *data,
    size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < length; i++)
    {
        crc ^=
            (uint16_t)data[i] << 8;

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000U)
            {
                crc =
                    (uint16_t)(
                        (crc << 1) ^
                        0x1021U
                    );
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// ============================================================
// D0
// ============================================================

size_t quadmd_encode_velocity(
    uint8_t *buffer,

    uint8_t robot_id,

    uint32_t sequence,

    float vx,
    float vy,
    float omega,

    uint8_t kick_power,

    bool brake)
{
    if (buffer == NULL)
    {
        return 0;
    }

    size_t i = 0;

    // Header
    buffer[i++] = QUADMD_SOF0;
    buffer[i++] = QUADMD_SOF1;

    // Tipo
    buffer[i++] =
        QUADMD_TYPE_VELOCITY;

    // Versão
    buffer[i++] =
        QUADMD_PROTOCOL_VERSION;

    // Robot ID
    buffer[i++] =
        robot_id;

    // Sequence
    write_u32(
        buffer,
        &i,
        sequence
    );

    // m/s -> mm/s
    write_i16(
        buffer,
        &i,
        saturate_i16(
            vx * 1000.0f
        )
    );

    write_i16(
        buffer,
        &i,
        saturate_i16(
            vy * 1000.0f
        )
    );

    // rad/s -> mrad/s
    write_i16(
        buffer,
        &i,
        saturate_i16(
            omega * 1000.0f
        )
    );

    if (kick_power > 100U)
    {
        kick_power = 100U;
    }

    buffer[i++] =
        kick_power;

    buffer[i++] =
        brake ? 1U : 0U;

    // CRC
    const uint16_t crc =
        quadmd_crc16(
            buffer,
            i
        );

    write_u16(
        buffer,
        &i,
        crc
    );

    return i;
}

// ============================================================
// E0
// ============================================================

size_t quadmd_encode_telemetry_request(
    uint8_t *buffer,

    uint8_t robot_id,

    uint16_t request_sequence,

    uint8_t flags)
{
    if (buffer == NULL)
    {
        return 0;
    }

    size_t i = 0;

    buffer[i++] = QUADMD_SOF0;
    buffer[i++] = QUADMD_SOF1;

    buffer[i++] =
        QUADMD_TYPE_TELEMETRY_REQUEST;

    buffer[i++] =
        QUADMD_PROTOCOL_VERSION;

    buffer[i++] =
        robot_id;

    write_u16(
        buffer,
        &i,
        request_sequence
    );

    buffer[i++] =
        flags &
        QUADMD_TELEMETRY_FULL;

    const uint16_t crc =
        quadmd_crc16(
            buffer,
            i
        );

    write_u16(
        buffer,
        &i,
        crc
    );

    return i;
}

// ============================================================
// TAMANHO E1
// ============================================================

size_t quadmd_telemetry_frame_size(
    uint8_t flags)
{
    size_t size = 11U;

    flags &=
        QUADMD_TELEMETRY_FULL;

    if (flags & QUADMD_TELEMETRY_BASIC)
    {
        size += 7U;
    }

    if (flags & QUADMD_TELEMETRY_MOTORS)
    {
        size += 16U;
    }

    if (flags & QUADMD_TELEMETRY_BATTERY)
    {
        size += 4U;
    }

    if (flags & QUADMD_TELEMETRY_DIAGNOSTICS)
    {
        size += 13U;
    }

    return size;
}

// ============================================================
// PARSER E1
// ============================================================

bool quadmd_parse_telemetry(
    const uint8_t *frame,

    size_t length,

    uint8_t expected_robot_id,

    quadmd_telemetry_t *t)
{
    if (
        frame == NULL ||
        t == NULL
    )
    {
        return false;
    }

    if (length < 11U)
    {
        return false;
    }

    // ========================================================
    // Header
    // ========================================================

    if (
        frame[0] != QUADMD_SOF0 ||
        frame[1] != QUADMD_SOF1
    )
    {
        return false;
    }

    if (
        frame[2] !=
        QUADMD_TYPE_TELEMETRY_RESPONSE
    )
    {
        return false;
    }

    if (
        frame[3] !=
        QUADMD_PROTOCOL_VERSION
    )
    {
        return false;
    }

    if (
        frame[4] !=
        expected_robot_id
    )
    {
        return false;
    }

    // ========================================================
    // Tamanho
    // ========================================================

    const uint8_t flags =
        frame[8] &
        QUADMD_TELEMETRY_FULL;

    const size_t expected_length =
        quadmd_telemetry_frame_size(
            flags
        );

    if (length != expected_length)
    {
        return false;
    }

    // ========================================================
    // CRC
    // ========================================================

    const uint16_t crc_rx =
        read_u16(
            &frame[length - 2U]
        );

    const uint16_t crc_calc =
        quadmd_crc16(
            frame,
            length - 2U
        );

    if (crc_rx != crc_calc)
    {
        return false;
    }

    // ========================================================
    // IMPORTANTE:
    //
    // NÃO usamos memset(t, 0, ...)
    //
    // FAST não pode apagar BATTERY/DIAGNOSTICS anteriores.
    // ========================================================

    t->valid = true;

    t->robot_id =
        frame[4];

    t->request_sequence =
        read_u16(
            &frame[5]
        );

    t->status =
        frame[7];

    t->fault_status =
        frame[7] >> 1;

    t->flags =
        flags;

    size_t offset = 9U;

    // ========================================================
    // BASIC
    // ========================================================

    if (flags & QUADMD_TELEMETRY_BASIC)
    {
        t->time_ms =
            read_u32(
                &frame[offset]
            );

        offset += 4U;

        t->communication_ok =
            frame[offset++];

        t->brake =
            frame[offset++];

        t->kick_power =
            frame[offset++];
    }

    // ========================================================
    // MOTORS
    // ========================================================

    if (flags & QUADMD_TELEMETRY_MOTORS)
    {
        for (int motor = 0; motor < 4; motor++)
        {
            t->rpm[motor] =
                (float)read_i16(
                    &frame[offset]
                ) / 10.0f;

            offset += 2U;
        }

        for (int motor = 0; motor < 4; motor++)
        {
            t->motor_command[motor] =
                read_i16(
                    &frame[offset]
                );

            offset += 2U;
        }
    }

    // ========================================================
    // BATTERY
    // ========================================================

    if (flags & QUADMD_TELEMETRY_BATTERY)
    {
        const uint16_t battery_mv =
            read_u16(
                &frame[offset]
            );

        offset += 2U;

        t->battery_voltage =
            (float)battery_mv /
            1000.0f;

        t->battery_adc =
            read_u16(
                &frame[offset]
            );

        offset += 2U;
    }

    // ========================================================
    // DIAGNOSTICS
    // ========================================================

    if (
        flags &
        QUADMD_TELEMETRY_DIAGNOSTICS
    )
    {
        t->crc_errors =
            read_u32(
                &frame[offset]
            );

        offset += 4U;

        t->received_packets =
            read_u32(
                &frame[offset]
            );

        offset += 4U;

        t->watchdog_ok =
            frame[offset++];

        t->last_command_sequence =
            read_u32(
                &frame[offset]
            );

        offset += 4U;
    }

    return true;
}