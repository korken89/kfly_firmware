/**
 *
 * OS layer for Serial Communication.
 * Handles package coding/decoding.
 *
 * THE TEXT BELLOW IS NOT VALID ANYMORE! CURRENTLY RUNNING SLIP.
 *
 * Serial Communication Protocol
 * -----------------------------
 * This was designed so that binary data could be sent while not
 * needing to "code" it as ASCII HEX. A simple SYNC byte is used
 * to denote the start of a transfer and after that a header
 * containing the command and size of the message. If the size is
 * greater than 0 a data package comes after the first CRC. If
 * the data contains a byte that has the same value as the sync
 * byte it will be replaced by two sync bytes "x" -> "xx" to denote
 * a byte of the value sync and not a data package sync.
 *
 *
 * Protocol:
 *      SYNC | HEADER | CRC8 | DATA | CRC16
 *      DATA and CRC16 is optional.
 *
 * HEADER:
 *      CMD     | DATA SIZE
 *      1 byte  | 1 byte
 *
 * DATA:
 *      BINARY DATA
 *      1 - 255 bytes
 *
 * SYNC: 1 byte
 *      Sent once = SYNC
 *      Sent twice = data byte with the value of SYNC ("escaping")
 *
 * CRC8: 1 byte
 *      CRC-8 of SYNC and HEADER
 *
 * CRC16: 2 bytes
 *      CCITT (16-bit) of whole message including SYNC and CRC8
 *      For more information about the CRCs look in crc.c/crc.h
 *
 *
 * -------------------- OBSERVE! --------------------
 * A command is build up by 8 bits (1 byte) and the 8-th bit (MSB) is the ACK
 * request bit. So all command must not use the ACK bit unless they need an ACK.
 * Command 0bAxxx xxxx <- A is ACK-bit.
 * Also the value 0xa6/166d/0b10100110 is reserved as the SYNC-byte.
 *
 * This gives 126 commands for the user.
 *
 */

#include "ch.h"
#include "hal.h"
#include "crc.h"
#include "kflypacket_parsers.h"
#include "kflypacket_generators.h"
#include "slip2kflypacket.h"


/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

 /*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief                   Initializes the data holder structure.
 *
 * @param[in/out] p         Pointer to kfly_parser_t structure.
 * @param[in]     port      Port used for data transfers.
 * @param[in]     buffer    Buffer where the data is saved (from the SLIP).
 */
void InitKFlyPacketParser(kfly_parser_t *p,
                          external_port_t port,
                          uint8_t *buffer)
{
    p->port = port;
    p->buffer = buffer;
    p->parser = NULL;
    p->rx_cmd_error = 0;
    p->rx_size_error = 0;
    p->rx_crc_error = 0;
    p->rx_success = 0;
}

/**
 * @brief                   Parses the data in a SLIP package to asess if it is
 *                          a valid KFly packet and then passes it on to the
 *                          usage parser.
 *
 * @param[in/out] slip      Pointer to slip_parser_t structure.
 * @param[in/out] p         Pointer to kfly_parser_t structure.
 */
void ParseKFlyPacketFromSLIP(slip_parser_t *slip, kfly_parser_t *p)
{
    uint8_t cmd;
    uint16_t crc16;
    const uint16_t size = slip->buffer_count;
    uint8_t *buffer = slip->buffer;

    /*
     * Check the CRC.
     */

    /* Remove the last 2 as they are the CRC. */
    crc16 = CRC16(buffer, size - 2);

    /* CRC comes LSB first. */
    if (((uint8_t)(crc16 >> 8) != buffer[size - 1]) ||
        ((uint8_t)(crc16) != buffer[size - 2]))
    {
        p->rx_crc_error++;
        return;
    }


    /*
     * Check the command.
     */

    cmd = buffer[0];

    /* 0 is not an allowed command (Cmd_None) */
    if ((cmd & ~ACK_BIT) > Cmd_None)
    {
        /* Get the correct parser from the parser lookup table. */
        p->parser = GetParser(cmd & ~ACK_BIT);

        /* If ACK is requested */
        if (cmd & ACK_BIT)
            p->ack = true;
        else
            p->ack = false;
    }
    else
    {
        p->rx_cmd_error++;
    }


    /*
     * Save the size and run parsers.
     */

    if ((size >= 4) && ((size - 4) == buffer[1]))
    {
        /* Receive success! Increment statistics counter. */
        p->data_length = size - 4;

        p->rx_success++;

        /* If an ACK was requested, send it. */
        if (p->ack == true)
            GenerateMessage(Cmd_ACK, p->port);

        /* If there is a parser for the message, execute it. */
        if (p->parser != NULL)
            p->parser(p);
    }
    else
    {
        p->rx_size_error++;
    }
}

