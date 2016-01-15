#ifndef __SLIP_H
#define __SLIP_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/**
 * @brief   SLIP reserved END byte.
 */
#define SLIP_END                0xC0
/**
 * @brief   SLIP reserved ESC byte, used for escaping sequences.
 */
#define SLIP_ESC                0xDB
/**
 * @brief   SLIP ESC_END byte, used after ESC to indicate an escaped END byte.
 */
#define SLIP_ESC_END            0xDC
/**
 * @brief   SLIP ESC_ESC byte, used after ESC to indicate an escaped ESC byte.
 */
#define SLIP_ESC_ESC            0xDD

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

bool GenerateSLIP(uint8_t *data,
                  const uint32_t size,
                  circular_buffer_t *cb);
bool GenerateSLIP_HBT(uint8_t *head,
                      const uint32_t h_size,
                      uint8_t *body,
                      const uint32_t b_size,
                      uint8_t *tail,
                      const uint32_t t_size,
                      circular_buffer_t *cb);

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/


#endif
