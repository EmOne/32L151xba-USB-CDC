/*
 * port.c
 *
 *  Created on: Jul 2, 2019
 *      Author: anolp
 */
#include "port.h"

void BoardCriticalSectionBegin( uint32_t *mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    __set_PRIMASK( *mask );
}
