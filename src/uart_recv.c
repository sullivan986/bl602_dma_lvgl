#include "bflb_dma.h"
#include "bflb_l1c.h"
#include "bflb_mtimer.h"
#include "bflb_uart.h"
#include "board.h"

static struct bflb_device_s *uartx;
static struct bflb_device_s *dma0_ch0;
static struct bflb_device_s *dma0_ch1;