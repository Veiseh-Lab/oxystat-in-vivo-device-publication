#include "spidrv.h"
#include "sl_spidrv_instances.h"
#include "sl_assert.h"


#include "sl_spidrv_dac_config.h"

SPIDRV_HandleData_t sl_spidrv_dac_handle_data;
SPIDRV_Handle_t sl_spidrv_dac_handle = &sl_spidrv_dac_handle_data;

SPIDRV_Init_t sl_spidrv_init_dac = {
  .port = SL_SPIDRV_DAC_PERIPHERAL,
#if defined(_USART_ROUTELOC0_MASK)
  .portLocationTx = SL_SPIDRV_DAC_TX_LOC,
  .portLocationRx = SL_SPIDRV_DAC_RX_LOC,
  .portLocationClk = SL_SPIDRV_DAC_CLK_LOC,
#if defined(SL_SPIDRV_DAC_CS_LOC)
  .portLocationCs = SL_SPIDRV_DAC_CS_LOC,
#endif
#elif defined(_GPIO_USART_ROUTEEN_MASK)
  .portTx = SL_SPIDRV_DAC_TX_PORT,
  .portRx = SL_SPIDRV_DAC_RX_PORT,
  .portClk = SL_SPIDRV_DAC_CLK_PORT,
#if defined(SL_SPIDRV_DAC_CS_PORT)
  .portCs = SL_SPIDRV_DAC_CS_PORT,
#endif
  .pinTx = SL_SPIDRV_DAC_TX_PIN,
  .pinRx = SL_SPIDRV_DAC_RX_PIN,
  .pinClk = SL_SPIDRV_DAC_CLK_PIN,
#if defined(SL_SPIDRV_DAC_CS_PIN)
  .pinCs = SL_SPIDRV_DAC_CS_PIN,
#endif
#else
  .portLocation = SL_SPIDRV_DAC_ROUTE_LOC,
#endif
  .bitRate = SL_SPIDRV_DAC_BITRATE,
  .frameLength = SL_SPIDRV_DAC_FRAME_LENGTH,
  .dummyTxValue = 0,
  .type = SL_SPIDRV_DAC_TYPE,
  .bitOrder = SL_SPIDRV_DAC_BIT_ORDER,
  .clockMode = SL_SPIDRV_DAC_CLOCK_MODE,
  .csControl = SL_SPIDRV_DAC_CS_CONTROL,
  .slaveStartMode = SL_SPIDRV_DAC_SLAVE_START_MODE,
};

void sl_spidrv_init_instances(void) {
  SPIDRV_Init(sl_spidrv_dac_handle, &sl_spidrv_init_dac);
}
