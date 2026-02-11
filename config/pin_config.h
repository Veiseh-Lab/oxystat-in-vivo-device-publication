#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[ADC0]
// [ADC0]$

// $[CMU]
// [CMU]$

// $[DBG]
// DBG SWCLKTCK on PF0
#ifndef DBG_SWCLKTCK_PORT                       
#define DBG_SWCLKTCK_PORT                        gpioPortF
#endif
#ifndef DBG_SWCLKTCK_PIN                        
#define DBG_SWCLKTCK_PIN                         0
#endif
#ifndef DBG_ROUTE_LOC                           
#define DBG_ROUTE_LOC                            0
#endif

// DBG SWDIOTMS on PF1
#ifndef DBG_SWDIOTMS_PORT                       
#define DBG_SWDIOTMS_PORT                        gpioPortF
#endif
#ifndef DBG_SWDIOTMS_PIN                        
#define DBG_SWDIOTMS_PIN                         1
#endif

// DBG SWV on PF2
#ifndef DBG_SWV_PORT                            
#define DBG_SWV_PORT                             gpioPortF
#endif
#ifndef DBG_SWV_PIN                             
#define DBG_SWV_PIN                              2
#endif
#ifndef DBG_SWV_LOC                             
#define DBG_SWV_LOC                              0
#endif

// [DBG]$

// $[PTI]
// [PTI]$

// $[GPIO]
// [GPIO]$

// $[I2C0]
// I2C0 SCL on PC9
#ifndef I2C0_SCL_PORT                           
#define I2C0_SCL_PORT                            gpioPortC
#endif
#ifndef I2C0_SCL_PIN                            
#define I2C0_SCL_PIN                             9
#endif
#ifndef I2C0_SCL_LOC                            
#define I2C0_SCL_LOC                             13
#endif

// I2C0 SDA on PC8
#ifndef I2C0_SDA_PORT                           
#define I2C0_SDA_PORT                            gpioPortC
#endif
#ifndef I2C0_SDA_PIN                            
#define I2C0_SDA_PIN                             8
#endif
#ifndef I2C0_SDA_LOC                            
#define I2C0_SDA_LOC                             13
#endif

// [I2C0]$

// $[IDAC0]
// [IDAC0]$

// $[LETIMER0]
// [LETIMER0]$

// $[LEUART0]
// [LEUART0]$

// $[LFXO]
// [LFXO]$

// $[MODEM]
// [MODEM]$

// $[PCNT0]
// [PCNT0]$

// $[PRS.CH0]
// [PRS.CH0]$

// $[PRS.CH1]
// [PRS.CH1]$

// $[PRS.CH2]
// [PRS.CH2]$

// $[PRS.CH3]
// [PRS.CH3]$

// $[PRS.CH4]
// [PRS.CH4]$

// $[PRS.CH5]
// [PRS.CH5]$

// $[PRS.CH6]
// [PRS.CH6]$

// $[PRS.CH7]
// [PRS.CH7]$

// $[PRS.CH8]
// [PRS.CH8]$

// $[PRS.CH9]
// [PRS.CH9]$

// $[PRS.CH10]
// [PRS.CH10]$

// $[PRS.CH11]
// [PRS.CH11]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[USART0]
// USART0 CLK on PB11
#ifndef USART0_CLK_PORT                         
#define USART0_CLK_PORT                          gpioPortB
#endif
#ifndef USART0_CLK_PIN                          
#define USART0_CLK_PIN                           11
#endif
#ifndef USART0_CLK_LOC                          
#define USART0_CLK_LOC                           4
#endif

// USART0 RX on PC10
#ifndef USART0_RX_PORT                          
#define USART0_RX_PORT                           gpioPortC
#endif
#ifndef USART0_RX_PIN                           
#define USART0_RX_PIN                            10
#endif
#ifndef USART0_RX_LOC                           
#define USART0_RX_LOC                            14
#endif

// USART0 TX on PB12
#ifndef USART0_TX_PORT                          
#define USART0_TX_PORT                           gpioPortB
#endif
#ifndef USART0_TX_PIN                           
#define USART0_TX_PIN                            12
#endif
#ifndef USART0_TX_LOC                           
#define USART0_TX_LOC                            7
#endif

// [USART0]$

// $[USART1]
// [USART1]$

// $[CUSTOM_PIN_NAME]
#ifndef _PORT                                   
#define _PORT                                    gpioPortA
#endif
#ifndef _PIN                                    
#define _PIN                                     0
#endif

#ifndef CHxP_PORT                               
#define CHxP_PORT                                gpioPortA
#endif
#ifndef CHxP_PIN                                
#define CHxP_PIN                                 2
#endif

#ifndef MAG_SW_PORT                             
#define MAG_SW_PORT                              gpioPortA
#endif
#ifndef MAG_SW_PIN                              
#define MAG_SW_PIN                               3
#endif

#ifndef CHxN_PORT                               
#define CHxN_PORT                                gpioPortA
#endif
#ifndef CHxN_PIN                                
#define CHxN_PIN                                 4
#endif

#ifndef CS_DAC12_PORT                           
#define CS_DAC12_PORT                            gpioPortC
#endif
#ifndef CS_DAC12_PIN                            
#define CS_DAC12_PIN                             6
#endif

#ifndef CS_DAC34_PORT                           
#define CS_DAC34_PORT                            gpioPortC
#endif
#ifndef CS_DAC34_PIN                            
#define CS_DAC34_PIN                             7
#endif

#ifndef B_IRQn_PORT                             
#define B_IRQn_PORT                              gpioPortD
#endif
#ifndef B_IRQn_PIN                              
#define B_IRQn_PIN                               9
#endif

#ifndef DEBUG_LED_PORT                          
#define DEBUG_LED_PORT                           gpioPortD
#endif
#ifndef DEBUG_LED_PIN                           
#define DEBUG_LED_PIN                            10
#endif

#ifndef CHMUX_A0_PORT                           
#define CHMUX_A0_PORT                            gpioPortD
#endif
#ifndef CHMUX_A0_PIN                            
#define CHMUX_A0_PIN                             12
#endif

#ifndef CHMUX_A1_PORT                           
#define CHMUX_A1_PORT                            gpioPortD
#endif
#ifndef CHMUX_A1_PIN                            
#define CHMUX_A1_PIN                             13
#endif

#ifndef CHMUX_EN_PORT                           
#define CHMUX_EN_PORT                            gpioPortD
#endif
#ifndef CHMUX_EN_PIN                            
#define CHMUX_EN_PIN                             14
#endif

#ifndef B_OK_PORT                               
#define B_OK_PORT                                gpioPortF
#endif
#ifndef B_OK_PIN                                
#define B_OK_PIN                                 5
#endif

#ifndef REFOUT_DAC12_PORT                       
#define REFOUT_DAC12_PORT                        gpioPortF
#endif
#ifndef REFOUT_DAC12_PIN                        
#define REFOUT_DAC12_PIN                         6
#endif

#ifndef REFOUT_DAC34_PORT                       
#define REFOUT_DAC34_PORT                        gpioPortF
#endif
#ifndef REFOUT_DAC34_PIN                        
#define REFOUT_DAC34_PIN                         7
#endif

// [CUSTOM_PIN_NAME]$

#endif // PIN_CONFIG_H

