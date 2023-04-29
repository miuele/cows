#ifndef CO_DRIVER_CUSTOM_H_INCLUDED
#define CO_DRIVER_CUSTOM_H_INCLUDED


/* Stack configuration override default values.
 * For more information see file CO_config.h. */
#ifdef CO_SINGLE_THREAD
#define CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE 0
#else
#define CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE CO_CONFIG_FLAG_CALLBACK_PRE
#endif
#define CO_CONFIG_GLOBAL_FLAG_TIMERNEXT CO_CONFIG_FLAG_TIMERNEXT

#define CO_CONFIG_NMT (CO_CONFIG_NMT_CALLBACK_CHANGE | \
                       CO_CONFIG_NMT_MASTER | \
                       CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                       CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)

#define CO_CONFIG_HB_CONS (CO_CONFIG_HB_CONS_ENABLE | \
                           CO_CONFIG_HB_CONS_CALLBACK_MULTI | \
                           CO_CONFIG_HB_CONS_QUERY_FUNCT | \
                           CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | \
                           CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)

#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER | \
                      CO_CONFIG_EM_PROD_CONFIGURABLE | \
                      CO_CONFIG_EM_PROD_INHIBIT | \
                      CO_CONFIG_EM_HISTORY | \
                      CO_CONFIG_EM_STATUS_BITS | \
                      CO_CONFIG_EM_CONSUMER | \
                      CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                      CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)

#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | \
                           CO_CONFIG_SDO_SRV_BLOCK | \
                           CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | \
                           CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)

#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 900

#define CO_CONFIG_SDO_CLI (CO_CONFIG_SDO_CLI_ENABLE | \
                           CO_CONFIG_SDO_CLI_SEGMENTED | \
                           CO_CONFIG_SDO_CLI_BLOCK | \
                           CO_CONFIG_SDO_CLI_LOCAL | \
                           CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | \
                           CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)

#define CO_CONFIG_TIME (CO_CONFIG_TIME_ENABLE | \
                        CO_CONFIG_TIME_PRODUCER | \
                        CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                        CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)

#define CO_CONFIG_LSS (CO_CONFIG_LSS_SLAVE | \
                       CO_CONFIG_LSS_SLAVE_FASTSCAN_DIRECT_RESPOND | \
                       CO_CONFIG_LSS_MASTER | \
                       CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE)

#define CO_CONFIG_GTW (CO_CONFIG_GTW_ASCII | \
                       CO_CONFIG_GTW_ASCII_SDO | \
                       CO_CONFIG_GTW_ASCII_NMT | \
                       CO_CONFIG_GTW_ASCII_LSS | \
                       CO_CONFIG_GTW_ASCII_LOG | \
                       CO_CONFIG_GTW_ASCII_ERROR_DESC | \
                       CO_CONFIG_GTW_ASCII_PRINT_HELP | \
                       CO_CONFIG_GTW_ASCII_PRINT_LEDS)
#define CO_CONFIG_GTW_BLOCK_DL_LOOP 3
#define CO_CONFIG_GTWA_COMM_BUF_SIZE 2000
#define CO_CONFIG_GTWA_LOG_BUF_SIZE 10000

#define CO_CONFIG_CRC16 (CO_CONFIG_CRC16_ENABLE)

#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE | \
                        CO_CONFIG_FIFO_ALT_READ | \
                        CO_CONFIG_FIFO_CRC16_CCITT | \
                        CO_CONFIG_FIFO_ASCII_COMMANDS | \
                        CO_CONFIG_FIFO_ASCII_DATATYPES)

#define CO_DRIVER_ERROR_REPORTING 1


#endif
