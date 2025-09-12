#ifndef INC_APP_PARSER_H_
#define INC_APP_PARSER_H_

#include <cmsis_os.h>

typedef struct
{
	osThreadId_t runner; // task handle
	osThreadAttr_t attr;
	struct {
		osMessageQueueId_t queue;
		//osMutexId_t lock;
	} mailbox;
} u_ctx ;

typedef struct {
    u_ctx* rx_ctx;
    u_ctx* tx_ctx;
} ctx_wrapper;


namespace AppParser {


}

#endif /* INC_APP_PARSER_H_ */
