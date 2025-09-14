#ifndef INC_APP_PARSER_H_
#define INC_APP_PARSER_H_

#include <cmsis_os.h>


typedef struct
{
	osThreadId_t runner; // task handle
	osThreadAttr_t attr;
	struct {
		osMessageQueueId_t queue;
	} mailbox; //

} u_ctx ; // struct containing context info for a  instance.
typedef struct {
    u_ctx* rx_ctx;
    u_ctx* tx_ctx;
} ctx_wrapper; // generic wrapper to contain two tasks contexts


namespace AppParser {


	typedef enum : uint32_t
	{
		MOVE_FWD = 0,
		MOVE_BWD,
		MOVE_RIGHT_FWD, // minimum turning radius tbd
		MOVE_LEFT_FWD,
		MOVE_RIGHT_BWD,
		MOVE_LEFT_BWD,
		MOVE_HALT,
		MOVE_T2_S180R,
		MOVE_T2_S90R,
		MOVE_T2_O1, // move around 1st obstacle
	} MOTION_CMD;


	typedef struct
	{
		MOTION_CMD cmd;
		uint32_t arg;
		bool is_absol;
		bool turn_opt;
		bool is_crawl;
		bool linear;
	} MOTION_PKT_t; // struct containing msg to send to motioncontroller queue


	typedef struct
	{
		uint8_t buffer[10];
		uint32_t length;
	} AppMessage_t ; // struct containing dma rxbuf captured by listener() within buf full cb fn
	typedef uint8_t BUF_CMP_t; // buf type

	class Listener
	{
		public:
			Listener(u_ctx *ctx);
			volatile void invoke(); // called within isr to memcpy rxbuf into queue
			~Listener( void );
		private:
			u_ctx *ctx;
	};

	class Processor
	{

	private:
	// static all to workaround the queue in instance error
		static void startImpl(void * _this); //unused
		template <typename T> // no type bounds enforcement, must be uint8_t, char etc.
		static bool isEq(const T &a, const T &b)
		{
			return a == b;
		}
		static MOTION_PKT_t *getMotionCmdFromBytes(BUF_CMP_t *bytes); // handle motion request and give the MOTION_PKT_t back
		static void returnSensorRequestCmd(BUF_CMP_t id); // handle the sensor request and return from ctx

		static bool _signal_obstr;
		static bool _obstr_txed;

	public:
		Processor(u_ctx *rx_ctx, u_ctx *tx_ctx);

		void start();
		~Processor( void );
		u_ctx *this_ctx; // context of this process
		u_ctx *o_ctx; // context of the other process, i.e. the motion controller.
		static void processorTask(void *pv);
		static void signalObstruction(void) { _signal_obstr = true; }
		static void signalNoObstruction(void) { _signal_obstr = false; _obstr_txed = false; }


	};

	/* PUBLIC CONSTANTS FOR UART CMDS AND APPLICATION-LAYER CMDS RESPECTIVELY */

	static constexpr char* ack = (char *)"ack";
	static constexpr char* nack = (char *)"nack";
	static constexpr char* obstr = (char *)"obst";
	//index 0
	static constexpr BUF_CMP_t START_CHAR = 'x';

	// index 1
	static constexpr BUF_CMP_t CMD_CHAR = 'c';
	static constexpr BUF_CMP_t REQ_CHAR = 'q';


	// index 2
	static constexpr BUF_CMP_t MOTOR_CHAR = 'm';
	static constexpr BUF_CMP_t SENSOR_CHAR = 's';
	static constexpr BUF_CMP_t AUX_CHAR = 'a';

	// index 3 iff [2] is MOTOR_CHAR
	static constexpr BUF_CMP_t FWD_CHAR = 'f';
	static constexpr BUF_CMP_t BWD_CHAR = 'b';
	static constexpr BUF_CMP_t RIGHT_CHAR = 'r';
	static constexpr BUF_CMP_t LEFT_CHAR = 'l';
	static constexpr BUF_CMP_t HALT_CHAR = 'h';

	static constexpr BUF_CMP_t CRAWL_CHAR = 'd';
	static constexpr BUF_CMP_t LINEAR_CHAR = 'j';

	static constexpr BUF_CMP_t IR_L_CHAR = 'w';
	static constexpr BUF_CMP_t IR_R_CHAR = 'e';
	static constexpr BUF_CMP_t USOUND_CHAR = 'u';
	static constexpr BUF_CMP_t GY_Z_CHAR = 'g'; // others are probably useless..
	static constexpr BUF_CMP_t QTRN_YAW_CHAR = 'y';
	static constexpr BUF_CMP_t QTRN_ALL_CHAR = 'k';

	static constexpr BUF_CMP_t LAST_HALT_CHAR = 'o';
	static constexpr BUF_CMP_t T2_180R_CHAR = '!';
	static constexpr BUF_CMP_t T2_90R_CHAR = '?';
	static constexpr BUF_CMP_t T2_O1_CHAR = '@';

	static constexpr BUF_CMP_t END_CHAR = 'z';



}




#endif /* INC_APP_PARSER_H_ */
