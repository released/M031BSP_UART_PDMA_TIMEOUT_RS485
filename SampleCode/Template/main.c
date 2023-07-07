/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_REVERSE1                              (flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                              (flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)

struct flag_8bit flag_uart_CTL;
#define FLAG_UART_rcv_data_finish                 	    (flag_uart_CTL.bit0)
#define FLAG_UART_send_data_PDMA                        (flag_uart_CTL.bit1)
#define FLAG_UART_PDMA_finish                           (flag_uart_CTL.bit2)
#define FLAG_UART_REVERSE3                              (flag_uart_CTL.bit3)
#define FLAG_UART_REVERSE4                              (flag_uart_CTL.bit4)
#define FLAG_UART_REVERSE5                              (flag_uart_CTL.bit5)
#define FLAG_UART_REVERSE6                              (flag_uart_CTL.bit6)
#define FLAG_UART_REVERSE7                              (flag_uart_CTL.bit7)

/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;

#define RECEIVE_MODE           	 		                (0)
#define TRANSMIT_MODE           		                (1)
#define FIFO_THRESHOLD 					                (4)
#define RX_BUFFER_SIZE 					                (64)
#define RX_TIMEOUT_CNT 					                (60)

#define nRTSPin0                 		                (PA0)
#define nRTSPin1                 		                (PC3)

#define CMD_HEAD                                        (0x5A)
#define CMD_TAIL                                        (0xA5)


#define UART2_TX_DMA_CH 		                        (0)
#define UART2_RX_DMA_CH 		                        (1)
// #define UART_PDMA_OPENED_CH   	                        (1 << UART2_RX_DMA_CH)
#define UART2_PDMA_OPENED_CH_TX   		                (1 << UART2_TX_DMA_CH)
#define UART2_PDMA_OPENED_CH_RX   	                    (1 << UART2_RX_DMA_CH)

#define PDMA_TIME                                       (0x100)
// #define PDMA_GET_TRANS_CNT(pdma,u32Ch)                  ((uint32_t)((pdma->DSCT[(u32Ch)].CTL&PDMA_DSCT_CTL_TXCNT_Msk) >> PDMA_DSCT_CTL_TXCNT_Pos))

uint8_t UART2_RxBuffer[RX_BUFFER_SIZE] = {0};
uint8_t UART2_TxBuffer[RX_BUFFER_SIZE] = {0};

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        dbg_printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

void delay_ms(uint16_t ms)
{
	#if 1
    uint32_t tickstart = get_tick();
    uint32_t wait = ms;
	uint32_t tmp = 0;
	
    while (1)
    {
		if (get_tick() > tickstart)	// tickstart = 59000 , tick_counter = 60000
		{
			tmp = get_tick() - tickstart;
		}
		else // tickstart = 59000 , tick_counter = 2048
		{
			tmp = 60000 -  tickstart + get_tick();
		}		
		
		if (tmp > wait)
			break;
    }
	
	#else
	TIMER_Delay(TIMER0, 1000*ms);
	#endif
}


//
// check_reset_source
//
uint8_t check_reset_source(void)
{
    uint32_t src = SYS_GetResetSrc();

    SYS->RSTSTS |= 0x1FF;
    printf("Reset Source <0x%08X>\r\n", src);

    #if 1   //DEBUG , list reset source
    if (src & BIT0)
    {
        printf("0)POR Reset Flag\r\n");       
    }
    if (src & BIT1)
    {
        printf("1)NRESET Pin Reset Flag\r\n");       
    }
    if (src & BIT2)
    {
        printf("2)WDT Reset Flag\r\n");       
    }
    if (src & BIT3)
    {
        printf("3)LVR Reset Flag\r\n");       
    }
    if (src & BIT4)
    {
        printf("4)BOD Reset Flag\r\n");       
    }
    if (src & BIT5)
    {
        printf("5)System Reset Flag \r\n");       
    }
    if (src & BIT6)
    {
        printf("6)Reserved.\r\n");       
    }
    if (src & BIT7)
    {
        printf("7)CPU Reset Flag\r\n");       
    }
    if (src & BIT8)
    {
        printf("8)CPU Lockup Reset Flag\r\n");       
    }
    #endif

    
    if (src & SYS_RSTSTS_PORF_Msk) {
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk);
        
        printf("power on from POR\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_CPURF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPURF_Msk);

        printf("power on from CPU reset\r\n");
        return FALSE;         
    }    
    else if (src & SYS_RSTSTS_PINRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_PINRF_Msk);
        
        printf("power on from nRESET pin\r\n");
        return FALSE;
    }
    
    printf("power on from unhandle reset source\r\n");
    return FALSE;
}

unsigned char cros_crc8_arg(const unsigned char *data, int len, unsigned char previous_crc)
{
	unsigned crc = previous_crc << 8;
	int i, j;

	for (j = len; j; j--, data++) {
		crc ^= (*data << 8);
		for (i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}

	return (unsigned char)(crc >> 8);
}

unsigned char cros_crc8(const unsigned char *data, int len)
{
	return cros_crc8_arg(data, len, 0);
}

void buffer_creation(void)
{    
    unsigned char i = 0;
    unsigned short index = 0;
    unsigned char crc8 = 0;
    static unsigned char counter = 0x10;

    reset_buffer((uint8_t*)UART2_TxBuffer , 0x00,RX_BUFFER_SIZE);

    // HEAD * 2
    UART2_TxBuffer[index++] = CMD_HEAD;
    UART2_TxBuffer[index++] = CMD_HEAD;
    UART2_TxBuffer[index++] = 0x55;
    UART2_TxBuffer[index++] = 0x66;

    for (i = 4 ; i < (RX_BUFFER_SIZE - 7 + 4) ; i++ , index++)
    {
        UART2_TxBuffer[i] = i + counter;
    }

    printf("index = 0x%2X (%2d)\r\n" , index , index);

    // CRC8
    crc8 = cros_crc8((unsigned char *)UART2_TxBuffer, index);
    UART2_TxBuffer[index++] = crc8;
    printf("CRC8 = 0x%2X (%2d)\r\n" , crc8 , index);

    // TAIL * 2
    UART2_TxBuffer[index++] = CMD_TAIL;
    UART2_TxBuffer[index++] = CMD_TAIL;

    counter += 0x10;
    
	#if 1   //debug
    dump_buffer_hex(UART2_TxBuffer,RX_BUFFER_SIZE);
    #endif
}

void set_RTS_transmit(void)
{
    nRTSPin1 = TRANSMIT_MODE;
    // printf("b-t)RTS =%d\r\n" , nRTSPin1);
}

void set_RTS_receive(void)
{
    nRTSPin1 = RECEIVE_MODE;
    // printf("b-r)RTS =%d\r\n" , nRTSPin1);  
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);    

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        printf("target abort interrupt !!:\r\n");
		#if 0
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else
        
        if (PDMA_GET_ABORT_STS(PDMA) & (UART2_PDMA_OPENED_CH_RX))
        {
            printf("UART2_PDMA_OPENED_CH_RX\r\n");
            PDMA_CLR_ABORT_FLAG(PDMA, (UART2_PDMA_OPENED_CH_RX));
        }
        
		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
		#if 1

        if (PDMA_GET_TD_STS(PDMA) & UART2_PDMA_OPENED_CH_RX)
        {
            PDMA_CLR_TD_FLAG(PDMA, UART2_PDMA_OPENED_CH_RX);
        } 
		
		#else
        if((PDMA_GET_TD_STS(PDMA) & UART_PDMA_OPENED_CH) == UART_PDMA_OPENED_CH)
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, UART_PDMA_OPENED_CH);
			//insert process
			/*
                DISABLE TRIGGER
            */

        } 
		#endif
    }
    else if (status & (PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
        // printf("UART2_RX time-out !!\r\n");
        /* Update receive count */
        
        // PDMA_SET_TRANS_CNT(PDMA, UART2_RX_DMA_CH,1);    
        /* restart timeout */
        PDMA_SetTimeOut(PDMA, UART2_RX_DMA_CH, DISABLE, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA, UART2_RX_DMA_CH);
        PDMA_SetTimeOut(PDMA, UART2_RX_DMA_CH, ENABLE, PDMA_TIME); 
        FLAG_UART_rcv_data_finish = 1;     
    }    
    else
    {
        printf("unknown interrupt !!\r\n");
    }	
}

void UART2_PDMA_TIMEOUT_Init(void)
{    
    FLAG_UART_rcv_data_finish = 0;     

    SYS_ResetModule(PDMA_RST);  

    PDMA_Open(PDMA, UART2_PDMA_OPENED_CH_TX | UART2_PDMA_OPENED_CH_RX);

    PDMA_SetBurstType(PDMA,UART2_TX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[UART2_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
	UART_DISABLE_INT(UART2,UART_INTEN_TXPDMAEN_Msk);        

    PDMA_SetTransferCnt(PDMA,UART2_RX_DMA_CH, PDMA_WIDTH_8, RX_BUFFER_SIZE);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,UART2_RX_DMA_CH, UART2_BASE, PDMA_SAR_FIX, ((uint32_t) (&UART2_RxBuffer[0])), PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,UART2_RX_DMA_CH, PDMA_UART2_RX, FALSE, 0);
    /* Single request type. */
    PDMA_SetBurstType(PDMA,UART2_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA_DisableInt(PDMA,UART2_RX_DMA_CH, PDMA_INT_TEMPTY );//PDMA->DSCT[UART2_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;  

    PDMA_EnableInt(PDMA, UART2_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, UART2_RX_DMA_CH, PDMA_INT_TIMEOUT); 

    // PDMA->TOUTPSC = (PDMA->TOUTPSC & (~PDMA_TOUTPSC_TOUTPSC1_Msk)) | (0x5 << PDMA_TOUTPSC_TOUTPSC1_Pos);
    PDMA_SetTimeOut(PDMA,UART2_RX_DMA_CH, ENABLE, PDMA_TIME );     
    NVIC_EnableIRQ(PDMA_IRQn);
}

void UART2_RX_PDMA_set(void)
{
    //RX	
    PDMA_SetTransferCnt(PDMA,UART2_RX_DMA_CH, PDMA_WIDTH_8, RX_BUFFER_SIZE);
    PDMA_SetTransferAddr(PDMA,UART2_RX_DMA_CH, UART2_BASE, PDMA_SAR_FIX, ((uint32_t) (&UART2_RxBuffer[0])), PDMA_DAR_INC);	
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,UART2_RX_DMA_CH, PDMA_UART2_RX, FALSE, 0);      
}

void UART2_TX_PDMA(void)
{
	uint32_t u32RegValue = 0;
	uint32_t u32Abort = 0;	
    // static uint8_t cnt = 0;    
	uint32_t len = SIZEOF (UART2_TxBuffer);
    	
    printf("len =%d\r\n" , len);

    set_RTS_transmit();

    FLAG_UART_PDMA_finish = 0;
	
	//U
    PDMA_SetTransferCnt(PDMA,UART2_TX_DMA_CH, PDMA_WIDTH_8, len);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,UART2_TX_DMA_CH, (uint32_t) (&UART2_TxBuffer[0]), PDMA_SAR_INC, (uint32_t) UART2_BASE , PDMA_DAR_FIX);
    /* Set request source; set basic mode. */

    PDMA_SetTransferMode(PDMA,UART2_TX_DMA_CH, PDMA_UART2_TX, FALSE, 0);

	UART_ENABLE_INT(UART2,UART_INTEN_TXPDMAEN_Msk);
 
    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS(PDMA) & UART2_PDMA_OPENED_CH_TX) == UART2_PDMA_OPENED_CH_TX)
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA , UART2_PDMA_OPENED_CH_TX);

                UART_DISABLE_INT(UART2,UART_INTEN_TXPDMAEN_Msk);
                FLAG_UART_PDMA_finish = 1;
                break;              
            }
        }
        /* Check the DMA transfer abort interrupt flag */
        if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
        {
            /* Get the target abort flag */
            u32Abort = PDMA_GET_ABORT_STS(PDMA);
            /* Clear the target abort flag */
            PDMA_CLR_ABORT_FLAG(PDMA,u32Abort);
            break;
        }		
    }

    // while(!FLAG_UART_PDMA_finish);
    while(!UART_IS_TX_EMPTY(UART2));

    set_RTS_receive();
}

void RS485_sendData(UART_T *uart)
{
    uint32_t i;
	// uint32_t len = strlen((const char*)UART2_TxBuffer);
	uint32_t len = SIZEOF (UART2_TxBuffer);
    	
    printf("len =%d\r\n" , len);

    set_RTS_transmit();

    for (i = 0; i < len; i++)
    {
        while (UART_GET_TX_FULL(uart));
        UART_WRITE(uart, UART2_TxBuffer[i]);
    }
    while (UART_GET_TX_EMPTY(uart) == 0);

    set_RTS_receive();
}

void RS485_process(void)
{    
    if (FLAG_UART_send_data_PDMA)
    {
        FLAG_UART_send_data_PDMA = 0;
        
        buffer_creation();
        UART2_TX_PDMA();
    }

    if (FLAG_UART_rcv_data_finish)
    {
        FLAG_UART_rcv_data_finish = 0;

        printf("UART2_RxBuffer : \r\n");
        dump_buffer(UART2_RxBuffer, RX_BUFFER_SIZE);
        reset_buffer(UART2_RxBuffer,0x00,RX_BUFFER_SIZE);  

        UART2_RX_PDMA_set();   
        set_RTS_receive();
    }
}

void RS485_port_register(void)
{
    // PC4 : UART 2 RX 
    // PC5 : UART 2 TX 
    // PC3 : RE/DE

    GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT);
    set_RTS_receive();

    SYS_ResetModule(UART2_RST);    
    UART_Open(UART2, 115200);
    UART_ENABLE_INT(UART2,UART_INTEN_RXPDMAEN_Msk);    

    printf("UART2 init\r\n"); 
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // dbg_printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // dbg_printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PB14 ^= 1;        
    }

    RS485_process();
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		dbg_printf("invalid command\r\n");
	}
	else
	{
		dbg_printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
                FLAG_UART_send_data_PDMA = 1;
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART02_IRQHandler(void)
{   
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	

}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	dbg_printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	dbg_printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	dbg_printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	dbg_printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	dbg_printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	dbg_printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

    #if 0
    dbg_printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    dbg_printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    dbg_printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    dbg_printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    dbg_printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    dbg_printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    dbg_printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    dbg_printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | (SYS_GPB_MFPH_PB15MFP_GPIO);

    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(PDMA_MODULE);

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
	
    CLK_EnableModuleClock(UART2_MODULE);
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));

    // CLK_EnableModuleClock(TMR0_MODULE);
  	// CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk)) |
                    (SYS_GPC_MFPL_PC4MFP_UART2_RXD | SYS_GPC_MFPL_PC5MFP_UART2_TXD);


   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();
    check_reset_source();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    UART2_PDMA_TIMEOUT_Init();
    RS485_port_register();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
