# M031BSP_UART_PDMA_TIMEOUT_RS485
 M031BSP_UART_PDMA_TIMEOUT_RS485

update @ 2023/07/07

1. use UART 2 + PDMA channel 0(TX) / 1(RX) , with PDMA time out to recieve RS485 RX data

2. press digit 1 , to send PDMA TX data ( 64 bytes )

- data buffer 0 / 1 with data header 0x5A

- data bufer 2 / 3 with data 0x55 , 0x66 

- data buffer 61 is CRC8 data

- data buffer 62/63 with data tail 0xA5

3. below is PDMA TX data log (left : MCU send PDMA TX , right : 2nd RS485 receive data)

![image](https://github.com/released/M031BSP_UART_PDMA_TIMEOUT_RS485/blob/main/uart_pdma_tx.jpg)

below is PDMA RX data log ( left MCU recieve RX by PMDA timeout , right : 2nd RS485 send data)

![image](https://github.com/released/M031BSP_UART_PDMA_TIMEOUT_RS485/blob/main/uart_pdma_rx.jpg)

below is RS485 schematic 

![image](https://github.com/released/M031BSP_UART_PDMA_TIMEOUT_RS485/blob/main/schematic_RS485.jpg)


