#ifndef __NETWORK_H__
#define __NETWORK_H__

#include <stdint.h>
#include <stdbool.h>
#include "wizchip_conf.h"
#include "socket.h"

#define SOCKN 0
#define CONTROLLER
#ifdef CONTROLLER
  #define SPI_SENSOR1_CSN_SELECT() HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, GPIO_PIN_RESET)
  #define SPI_SENSOR2_CSN_SELECT() NULL
  #define SPI_SENSOR3_CSN_SELECT() NULL
  #define SPI_SENSOR4_CSN_SELECT() NULL
  #define SPI_MB_CSN_SELECT() HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET)

  #define SPI_SENSOR1_CSN_DEELECT() HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, GPIO_PIN_SET)
  #define SPI_SENSOR2_CSN_DEELECT() NULL
  #define SPI_SENSOR3_CSN_DEELECT() NULL
  #define SPI_SENSOR4_CSN_DEELECT() NULL
  #define SPI_MB_CSN_DELECT() HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET)

  #define SPI_SENSOR1_READ_BUF(pu8Buf, u16Size) HAL_SPI_Receive(&hspi3, pu8Buf, u16Size, HAL_MAX_DELAY)
  #define SPI_SENSOR2_READ_BUF(pu8Buf, u16Size) NULL
  #define SPI_SENSOR3_READ_BUF(pu8Buf, u16Size) NULL
  #define SPI_SENSOR4_READ_BUF(pu8Buf, u16Size) NULL
  #define SPI_MB_READ_BUF(pu8Buf, u16Size) HAL_SPI_Receive(&hspi1, pu8Buf, u16Size, HAL_MAX_DELAY)

  #define SPI_SENSOR1_WRITE_BUF(pu8Buf, u16Size) HAL_SPI_Transmit(&hspi3, pu8Buf, u16Size, HAL_MAX_DELAY)
  #define SPI_SENSOR2_WRITE_BUF(pu8Buf, u16Size) NULL
  #define SPI_SENSOR3_WRITE_BUF(pu8Buf, u16Size) NULL
  #define SPI_SENSOR4_WRITE_BUF(pu8Buf, u16Size) NULL
  #define SPI_MB_WRITE_BUF(pu8Buf, u16Size) HAL_SPI_Transmit(&hspi1, pu8Buf, u16Size, HAL_MAX_DELAY)

  #define SPI_SENSOR1_RSTN_LOW() HAL_GPIO_WritePin(SPI3_RSTN_GPIO_Port, SPI3_RSTN_Pin, GPIO_PIN_RESET)
  #define SPI_SENSOR2_RSTN_LOW() NULL
  #define SPI_SENSOR3_RSTN_LOW() NULL
  #define SPI_SENSOR4_RSTN_LOW() NULL
  #define SPI_MB_RSTN_LOW() HAL_GPIO_WritePin(SPI1_RSTN_GPIO_Port, SPI1_RSTN_Pin, GPIO_PIN_RESET)

  #define SPI_SENSOR1_RSTN_HIGH() HAL_GPIO_WritePin(SPI3_RSTN_GPIO_Port, SPI3_RSTN_Pin, GPIO_PIN_SET)
  #define SPI_SENSOR2_RSTN_HIGH() NULL
  #define SPI_SENSOR3_RSTN_HIGH() NULL
  #define SPI_SENSOR4_RSTN_HIGH() NULL
  #define SPI_MB_RSTN_HIGH() HAL_GPIO_WritePin(SPI1_RSTN_GPIO_Port, SPI1_RSTN_Pin, GPIO_PIN_SET)
#else
  #define SPI_SENSOR1_CSN_SELECT() HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, GPIO_PIN_RESET)
  #define SPI_SENSOR2_CSN_SELECT() NULL
  #define SPI_SENSOR3_CSN_SELECT() NULL
  #define SPI_SENSOR4_CSN_SELECT() NULL
  #define SPI_MB_CSN_SELECT() NULL

  #define SPI_SENSOR1_CSN_DEELECT() HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, GPIO_PIN_SET)
  #define SPI_SENSOR2_CSN_DEELECT() NULL
  #define SPI_SENSOR3_CSN_DEELECT() NULL
  #define SPI_SENSOR4_CSN_DEELECT() NULL
  #define SPI_MB_CSN_DELECT() NULL

  #define SPI_SENSOR1_READ_BUF(pu8Buf, u16Size) HAL_SPI_Receive(&hspi3, pu8Buf, u16Size, HAL_MAX_DELAY)
  #define SPI_SENSOR2_READ_BUF(pu8Buf, u16Size) NULL
  #define SPI_SENSOR3_READ_BUF(pu8Buf, u16Size) NULL
  #define SPI_SENSOR4_READ_BUF(pu8Buf, u16Size) NULL
  #define SPI_MB_READ_BUF(pu8Buf, u16Size) NULL

  #define SPI_SENSOR1_WRITE_BUF(pu8Buf, u16Size) HAL_SPI_Transmit(&hspi3, pu8Buf, u16Size, HAL_MAX_DELAY)
  #define SPI_SENSOR2_WRITE_BUF(pu8Buf, u16Size) NULL
  #define SPI_SENSOR3_WRITE_BUF(pu8Buf, u16Size) NULL
  #define SPI_SENSOR4_WRITE_BUF(pu8Buf, u16Size) NULL
  #define SPI_MB_WRITE_BUF(pu8Buf, u16Size) NULL

  #define SPI_SENSOR1_RSTN_LOW() HAL_GPIO_WritePin(SPI3_RSTN_GPIO_Port, SPI3_RSTN_Pin, GPIO_PIN_RESET)
  #define SPI_SENSOR2_RSTN_LOW() NULL
  #define SPI_SENSOR3_RSTN_LOW() NULL
  #define SPI_SENSOR4_RSTN_LOW() NULL
  #define SPI_MB_RSTN_LOW() NULL

  #define SPI_SENSOR1_RSTN_HIGH() HAL_GPIO_WritePin(SPI3_RSTN_GPIO_Port, SPI3_RSTN_Pin, GPIO_PIN_SET)
  #define SPI_SENSOR2_RSTN_HIGH() NULL
  #define SPI_SENSOR3_RSTN_HIGH() NULL
  #define SPI_SENSOR4_RSTN_HIGH() NULL
  #define SPI_MB_RSTN_HIGH() NULL
#endif
/* Exported enum & structure */
typedef enum
{
  W5500SPI_NONE,
  W5500SPI1,
  W5500SPI2,
  W5500SPI3,
  W5500SPI4,
  W5500SPIMBTCP
}W5500SPI_TypeDef;

typedef enum
{
  CLOSED = 0x00,      //SOCK_CLOSED
  INIT = 0x13,        //SOCK_INIT
  LISTEN = 0x14,      //SOCK_LISTEN
  ESTABLISHED = 0x17, //SOCK_ESTABLISHED
  CLOSE_WAIT = 0x1C   //SOCK_CLOSE_WAIT
}SocketState_TypeDef;

typedef enum
{
  NOERR,
  INITIAL_ERR,
  PHY_DISCON,
  SOCK_CLOSE,
  SOCK_ERR,
  PTR_BOUNDS_ERR
}W5500TcpSocketErr_TypeDef;

typedef struct
{
  W5500SPI_TypeDef eSpiPort;
  wiz_NetInfo stWizNetinfo;
  uint16_t u16Port;
  volatile bool bIsPhyConnected;
  SocketState_TypeDef eSockState;
  bool bIsSocketConnected;
  bool bIsSocketTxEnable;
  bool bIsSocketTxSent;
  bool bIsSocketRxEnable;
  bool bIsSocketRxReceived;
  uint8_t* pu8TxData;
  uint16_t u16TxSize;
  uint8_t* pu8RxData;
  uint16_t u16RxSize;
}W5500TcpSocket_TypeDef;

extern W5500TcpSocket_TypeDef hW55001;
extern W5500TcpSocket_TypeDef hW5500MBTCP;

/* Exported functions prototypes */
/* W5500 TCP Socket HAL Functions */
W5500TcpSocketErr_TypeDef eW5500Init(void);
W5500TcpSocketErr_TypeDef eW5500SPI_TCPSocket_Init(void);

void vW5500SetNetInfo(W5500TcpSocket_TypeDef* pstW5500TcpSocketToSet, wiz_NetInfo* pstNetinfoToSet, uint16_t u16PortToSet);
void vSetCurSpiPort(W5500SPI_TypeDef stSpiPortToSet);
W5500SPI_TypeDef eGetCurSpiPort(void);
void vReleaseSocket(void);
W5500TcpSocketErr_TypeDef eControllerTCPStatePoll(W5500TcpSocket_TypeDef* stControllerW5500TcpSocket);
W5500TcpSocketErr_TypeDef eW5500Tcp_Transmit(W5500TcpSocket_TypeDef* pstW5500TcpSocketToUse, uint8_t *pData, uint16_t u16Size);
W5500TcpSocketErr_TypeDef eW5500Tcp_Receive(W5500TcpSocket_TypeDef* pstW5500TcpSocketToUse, uint8_t *pData, uint16_t u16Size);

#ifdef CONTROLLER
W5500TcpSocketErr_TypeDef eW5500SPIMBTCP_TCPSocket_Init(void);
//implement in FortAPI/freemodbus/porttcp.c
void vModbusTCPServerPoll(W5500TcpSocket_TypeDef* stMBW5500TcpSocket);
#endif

#endif /* __NETWORK_H__ */
