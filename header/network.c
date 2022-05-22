/**
  ***************************************************************************************
  * @file     network.c
  * @version  1.0
  * @brief    wiznet W5500 communication interface callback function implementation
  *           and tcp socket connection interface
  * @change   DATE        VERSION        DETAIL
  *           20210830    1.0            JackZhang: basic architecture implementation
  ***************************************************************************************
  */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "cmsis_armcc.h"
#include "spi.h"
#include "network.h"

#include "debug.h"
W5500Debugger_TypeDef stControllerW5500Debugger;

W5500TcpSocket_TypeDef hW55001;
//determine which spi port to use
volatile static W5500SPI_TypeDef gs_eCurW5500SPI = W5500SPI_NONE;

#define TCP_BUF_SIZE 2048
uint8_t au8TcpReceiveBuffer[TCP_BUF_SIZE];
uint16_t u16TcpReceiveSize;
uint8_t au8TcpTransmitBuffer[TCP_BUF_SIZE];
uint16_t u16TcpTransmitSize;

//default initial settings
static const uint16_t u16DefaultPort = 99;

//TODO: check if each mac address is unique in lan
//mac head {0x00, 0x08, 0xdc} belongs to wiznet
static const wiz_NetInfo stControllerNetInfo =
    {
        .mac = {0x00, 0x08, 0xdc, 0x00, 0x00, 0x00},
        .ip = {192, 168, 1, 101},
        .sn = {255, 255, 255, 0},
        .gw = {192, 168, 1, 1},
        .dns = {0, 0, 0, 0},
        .dhcp = NETINFO_STATIC};

static const wiz_NetInfo stSensorNetInfo =
    {
        .mac = {0x00, 0x08, 0xdc, 0x00, 0x00, 0x01},
        .ip = {192, 168, 1, 102},
        .sn = {255, 255, 255, 0},
        .gw = {192, 168, 1, 1},
        .dns = {0, 0, 0, 0},
        .dhcp = NETINFO_STATIC};

/* W5500 SPI Callback Functions */
static void vW5500_Select(void);
static void vW5500_Deselect(void);
static void vW5500_HardwareReset(void);
static void vSPI_CrisEnter(void);
static void vSPI_CrisExit(void);
static uint8_t u8W5500_Readbyte(void);
static void vW5500_WriteByte(uint8_t data);
static void vW5500_ReadBuf(uint8_t *buff, uint16_t len);
static void vW5500_WriteBuf(uint8_t *buff, uint16_t len);
static void vW5500IntEventHandler(W5500TcpSocket_TypeDef *stControllerW5500TcpSocket);

/**
  * @brief  spi chip selection callback
  * @param  void
  * @retval void
  */
static void vW5500_Select(void)
{
  switch (eGetCurSpiPort())
  {
  case W5500SPI_NONE:
    break;

  case W5500SPI1:
    SPI_SENSOR1_CSN_SELECT();
    break;
  case W5500SPI2:
    SPI_SENSOR2_CSN_SELECT();
    break;
  case W5500SPI3:
    SPI_SENSOR3_CSN_SELECT();
    break;
  case W5500SPI4:
    SPI_SENSOR4_CSN_SELECT();
    break;

  case W5500SPIMBTCP:
    SPI_MB_CSN_SELECT();
    break;

  default:
    break;
  }
}

/**
  * @brief  spi chip deselection callback
  * @param  void
  * @retval void
  */
static void vW5500_Deselect(void)
{
  switch (eGetCurSpiPort())
  {
  case W5500SPI_NONE:
    break;

  case W5500SPI1:
    SPI_SENSOR1_CSN_DEELECT();
    break;
  case W5500SPI2:
    SPI_SENSOR2_CSN_DEELECT();
    break;
  case W5500SPI3:
    SPI_SENSOR3_CSN_DEELECT();
    break;
  case W5500SPI4:
    SPI_SENSOR4_CSN_DEELECT();
    break;

  case W5500SPIMBTCP:
    SPI_MB_CSN_DELECT();
    break;

  default:
    break;
  }
}

/**
  * @brief  w5500 chip reset function
  * @param  void
  * @retval void
  */
static void vW5500_HardwareReset(void)
{
  SPI_SENSOR1_RSTN_LOW();
  SPI_SENSOR2_RSTN_LOW();
  SPI_SENSOR3_RSTN_LOW();
  SPI_SENSOR4_RSTN_LOW();
  SPI_MB_RSTN_LOW();

  HAL_Delay(1);

  SPI_SENSOR1_RSTN_HIGH();
  SPI_SENSOR2_RSTN_HIGH();
  SPI_SENSOR3_RSTN_HIGH();
  SPI_SENSOR4_RSTN_HIGH();
  SPI_MB_RSTN_HIGH();
}

/**
  * @brief  spi enter critical section callback
  * @param  void
  * @retval void
  */
static void vSPI_CrisEnter(void)
{
//  __set_PRIMASK(1);
}

/**
  * @brief  spi exit critical section callback
  * @param  void
  * @retval void
  */
static void vSPI_CrisExit(void)
{
//  __set_PRIMASK(0);
}

/**
  * @brief  spi read multi bytes callback
  * @param  void
  * @retval void
  */
static void vW5500_ReadBuf(uint8_t *pu8Buf, uint16_t u16Size)
{
  switch (eGetCurSpiPort())
  {
  case W5500SPI_NONE:
    break;

  case W5500SPI1:
    SPI_SENSOR1_READ_BUF(pu8Buf, u16Size);
    break;
  case W5500SPI2:
    SPI_SENSOR2_READ_BUF(pu8Buf, u16Size);
    break;
  case W5500SPI3:
    SPI_SENSOR3_READ_BUF(pu8Buf, u16Size);
    break;
  case W5500SPI4:
    SPI_SENSOR4_READ_BUF(pu8Buf, u16Size);
    break;

  case W5500SPIMBTCP:
    SPI_MB_READ_BUF(pu8Buf, u16Size);
    break;

  default:
    break;
  }
}

/**
  * @brief  spi write multi bytes callback
  * @param  void
  * @retval void
  */
static void vW5500_WriteBuf(uint8_t *pu8Buf, uint16_t u16Size)
{
  switch (eGetCurSpiPort())
  {
  case W5500SPI_NONE:
    break;

  case W5500SPI1:
    SPI_SENSOR1_WRITE_BUF(pu8Buf, u16Size);
    break;
  case W5500SPI2:
    SPI_SENSOR2_WRITE_BUF(pu8Buf, u16Size);
    break;
  case W5500SPI3:
    SPI_SENSOR3_WRITE_BUF(pu8Buf, u16Size);
    break;
  case W5500SPI4:
    SPI_SENSOR4_WRITE_BUF(pu8Buf, u16Size);
    break;

  case W5500SPIMBTCP:
    SPI_MB_WRITE_BUF(pu8Buf, u16Size);
    break;

  default:
    break;
  }
}

/**
  * @brief  spi read byte callback
  * @param  void
  * @retval void
  */
static uint8_t u8W5500_Readbyte(void)
{
  uint8_t data = 0;
  vW5500_ReadBuf(&data, 1);
  return data;
}

/**
  * @brief  spi write byte callback
  * @param  void
  * @retval void
  */
static void vW5500_WriteByte(uint8_t u8Data)
{
  vW5500_WriteBuf(&u8Data, 1);
  return;
}

/**
  * @brief  tcp socket connection initial function
  * @param  void
  * @retval void
  */
W5500TcpSocketErr_TypeDef eW5500Init(void)
{
  W5500TcpSocketErr_TypeDef stError = NOERR;

  /* Critical section callback */
  reg_wizchip_cris_cbfunc(vSPI_CrisEnter, vSPI_CrisExit);

  /* SPI Chip selection call back */
  reg_wizchip_cs_cbfunc(vW5500_Select, vW5500_Deselect);

  /* SPI Read & Write callback function */
  reg_wizchip_spi_cbfunc(u8W5500_Readbyte, vW5500_WriteByte);
  reg_wizchip_spiburst_cbfunc(vW5500_ReadBuf, vW5500_WriteBuf);

  vW5500_HardwareReset();
  stError = eW5500SPI_TCPSocket_Init();
  if (stError == INITIAL_ERR)
  {
    return stError;
  }
#ifdef CONTROLLER
  eW5500SPIMBTCP_TCPSocket_Init();
#endif
  return stError;
}

/**
  * @brief  tcp socket connection sub initial function
  * @param  void
  * @retval void
  */
W5500TcpSocketErr_TypeDef eW5500SPI_TCPSocket_Init(void)
{
  W5500TcpSocketErr_TypeDef stError = NOERR;

  vSetCurSpiPort(W5500SPI1);

  hW55001.eSpiPort = W5500SPI1;
#ifdef CONTROLLER
  hW55001.stWizNetinfo = stControllerNetInfo;
#else
  hW55001.stWizNetinfo = stSensorNetInfo;
#endif
  hW55001.u16Port = u16DefaultPort;
  hW55001.bIsPhyConnected = false;
  hW55001.eSockState = CLOSED;
  hW55001.bIsSocketConnected = false;
  hW55001.bIsSocketTxSent = false;
  hW55001.bIsSocketRxReceived = false;

  wiz_NetTimeout timeout;
  //retry times
  timeout.retry_cnt = 5;
  //timeout value 2000 * 100us = 200ms, totally 5*200ms = 1s
  timeout.time_100us = 2000;
  wizchip_settimeout(&timeout);

  // WIZCHIP SOCKET Buffer initialize
  uint8_t u8SocketBufSize[2][8] = {{16, 0, 0, 0, 0, 0, 0, 0},
                                   {16, 0, 0, 0, 0, 0, 0, 0}};
  wizchip_init((uint8_t *)u8SocketBufSize, (uint8_t *)u8SocketBufSize + _WIZCHIP_SOCK_NUM_);

  wizchip_setnetinfo(&hW55001.stWizNetinfo);
  wiz_NetInfo ref;
  wizchip_getnetinfo(&ref);
  if (memcmp(&ref, &hW55001.stWizNetinfo, sizeof(wiz_NetInfo)) != 0)
  {
    //TODO: use another indication to check if initialize error occur.
    hW55001.eSpiPort = W5500SPI_NONE;
    stError = INITIAL_ERR;
  }

  //TODO: test, sendto DISCARD_PORT 9, for arp table
  uint8_t broadcastIp[] = {255, 255, 255, 255};
  uint8_t discardport = 9;
  sendto(SOCKN, (uint8_t *)"0", sizeof("0"), broadcastIp, discardport);
  close(SOCKN);

  //enable socket 0 Sn_IR interrupt
//  setSIMR(0x1);

  return stError;
}

/**
  * @brief  tcp socket connection set net configuration
  * @param  pstW5500TcpSocketToSet: which w5500 chip to set
  *         pstNetinfoToSet: net config to set
  *         u16PortToSet: using port
  * @retval void
  */
void vW5500SetNetInfo(W5500TcpSocket_TypeDef *pstW5500TcpSocketToSet, wiz_NetInfo *pstNetinfoToSet, uint16_t u16PortToSet)
{
  vSetCurSpiPort(pstW5500TcpSocketToSet->eSpiPort);

  disconnect(SOCKN);

  pstW5500TcpSocketToSet->stWizNetinfo = *pstNetinfoToSet;
  pstW5500TcpSocketToSet->u16Port = u16PortToSet;
  pstW5500TcpSocketToSet->eSockState = CLOSED;
  pstW5500TcpSocketToSet->bIsSocketConnected = false;
  pstW5500TcpSocketToSet->bIsSocketTxEnable = false;
  pstW5500TcpSocketToSet->bIsSocketTxSent = false;
  pstW5500TcpSocketToSet->bIsSocketRxEnable = false;
  pstW5500TcpSocketToSet->bIsSocketRxReceived = false;

  wizchip_setnetinfo(pstNetinfoToSet);
  wizchip_sw_reset();
}

/**
  * @brief  tcp socket connection set using spi port of current target w5500 chip
  * @param  stSpiPortToSet: which spi port of w5500 chip to use
  * @retval void
  */
void vSetCurSpiPort(W5500SPI_TypeDef stSpiPortToSet)
{
  gs_eCurW5500SPI = stSpiPortToSet;
}

/**
  * @brief  tcp socket connection get using spi port of current target w5500 chip
  * @param  void
  * @retval W5500SPI_TypeDef: which spi port of w5500 chip is using
  */
W5500SPI_TypeDef eGetCurSpiPort(void)
{
  return gs_eCurW5500SPI;
}

/**
  * @brief  tcp socket connection release socket
  * @param  void
  * @retval void
  */
void vReleaseSocket(void)
{
  disconnect(SOCKN);
  //softwarereset
  wizchip_sw_reset();
}

/**
  * @brief  w5500 interupt event handler
  * @param  W5500TcpSocket_TypeDef*, which to operate
  * @retval void
  */
static void vW5500IntEventHandler(W5500TcpSocket_TypeDef *pstControllerW5500TcpSocket)
{
  uint8_t u8EventIR = getIR();
  uint8_t u8EventSIR = getSIR();
  uint8_t u8EventSn_IR = getSn_IR(SOCKN);

  if (u8EventSn_IR & Sn_IR_SENDOK)
  {
    setSn_IR(SOCKN, Sn_IR_SENDOK);
  }
  if (u8EventSn_IR & Sn_IR_TIMEOUT)
  {
    setSn_IR(SOCKN, Sn_IR_TIMEOUT);
  }
  if (u8EventSn_IR & Sn_IR_RECV)
  {
    setSn_IR(SOCKN, Sn_IR_RECV);
  }
  if (u8EventSn_IR & Sn_IR_DISCON)
  {
    setSn_IR(SOCKN, Sn_IR_DISCON);
  }
  if (u8EventSn_IR & Sn_IR_CON)
  {
    setSn_IR(SOCKN, Sn_IR_CON);
  }
}

/**
  * @brief  tcp socket connection maintain polling function
  * @param  stControllerW5500TcpSocket: a set of configuration of controller-sensor
  *                                     tcp socket connection, controller side
  * @retval W5500TcpSocketErr_TypeDef: error state of tcp socket connection
  */
W5500TcpSocketErr_TypeDef
eControllerTCPStatePoll(W5500TcpSocket_TypeDef *stControllerW5500TcpSocket)
{
  W5500TcpSocketErr_TypeDef stErrReturn = SOCK_CLOSE;

  if (stControllerW5500TcpSocket->eSpiPort == W5500SPI_NONE)
  {
    stErrReturn = INITIAL_ERR;
    eW5500SPI_TCPSocket_Init();
  }

  vSetCurSpiPort(stControllerW5500TcpSocket->eSpiPort);

  //TODO: for debug use, remember to comment out when finish debugging
  vW5500Debugger(&stControllerW5500Debugger, eGetCurSpiPort());

  //check phy connection
  stControllerW5500TcpSocket->bIsPhyConnected = (bool)(getPHYCFGR() & PHYCFGR_LNK_ON);
  if (!(stControllerW5500TcpSocket->bIsPhyConnected))
  {
    if (stControllerW5500TcpSocket->bIsSocketConnected)
    {
      vReleaseSocket();
      stControllerW5500TcpSocket->bIsSocketConnected = false;
      stErrReturn = PHY_DISCON;
    }
  }

  //get tcp socket state
  stControllerW5500TcpSocket->eSockState = (SocketState_TypeDef)getSn_SR(SOCKN);
  switch (stControllerW5500TcpSocket->eSockState)
  {
  case SOCK_CLOSED:
    socket(SOCKN, Sn_MR_TCP, stControllerW5500TcpSocket->u16Port, SF_TCP_NODELAY | SF_IO_NONBLOCK);
    stControllerW5500TcpSocket->bIsSocketConnected = false;
    if (!stControllerW5500TcpSocket->bIsPhyConnected)
    {
      break;
    }
  case SOCK_INIT:
#ifdef CONTROLLER
    connect(SOCKN, (uint8_t *)&stSensorNetInfo.ip, stControllerW5500TcpSocket->u16Port);
#else
    listen(SOCKN);
#endif
    break;
  case SOCK_LISTEN:
    break;
  case SOCK_ESTABLISHED:
    stControllerW5500TcpSocket->bIsSocketConnected = true;
    stErrReturn = NOERR;
    //rx
    while(getSn_RX_RSR(SOCKN) > 0)
    {
      if(stControllerW5500TcpSocket->bIsSocketRxEnable)
      {
        recv(SOCKN, stControllerW5500TcpSocket->pu8RxData, stControllerW5500TcpSocket->u16RxSize);
        stControllerW5500TcpSocket->bIsSocketRxReceived = true;
        stControllerW5500TcpSocket->bIsSocketRxEnable = false;
      }
      else
      {
        stControllerW5500TcpSocket->pu8RxData = au8TcpReceiveBuffer;
        stControllerW5500TcpSocket->u16RxSize = TCP_BUF_SIZE;
        recv(SOCKN, stControllerW5500TcpSocket->pu8RxData, TCP_BUF_SIZE);
      }
      if (getSn_IR(SOCKN) & Sn_IR_RECV)
      {
        setSn_IR(SOCKN, Sn_IR_RECV);
      }
    }
    break;
  case SOCK_CLOSE_WAIT:
    disconnect(SOCKN);
    break;
  default:
    break;
  } //switch end

  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SPI3_INTN_GPIO_Port, SPI3_INTN_Pin))
  {
    vW5500IntEventHandler(stControllerW5500TcpSocket);
  }

  vW5500Debugger(&stControllerW5500Debugger, eGetCurSpiPort());

  return stErrReturn;
}

/**
  * @brief  tcp socket connection data transmit interface
  * @param  pstW5500TcpSocketToUse: using handle of tcp socket connection
  *         pData: pointer to data buffer
  *         u16Size: size of data to transfer
  * @retval W5500TcpSocketErr_TypeDef: error state of tcp socket connection
  */
W5500TcpSocketErr_TypeDef
eW5500Tcp_Transmit(W5500TcpSocket_TypeDef *pstW5500TcpSocketToUse, uint8_t *pData, uint16_t u16Size)
{
  vSetCurSpiPort(pstW5500TcpSocketToUse->eSpiPort);

  W5500TcpSocketErr_TypeDef stErrReturn = NOERR;

  if ((pData == NULL) || (u16Size <= 0))
  {
    stErrReturn = PTR_BOUNDS_ERR;
    return stErrReturn;
  }

  if(SOCK_ERROR >= send(SOCKN, pData, u16Size))
  {
    stErrReturn = SOCK_ERR;
    return stErrReturn;
  }

  pstW5500TcpSocketToUse->bIsSocketTxSent = true;

  return stErrReturn;
}

/**
  * @brief  tcp socket connection data receive interface
  * @param  pstW5500TcpSocketToUse: using handle of tcp socket connection
  *         pData: pointer to data buffer
  *         u16Size: size of data to transfer
  * @retval W5500TcpSocketErr_TypeDef: error state of tcp socket connection
  */
W5500TcpSocketErr_TypeDef
eW5500Tcp_Receive(W5500TcpSocket_TypeDef *pstW5500TcpSocketToUse, uint8_t *pData, uint16_t u16Size)
{
  vSetCurSpiPort(pstW5500TcpSocketToUse->eSpiPort);

  W5500TcpSocketErr_TypeDef stErrReturn = NOERR;

  if ((pData == NULL) || (u16Size == 0))
  {
    stErrReturn = PTR_BOUNDS_ERR;
    return stErrReturn;
  }

  pstW5500TcpSocketToUse->eSockState = (SocketState_TypeDef)getSn_SR(SOCKN);
  if (pstW5500TcpSocketToUse->eSockState != SOCK_ESTABLISHED)
  {
    stErrReturn = SOCK_ERR;
    return stErrReturn;
  }

  pstW5500TcpSocketToUse->pu8RxData = pData;
  pstW5500TcpSocketToUse->u16RxSize = u16Size;
  pstW5500TcpSocketToUse->bIsSocketRxEnable = true;

  return stErrReturn;
}
