# SX1231_STM32_Driver

## 说明

SX1231无线模块的驱动程序，目前用于STM32平台，基于LL库

## API

### Typedef

#### SX1231_Typedef

用于储存SX1231模块配置的结构体，可由SX1231_Typedef_Init()函数初始化此结构体为上电默认值，再将需要修改的值进行修改，完成后调用SX1231_Init()函数，对SX1231模块进行初始化

```c
typedef struct
{
	SPI_TypeDef* SPI_Port;
	GPIO_TypeDef* NSS_GPIO_Port;
	uint32_t NSS_GPIO_Pin;
	uint8_t DMA_Enable;

	struct
	{
		uint8_t SequencerOff;
		uint8_t ListenOn;
		uint8_t Mode;
		uint8_t DataMode;
		uint8_t ModulationType;
		uint8_t ModulationShaping;
		uint32_t BitRate;
		uint32_t Fdev;
		uint32_t Frf;
		uint8_t LowBatOn;
		uint8_t LowBatTrim;
		uint8_t ListenResol;
		uint8_t ListenCriteria;
		uint8_t ListenEnd;
		uint8_t ListenCoefIdle;
		uint8_t ListenCoefRx;
	} Common;

	struct
	{
		uint8_t PaOn;
		uint8_t OutputPower;
		uint8_t PaRamp;
		uint8_t OcpOn;
		uint8_t OcpTrim;
	} Transmitter;

	struct
	{
		uint8_t AgcAutoReferenceOn;
		uint8_t AgcReferenceLevel;
		uint8_t AgcSnrMargin;
		uint8_t AgcStep[5];
		uint8_t LnaZin;
		uint8_t LnaLowPowerOn;
		uint8_t LnaGainSelect;
		uint8_t DccFreq;
		uint8_t RxBwMant;
		uint8_t RxBwExp;
		uint8_t DccFreqAfc;
		uint8_t RxBwMantAfc;
		uint8_t RxBwExpAfc;
		uint8_t OokThreshType;
		uint8_t OokPeakTheshStep;
		uint8_t OokPeakThreshDec;
		uint8_t OokAverageThreshFilt;
		uint8_t OokFixedThresh;
		uint8_t AfcAutoclearOn;
		uint8_t AfcAutoOn;
		uint16_t AfcValue;
		uint16_t FeiValue;
		uint8_t FastRx;
	} Receiver;

	struct
	{
		uint8_t DioMapping[6];
		uint8_t ClkOut;
		uint8_t RssiThreshold;
		uint8_t TimeoutRxStart;
		uint8_t TimeoutRssiThresh;
	} IRQ_PinMapping;

	struct
	{
		uint16_t PreambleSize;
		uint8_t SyncOn;
		uint8_t FifoFillCondition;
		uint8_t SyncSize;
		uint8_t SyncTol;
		uint8_t SyncValue[8];
		uint8_t PacketFormat;
		uint8_t DcFree;
		uint8_t CrcOn;
		uint8_t CrcAutoClearOff;
		uint8_t AddressFiltering;
		uint8_t PayloadLength;
		uint8_t NodeAddress;
		uint8_t BroadcastAddress;
		uint8_t IntermediateEnterCondition;
		uint8_t IntermediateExitCondition;
		uint8_t IntermediateMode;
		uint8_t TxStartCondition;
		uint8_t FifoThreshold;
		uint8_t InterPacketRxDelay;
		uint8_t RestartRx;
		uint8_t AutoRxRestartOn;
		uint8_t AesOn;
		uint8_t AesKey[16];
	} PacketEngine;

	struct
	{
		uint8_t AdcLowPowerOn;
	} TempSensor;

	struct
	{
		uint8_t SensitivityBoost;
		uint8_t OokDeltaThreshold;
	} Test;

} SX1231_Typedef;

```

##### SPI_TypeDef* SPI_Port;

​	SX1231模块的SPI端口

##### GPIO_TypeDef* NSS_GPIO_Port;

​	SX1231模块NSS引脚所在GPIO组

##### uint32_t NSS_GPIO_Pin;

​	SX1231模块NSS引脚所在GPIO引脚

##### uint8_t DMA_Enable;

​	是否使用DMA传输。由于时间原因DMA传输并未进行实现，在后续版本中更新



### API列表

#### void SX1231_Typedef_Init(SX1231_Typedef* SX1231)

##### 功能

​		初始化SX1231的配置结构体

##### 参数

###### 	SX1231_Typedef* SX1231

​		未经过初始化的SX1231配置结构体

##### 返回值

​	无



#### void SX1231_Init(SX1231_Typedef* SX1231)

##### 功能

​		根据SX1231配置结构体内容初始化SX1231模块

##### 参数

###### 	SX1231_Typedef* SX1231

​		SX1231的定义结构体，包含SX1231的配置参数

##### 返回值

​	无



#### void SX1231_WriteFifo(SX1231_Typedef* SX1231, uint8_t bfr[], uint32_t length)

##### 功能

​		向SX1231的FIFO中写数据

##### 参数

###### 	SX1231_Typedef* SX1231

​		SX1231的定义结构体，包含SX1231的配置参数

###### uint8_t bfr[]

​		读写缓冲区，用于存放FIFO数据

###### uint32_t length

​		FIFO读写字节数目

##### 返回值

​	无



#### void SX1231_ReadFifo(SX1231_Typedef* SX1231, uint8_t bfr[], uint32_t length)

##### 功能

​		从SX1231的FIFO中读数据

##### 参数

###### 	SX1231_Typedef* SX1231

​		SX1231的定义结构体，包含SX1231的配置参数

###### uint8_t bfr[]

​		读写缓冲区，用于存放FIFO数据

###### uint32_t length

​		FIFO读写字节数目

##### 返回值

​	无



#### void SX1231_ClearFifo(SX1231_Typedef* SX1231)

##### 功能

​		清空SX1231的FIFO

##### 参数

###### 	SX1231_Typedef* SX1231

​		SX1231的定义结构体，包含SX1231的配置参数

##### 返回值

​	无



#### 配置寄存器操作

由于精力有限，无法对SX1231的每个寄存器进行详细的讲解。此处的函数分为三类，SX1231_Get为读取对应配置位的内容，SX1231_Set为向对应配置位写入参数，SX1231_IsActiveFlag则为判断相应的标志位是否有效

```c
uint8_t SX1231_GetSensitivityBoost(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTempValue(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTempAdcLowPowerOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTempMeasRunning(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAesOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAutoRxRestartOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetInterPacketRxDelay(SX1231_Typedef* SX1231);
uint8_t SX1231_GetFifoThreshold(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTxStartCondition(SX1231_Typedef* SX1231);
uint8_t SX1231_GetIntermediateMode(SX1231_Typedef* SX1231);
uint8_t SX1231_GetIntermediateExitCondition(SX1231_Typedef* SX1231);
uint8_t SX1231_GetIntermediateEnterCondition(SX1231_Typedef* SX1231);
uint8_t SX1231_GetBroadcastAddress(SX1231_Typedef* SX1231);
uint8_t SX1231_GetNodeAddress(SX1231_Typedef* SX1231);
uint8_t SX1231_GetPayloadLength(SX1231_Typedef* SX1231);
uint8_t SX1231_GetPacketFormat(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDcFree(SX1231_Typedef* SX1231);
uint8_t SX1231_GetCrcOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetCrcAutoClearOff(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAddressFiltering(SX1231_Typedef* SX1231);
uint8_t SX1231_GetSyncTol(SX1231_Typedef* SX1231);
uint8_t SX1231_GetSyncSize(SX1231_Typedef* SX1231);
uint8_t SX1231_GetFifoFillCondition(SX1231_Typedef* SX1231);
uint8_t SX1231_GetSyncOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTimeoutRssiThresh(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTimeoutRxStart(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRssiThreshold(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_FifoFull(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_FifoNotEmpty(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_FifoLevel(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_FifoOverrun(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_PacketSent(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_PayloadReady(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_CrcOk(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_LowBat(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_ModeReady(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_RxReady(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_TxReady(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_PllLock(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_Rssi(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_Timeout(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_AutoMode(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_SyncAddressMatch(SX1231_Typedef* SX1231);
uint8_t SX1231_GetClkOut(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDioMapping(SX1231_Typedef* SX1231, uint8_t DioID);
uint8_t SX1231_GetRssiValue(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRssiDone(SX1231_Typedef* SX1231);
uint8_t SX1231_GetFastRx(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAfcAutoOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAfcAutoclearOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAfcClear(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAfcDone(SX1231_Typedef* SX1231);
uint8_t SX1231_GetFeiDone(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokFixedThresh(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokAverageThreshFilt(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokPeakThreshDec(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokPeakTheshStep(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokThreshType(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRxBwExpAfc(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRxBwMantAfc(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDccFreqAfc(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRxBwExp(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRxBwMant(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDccFreq(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLnaCurrentGain(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLnaGainSelect(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLnaLowPowerOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLnaZin(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAgcSnrMargin(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAgcReferenceLevel(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAgcAutoReferenceOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOcpTrim(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOcpOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetPaRamp(SX1231_Typedef* SX1231);
uint8_t SX1231_GetPaOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOutputPower(SX1231_Typedef* SX1231);
uint8_t SX1231_GetVersion(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenCoefRx(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenCoefIdle(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenEnd(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenCriteria(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenResol(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLowBatTrim(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLowBatOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLowBatMonitor(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRcCalDone(SX1231_Typedef* SX1231);
uint8_t SX1231_GetModulationShaping(SX1231_Typedef* SX1231);
uint8_t SX1231_GetModulationType(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDataMode(SX1231_Typedef* SX1231);
uint8_t SX1231_GetSequencerOff(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetMode(SX1231_Typedef* SX1231);
uint16_t SX1231_GetPreambleSize(SX1231_Typedef* SX1231);
uint16_t SX1231_GetFeiValue(SX1231_Typedef* SX1231);
uint16_t SX1231_GetAfcValue(SX1231_Typedef* SX1231);
void SX1231_GetAgcStep(SX1231_Typedef* SX1231, uint8_t AgcStep[5]);
void SX1231_SetSensitivityBoost(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetTempAdcLowPowerOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetTempMeasStart(SX1231_Typedef* SX1231);
void SX1231_SetAesKey(SX1231_Typedef* SX1231, uint8_t AesKey[16]);
void SX1231_SetAesOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAutoRxRestartOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRestartRx(SX1231_Typedef* SX1231);
void SX1231_SetInterPacketRxDelay(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetFifoThreshold(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetTxStartCondition(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetIntermediateMode(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetIntermediateExitCondition(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetIntermediateEnterCondition(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetBroadcastAddress(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetNodeAddress(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetPayloadLength(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetPacketFormat(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDcFree(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetCrcOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetCrcAutoClearOff(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAddressFiltering(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_GetSyncValue(SX1231_Typedef* SX1231, uint8_t SyncValue[8]);
void SX1231_SetSyncValue(SX1231_Typedef* SX1231, uint8_t SyncValue[8]);
void SX1231_SetSyncTol(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetSyncSize(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetFifoFillCondition(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetSyncOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetPreambleSize(SX1231_Typedef* SX1231, uint16_t val);
void SX1231_SetTimeoutRssiThresh(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetTimeoutRxStart(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRssiThreshold(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetClkOut(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDioMapping(SX1231_Typedef* SX1231, uint8_t DioID, uint8_t Mapping);
void SX1231_SetRssiStart(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetFastRx(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetFeiValue(SX1231_Typedef* SX1231, uint16_t val);
void SX1231_SetAfcValue(SX1231_Typedef* SX1231, uint16_t val);
void SX1231_SetAfcAutoOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAfcAutoclearOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAfcStart(SX1231_Typedef* SX1231l);
void SX1231_SetFeiStart(SX1231_Typedef* SX1231);
void SX1231_SetOokFixedThresh(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokAverageThreshFilt(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokPeakThreshDec(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokPeakTheshStep(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokThreshType(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRxBwExpAfc(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRxBwMantAfc(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDccFreqAfc(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRxBwExp(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRxBwMant(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDccFreq(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLnaGainSelect(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLnaLowPowerOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLnaZin(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokDeltaThreshold(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAgcStep(SX1231_Typedef* SX1231, uint8_t AgcStep[5]);
void SX1231_SetAgcSnrMargin(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAgcReferenceLevel(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAgcAutoReferenceOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOcpTrim(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOcpOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetPaRamp(SX1231_Typedef* SX1231, uint8_t mask);
void SX1231_SetPaOn(SX1231_Typedef* SX1231, uint8_t mask);
void SX1231_SetOutputPower(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenCoefRx(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenCoefIdle(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenEnd(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenCriteria(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenResol(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLowBatTrim(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLowBatOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRcCalStart(SX1231_Typedef* SX1231);
void SX1231_SetCarrierFreq(SX1231_Typedef* SX1231, uint32_t freq);
void SX1231_SetFreqDev(SX1231_Typedef* SX1231, uint32_t FreqDev);
void SX1231_SetBitRate(SX1231_Typedef* SX1231, uint32_t bitRate);
void SX1231_SetModulationShaping(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetModulationType(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDataMode(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenAbort(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetSequencerOff(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetMode(SX1231_Typedef* SX1231, uint8_t val);
uint32_t SX1231_GetFreqDev(SX1231_Typedef* SX1231);
uint32_t SX1231_GetBitRate(SX1231_Typedef* SX1231);
uint32_t SX1231_GetCarrierFreq(SX1231_Typedef* SX1231);
```

### 宏定义

此处的宏定义为按照手册对相应寄存器设置值进行定义得到的，主要目的是方便配置对应的寄存器

例如在使用 SX1231_SetModulationType(SX1231_Typedef* SX1231, uint8_t val)时，可选的val有SX1231_MODULATION_TYPE_FSK与SX1231_MODULATION_TYPE_OOK，请配合适当的编辑器的代码补全与提示进行操作，能够显著提高效率

```c
#define SX1231_MODE_SLEEP 0x0
#define SX1231_MODE_STDBY 0x1
#define SX1231_MODE_FS 0x2
#define SX1231_MODE_TX 0x3
#define SX1231_MODE_RX 0x4

#define SX1231_DATA_MODE_PKT 0
#define SX1231_DATA_MODE_CONT_W_SYN 2
#define SX1231_DATA_MODE_CONT_WO_SYN 3

#define SX1231_MODULATION_TYPE_FSK 0
#define SX1231_MODULATION_TYPE_OOK 1

#define SX1231_MODULATION_SHAPING_NO 0
#define SX1231_MODULATION_SHAPING_GAUSS_FIL_BT_1_0 1
#define SX1231_MODULATION_SHAPING_GAUSS_FIL_BT_0_5 2
#define SX1231_MODULATION_SHAPING_GAUSS_FIL_BT_0_3 3
#define SX1231_MODULATION_SHAPING_FIL_BR 1
#define SX1231_MODULATION_SHAPING_FIL_2BR 2

#define SX1231_RC_CAL_IN_PROGRESS 0
#define SX1231_RC_CAL_DONE 1

#define SX1231_LOW_BAT_ON 1
#define SX1231_LOW_BAT_OFF 0

#define SX1231_LOW_BAT_TH_1V695 0x0
#define SX1231_LOW_BAT_TH_1V764 0x1
#define SX1231_LOW_BAT_TH_1V835 0x2
#define SX1231_LOW_BAT_TH_1V905 0x3
#define SX1231_LOW_BAT_TH_1V976 0x4
#define SX1231_LOW_BAT_TH_2V045 0x5
#define SX1231_LOW_BAT_TH_2V116 0x6
#define SX1231_LOW_BAT_TH_2V185 0x7

#define SX1231_LISTEN_RESOLUTION_64us 0x5
#define SX1231_LISTEN_RESOLUTION_4_1ms 0xa
#define SX1231_LISTEN_RESOLUTION_262ms 0xf

#define SX1231_LISTEN_CRITERIA_SIGNAL_STRENGTH_ABOVE_RSSI_THRESHOLD 0
#define SX1231_LISTEN_CRITERIA_SIGNAL_STRENGTH_ABOVE_RSSI_THRESHOLD_AND_SYNC_ADDRESS_MATCHED 1

#define SX1231_LISTEN_END_STAY_IN_RX_AND_LISTEN_STOPS_AND_MUST_BE_DISABLED 0
#define SX1231_LISTEN_END_STAY_IN_RX_UNTIL_INTERRUPT 1
#define SX1231_LISTEN_END_STAY_IN_RX_UNTIL_INTERRUPT_THEN_RESUME 2

#define SX1231_PA0_ON 0x04
#define SX1231_PA1_ON 0x02
#define SX1231_PA2_ON 0x01

#define SX1231_PA_RAMP_3_4_ms 0x0
#define SX1231_PA_RAMP_2_ms 0x1
#define SX1231_PA_RAMP_1_ms 0x2
#define SX1231_PA_RAMP_500_us 0x3
#define SX1231_PA_RAMP_250_us 0x4
#define SX1231_PA_RAMP_125_us 0x5
#define SX1231_PA_RAMP_100_us 0x6
#define SX1231_PA_RAMP_62_us 0x7
#define SX1231_PA_RAMP_50_us 0x8
#define SX1231_PA_RAMP_40_us 0x9
#define SX1231_PA_RAMP_31_us 0xa
#define SX1231_PA_RAMP_25_us 0xb
#define SX1231_PA_RAMP_20_us 0xc
#define SX1231_PA_RAMP_15_us 0xd
#define SX1231_PA_RAMP_12_us 0xe
#define SX1231_PA_RAMP_10_us 0xf

#define SX1231_OCP_ENABLE 1
#define SX1231_OCP_DISABLE 0

#define SX1231_AGC_REFERENCE_FORCED 0
#define SX1231_AGC_REFERENCE_AUTO 1

#define SX1231_LNA_ZIN_50_OHMS 0
#define SX1231_LNA_ZIN_200_OHMS 1

#define SX1231_LNA_LOW_POWER_NORMAL_MODE 0
#define SX1231_LNA_LOW_POWER_MODE_ENABLED 1

#define SX1231_LNA_GAIN_SET_BY_AGC 0x0
#define SX1231_LNA_GAIN_G1 0x1
#define SX1231_LNA_GAIN_G2 0x2
#define SX1231_LNA_GAIN_G3 0x3
#define SX1231_LNA_GAIN_G4 0x4
#define SX1231_LNA_GAIN_G5 0x5
#define SX1231_LNA_GAIN_G6 0x6
#define SX1231_LNA_GAIN_G7 0x7

#define SX1231_RX_BW_MANT_16 0x0
#define SX1231_RX_BW_MANT_20 0x1
#define SX1231_RX_BW_MANT_24 0x2

#define SX1231_OOK_THRESH_FIXED 0x0
#define SX1231_OOK_THRESH_PEAK 0x1
#define SX1231_OOK_THRESH_AVERAGE 0x2

#define SX1231_OOK_PEAK_THRESH_STEP_0_5_dB 0x0
#define SX1231_OOK_PEAK_THRESH_STEP_1_0_dB 0x1
#define SX1231_OOK_PEAK_THRESH_STEP_1_5_dB 0x2
#define SX1231_OOK_PEAK_THRESH_STEP_2_0_dB 0x3
#define SX1231_OOK_PEAK_THRESH_STEP_3_0_dB 0x4
#define SX1231_OOK_PEAK_THRESH_STEP_4_0_dB 0x5
#define SX1231_OOK_PEAK_THRESH_STEP_5_0_dB 0x6
#define SX1231_OOK_PEAK_THRESH_STEP_6_0_dB 0x7

#define SX1231_OOK_PEAK_THRESH_DEC_ONCE_PER_CHIP 0x0
#define SX1231_OOK_PEAK_THRESH_DEC_ONCE_EVERY_2_CHIPS 0x1
#define SX1231_OOK_PEAK_THRESH_DEC_ONCE_EVERY_4_CHIPS 0x2
#define SX1231_OOK_PEAK_THRESH_DEC_ONCE_EVERY_8_CHIPS 0x3
#define SX1231_OOK_PEAK_THRESH_DEC_TWICE_IN_EACH_CHIP 0x4
#define SX1231_OOK_PEAK_THRESH_DEC_4_TIMES_IN_EACH_CHIP 0x5
#define SX1231_OOK_PEAK_THRESH_DEC_8_TIMES_IN_EACH_CHIP 0x6
#define SX1231_OOK_PEAK_THRESH_DEC_16_TIMES_IN_EACH_CHIP 0x7

#define SX1231_FEI_ONGOING 0
#define SX1231_FEI_FINISHED 1

#define SX1231_AFC_ONGOING 0
#define SX1231_AFC_FINISHED 1

#define SX1231_AFC_AUTOCLEAR_DISABLED 0
#define SX1231_AFC_AUTOCLEAR_ENABLED 1

#define SX1231_AFC_AUTO_DISABLED 0
#define SX1231_AFC_AUTO_ENABLED 1

#define SX1231_FAST_RX_2_RSSI_SAMPLE 0
#define SX1231_FAST_RX_1_RSSI_SAMPLE 1

#define SX1231_RSSI_ONGOING 0
#define SX1231_RSSI_FINISHED 1

#define SX1231_FIFO_FILL_CONDITION_SYNC_ADDRESS_INTERRUPT 0
#define SX1231_FIFO_FILL_CONDITION_FIFO_FILL_CONDITION_IS_SET 1

#define SX1231_PACKET_FORMAT_FIXED_LENGTH 0
#define SX1231_PACKET_FORMAT_VARIABLE_LENGTH 1

#define SX1231_DC_FREE_OFF 0
#define SX1231_DC_FREE_MANCHESTER 1
#define SX1231_DC_FREE_WHITENING 2

#define SX1231_CRC_OFF 0
#define SX1231_CRC_ON 1

#define SX1231_CRC_AUTO_CLEAR_OFF 0
#define SX1231_CRC_AUTO_CLEAR_ON 1

#define SX1231_ADDRESS_FILTERING_NONE 0
#define SX1231_ADDRESS_FILTERING_MATCH_NODE_ADDRESS 1
#define SX1231_ADDRESS_FILTERING_MATCH_NODE_OR_BROADCAST_ADDRESS 1

#define SX1231_INTERMEDIATE_ENTER_CONDITION_NONE 0x0
#define SX1231_INTERMEDIATE_ENTER_CONDITION_FIFO_NOT_EMPTY_SET 0x1
#define SX1231_INTERMEDIATE_ENTER_CONDITION_FIFO_LEVEL_SET 0x2
#define SX1231_INTERMEDIATE_ENTER_CONDITION_CRC_OK_SET 0x3
#define SX1231_INTERMEDIATE_ENTER_CONDITION_PAY_LOAD_READY_SET 0x4
#define SX1231_INTERMEDIATE_ENTER_CONDITION_SYNC_ADDRESS_SET 0x5
#define SX1231_INTERMEDIATE_ENTER_CONDITION_PACKET_SENT_SET 0x6
#define SX1231_INTERMEDIATE_ENTER_CONDITION_FIFO_NOT_EMPTY_RESET 0x7

#define SX1231_INTERMEDIATE_EXIT_CONDITION_NONE 0x0
#define SX1231_INTERMEDIATE_EXIT_CONDITION_FIFO_NOT_EMPTY_RESET 0x1
#define SX1231_INTERMEDIATE_EXIT_CONDITION_FIFO_LEVEL_OR_TIMEOUT_SET 0x2
#define SX1231_INTERMEDIATE_EXIT_CONDITION_CRC_OK_OR_TIMEOUT_SET 0x3
#define SX1231_INTERMEDIATE_EXIT_CONDITION_PAY_LOAD_READY_OR_TIMEOUT_SET 0x4
#define SX1231_INTERMEDIATE_EXIT_CONDITION_SYNC_ADDRESS_OR_TIMEOUT_SET 0x5
#define SX1231_INTERMEDIATE_EXIT_CONDITION_PACKET_SENT_SET 0x6
#define SX1231_INTERMEDIATE_EXIT_CONDITION_TIMEOUT_SET 0x7

#define SX1231_INTERMEDIATE_MODE_SLEEP 0x0
#define SX1231_INTERMEDIATE_MODE_STAND_BY 0x1
#define SX1231_INTERMEDIATE_MODE_RECEIVER 0x2
#define SX1231_INTERMEDIATE_MODE_TRANSMITTER 0x3

#define SX1231_TX_START_CONDITION_FIFO_LEVEL 0x0
#define SX1231_TX_START_CONDITION_FIFI_NOT_EMPTY 0x1

#define SX1231_AUTO_RX_RESTART_ENABLED 1
#define SX1231_AUTO_RX_RESTART_DISABLED 0

#define SX1231_AES_ENABLED 1
#define SX1231_AES_DISABLED 0

#define SX1231_TEMP_ADC_LOW_POWER_ENABLED 1
#define SX1231_TEMP_ADC_LOW_POWER_DISABLED 0

#define SX1231_SENSITIVITY_BOOST_NORMAL 0x1B
#define SX1231_SENSITIVITY_BOOST_HIGH_SENSITIVITY 0x2D

#define SX1231_SYNC_ENABLED 1
#define SX1231_SYNC_DISABLED 0

#define SX1231_SEQUENCER_OFF_ENABLED 1
#define SX1231_SEQUENCER_OFF_DISABLED 0

#define SX1231_LISTEN_ENABLED 1;
#define SX1231_LISTEN_DISABLED 0;
```



## 移植

#### 需要实现的函数

##### __STATIC_INLINE void SX1231_SPI_Tx(SX1231_Typedef* SX1231, uint8_t dat);

##### 功能

​	通过SPI接口向SX1231发送8位数据

##### 参数

###### 	SX1231_Typedef* SX1231

​		SX1231的定义结构体，包含SX1231的配置参数

###### uint8_t dat

​		要发送的数据

##### 返回值

​	无



##### __STATIC_INLINE uint8_t SX1231_SPI_Rx(SX1231_Typedef* SX1231);

##### 功能

​	通过SPI从SX1231接收8位数据

##### 参数

###### 	SX1231_Typedef* SX1231

​		SX1231的定义结构体，包含SX1231的配置参数

##### 返回值

​	接收到的数据



##### __STATIC_INLINE void SX1231_SPI_Enable(SX1231_Typedef* SX1231);

##### 功能

​	使能SX1231的SPI

##### 参数

###### 	SX1231_Typedef* SX1231

​		SX1231的定义结构体，包含SX1231的配置参数

##### 返回值

​	无

##### __STATIC_INLINE void SX1231_SPI_Disable(SX1231_Typedef* SX1231);

##### 功能

​	失能SX1231的SPI

##### 参数

###### 	SX1231_Typedef* SX1231

​		SX1231的定义结构体，包含SX1231的配置参数

##### 返回值

​	无

## 示例

接收端和发送端均提供了示例代码(SX1231_APP)

### 接收端main调用

初始化完成后采用轮询方式查询模块是否收到数据包，当接收有效时进行相关操作

```c
#include "SX1231_APP.h"

void main()
{
	uint8_t rx_Valid;
	uint32_t invalid_cnt = 0;
	uint8_t bfr[SX1231_PayloadLength];

	SPIx_Init();  //初始化无线模块之前应当初始化SPI接口
	SX1231_Init_Secondary();  //SX1231从机初始化

	while (!SX1231_IsActiveFlag_ModeReady(&SX1231_Secondary))  //等待SX1231就绪
	{
		printf("wait");
	}
    
	while(1)  //主循环
	{
		rx_Valid = 0;  //先将接受有效标志置0
		if(SX1231_Secondary_ReceivePacket(bfr))  //接收有效
			if(bfr[0] == 0x23 && bfr[1] == 0x33)  //此处为自定义的数据包前缀，非必须
			{
				rx_Valid = 1;  //接收有效标注置1
				invalid_cnt = 0;  //无效接收次数清零
				//数据有效，此处进行接收到数据的操作
	    		}

		if(! rx_Valid)  //当一次轮询的接收无效时
		{
			invalid_cnt++;   //无效次数增加
			if(invalid_cnt > 20000)  //当无效接受次数大于某一值时，可能是发送端已经关闭
			{
				invalid_cnt = 20000+1;  //始终维持在接受无效状态，直到再次接收有效
				//此处进行长时间接收无效的操作
			}
		}

	}
}
```

### 发送端main调用

注意发送函数是阻塞形式的

```c
#include "SX1231_APP.h"

void main()
{
	uint8_t bfr[SX1231_PayloadLength];

	SPIx_Init();  //初始化无线模块之前应当初始化SPI接口
	SX1231_Init_Primary();  //SX1231主机初始化

	while (!SX1231_IsActiveFlag_ModeReady(&SX1231_Primary))  //等待SX1231就绪
	{
		printf("wait");
	}

	while(1)  //主循环
	{
		//应先将要发送的数据写入bfr中
		SX1231_Primary_TransmitPacket(bfr);  //发送数据
	}
}
```

