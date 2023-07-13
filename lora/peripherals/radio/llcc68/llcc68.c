/*!
 * \file      LLCC68.c
 *
 * \brief     LLCC68 driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <string.h>
#include "radio.h"
// #include "delay.h"
#include "LLCC68.h"
#include "LLCC68-board.h"

/*!
 * \brief Internal frequency of the radio
 */
#define LLCC68_XTAL_FREQ                            32000000UL

/*!
 * \brief Scaling factor used to perform fixed-point operations
 */
#define LLCC68_PLL_STEP_SHIFT_AMOUNT                ( 14 )

/*!
 * \brief PLL step - scaled with LLCC68_PLL_STEP_SHIFT_AMOUNT
 */
#define LLCC68_PLL_STEP_SCALED                      ( LLCC68_XTAL_FREQ >> ( 25 - LLCC68_PLL_STEP_SHIFT_AMOUNT ) )

/*!
 * \brief Maximum value for parameter symbNum in \ref LLCC68SetLoRaSymbNumTimeout
 */
#define LLCC68_MAX_LORA_SYMB_NUM_TIMEOUT            248

/*!
 * \brief Radio registers definition
 */
typedef struct
{
    uint16_t      Addr;                             //!< The address of the register
    uint8_t       Value;                            //!< The value of the register
}RadioRegisters_t;

/*!
 * \brief Stores the current packet type set in the radio
 */
static RadioPacketTypes_t PacketType;

/*!
 * \brief Stores the current packet header type set in the radio
 */
static volatile RadioLoRaPacketLengthsMode_t LoRaHeaderType;

/*!
 * \brief Stores the last frequency error measured on LoRa received packet
 */
volatile uint32_t FrequencyError = 0;

/*!
 * \brief Hold the status of the Image calibration
 */
static bool ImageCalibrated = false;

/*!
 * \brief Get the number of PLL steps for a given frequency in Hertz
 *
 * \param [in] freqInHz Frequency in Hertz
 *
 * \returns Number of PLL steps
 */
static uint32_t LLCC68ConvertFreqInHzToPllStep( uint32_t freqInHz );

/*
 * LLCC68 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void LLCC68OnDioIrq( void );

/*!
 * \brief DIO 0 IRQ callback
 */
void LLCC68SetPollingMode( void );

/*!
 * \brief DIO 0 IRQ callback
 */
void LLCC68SetInterruptMode( void );

/*
 * \brief Process the IRQ if handled by the driver
 */
void LLCC68ProcessIrqs( void );

void LLCC68Init( DioIrqHandler dioIrq )
{
    LLCC68Reset( );

    LLCC68IoIrqInit( dioIrq );

    LLCC68Wakeup( );
    LLCC68SetStandby( STDBY_RC );

    // Initialize TCXO control
    LLCC68IoTcxoInit( );

    // Initialize RF switch control
    LLCC68IoRfSwitchInit( );

    LLCC68SetOperatingMode( MODE_STDBY_RC );
}

void LLCC68CheckDeviceReady( void )
{
    if( ( LLCC68GetOperatingMode( ) == MODE_SLEEP ) || ( LLCC68GetOperatingMode( ) == MODE_RX_DC ) )
    {
        LLCC68Wakeup( );
        // Switch is turned off when device is in sleep mode and turned on is all other modes
        LLCC68AntSwOn( );
    }
    LLCC68WaitOnBusy( );
}

void LLCC68SetPayload( uint8_t *payload, uint8_t size )
{
    LLCC68WriteBuffer( 0x00, payload, size );
}

uint8_t LLCC68GetPayload( uint8_t *buffer, uint8_t *size,  uint8_t maxSize )
{
    uint8_t offset = 0;

    LLCC68GetRxBufferStatus( size, &offset );
    if( *size > maxSize )
    {
        return 1;
    }
    LLCC68ReadBuffer( offset, buffer, *size );
    return 0;
}

void LLCC68SendPayload( uint8_t *payload, uint8_t size, uint32_t timeout )
{
    LLCC68SetPayload( payload, size );
    LLCC68SetTx( timeout );
}

uint8_t LLCC68SetSyncWord( uint8_t *syncWord )
{
    LLCC68WriteRegisters( REG_LR_SYNCWORDBASEADDRESS, syncWord, 8 );
    return 0;
}

void LLCC68SetCrcSeed( uint16_t seed )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( seed >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( seed & 0xFF );

    switch( LLCC68GetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            LLCC68WriteRegisters( REG_LR_CRCSEEDBASEADDR, buf, 2 );
            break;

        default:
            break;
    }
}

void LLCC68SetCrcPolynomial( uint16_t polynomial )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( polynomial >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( polynomial & 0xFF );

    switch( LLCC68GetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            LLCC68WriteRegisters( REG_LR_CRCPOLYBASEADDR, buf, 2 );
            break;

        default:
            break;
    }
}

void LLCC68SetWhiteningSeed( uint16_t seed )
{
    uint8_t regValue = 0;
    
    switch( LLCC68GetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            regValue = LLCC68ReadRegister( REG_LR_WHITSEEDBASEADDR_MSB ) & 0xFE;
            regValue = ( ( seed >> 8 ) & 0x01 ) | regValue;
            LLCC68WriteRegister( REG_LR_WHITSEEDBASEADDR_MSB, regValue ); // only 1 bit.
            LLCC68WriteRegister( REG_LR_WHITSEEDBASEADDR_LSB, ( uint8_t )seed );
            break;

        default:
            break;
    }
}

uint32_t LLCC68GetRandom( void )
{
    uint32_t number = 0;
    uint8_t regAnaLna = 0;
    uint8_t regAnaMixer = 0;

    regAnaLna = LLCC68ReadRegister( REG_ANA_LNA );
    LLCC68WriteRegister( REG_ANA_LNA, regAnaLna & ~( 1 << 0 ) );

    regAnaMixer = LLCC68ReadRegister( REG_ANA_MIXER );
    LLCC68WriteRegister( REG_ANA_MIXER, regAnaMixer & ~( 1 << 7 ) );

    // Set radio in continuous reception
    LLCC68SetRx( 0xFFFFFF ); // Rx Continuous

    LLCC68ReadRegisters( RANDOM_NUMBER_GENERATORBASEADDR, ( uint8_t* )&number, 4 );

    LLCC68SetStandby( STDBY_RC );

    LLCC68WriteRegister( REG_ANA_LNA, regAnaLna );
    LLCC68WriteRegister( REG_ANA_MIXER, regAnaMixer );

    return number;
}

void LLCC68SetSleep( SleepParams_t sleepConfig )
{
    uint8_t value=0;

    LLCC68AntSwOff( );

    value = ( ( ( uint8_t )sleepConfig.Fields.WarmStart << 2 ) |
                      ( ( uint8_t )sleepConfig.Fields.Reset << 1 ) |
                      ( ( uint8_t )sleepConfig.Fields.WakeUpRTC ) );
    LLCC68WriteCommand( RADIO_SET_SLEEP, &value, 1 );
    LLCC68SetOperatingMode( MODE_SLEEP );
}

void LLCC68SetStandby( RadioStandbyModes_t standbyConfig )
{
    LLCC68WriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
    if( standbyConfig == STDBY_RC )
    {
        LLCC68SetOperatingMode( MODE_STDBY_RC );
    }
    else
    {
        LLCC68SetOperatingMode( MODE_STDBY_XOSC );
    }
}

void LLCC68SetFs( void )
{
    LLCC68WriteCommand( RADIO_SET_FS, 0, 0 );
    LLCC68SetOperatingMode( MODE_FS );
}

void LLCC68SetTx( uint32_t timeout )
{
    uint8_t buf[3];

    LLCC68SetOperatingMode( MODE_TX );

    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    LLCC68WriteCommand( RADIO_SET_TX, buf, 3 );
}

void LLCC68SetRx( uint32_t timeout )
{
    uint8_t buf[3];

    LLCC68SetOperatingMode( MODE_RX );

    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    LLCC68WriteCommand( RADIO_SET_RX, buf, 3 );
}

void LLCC68SetRxBoosted( uint32_t timeout )
{
    uint8_t buf[3];

    LLCC68SetOperatingMode( MODE_RX );

    LLCC68WriteRegister( REG_RX_GAIN, 0x96 ); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity

    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    LLCC68WriteCommand( RADIO_SET_RX, buf, 3 );
}

void LLCC68SetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime )
{
    uint8_t buf[6];

    buf[0] = ( uint8_t )( ( rxTime >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( rxTime >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( rxTime & 0xFF );
    buf[3] = ( uint8_t )( ( sleepTime >> 16 ) & 0xFF );
    buf[4] = ( uint8_t )( ( sleepTime >> 8 ) & 0xFF );
    buf[5] = ( uint8_t )( sleepTime & 0xFF );
    LLCC68WriteCommand( RADIO_SET_RXDUTYCYCLE, buf, 6 );
    LLCC68SetOperatingMode( MODE_RX_DC );
}

void LLCC68SetCad( void )
{
    LLCC68WriteCommand( RADIO_SET_CAD, 0, 0 );
    LLCC68SetOperatingMode( MODE_CAD );
}

void LLCC68SetTxContinuousWave( void )
{
    LLCC68WriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
    LLCC68SetOperatingMode( MODE_TX );
}

void LLCC68SetTxInfinitePreamble( void )
{
    LLCC68WriteCommand( RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
    LLCC68SetOperatingMode( MODE_TX );
}

void LLCC68SetStopRxTimerOnPreambleDetect( bool enable )
{
    LLCC68WriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, ( uint8_t* )&enable, 1 );
}

void LLCC68SetLoRaSymbNumTimeout( uint8_t symbNum )
{
    uint8_t mant = ( ( ( symbNum > LLCC68_MAX_LORA_SYMB_NUM_TIMEOUT ) ?
                       LLCC68_MAX_LORA_SYMB_NUM_TIMEOUT : 
                       symbNum ) + 1 ) >> 1;
    uint8_t exp  = 0;
    uint8_t reg  = 0;

    while( mant > 31 )
    {
        mant = ( mant + 3 ) >> 2;
        exp++;
    }

    reg = mant << ( 2 * exp + 1 );
    LLCC68WriteCommand( RADIO_SET_LORASYMBTIMEOUT, &reg, 1 );

    if( symbNum != 0 )
    {
        reg = exp + ( mant << 3 );
        LLCC68WriteRegister( REG_LR_SYNCH_TIMEOUT, reg );
    }
}

void LLCC68SetRegulatorMode( RadioRegulatorMode_t mode )
{
    LLCC68WriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}

void LLCC68Calibrate( CalibrationParams_t calibParam )
{
    uint8_t value = ( ( ( uint8_t )calibParam.Fields.ImgEnable << 6 ) |
                      ( ( uint8_t )calibParam.Fields.ADCBulkPEnable << 5 ) |
                      ( ( uint8_t )calibParam.Fields.ADCBulkNEnable << 4 ) |
                      ( ( uint8_t )calibParam.Fields.ADCPulseEnable << 3 ) |
                      ( ( uint8_t )calibParam.Fields.PLLEnable << 2 ) |
                      ( ( uint8_t )calibParam.Fields.RC13MEnable << 1 ) |
                      ( ( uint8_t )calibParam.Fields.RC64KEnable ) );

    LLCC68WriteCommand( RADIO_CALIBRATE, &value, 1 );
}

void LLCC68CalibrateImage( uint32_t freq )
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    LLCC68WriteCommand( RADIO_CALIBRATEIMAGE, calFreq, 2 );
}

void LLCC68SetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut )
{
    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    LLCC68WriteCommand( RADIO_SET_PACONFIG, buf, 4 );
}

void LLCC68SetRxTxFallbackMode( uint8_t fallbackMode )
{
    LLCC68WriteCommand( RADIO_SET_TXFALLBACKMODE, &fallbackMode, 1 );
}

void LLCC68SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];

    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    LLCC68WriteCommand( RADIO_CFG_DIOIRQ, buf, 8 );
}

uint16_t LLCC68GetIrqStatus( void )
{
    uint8_t irqStatus[2];

    LLCC68ReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
    return ( irqStatus[0] << 8 ) | irqStatus[1];
}

void LLCC68SetDio2AsRfSwitchCtrl( uint8_t enable )
{
    LLCC68WriteCommand( RADIO_SET_RFSWITCHMODE, &enable, 1 );
}

void LLCC68SetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout )
{
    uint8_t buf[4];

    buf[0] = tcxoVoltage & 0x07;
    buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( timeout & 0xFF );

    LLCC68WriteCommand( RADIO_SET_TCXOMODE, buf, 4 );
}

void LLCC68SetRfFrequency( uint32_t frequency )
{
    uint8_t buf[4];
	uint32_t freqInPllSteps = 0;

    if( ImageCalibrated == false )
    {
        LLCC68CalibrateImage( frequency );
        ImageCalibrated = true;
    }

    freqInPllSteps = LLCC68ConvertFreqInHzToPllStep( frequency );

    buf[0] = ( uint8_t )( ( freqInPllSteps >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freqInPllSteps >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freqInPllSteps >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freqInPllSteps & 0xFF );
    LLCC68WriteCommand( RADIO_SET_RFFREQUENCY, buf, 4 );
}

void LLCC68SetPacketType( RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    PacketType = packetType;
    LLCC68WriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
}

RadioPacketTypes_t LLCC68GetPacketType( void )
{
    return PacketType;
}

void LLCC68SetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];

    if( LLCC68GetDeviceId( ) == SX1261 )
    {
        if( power == 15 )
        {
            LLCC68SetPaConfig( 0x06, 0x00, 0x01, 0x01 );
        }
        else
        {
            LLCC68SetPaConfig( 0x04, 0x00, 0x01, 0x01 );
        }
        if( power >= 14 )
        {
            power = 14;
        }
        else if( power < -17 )
        {
            power = -17;
        }
    }
    else // sx1262
    {
        // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
        // RegTxClampConfig = @address 0x08D8
        LLCC68WriteRegister( 0x08D8, LLCC68ReadRegister( 0x08D8 ) | ( 0x0F << 1 ) );
        // WORKAROUND END

        LLCC68SetPaConfig( 0x04, 0x07, 0x00, 0x01 );
        if( power > 22 )
        {
            power = 22;
        }
        else if( power < -9 )
        {
            power = -9;
        }
    }
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    LLCC68WriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

void LLCC68SetModulationParams( ModulationParams_t *modulationParams )
{
    uint8_t n;
    uint32_t tempVal = 0;
    uint8_t buf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != modulationParams->PacketType )
    {
        LLCC68SetPacketType( modulationParams->PacketType );
    }

    switch( modulationParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        n = 8;
        tempVal = ( uint32_t )( 32 * LLCC68_XTAL_FREQ / modulationParams->Params.Gfsk.BitRate );
        buf[0] = ( tempVal >> 16 ) & 0xFF;
        buf[1] = ( tempVal >> 8 ) & 0xFF;
        buf[2] = tempVal & 0xFF;
        buf[3] = modulationParams->Params.Gfsk.ModulationShaping;
        buf[4] = modulationParams->Params.Gfsk.Bandwidth;
        tempVal = LLCC68ConvertFreqInHzToPllStep( modulationParams->Params.Gfsk.Fdev );
        buf[5] = ( tempVal >> 16 ) & 0xFF;
        buf[6] = ( tempVal >> 8 ) & 0xFF;
        buf[7] = ( tempVal& 0xFF );
        LLCC68WriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );
        break;
    case PACKET_TYPE_LORA:
        n = 4;
        buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
        buf[1] = modulationParams->Params.LoRa.Bandwidth;
        buf[2] = modulationParams->Params.LoRa.CodingRate;
        buf[3] = modulationParams->Params.LoRa.LowDatarateOptimize;

        LLCC68WriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );

        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }
}

void LLCC68SetPacketParams( PacketParams_t *packetParams )
{
    uint8_t n;
    uint8_t crcVal = 0;
    uint8_t buf[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != packetParams->PacketType )
    {
        LLCC68SetPacketType( packetParams->PacketType );
    }

    switch( packetParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        if( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_IBM )
        {
            LLCC68SetCrcSeed( CRC_IBM_SEED );
            LLCC68SetCrcPolynomial( CRC_POLYNOMIAL_IBM );
            crcVal = RADIO_CRC_2_BYTES;
        }
        else if( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_CCIT )
        {
            LLCC68SetCrcSeed( CRC_CCITT_SEED );
            LLCC68SetCrcPolynomial( CRC_POLYNOMIAL_CCITT );
            crcVal = RADIO_CRC_2_BYTES_INV;
        }
        else
        {
            crcVal = packetParams->Params.Gfsk.CrcLength;
        }
        n = 9;
        buf[0] = ( packetParams->Params.Gfsk.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.Gfsk.PreambleLength;
        buf[2] = packetParams->Params.Gfsk.PreambleMinDetect;
        buf[3] = ( packetParams->Params.Gfsk.SyncWordLength /*<< 3*/ ); // convert from byte to bit
        buf[4] = packetParams->Params.Gfsk.AddrComp;
        buf[5] = packetParams->Params.Gfsk.HeaderType;
        buf[6] = packetParams->Params.Gfsk.PayloadLength;
        buf[7] = crcVal;
        buf[8] = packetParams->Params.Gfsk.DcFree;
        break;
    case PACKET_TYPE_LORA:
        n = 6;
        buf[0] = ( packetParams->Params.LoRa.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.LoRa.PreambleLength;
        buf[2] = LoRaHeaderType = packetParams->Params.LoRa.HeaderType;
        buf[3] = packetParams->Params.LoRa.PayloadLength;
        buf[4] = packetParams->Params.LoRa.CrcMode;
        buf[5] = packetParams->Params.LoRa.InvertIQ;
        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }
    LLCC68WriteCommand( RADIO_SET_PACKETPARAMS, buf, n );
}

void LLCC68SetCadParams( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode, uint32_t cadTimeout )
{
    uint8_t buf[7];

    buf[0] = ( uint8_t )cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = ( uint8_t )cadExitMode;
    buf[4] = ( uint8_t )( ( cadTimeout >> 16 ) & 0xFF );
    buf[5] = ( uint8_t )( ( cadTimeout >> 8 ) & 0xFF );
    buf[6] = ( uint8_t )( cadTimeout & 0xFF );
    LLCC68WriteCommand( RADIO_SET_CADPARAMS, buf, 7 );
    LLCC68SetOperatingMode( MODE_CAD );
}

void LLCC68SetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    LLCC68WriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}

RadioStatus_t LLCC68GetStatus( void )
{
    uint8_t stat = 0;
    RadioStatus_t status = { 0 };
		status.Value = 0;

    stat = LLCC68ReadCommand( RADIO_GET_STATUS, NULL, 0 );
    status.Fields.CmdStatus = ( stat & ( 0x07 << 1 ) ) >> 1;
    status.Fields.ChipMode = ( stat & ( 0x07 << 4 ) ) >> 4;
    return status;
}

int8_t LLCC68GetRssiInst( void )
{
    uint8_t buf[1];
    int8_t rssi = 0;

    LLCC68ReadCommand( RADIO_GET_RSSIINST, buf, 1 );
    rssi = -buf[0] >> 1;
    return rssi;
}

void LLCC68GetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBufferPointer )
{
    uint8_t status[2];

    LLCC68ReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

    // In case of LORA fixed header, the payloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    if( ( LLCC68GetPacketType( ) == PACKET_TYPE_LORA ) && ( LoRaHeaderType == LORA_PACKET_FIXED_LENGTH ) )
    {
        *payloadLength = LLCC68ReadRegister( REG_LR_PAYLOADLENGTH );
    }
    else
    {
        *payloadLength = status[0];
    }
    *rxStartBufferPointer = status[1];
}

void LLCC68GetPacketStatus( PacketStatus_t *pktStatus )
{
    uint8_t status[3];

    LLCC68ReadCommand( RADIO_GET_PACKETSTATUS, status, 3 );

    pktStatus->packetType = LLCC68GetPacketType( );
    switch( pktStatus->packetType )
    {
        case PACKET_TYPE_GFSK:
            pktStatus->Params.Gfsk.RxStatus = status[0];
            pktStatus->Params.Gfsk.RssiSync = -status[1] >> 1;
            pktStatus->Params.Gfsk.RssiAvg = -status[2] >> 1;
            pktStatus->Params.Gfsk.FreqError = 0;
            break;

        case PACKET_TYPE_LORA:
            pktStatus->Params.LoRa.RssiPkt = -status[0] >> 1;
            // Returns SNR value [dB] rounded to the nearest integer value
            pktStatus->Params.LoRa.SnrPkt = ( ( ( int8_t )status[1] ) + 2 ) >> 2;
            pktStatus->Params.LoRa.SignalRssiPkt = -status[2] >> 1;
            pktStatus->Params.LoRa.FreqError = FrequencyError;
            break;

        default:
        case PACKET_TYPE_NONE:
            // In that specific case, we set everything in the pktStatus to zeros
            // and reset the packet type accordingly
            memset( pktStatus, 0, sizeof( PacketStatus_t ) );
            pktStatus->packetType = PACKET_TYPE_NONE;
            break;
    }
}

RadioError_t LLCC68GetDeviceErrors( void )
{
    uint8_t err[] = { 0, 0 };
    RadioError_t error = { 0 };
		error.Value = 0;

    LLCC68ReadCommand( RADIO_GET_ERROR, ( uint8_t* )err, 2 );
    error.Fields.PaRamp     = ( err[0] & ( 1 << 0 ) ) >> 0;
    error.Fields.PllLock    = ( err[1] & ( 1 << 6 ) ) >> 6;
    error.Fields.XoscStart  = ( err[1] & ( 1 << 5 ) ) >> 5;
    error.Fields.ImgCalib   = ( err[1] & ( 1 << 4 ) ) >> 4;
    error.Fields.AdcCalib   = ( err[1] & ( 1 << 3 ) ) >> 3;
    error.Fields.PllCalib   = ( err[1] & ( 1 << 2 ) ) >> 2;
    error.Fields.Rc13mCalib = ( err[1] & ( 1 << 1 ) ) >> 1;
    error.Fields.Rc64kCalib = ( err[1] & ( 1 << 0 ) ) >> 0;
    return error;
}

void LLCC68ClearDeviceErrors( void )
{
    uint8_t buf[2] = { 0x00, 0x00 };
    LLCC68WriteCommand( RADIO_CLR_ERROR, buf, 2 );
}

void LLCC68ClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    LLCC68WriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}

static uint32_t LLCC68ConvertFreqInHzToPllStep( uint32_t freqInHz )
{
    uint32_t stepsInt;
    uint32_t stepsFrac;

    // pllSteps = freqInHz / (LLCC68_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    stepsInt = freqInHz / LLCC68_PLL_STEP_SCALED;
    stepsFrac = freqInHz - ( stepsInt * LLCC68_PLL_STEP_SCALED );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( stepsInt << LLCC68_PLL_STEP_SHIFT_AMOUNT ) + 
           ( ( ( stepsFrac << LLCC68_PLL_STEP_SHIFT_AMOUNT ) + ( LLCC68_PLL_STEP_SCALED >> 1 ) ) /
             LLCC68_PLL_STEP_SCALED );
}
