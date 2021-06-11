/*
 * usb_msc.c
 *
 *  Created on: 9 июн. 2021 г.
 *      Author: sa100 (sergio.rudenko@gmail.com)
 *
 * Description: Mass Storage Device (MSC)
 *
 */

#include <string.h>

#include "usb_lib.h"
#include "hal_uid.h"


/* ---[ private macros ]--------------------------------------------------- */
//#define LOBYTE(x)  ((uint8_t)(((x) & 0x00FFU) >> 0U))
//#define HIBYTE(x)  ((uint8_t)(((x) & 0xFF00U) >> 8U))


/* ---[ private definitions ]---------------------------------------------- */
#define GET_MAX_LUN				0xFE
#define MASS_STORAGE_RESET		0xFF
#define LUN_DATA_LENGTH         1

#define BOT_CBW_SIGNATURE		0x43425355
#define BOT_CSW_SIGNATURE       0x53425355
#define BOT_CBW_PACKET_LENGTH   31
#define BOT_CSW_DATA_LENGTH		13
#define BOT_MAX_PACKET_SIZE		64


typedef enum BOT_Direction_ {
  DIR_IN		= 0,
  DIR_OUT,
  BOTH_DIR,
} BOT_Direction_t;


typedef enum SCSI_Command_ {
  SCSI_TEST_UNIT_READY			= 0x00,
  SCSI_REQUEST_SENSE        	= 0x03,
  SCSI_FORMAT_UNIT				= 0x04,
  SCSI_READ6                	= 0x08,
  SCSI_WRITE6               	= 0x0A,
  SCSI_INQUIRY              	= 0x12,
  SCSI_MODE_SELECT6         	= 0x15,
  SCSI_MODE_SENSE6          	= 0x1A,
  SCSI_START_STOP_UNIT      	= 0x1B,
  SCSI_SEND_DIAGNOSTIC      	= 0x1D,
  SCSI_ALLOW_MEDIUM_REMOVAL		= 0x1E,
  SCSI_READ_FORMAT_CAPACITIES 	= 0x23,
  SCSI_READ_CAPACITY10      	= 0x25,
  SCSI_READ10               	= 0x28,
  SCSI_WRITE10              	= 0x2A,
  SCSI_VERIFY10             	= 0x2F,
  SCSI_MODE_SELECT10        	= 0x55,
  SCSI_MODE_SENSE10         	= 0x5A,
  SCSI_READ12               	= 0xA8,
  SCSI_WRITE12              	= 0xAA,
  SCSI_VERIFY12             	= 0xAF,
  SCSI_READ16               	= 0x88,
  SCSI_WRITE16              	= 0x8A,
  SCSI_VERIFY16             	= 0x8F,
  SCSI_READ_CAPACITY16      	= 0x9E,
}
SCSI_Command_t;


typedef enum CSW_Status_ {
  CSW_CMD_PASSED 	= 0,
  CSW_CMD_FAILED,
  CSW_PHASE_ERROR,
} CSW_Status_t;


typedef enum CSW_SendPermission_ {
  SEND_CSW_DISABLE	= 0,
  SEND_CSW_ENABLE,
} CSW_SendPermission_t;


#define NO_SENSE						0
#define RECOVERED_ERROR		    		1
#define NOT_READY		        		2
#define MEDIUM_ERROR		    		3
#define HARDWARE_ERROR		    		4
#define ILLEGAL_REQUEST		    		5
#define UNIT_ATTENTION		   			6
#define DATA_PROTECT		    		7
#define BLANK_CHECK		        		8
#define VENDOR_SPECIFIC		    		9
#define COPY_ABORTED		    		10
#define ABORTED_COMMAND		    		11
#define VOLUME_OVERFLOW		    		13
#define MISCOMPARE		        		14

#define INVALID_COMMAND                 0x20
#define INVALID_FIELED_IN_COMMAND       0x24
#define PARAMETER_LIST_LENGTH_ERROR     0x1A
#define INVALID_FIELD_IN_PARAMETER_LIST 0x26
#define ADDRESS_OUT_OF_RANGE            0x21
#define MEDIUM_NOT_PRESENT 			    0x3A
#define MEDIUM_HAVE_CHANGED			    0x28


/* ---[ function prototypes ]---------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

  /* ======================================================================== */
  /* ===[ DEVICE_PROP ]====================================================== */
  /* ======================================================================== */
  static void		init_device();
  static void 	reset_device();
  static void 	process_status_in();
  static void 	process_status_out();
  static RESULT 	setup_data_class(u8 RequestNo);
  static RESULT	setup_nodata_class(u8 RequestNo);
  static RESULT	get_interface_settings(u8 Interface, u8 AlternateSetting);
  static uint8_t*	get_usb_device_descriptor(uint16_t len);
  static uint8_t*	get_usb_config_descriptor(uint16_t len);
  static uint8_t*	get_usb_string_descriptor(uint16_t len);


  /* ======================================================================== */
  /* ===[ USER_STANDARD_REQUESTS ]=========================================== */
  /* ======================================================================== */
  static void 	clear_feature();
  static void 	set_configuration();
  static void 	set_device_address();


  /* ======================================================================== */
  /* ===[ Endpoint Callback`s ]============================================== */
  /* ======================================================================== */
  static void 	EP1_IN_Callback();
  static void	EP2_OUT_Callback();


  /* ======================================================================== */
  /* ===[ Bulk Only Transport ]============================================== */
  /* ======================================================================== */
  static void 	BOT_Abort(BOT_Direction_t);
  static void 	BOT_SetCSW(CSW_Status_t, CSW_SendPermission_t);
  static void 	BOT_DecodeCBW();
  static void 	BOT_TransferDataRequest(uint8_t* data, uint16_t size);


  /* ======================================================================== */
  /* ===[ Small Computer System Interface ]================================== */
  /* ======================================================================== */
  static void 	SCSI_Valid_Cmd(uint8_t lun);
  static void 	SCSI_Invalid_Cmd(uint8_t lun);
  static void 	SCSI_Read10_Cmd(uint8_t lun , uint32_t LBA , uint32_t BlockNbr);
  static void 	SCSI_Write10_Cmd(uint8_t lun , uint32_t LBA , uint32_t BlockNbr);
  static void 	SCSI_Verify10_Cmd(uint8_t lun);
  static void 	SCSI_ModeSense6_Cmd (uint8_t lun);
  static void 	SCSI_ModeSense10_Cmd (uint8_t lun);
  static void 	SCSI_ReadCapacity10_Cmd(uint8_t lun);
  static void 	SCSI_Format_Cmd(uint8_t lun);
  static void 	SCSI_Inquiry_Cmd(uint8_t lun);
  static void 	SCSI_RequestSense_Cmd (uint8_t lun);
  static void 	SCSI_TestUnitReady_Cmd (uint8_t lun);
  static void 	SCSI_StartStopUnit_Cmd(uint8_t lun);
  static void 	SCSI_ReadFormatCapacity_Cmd(uint8_t lun);
  static bool 	SCSI_Address_Management(uint8_t lun , uint8_t Cmd , uint32_t LBA , uint32_t BlockNbr);
  /* --- Valid (Unsupported) commands -------------------------------------- */
  static void	SCSI_Prevent_Removal_Cmd(uint8_t) 	__attribute__ ((alias("SCSI_Valid_Cmd")));
  /* --- Invalid (Unsupported) commands ------------------------------------ */
  static void	SCSI_Read6_Cmd(uint8_t) 			__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Read12_Cmd(uint8_t) 			__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Read16_Cmd(uint8_t) 			__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Write6_Cmd(uint8_t) 			__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Write12_Cmd(uint8_t) 			__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Write16_Cmd(uint8_t) 			__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Verify12_Cmd(uint8_t)			__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Verify16_Cmd(uint8_t)			__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Mode_Select6_Cmd(uint8_t)		__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Mode_Select10_Cmd(uint8_t)		__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Send_Diagnostic_Cmd(uint8_t)	__attribute__ ((alias("SCSI_Invalid_Cmd")));
  static void	SCSI_Read_Capacity16_Cmd(uint8_t) 	__attribute__ ((alias("SCSI_Invalid_Cmd")));


  /* ======================================================================== */
  /* ===[ Mass Storage Device ]============================================== */
  /* ======================================================================== */
  static void	MSD_ReadMemory(uint8_t lun, uint32_t offset, uint32_t length);
  static void 	MSD_WriteMemory (uint8_t lun, uint32_t offset, uint32_t length);

#ifdef __cplusplus
}
#endif


/* ---[ private variables ]------------------------------------------------ */
static uint8_t buf[512/*FIXME!*/];
static ONE_DESCRIPTOR descriptor = { buf, 0 };

static uint8_t botData[BOT_MAX_PACKET_SIZE];
static MassStorageDevice_t MSD_ =
  {
    .State 					= UNCONNECTED,
    .maxUnitNumber 	= 1,

    .Transfer =
      {
        .State				= TXFR_IDLE,
        .data				= buf,
      },

    .BOT =
      {
        .State 				= BOT_IDLE,
        .data 				= botData,
      },
  };


/* ---[ exported variables ]----------------------------------------------- */
MassStorageDevice_t *pMSD = &MSD_;

DEVICE Device_Table =
  {
    .Total_Endpoint 				= 3, /* Number of endpoints that are used */
    .Total_Configuration 			= 1, /* Number of configuration available */
  };

DEVICE_PROP Device_Property =
  {
    .Init 							= init_device,
    .Reset							= reset_device,
    .Process_Status_IN 				= process_status_in,
    .Process_Status_OUT				= process_status_out,
    .Class_Data_Setup				= setup_data_class,
    .Class_NoData_Setup 			= setup_nodata_class,
    .Class_Get_Interface_Setting 	= get_interface_settings,
    .GetDeviceDescriptor			= get_usb_device_descriptor,
    .GetConfigDescriptor 			= get_usb_config_descriptor,
    .GetStringDescriptor			= get_usb_string_descriptor,
    .RxEP_buffer 					= NULL, /* RxEP_buffer */
    .MaxPacketSize					= 64,	/* max packet size */
  };

USER_STANDARD_REQUESTS User_Standard_Requests =
  {
    .User_GetConfiguration 			= NOP_Process,
    .User_SetConfiguration 			= set_configuration,
    .User_GetInterface 				= NOP_Process,
    .User_SetInterface				= NOP_Process,
    .User_GetStatus					= NOP_Process,
    .User_ClearFeature				= clear_feature,
    .User_SetEndPointFeature 		= NOP_Process,
    .User_SetDeviceFeature			= NOP_Process,
    .User_SetDeviceAddress			= set_device_address,
  };

void (*pEpInt_IN[7])(void) =
  {
    EP1_IN_Callback,
    NOP_Process,
    NOP_Process,
    NOP_Process,
    NOP_Process,
    NOP_Process,
    NOP_Process,
  };

void (*pEpInt_OUT[7])(void) =
  {
    NOP_Process,
    EP2_OUT_Callback,
    NOP_Process,
    NOP_Process,
    NOP_Process,
    NOP_Process,
    NOP_Process,
  };

/* ---[ imported variables ]----------------------------------------------- */
extern const 	uint8_t		USB_DeviceDescriptor[];
extern const 	uint16_t 	USB_DeviceDescriptorSize;
extern const 	uint8_t 	USB_ConfigDescriptor[];
extern const 	uint16_t 	USB_ConfigDescriptorSize;
extern const 	uint8_t 	USB_LangIdDescriptor[];
extern const 	uint16_t 	USB_LangIdDescriptorSize;
extern const 	uint8_t 	USB_VendorDescriptor[];
extern const 	uint16_t 	USB_VendorDescriptorSize;
extern const 	uint8_t 	USB_ProductDescriptor[];
extern const 	uint16_t 	USB_ProductDescriptorSize;
extern const 	uint8_t 	USB_InterfaceDescriptor[];
extern const	uint16_t 	USB_InterfaceDescriptorSize;

extern 			uint8_t 	Page00_Inquiry_Data[];
extern			uint8_t 	Standard_Inquiry_Data[];
extern			uint8_t 	Standard_Inquiry_Data2[];
extern 			uint8_t 	Mode_Sense6_data[];
extern 			uint8_t 	Mode_Sense10_data[];
extern 			uint8_t 	Scsi_Sense_Data[];
extern 			uint8_t 	ReadCapacity10_Data[];
extern 			uint8_t 	ReadFormatCapacity_Data[];

/* ------------------------------------------------------------------------ */

/**
 * common powerOn|Reset
 */
static void
initialisation()
{
  /* Disable all endpoints */
  _ClrEP_EN(
    (1 << ENDP0) |
    (1 << ENDP1) |
    (1 << ENDP2) |
    (1 << ENDP3) |
    (1 << ENDP4));

  /* Enabler USB EP0 interrupts */
  _SetEP0_INT_EN(
    EPn_INT_EN_SETUPIE |
    EPn_INT_EN_INNACKIE |
    EPn_INT_EN_OUTACKIE);	//|EPn_INT_EN_OUTNACKIE |EPn_INT_EN_INACKIE

  /* Clear USB EP0 interrupts */
  _ClrEP0_INT_STA(
    EPn_INT_STATE_SETUP |
    EPn_INT_STATE_INACK |
    EPn_INT_STATE_INNACK |
    EPn_INT_STATE_OUTACK |
    EPn_INT_STATE_OUTNACK);

  /* Enabler USB EP1 interrupts */
  _SetEP1_INT_EN(EPn_INT_EN_INNACKIE);
  _ClrEP1_INT_STA(EPn_INT_STATE_INNACK);

  /* Enabler USB EP2 interrupts */
  _SetEP2_INT_EN(EPn_INT_EN_OUTACKIE);
  _ClrEP2_INT_STA(EPn_INT_STATE_OUTACK);

  /* Enable EP0, EP1, EP2 */
  _SetEP_EN(
    (1 << ENDP0) |
    (1 << ENDP1) |
    (1 << ENDP2));

  /* Enable EP0, EP1, EP2 interrupts */
  _SetEP_INT_EN(
    (1 << ENDP0) |
    (1 << ENDP1) |
    (1 << ENDP2));
}

/**
 * Emulate USB cable connect|disconnect
 * ( 1.5K internal pull-up resistor )
 */
static void
connect_cable(FunctionalState NewState)
{
  if (NewState == ENABLE) {
    _SetUSB_TOP(USB_TOP_CONNECT);
  }
  else {
    _ClrUSB_TOP(USB_TOP_CONNECT);
  }
}


/**
 * Initialization.
 */
static void
init_device()
{
  /* for the serial number string descriptor
   * with the data from the unique ID */
  GetChipUID();

  /* hardware */
  USB_HardwareInit();

  pInformation->Current_Configuration = 0;

  connect_cable(DISABLE);	// USB connection 1.5K internal pull-up resistor

  /* USB Reset */
  _SetUSB_TOP(USB_TOP_RESET);
  _ClrUSB_TOP(USB_TOP_RESET);

  /* Clear USB interrupts */
  _ClrUSB_INT_STA(
    USB_INT_STATE_RSTF |
    USB_INT_STATE_EPINTF);	// USB_INT_STATE_SOFF|

  /* Enabler USB interrupts */
  _SetUSB_INT_EN(
    USB_INT_EN_RSTIE |
    USB_INT_EN_EPINTIE);	// USB_INT_EN_SOFIE|

  initialisation();

  connect_cable(ENABLE);	// USB connection 1.5K internal pull-up resistor

  /* USB interrupts clear states */
  _ClrUSB_INT_STA(0xff);

  pMSD->State = UNCONNECTED;
}


/**
 * Reset routine.
 */
static void
reset_device()
{
  /* Set DEVICE as not configured */
  pInformation->Current_Configuration = 0;

  /* Current Feature initialization */
  pInformation->Current_Feature = USB_DEVICE_POWER_SOURCE;

  /* Set Virtual_Com_Port DEVICE with the default Interface */
  pInformation->Current_Interface = 0;

  initialisation();
  SetDeviceAddress(0);

  /* USB Reset */
  _SetUSB_TOP(USB_TOP_RESET);
  _ClrUSB_TOP(USB_TOP_RESET);

  pMSD->State = ATTACHED;
}


/**
 *  @brief  Update the device state to addressed.
 *  @param  None.
 *  @retval None.
 */
static void
process_status_in()
{
}


/**
 *  @brief  status OUT routine.
 *  @param  None.
 *  @retval None.
 */
static void
process_status_out()
{
}


/**
 *  @brief  Handle the Get Max Lun request.
 *  @param  uint16_t Length.
 *  @retval uint8_t *ptr.
 */
static uint8_t*
get_max_lun(uint16_t length)
{
  if (length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = LUN_DATA_LENGTH;
    return NULL;
  }
  else
  {
    return((uint8_t*)(&pMSD->maxUnitNumber));
  }

}


/**
 *  @brief  Handle the data class specific requests.
 *  @param  Request No.
 *  @retval USB_UNSUPPORT or USB_SUCCESS.
 */
static RESULT
setup_data_class(u8 RequestNo)
{
  u8* (*CopyRoutine)(u16);
  CopyRoutine = NULL;

  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) &&
      (RequestNo == GET_MAX_LUN) &&
      (pInformation->USBwValue == 0) &&
      (pInformation->USBwIndex == 0) &&
      (pInformation->USBwLength == 0x01))
  {
    CopyRoutine = get_max_lun;
  }
  else
  {
    return USB_UNSUPPORT;
  }

  if (CopyRoutine == NULL)
  {
    return USB_UNSUPPORT;
  }

  pInformation->Ctrl_Info.CopyData = CopyRoutine;
  pInformation->Ctrl_Info.Usb_wOffset = 0;
  (*CopyRoutine)(0);

  return USB_SUCCESS;
}


/**
 *  @brief  handle the no data class specific requests.
 *  @param  Request No.
 *  @retval USB_UNSUPPORT or USB_SUCCESS.
 */
static RESULT
setup_nodata_class(u8 RequestNo)
{
  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) &&
      (RequestNo == MASS_STORAGE_RESET) &&
      (pInformation->USBwValue == 0) &&
      (pInformation->USBwIndex == 0) &&
      (pInformation->USBwLength == 0x00))
  {
    /* Initialize Endpoint 1 */
    //TODO: ClearDTOG_TX(ENDP1);

    /* Initialize Endpoint 2 */
    //TODO: ClearDTOG_RX(ENDP2);

    /*initialize the CBW signature to enable the clear feature*/
    pMSD->BOT.CBW.dSignature = BOT_CBW_SIGNATURE;
    pMSD->BOT.State = BOT_IDLE;

    return USB_SUCCESS;
  }
  return USB_UNSUPPORT;
}


/**
 *  @brief  test the interface and the alternate setting according to the
 *          supported one.
 *  @param  u8: Interface : interface number.
 *          u8: AlternateSetting : Alternate Setting number.
 *  @retval USB_UNSUPPORT or USB_SUCCESS.
 */
static RESULT
get_interface_settings(u8 Interface, u8 AlternateSetting)
{
  if (AlternateSetting > 0)
  {
    return USB_UNSUPPORT;	/* in this application we don't have AlternateSetting*/
  }
  else if (Interface > 0)
  {
    return USB_UNSUPPORT;	/*in this application we have only 1 interfaces*/
  }
  return USB_SUCCESS;
}


/**
 * convert half byte to char
 */
static uint8_t
nibble_to_char(uint8_t halfByte)
{
  uint8_t result;

  if (halfByte <= 0x9) {
    result = '0' + halfByte;
  }
  else if (halfByte >= 0xA &&
           halfByte <= 0xF) {
    result = 'A' + halfByte - 0xA;
  }
  else {
    result = '-';
  }
  return result;
}


/**
 *  @brief  Gets the device descriptor.
 *  @param  Length.
 *  @retval The address of the device descriptor.
 */
static uint8_t*
get_usb_device_descriptor(uint16_t len)
{
  descriptor.Descriptor_Size = USB_DeviceDescriptorSize;
  memcpy(descriptor.Descriptor, USB_DeviceDescriptor, descriptor.Descriptor_Size);

  return Standard_GetDescriptorData(len, &descriptor);
}


/**
 *  @brief  Gets the configuration descriptor.
 *  @param  Length.
 *  @retval The address of the configuration descriptor.
 */
static uint8_t*
get_usb_config_descriptor(uint16_t len)
{
  descriptor.Descriptor_Size = USB_ConfigDescriptorSize;
  memcpy(descriptor.Descriptor, USB_ConfigDescriptor, descriptor.Descriptor_Size);

  return Standard_GetDescriptorData(len, &descriptor);
}


/**
 *  @brief  Gets the string descriptors according to the needed index.
 *  @param  Length.
 *  @retval The address of the string descriptors.
 */
static uint8_t*
get_usb_string_descriptor(uint16_t len)
{
  uint8_t index = pInformation->USBwValue0;
  uint8_t *result;

  switch(index) {
    case USB_DESC_LANGUAGE_ID_INDEX:
      descriptor.Descriptor_Size = 2 + USB_LangIdDescriptorSize;
      memcpy(&descriptor.Descriptor[2], USB_LangIdDescriptor, descriptor.Descriptor_Size);
      break;

    case USB_DESC_VENDOR_INDEX:
      descriptor.Descriptor_Size = 2 + USB_VendorDescriptorSize;
      memcpy(&descriptor.Descriptor[2], USB_VendorDescriptor, descriptor.Descriptor_Size);
      break;

    case USB_DESC_PRODUCT_INDEX:
      descriptor.Descriptor_Size = 2 + USB_ProductDescriptorSize;
      memcpy(&descriptor.Descriptor[2], USB_ProductDescriptor, descriptor.Descriptor_Size);
      break;

    case USB_DESC_SERIAL_NUMBER_INDEX:
      memset(&descriptor.Descriptor[0], 0x00, sizeof(buf));
      descriptor.Descriptor_Size = 2 + sizeof(device_id_data) * 2;
      for(uint8_t i = 0; i < sizeof(device_id_data); i++) {
        descriptor.Descriptor[2 + i * 4] = nibble_to_char((device_id_data[i] >> 0) & 0x0F);
        descriptor.Descriptor[4 + i * 4] = nibble_to_char((device_id_data[i] >> 4) & 0x0F);
      }
      break;

    case USB_DESC_INTERFACE_INDEX:
      descriptor.Descriptor_Size = 2 + USB_InterfaceDescriptorSize;
      memcpy(&descriptor.Descriptor[2], USB_InterfaceDescriptor, descriptor.Descriptor_Size);
      break;

    default:
      result = NULL;
  }

  if (index < USB_DESC_STRING_COUNT) {
    descriptor.Descriptor[0] = descriptor.Descriptor_Size; 	/*bLength*/
    descriptor.Descriptor[1] = USB_DESC_TYPE_STRING; 		/*bDescriptorType*/
    result = Standard_GetDescriptorData(len, &descriptor);
  }
  return result;
}


/* ------------------------------------------------------------------------ */

/**
 *  @brief  Handle the SetConfiguration request.
 *  @param  none
 *  @retval none
 */
static void set_configuration()
{
  if (pInformation->Current_Configuration != 0)
  {
    /* Device configured */
    pMSD->State = CONFIGURED;

    //TODO: ClearDTOG_TX(ENDP1);
    //TODO: ClearDTOG_RX(ENDP2);

    /* set the Bot state machine to the IDLE state */
    pMSD->BOT.State = BOT_IDLE;
  }
}

/**
 *  @brief  Handle the ClearFeature request.
 *  @param  none
 *  @retval none
 */
static void clear_feature()
{
  /* when the host send a CBW with invalid signature or invalid length the two
   Endpoints (IN & OUT) shall stall until receiving a Mass Storage Reset     */
  if (pMSD->BOT.CBW.dSignature != BOT_CBW_SIGNATURE)
  {
    BOT_Abort(BOTH_DIR);
  }
}

/**
 *  @brief  Update the device state to addressed.
 *  @param  none
 *  @retval none
 */
static void set_device_address()
{
  pMSD->State = ADDRESSED;
}


/* ------------------------------------------------------------------------ */

/*******************************************************************************
* Function Name  : Set_Scsi_Sense_Data
* Description    : Set Scsi Sense Data routine.
* Input          : uint8_t Sens_Key
                   uint8_t Asc.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void
set_scsi_sense_data(uint8_t lun, uint8_t Sens_Key, uint8_t Asc)
{
  (void) lun;

  Scsi_Sense_Data[ 2] = Sens_Key;
  Scsi_Sense_Data[12] = Asc;
}


/*******************************************************************************
* Function Name  : BOT_Abort
* Description    : Stall the needed Endpoint according to the selected direction.
* Input          : Endpoint direction IN, OUT or both directions
* Output         : None.
* Return         : None.
*******************************************************************************/
static void
BOT_Abort(BOT_Direction_t Direction)
{
  switch (Direction)
  {
    case DIR_IN :
      //TODO: SetEPTxStatus(ENDP1, EP_TX_STALL);
      break;

    case DIR_OUT :
      //TODO: SetEPRxStatus(ENDP2, EP_RX_STALL);
      break;

    case BOTH_DIR :
      //TODO: SetEPTxStatus(ENDP1, EP_TX_STALL);
      //TODO: SetEPRxStatus(ENDP2, EP_RX_STALL);
      break;

    default:
      break;
  }
}


/*******************************************************************************
* Function Name  : Set_CSW
* Description    : Set the SCW with the needed fields.
* Input          : uint8_t CSW_Status this filed can be CSW_CMD_PASSED,CSW_CMD_FAILED,
*                  or CSW_PHASE_ERROR.
* Output         : None.
* Return         : None.
*******************************************************************************/
void BOT_SetCSW(CSW_Status_t CSW_Status, CSW_SendPermission_t Send_Permission)
{
  pMSD->BOT.CSW.dSignature = BOT_CSW_SIGNATURE;
  pMSD->BOT.CSW.bStatus = CSW_Status;

  //USB_SIL_Write(EP1_IN, (uint8_t *)&CSW, CSW_DATA_LENGTH);
  UserToPMABufferCopy((uint8_t*)&pMSD->BOT.CSW, ENDP1, BOT_CSW_DATA_LENGTH);
  _SetUSB_CTRL1(EP1_CTRL_TRANEN | BOT_CSW_DATA_LENGTH);

  if (Send_Permission)
  {
    pMSD->BOT.State = BOT_CSW_SEND;
    //TODO: SetEPTxStatus(ENDP1, EP_TX_VALID);
  }
  else
  {
    pMSD->BOT.State = BOT_ERROR;
  }
}


/*******************************************************************************
* Function Name  : BOT_DecodeCBW (Command Block Wrapper)
* Description    : Decode the received CBW and call the related SCSI command routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void
BOT_DecodeCBW()
{
  uint32_t blockAddr;
  uint16_t blockNum;

  memcpy((uint8_t *)&pMSD->BOT.CBW, pMSD->BOT.data, pMSD->BOT.dataSize);

  pMSD->BOT.CSW.dTag = pMSD->BOT.CBW.dTag;
  pMSD->BOT.CSW.dDataResidue = pMSD->BOT.CBW.dDataLength;

  if (pMSD->BOT.dataSize != BOT_CBW_PACKET_LENGTH)
  {
    BOT_Abort(BOTH_DIR);

    /* reset the CBW.dSignature to disable the clear feature until receiving a Mass storage reset */
    pMSD->BOT.CBW.dSignature = 0;
    set_scsi_sense_data(pMSD->BOT.CBW.bLUN, ILLEGAL_REQUEST, PARAMETER_LIST_LENGTH_ERROR);
    BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_DISABLE);
    return;
  }

  if ((pMSD->BOT.CBW.CB[0] == SCSI_READ10) ||
      (pMSD->BOT.CBW.CB[0] == SCSI_WRITE10))
  {
    /* Calculate Logical Block Address */
    blockAddr = (pMSD->BOT.CBW.CB[2] << 24) |
                (pMSD->BOT.CBW.CB[3] << 16) |
                (pMSD->BOT.CBW.CB[4] <<  8) |
                (pMSD->BOT.CBW.CB[5]);

    /* Calculate the Number of Blocks to transfer */
    blockNum  = (pMSD->BOT.CBW.CB[7] <<  8) |
                (pMSD->BOT.CBW.CB[8]);
  }

  if (pMSD->BOT.CBW.dSignature == BOT_CBW_SIGNATURE)
  {
    /* Valid CBW */
    if ((pMSD->BOT.CBW.bLUN > pMSD->maxUnitNumber) ||
        (pMSD->BOT.CBW.bCBLength < 1) || (pMSD->BOT.CBW.bCBLength > 16))
    {
      BOT_Abort(BOTH_DIR);
      set_scsi_sense_data(pMSD->BOT.CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
      BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_DISABLE);
    }
    else
    {
      switch (pMSD->BOT.CBW.CB[0])
      {
        case SCSI_REQUEST_SENSE:
          SCSI_RequestSense_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_INQUIRY:
          SCSI_Inquiry_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_START_STOP_UNIT:
          SCSI_StartStopUnit_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_ALLOW_MEDIUM_REMOVAL:
          SCSI_StartStopUnit_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_MODE_SENSE6:
          SCSI_ModeSense6_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_MODE_SENSE10:
          SCSI_ModeSense10_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_READ_FORMAT_CAPACITIES:
          SCSI_ReadFormatCapacity_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_READ_CAPACITY10:
          SCSI_ReadCapacity10_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_TEST_UNIT_READY:
          SCSI_TestUnitReady_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_READ10:
          SCSI_Read10_Cmd(pMSD->BOT.CBW.bLUN, blockAddr, blockNum);
          break;

        case SCSI_WRITE10:
          SCSI_Write10_Cmd(pMSD->BOT.CBW.bLUN, blockAddr, blockNum);
          break;

        case SCSI_VERIFY10:
          SCSI_Verify10_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_FORMAT_UNIT:
          SCSI_Format_Cmd(pMSD->BOT.CBW.bLUN);
          break;

          /* Unsupported command */
        case SCSI_MODE_SELECT10:
          SCSI_Mode_Select10_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_MODE_SELECT6:
          SCSI_Mode_Select6_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_SEND_DIAGNOSTIC:
          SCSI_Send_Diagnostic_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_READ6:
          SCSI_Read6_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_READ12:
          SCSI_Read12_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_READ16:
          SCSI_Read16_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_READ_CAPACITY16:
          SCSI_Read_Capacity16_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_WRITE6:
          SCSI_Write6_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_WRITE12:
          SCSI_Write12_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_WRITE16:
          SCSI_Write16_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_VERIFY12:
          SCSI_Verify12_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        case SCSI_VERIFY16:
          SCSI_Verify16_Cmd(pMSD->BOT.CBW.bLUN);
          break;

        default:
          {
            BOT_Abort(BOTH_DIR);
            set_scsi_sense_data(pMSD->BOT.CBW.bLUN, ILLEGAL_REQUEST, INVALID_COMMAND);
            BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_DISABLE);
          }
      }
    }
  }
  else
  {
    /* Invalid CBW */
    BOT_Abort(BOTH_DIR);
    set_scsi_sense_data(pMSD->BOT.CBW.bLUN, ILLEGAL_REQUEST, INVALID_COMMAND);
    BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_DISABLE);
  }
}


/*******************************************************************************
* Function Name  : SCSI_Valid_Cmd
* Description    : Valid Commands routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void
SCSI_Valid_Cmd(uint8_t lun)
{
  (void) lun;

  if (pMSD->BOT.CBW.dDataLength != 0)
  {
    BOT_Abort(BOTH_DIR);
    set_scsi_sense_data(pMSD->BOT.CBW.bLUN, ILLEGAL_REQUEST, INVALID_COMMAND);
    BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_DISABLE);
  }
  else
  {
    BOT_SetCSW(CSW_CMD_PASSED, SEND_CSW_ENABLE);
  }
}


/*******************************************************************************
* Function Name  : SCSI_Invalid_Cmd
* Description    : Invalid Commands routine
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void
SCSI_Invalid_Cmd(uint8_t lun)
{
  (void) lun;

  if (pMSD->BOT.CBW.dDataLength == 0)
  {
    BOT_Abort(DIR_IN);
  }
  else
  {
    if ((pMSD->BOT.CBW.bmFlags & 0x80) != 0)
    {
      BOT_Abort(DIR_IN);
    }
    else
    {
      BOT_Abort(BOTH_DIR);
    }
  }
  set_scsi_sense_data(pMSD->BOT.CBW.bLUN, ILLEGAL_REQUEST, INVALID_COMMAND);
  BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_DISABLE);
}


/*******************************************************************************
* Function Name  : SCSI_Address_Management
* Description    : Test the received address.
* Input          : uint8_t Cmd : the command can be SCSI_READ10 or SCSI_WRITE10.
* Output         : None.
* Return         : Read\Write status (bool).
*******************************************************************************/
static bool
SCSI_Address_Management(uint8_t lun , uint8_t Cmd , uint32_t blockIndex , uint32_t blocksCount)
{
  if ((blockIndex + blocksCount) > pMSD->Unit[lun].blocksTotalCount)
  {
    if (Cmd == SCSI_WRITE10)
    {
      BOT_Abort(BOTH_DIR);
    }
    else
    {
      BOT_Abort(DIR_IN);
    }
    set_scsi_sense_data(pMSD->BOT.CBW.bLUN, ILLEGAL_REQUEST, ADDRESS_OUT_OF_RANGE);
    BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_DISABLE);
    return false;
  }

  if (pMSD->BOT.CBW.dDataLength != blocksCount * pMSD->Unit[lun].blockSize)
  {
    if (Cmd == SCSI_WRITE10)
    {
      BOT_Abort(BOTH_DIR);
    }
    else
    {
      BOT_Abort(DIR_IN);
    }
    set_scsi_sense_data(pMSD->BOT.CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
    BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_DISABLE);
    return false;
  }
  return true;
}


/*******************************************************************************
* Function Name  : SCSI_Read10_Cmd
* Description    : SCSI Read10 Command routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Read10_Cmd(uint8_t lun , uint32_t startBlock , uint32_t blocksCount)
{
  if (pMSD->BOT.State == BOT_IDLE)
  {
    if (!SCSI_Address_Management(lun, SCSI_READ10, startBlock, blocksCount))
    {
      /* address out of range */
      return;
    }

    if ((pMSD->BOT.CBW.bmFlags & 0x80) != 0)
    {
      pMSD->BOT.State = BOT_DATA_IN;
      MSD_ReadMemory(lun, startBlock, blocksCount);
    }
    else
    {
      BOT_Abort(BOTH_DIR);
      set_scsi_sense_data(pMSD->BOT.CBW.bLUN,
                          ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
      BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_ENABLE);
    }
    return;
  }
  else if (pMSD->BOT.State == BOT_DATA_IN)
  {
    MSD_ReadMemory(lun, startBlock, blocksCount);
  }
}

/*******************************************************************************
* Function Name  : SCSI_Write10_Cmd
* Description    : SCSI Write10 Command routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Write10_Cmd(uint8_t lun , uint32_t LBA , uint32_t BlockNbr)
{
  if (pMSD->BOT.State == BOT_IDLE)
  {
    if (!SCSI_Address_Management(lun, SCSI_WRITE10 , LBA, BlockNbr)) /* address out of range */
    {
      return;
    }

    if ((pMSD->BOT.CBW.bmFlags & 0x80) == 0)
    {
      pMSD->BOT.State = BOT_DATA_OUT;
      //TODO: SetEPRxStatus(ENDP2, EP_RX_VALID);
    }
    else
    {
      BOT_Abort(DIR_IN);
      set_scsi_sense_data(pMSD->BOT.CBW.bLUN,
                          ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
      BOT_SetCSW(CSW_CMD_FAILED, SEND_CSW_ENABLE);
    }
    return;
  }
  else if (pMSD->BOT.State == BOT_DATA_OUT)
  {
    Write_Memory(lun , LBA , BlockNbr);
  }
}


/*******************************************************************************
* Function Name  : MSD_ReadMemory
* Description    : Handle the Read operation from the block device.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void
MSD_ReadMemory(uint8_t lun, uint32_t startBlock, uint32_t blocksCount)
{
  if (pMSD->Transfer.State == TXFR_IDLE)
  {
    pMSD->Transfer.State = TXFR_ONGOING;
    pMSD->Transfer.size = pMSD->Unit[lun].blockSize * blocksCount;
    pMSD->Transfer.offset = pMSD->Unit[lun].blockSize * startBlock;
  }

  if (pMSD->Transfer.State == TXFR_ONGOING)
  {
    if ((pMSD->Transfer.dataOffset % pMSD->Unit[lun].blockSize) == 0)
    {
      STORAGE_Read(lun,
                   pMSD->Transfer.data,
                   pMSD->Transfer.offset,
                   pMSD->Unit[lun].blockSize);

      UserToPMABufferCopy(pMSD->Transfer.data,
                          ENDP1, BOT_MAX_PACKET_SIZE);

      pMSD->Transfer.dataOffset = BOT_MAX_PACKET_SIZE;
    }
    else
    {
      UserToPMABufferCopy(pMSD->Transfer.data +
                          pMSD->Transfer.dataOffset,
                          ENDP1, BOT_MAX_PACKET_SIZE);

      pMSD->Transfer.dataOffset += BOT_MAX_PACKET_SIZE;
    }
    //TODO: SetEPTxCount(ENDP1, BULK_MAX_PACKET_SIZE);
    //TODO: SetEPTxStatus(ENDP1, EP_TX_VALID);

    pMSD->Transfer.offset += BOT_MAX_PACKET_SIZE;
    pMSD->Transfer.size -= BOT_MAX_PACKET_SIZE;

    pMSD->BOT.CSW.dDataResidue -= BOT_MAX_PACKET_SIZE;

    STORAGE_onReadWrite(true);
  }

  if (pMSD->Transfer.size == 0)
  {
    pMSD->BOT.State = BOT_DATA_IN_LAST;
    pMSD->Transfer.State = TXFR_IDLE;
    pMSD->Transfer.offset = 0;

    STORAGE_onReadWrite(false);
  }
}


/*******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void
MSD_WriteMemory (uint8_t lun, uint32_t startBlock, uint32_t blocksCount)
{
  if (pMSD->Transfer.State == TXFR_IDLE)
  {
    pMSD->Transfer.State = TXFR_ONGOING;
    pMSD->Transfer.size = pMSD->Unit[lun].blockSize * blocksCount;
    pMSD->Transfer.offset = pMSD->Unit[lun].blockSize * startBlock;
  }

  if (pMSD->Transfer.State == TXFR_ONGOING)
  {
    memcpy(pMSD->Transfer.data +
           pMSD->Transfer.dataOffset,
           pMSD->BOT.data, BOT_MAX_PACKET_SIZE);

    pMSD->Transfer.dataOffset += BOT_MAX_PACKET_SIZE;

    pMSD->Transfer.offset += BOT_MAX_PACKET_SIZE;
    pMSD->Transfer.size -= BOT_MAX_PACKET_SIZE;

    if ((pMSD->Transfer.dataOffset > 0) &&
        (pMSD->Transfer.dataOffset % pMSD->Unit[lun].blockSize) == 0)
    {
      STORAGE_Write(lun,
                    pMSD->Transfer.data +
                    pMSD->Transfer.dataOffset,
                    pMSD->Transfer.offset,
                    pMSD->Unit[lun].blockSize);
    }

    pMSD->BOT.CSW.dDataResidue -= BOT_MAX_PACKET_SIZE;
    //TODO: SetEPRxStatus(ENDP2, EP_RX_VALID); /* enable the next transaction*/

    STORAGE_onReadWrite(true);
  }

  if ((pMSD->Transfer.size == 0) ||
      (pMSD->BOT.State == BOT_CSW_SEND))
  {
    pMSD->Transfer.State = TXFR_IDLE;
    pMSD->Transfer.dataOffset = 0;

    BOT_SetCSW(CSW_CMD_PASSED, SEND_CSW_ENABLE);

    STORAGE_onReadWrite(false);
  }
}


/* ------------------------------------------------------------------------ */

/**
 *  @brief  EP1 IN (port -> host) Callback.
 *  @param  None.
 *  @retval None.
 */
static void
EP1_IN_Callback()
{
  // TODO:
  switch (pMSD->BOT.State)
  {
    case BOT_DATA_IN:
      switch (pMSD->BOT.CBW.CB[0])
      {
        case SCSI_READ10:
          SCSI_Read10_Cmd(pMSD->BOT.CBW.bLUN,
                          pMSD->SCSI.LBA,
                          pMSD->SCSI.blockLen);
          break;
      }
      break;

    case BOT_DATA_IN_LAST:
      BOT_SetCSW(CSW_CMD_PASSED, SEND_CSW_ENABLE);
      //TODO: SetEPRxStatus(ENDP2, EP_RX_VALID);
      break;

    case BOT_ERROR:
    case BOT_CSW_SEND:
      pMSD->BOT.State = BOT_IDLE;
      //TODO: SetEPRxStatus(ENDP2, EP_RX_VALID);	/* enable the Endpoint to receive the next cmd*/
      //      if (GetEPRxStatus(EP2_OUT) == EP_RX_STALL)
      //      {
      //        SetEPRxStatus(EP2_OUT, EP_RX_VALID);/* enable the Endpoint to receive the next cmd*/
      //      }
      break;

    default:
      break;
  }


  //  uint8_t data[CDC_DATA_FS_MAX_PACKET_SIZE];
  //  size_t  size = ring_buffer_available(&pVirtualComPort->rbToHost);
  //
  //  if (size && ((_GetUSB_CTRL1() & EP1_CTRL_TRANEN ) == 0))
  //  {
  //    if (size >= CDC_DATA_FS_MAX_PACKET_SIZE) {
  //      size = CDC_DATA_FS_MAX_PACKET_SIZE;
  //    }
  //    ring_buffer_read_bytes(&pVirtualComPort->rbToHost, data, size);
  //    UserToPMABufferCopy((uint8_t*)data, ENDP1, size);
  //    _SetUSB_CTRL1(EP1_CTRL_TRANEN | size);
  //  }
}



/* ------------------------------------------------------------------------ */

/**
 *  @brief  EP3 OUT (host -> port) Callback.
 *  @param  None.
 *  @retval None.
 */
static void
EP2_OUT_Callback()
{
  // TODO:
  //  uint8_t data[CDC_DATA_FS_MAX_PACKET_SIZE + 1/* NOTE: \0 */] = {0};
  //  size_t  size = _GetUSB_AVILn(ENDP3);
  //
  //  if (size > CDC_DATA_FS_MAX_PACKET_SIZE) {
  //    size %= CDC_DATA_FS_MAX_PACKET_SIZE;
  //  }
  //  PMAToUserBufferCopy(data, ENDP3, size);
  //  VCOM_onDataReceivedFromHost(data, size);
}


/* ------------------------------------------------------------------------ */

/**
 * Interrupt Handler
 */
void USB_IRQHandler()
{
  if(_GetUSB_INT_STA() & USB_INT_STATE_SOFF)
  {
    _ClrUSB_INT_STA(USB_INT_STATE_SOFF);
    /* TODO? 1ms callback */
  }

  if (_GetUSB_INT_STA() & USB_INT_STATE_SUSPENDF)
  {
    _ClrUSB_INT_STA(USB_INT_STATE_SUSPENDF);
    /* TODO? Suspend callback */
  }

  if (_GetUSB_INT_STA() & USB_INT_STATE_RESUMF)
  {
    _ClrUSB_INT_STA(USB_INT_STATE_RESUMF);
    /* TODO? Resume callback */
  }

  if(_GetUSB_INT_STA() & USB_INT_STATE_RSTF)
  {
    _ClrUSB_INT_STA(USB_INT_STATE_RSTF);
    Device_Property.Reset();
  }

  if(_GetUSB_INT_STA() & USB_INT_STATE_EPINTF)
  {
    CTR_LP();
  }
}


/* ------------------------------------------------------------------------ */

/**
 * USB Hardware initialization
 */
__attribute__((weak)) void
USB_HardwareInit()
{
}

