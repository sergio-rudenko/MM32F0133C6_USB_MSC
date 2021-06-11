/*
 * usb_conf.h
 */

#ifndef USB_CONF_H_
#define USB_CONF_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


#define USB_DEVICE_POWER_SOURCE				   0x80U	/* Bus powered */

#define USB_DESC_TYPE_DEVICE				   0x01U
#define USB_DESC_TYPE_CONFIGURATION            0x02U
#define USB_DESC_TYPE_STRING                   0x03U
#define USB_DESC_TYPE_INTERFACE                0x04U
#define USB_DESC_TYPE_ENDPOINT                 0x05U

#define USB_DESC_LANGUAGE_ID_INDEX		    	0x0U
#define USB_DESC_VENDOR_INDEX					0x1U
#define USB_DESC_PRODUCT_INDEX					0x2U
#define USB_DESC_SERIAL_NUMBER_INDEX			0x3U
#define USB_DESC_INTERFACE_INDEX				0x4U
#define USB_DESC_STRING_COUNT			  		0x5U


/* ---[ exported types ]--------------------------------------------------- */

typedef enum USB_DeviceState_ {
  UNCONNECTED 	= 0,
  ATTACHED,
  POWERED,
  SUSPENDED,
  ADDRESSED,
  CONFIGURED
}
USB_DeviceState_t;


typedef enum BOT_State_ {
  BOT_IDLE		= 0,	/* Idle state */
  BOT_DATA_OUT,			/* Data Out state */
  BOT_DATA_IN,			/* Data In state */
  BOT_DATA_IN_LAST,		/* Last Data In Last */
  BOT_CSW_SEND,			/* Command Status Wrapper */
  BOT_ERROR,			/* error state */
}
BOT_State_t;

/* Command Block Wrapper */
typedef struct BOT_CBW_
{
  uint32_t dSignature;
  uint32_t dTag;
  uint32_t dDataLength;
  uint8_t  bmFlags;
  uint8_t  bLUN;
  uint8_t  bCBLength;
  uint8_t  CB[16];
}
BOT_CBW_t;


/* Command Status Wrapper */
typedef struct BOT_CSW_
{
  uint32_t dSignature;
  uint32_t dTag;
  uint32_t dDataResidue;
  uint8_t  bStatus;
}
BOT_CSW_t;

typedef enum MSD_TransferState_ {
  TXFR_IDLE		= 0,
  TXFR_ONGOING,
} MSD_TransferState_t;


typedef struct MSD_LogicalUnit_ {
  uint32_t	blockSize;
  uint32_t	blocksTotalCount;
} MSD_LogicalUnit_t;


typedef struct MassStorageDevice_ {
  USB_DeviceState_t 	State;

  /* Logical Unit (s) */
  size_t				maxUnitNumber;
  MSD_LogicalUnit_t*	Unit;

  struct {
    MSD_TransferState_t	State;
    uint32_t			offset;
    uint32_t			size;

    uint8_t*			data;
    uint16_t			dataOffset;
  } Transfer;

  struct {
    BOT_State_t			State;
    BOT_CBW_t 			CBW;
    BOT_CSW_t 			CSW;

    uint8_t*			data;
    uint16_t			dataSize;
  } BOT;

  struct {
    uint32_t			blockIndex;
    uint32_t 			blocksCount;
  } SCSI;
}
MassStorageDevice_t;


/* ---[ exported variables ]----------------------------------------------- */
extern MassStorageDevice_t *pMSD;


/* ---[ function prototypes ]---------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

  void 		USB_HardwareInit();

//  void		STORAGE_Init(MSD_LogicalUnit_t* Unit, uint8_t maxUnitNumber);
//  bool 		STORAGE_IsReady(uint8_t lun);
//  bool		STORAGE_IsWriteProtected(uint8_t lun);
//  size_t 	STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
//  size_t	STORAGE_GetMaxLun();
  size_t	STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  size_t	STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);

  void		STORAGE_onReadWrite(bool readWriteOngoing);

#ifdef __cplusplus
}
#endif

#endif /* USB_CONF_H_ */
