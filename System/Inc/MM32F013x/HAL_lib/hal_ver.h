////////////////////////////////////////////////////////////////////////////////
/// @file     hal_ver.h
/// @author   AE TEAM
/// @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE UART
///           FIRMWARE LIBRARY.
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////

// Define to prevent recursive inclusion
#ifndef __HAL_VER_H
#define __HAL_VER_H

// Files includes
#include "reg_common.h"
#include "reg_dbg.h"

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup MM32_Hardware_Abstract_Layer
/// @{

/////////////////////////////////////1///////////////////////////////////////////
/// @defgroup UART_HAL
/// @brief UART HAL modules
/// @{


////////////////////////////////////////////////////////////////////////////////
/// @defgroup UART_Exported_Types
/// @{
///

////////////////////////////////////////////////////////////////////////////////
/// @brief UART Word Length Enumerate definition
/// @anchor UART_Word_Length
////////////////////////////////////////////////////////////////////////////////

/// @}

////////////////////////////////////////////////////////////////////////////////
/// @defgroup UART_Exported_Constants
/// @{

/// @}

////////////////////////////////////////////////////////////////////////////////
/// @defgroup UART_Exported_Variables
/// @{
#ifdef _HAL_VER_C_

#define GLOBAL
#else
#define GLOBAL extern
#endif

#undef GLOBAL
/// @}

#ifdef __cplusplus
 extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
/// @defgroup UART_Exported_Functions
/// @{

u32            Get_MM32LibVersion(void);
u32            Get_ChipsetREVID(void);
u32            Get_ChipsetDEVID(void);
u32            Get_ChipsetUIDw0(void);
u32            Get_ChipsetUIDw1(void);
u32            Get_ChipsetUIDw2(void);

/// @}

#ifdef __cplusplus
 }
#endif

/// @}

/// @}

////////////////////////////////////////////////////////////////////////////////
#endif // __HAL_VER_H 
////////////////////////////////////////////////////////////////////////////////
