/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v1.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
/**
 * @todo Change wheel state from analog for zoom and digital for modes
 */
extern uint8_t USB_RX_Buffer[16];
extern uint8_t wheel_arr_[18] =
		{
		0x09, 0x38,          //     UsageId(Wheel[0x0038])
	    0x46, 0x0E, 0x01,    //     PhysicalMaximum(270)
	    0x65, 0x14,          //     Unit('degrees', EnglishRotation, Degrees:1)
	    0x15, 0x00,          //     LogicalMinimum(0)
	    0x25, 0x05,          //     LogicalMaximum(5)
	    0x95, 0x01,          //     ReportCount(1)
	    0x75, 0x02,          //     ReportSize(2)
	    0x81, 0x02//     Input(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
		};
extern uint8_t wheel_arr__[18] =
		{
		0x09, 0x38,          //     UsageId(Wheel[0x0038])
	    0x46, 0x0E, 0x01,    //     PhysicalMaximum(270)
	    0x65, 0x14,          //     Unit('degrees', EnglishRotation, Degrees:1)
	    0x15, 0x00,          //     LogicalMinimum(0)
	    0x25, 0x05,          //     LogicalMaximum(5)
	    0x95, 0x01,          //     ReportCount(1)
	    0x75, 0x02,          //     ReportSize(2)
	    0x81, 0x02//     Input(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
		};
extern uint8_t wheel_arr___[18] = {0};

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
		/* USER CODE BEGIN 0 */
	    0x05, 0x01,          // UsagePage(Generic Desktop[0x0001])
	    0x09, 0x04,          // UsageId(Joystick[0x0004])
	    0xA1, 0x01,          // Collection(Application)
	    0x85, 0x01,          //     ReportId(1)
	    0x09, 0x01,          //     UsageId(Pointer[0x0001])
	    0xA1, 0x00,          //     Collection(Physical)
	    0x09, 0x30,          //         UsageId(X[0x0030])
	    0x09, 0x31,          //         UsageId(Y[0x0031])
	    0x15, 0x80,          //         LogicalMinimum(-128)
	    0x25, 0x7F,          //         LogicalMaximum(127)
	    0x95, 0x02,          //         ReportCount(2)
	    0x75, 0x08,          //         ReportSize(8)
	    0x81, 0x02,          //         Input(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)

		wheel_arr___,

		0x05, 0x09,          //     UsagePage(Button[0x0009])
	    0x19, 0x01,          //     UsageIdMin(Button 1[0x0001])
	    0x29, 0x07,          //     UsageIdMax(Button 7[0x0007])
	    0x45, 0x00,          //     PhysicalMaximum(0)
	    0x65, 0x00,          //     Unit(None)
	    0x25, 0x01,          //     LogicalMaximum(1)
	    0x95, 0x07,          //     ReportCount(7)
	    0x75, 0x01,          //     ReportSize(1)
	    0x81, 0x02,          //     Input(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
	    0x95, 0x01,          //     ReportCount(1)
	    0x75, 0x07,          //     ReportSize(7)
	    0x81, 0x03,          //     Input(Constant, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
		/* USER CODE BEGIN 0 */
		0xC0                // EndCollection()
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
  UNUSED(event_idx);
  UNUSED(state);

  /* Start next USB packet transfer once data processing is completed */
  if (USBD_CUSTOM_HID_ReceivePacket(&hUsbDeviceFS) != (uint8_t)USBD_OK)
  {
    return -1;
  }

  USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*) hUsbDeviceFS.pClassData;
  for(uint8_t i = 0; i < 16; i++){
	  USB_RX_Buffer[i] = hhid->Report_buf[i];
  }

  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

