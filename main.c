/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK2_QSPI_Flash_XIP
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-11-28
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_qspi_memslot.h"
#include "cy_serial_flash_qspi.h"

#define PACKET_SIZE             (256u)     /* Memory Read/Write size */
#define NUM_BYTES_PER_LINE      (16u)     /* Used when array of data is printed on the console */
#define FLASH_MEM_SLOT_NUM      (1u)      /* Slot number of the memory to use */
#define QSPI_BUS_FREQUENCY_HZ   (48000000lu)

CY_SECTION(".cy_xip_code") __attribute__((used))
void print_from_external_memory(const char *buf);
void check_address(char *message, uint32_t addr);
void print_array(char *message, uint8_t *buf, uint32_t size);
void check_status(char *message, uint32_t status);
void handle_error(void);

/* String placed in external memory on programming */
const char hi_word[] CY_SECTION(".cy_xip") __attribute__((used)) = "\r\n >>> This message is from the external memory. <<<\r\n\r\n";

int main(void)
{
    cy_rslt_t result;
    uint32_t addr;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("RDK2 QSPI NOR Flash XIP Example.\r\n");

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /* Initialize the QSPI block */
    result = cy_serial_flash_qspi_init(smifMemConfigs[FLASH_MEM_SLOT_NUM],
    		QSPI_IO0, QSPI_IO1,
    		QSPI_IO2, QSPI_IO3, NC, NC, NC, NC, QSPI_CLK,
			FLASH_SSEL, QSPI_BUS_FREQUENCY_HZ);
    check_status("Serial Flash initialization failed", result);

    /* Initialize the transfer buffers */
    uint8_t txBuffer[PACKET_SIZE];
    uint8_t rxBuffer[PACKET_SIZE];

    /* Initialize tx buffer and rx buffer */
    for(uint32_t index = 0; index < PACKET_SIZE; index++)
    {
        txBuffer[index] = (uint8_t) (index & 0xFF);
        rxBuffer[index] = 0;
    }

    /* Set the address to transact with to the start of the third sector */
    uint32_t extMemAddress = 0x00;
    size_t sectorSize = cy_serial_flash_qspi_get_erase_size(extMemAddress);
    extMemAddress = sectorSize * 2;

    printf(" Total Flash Size: %u bytes.\r\n", cy_serial_flash_qspi_get_size());

    /* Erase before write */
    printf(" Erasing %u bytes of memory.\r\n", sectorSize);
    result = cy_serial_flash_qspi_erase(extMemAddress, sectorSize);
    check_status("Erasing memory failed", result);

    /* Read after Erase to confirm that all data is 0xFF */
    printf(" Reading after Erase. Ensure that the data read is 0xFF for each byte.\r\n");
    result = cy_serial_flash_qspi_read(extMemAddress, PACKET_SIZE, rxBuffer);
    check_status("Reading memory failed", result);
    print_array("Received Data", rxBuffer, PACKET_SIZE);

    /* Write the content of the txBuffer to the memory */
    printf(" Writing data to memory.\r\n");
    result = cy_serial_flash_qspi_write(extMemAddress, PACKET_SIZE, txBuffer);
    check_status("Writing to memory failed", result);
    print_array("Written Data", txBuffer, PACKET_SIZE);

    /* Read back after Write for verification */
    printf(" Reading back for verification.\r\n");
    result = cy_serial_flash_qspi_read(extMemAddress, PACKET_SIZE, rxBuffer);
    check_status("Reading memory failed", result);
    print_array("Received Data", rxBuffer, PACKET_SIZE);

    /* Check if the transmitted and received arrays are equal */
    check_status("Read data does not match with written data. Read/Write operation failed.", memcmp(txBuffer, rxBuffer, PACKET_SIZE));
    printf("================================================================================\r\n");
    printf("  SUCCESS: Read data matches with written data!\r\n");
    printf("================================================================================\r\n");

    /* Put the device in XIP mode */
    printf("Entering XIP Mode.\r\n");
    cy_serial_flash_qspi_enable_xip(true);

    addr = (uint32_t)&hi_word;
    check_address("String not found in external memory.\r\n", addr);
    /* Print the string from external memory */
    printf("  String in the external memory at address: 0x%lx\r\n", addr);
    printf("-------------------------------------------------------\r\n%s", (const char*)&hi_word);

    addr = (uint32_t)&print_from_external_memory;
    check_address("Function not found in external memory.\r\n", addr);
    /* Print by calling function that lives in external memory */
    printf("  Function call from external memory address: 0x%lx\r\n", addr);
    printf("-------------------------------------------------------\r\n");
    print_from_external_memory("\r\n >>> Hello from the external function!<<<\r\n\r\n");

    printf("================================================================================\r\n");
    printf("  SUCCESS: Data successfully accessed in XIP mode!\r\n");
    printf("================================================================================\r\n");

    for (;;)
    {
    	CyDelay(500);
    	cyhal_gpio_toggle(LED1);
    }
}

/*******************************************************************************
* Function Name: check_status
****************************************************************************//**
* Summary:
*  Prints the message, indicates the non-zero status by turning the LED on, and
*  asserts the non-zero status.
*
* Parameters:
*  message - message to print if status is non-zero.
*  status - status for evaluation.
*
*******************************************************************************/
void check_status(char *message, uint32_t status)
{
    if(0u != status)
    {
        printf("================================================================================\r\n");
        printf("\nFAIL: %s\r\n", message);
        printf("Error Code: 0x%08lX\r\n", (unsigned long)status);
        printf("================================================================================\r\n");

        /* On failure, turn the LED ON */
        cyhal_gpio_write(LED2, CYBSP_LED_STATE_ON);
        while(true); /* Wait forever here when error occurs. */
    }
}

/*******************************************************************************
* Function Name: print_array
****************************************************************************//**
* Summary:
*  Prints the content of the buffer to the UART console.
*
* Parameters:
*  message - message to print before array output
*  buf - buffer to print on the console.
*  size - size of the buffer.
*
*******************************************************************************/
void print_array(char *message, uint8_t *buf, uint32_t size)
{
    printf("%s (%lu bytes):\r\n", message, (unsigned long)size);
    printf("-------------------------\r\n");

    for(uint32_t index = 0; index < size; index++)
    {
        printf("0x%02X ", buf[index]);

        if(0u == ((index + 1) % NUM_BYTES_PER_LINE))
        {
            printf("\r\n");
        }
    }
}

/*******************************************************************************
* Function Name: check_address
****************************************************************************//**
* Summary:
*  Checks the address of the passed function. If the address is not in the
*  external memory region (addr >= 0x18000000), then prints a failure message
*  and exits.
*
* Parameters:
*  message - message to print if address is not in external memory.
*  addr - address for evaluation.
*
*******************************************************************************/
void check_address(char *message, uint32_t addr)
{
    if(smifMemConfigs[FLASH_MEM_SLOT_NUM]->baseAddress > addr)
    {
        printf("================================================================================\r\n");
        printf("FAIL: %s\r\n", message);
        printf("Address: 0x%lx\r\n", addr);
        printf("================================================================================\r\n");

        /* On failure, turn the LED ON */
        cyhal_gpio_write(LED2, CYBSP_LED_STATE_ON);
        while(true); /* Wait forever here when error occurs. */
    }
}

/********************************************************
* Function Name: print_from_external_memory
*********************************************************
* Summary:
*  Prints the passed string to a UART.
*  Executes from external memory.
*
* Parameters:
*   buf: The string to be printed
********************************************************/
void print_from_external_memory(const char *buf)
{
    printf("%s", buf);
}

/*If initialization fails, program ends up here.*/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */
