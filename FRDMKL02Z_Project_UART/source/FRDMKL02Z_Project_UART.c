/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    FRDMKL02Z_Project_UART.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL02Z4.h"
#include "fsl_debug_console.h"


#include "sdk_hal_uart0.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
/*
 * @brief   Application entry point.
 */


void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 10000000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}



int main(void) {

	/* Define the init structure for the output LED pin*/
			    gpio_pin_config_t led_config = {
			        kGPIO_DigitalOutput, 0,
			    };


  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif


    /* Print a note to terminal. */
     PRINTF("\r\n Programa para encender y apagar los leds RGB por teclado");
     PRINTF("\r\n Para cambiar el estado de los leds vea el siguiente menu:\r\n");
     PRINTF("\r\n           LED ROJO             ||            LED VERDE           ||            LED AZUL           ");
     PRINTF("\r\n     ON        ||      OFF      ||       ON      ||      OFF      ||       ON      ||      OFF     ");
     PRINTF("\r\n Presione : R  ||  Presione: r  ||  Presione: V  ||  Presione: v  ||  Presione: A  ||  Presione: a \r\n");
     PRINTF("\r\n Presione T para ver una secuencia de colores y combinaciones");
     //PRINTF("\r\n Presione t para apagar todos los leds\r\n");
     PRINTF("\r\n");


    (void)uart0Inicializar(115200);


    /* Init output LED GPIO. */
            //Configura LED ROJO (PTB6) como salida
            GPIO_PinInit(GPIOB, 6U, &led_config);

            //Configura LED VERDE (PTB7) como salida
            GPIO_PinInit(GPIOB, 7U, &led_config);

            //Configura LED AZUL (PTB10) como salida
            GPIO_PinInit(GPIOB, 10U, &led_config);




            //Sacar 1 por pin LED AZUL (apaga)
                	 GPIO_PortSet(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);

                	 //Sacar 1 por pin LED VERDE (apaga)
                	 GPIO_PortSet(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);

                	 //Sacar 1 por pin LED ROJO (apaga)
                	 GPIO_PortSet(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);



    while(1) {
    	status_t status;
    	uint8_t nuevo_byte;

    	if(uart0NuevosDatosEnBuffer()>0){
    		status=uart0LeerByteDesdeBufferCircular(&nuevo_byte);

    		if(status==kStatus_Success){


    			if(nuevo_byte==65){
    				//Sacar 0 por pin LED AZUL (enciende)
    				GPIO_PortClear(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);
    				printf("Azul enciende:%c\r\n",nuevo_byte);
    			}

    			if(nuevo_byte==97){
    				//Sacar 1 por pin LED AZUL (apaga)
    				GPIO_PortSet(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);
    			    printf("Azul apagado:%c\r\n",nuevo_byte);
    			}



    			if(nuevo_byte==86){
    				//Sacar 0 por pin LED VERDE (enciende)
    				GPIO_PortClear(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);
    			    printf("Verde enciende:%c\r\n",nuevo_byte);
    			}

    			if(nuevo_byte==118){
    				//Sacar 1 por pin LED VERDE (apaga)
    				GPIO_PortSet(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);
    			    printf("Verde apaga:%c\r\n",nuevo_byte);
    			}



    			if(nuevo_byte==82){
    				//Sacar 0 por pin LED ROJO (enciende)
    				GPIO_PortClear(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);
    			    printf("Rojo enciende:%c\r\n",nuevo_byte);
    			}

    			if(nuevo_byte==114){
    				//Sacar 1 por pin LED ROJO (apaga)
    				GPIO_PortSet(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);
    			    printf("Rojo apaga:%c\r\n",nuevo_byte);
    			}

    			if(nuevo_byte==84){


    				    	//Sacar 0 por pin LED AZUL (enciende)
    				    	GPIO_PortClear(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);
    				    	printf("AZUL\r\n");
    				    	delay();
    				    	//Sacar 1 por pin LED AZUL (apaga)
    				    	GPIO_PortSet(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);


    				    	//Sacar 0 por pin LED VERDE (enciende)
    				    	GPIO_PortClear(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);
    				    	printf("VERDE\r\n");
    				    	delay();
    				    	//Sacar 1 por pin LED VERDE (apaga)
    				    	GPIO_PortSet(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);


    				    	//Sacar 0 por pin LED ROJO (enciende)
    				    	GPIO_PortClear(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);
    				    	printf("ROJO\r\n");
    				    	delay();
    				    	//Sacar 1 por pin LED ROJO (apaga)
    				    	GPIO_PortSet(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);


    				    	//Sacar 0 por pin LED VERDE (enciende)
    				    	GPIO_PortClear(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);
    				    	//Sacar 0 por pin LED AZUL (enciende)
    				    	GPIO_PortClear(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);
    				    	printf("AZUL Y VERDE\r\n");
    				    	delay();
    				    	//Sacar 1 por pin LED VERDE (apaga)
    				    	 GPIO_PortSet(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);
    				    	 //Sacar 1 por pin LED AZUL (apaga)
    				    	 GPIO_PortSet(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);


    				    	//Sacar 0 por pin LED VERDE (enciende)
    				  GPIO_PortClear(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);
    				    	//Sacar 0 por pin LED ROJO (enciende)
    				    GPIO_PortClear(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);
    				    printf("VERDE Y ROJO\r\n");
    				        delay();
    				        //Sacar 1 por pin LED VERDE (apaga)
    				         GPIO_PortSet(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);
    				         //Sacar 1 por pin LED ROJO (apaga)
    				         GPIO_PortSet(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);



    				        //Sacar 0 por pin LED AZUL (enciende)
    				       GPIO_PortClear(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);
    				       //Sacar 0 por pin LED ROJO (enciende)
    				      GPIO_PortClear(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);
    				      printf("AZUL Y ROJO\r\n");
    				      delay();
    				      //Sacar 1 por pin LED AZUL (apaga)
    				      GPIO_PortSet(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);
    				      //Sacar 1 por pin LED ROJO (apaga)
    				      GPIO_PortSet(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);



    				      //Sacar 0 por pin LED AZUL (enciende)
    				       GPIO_PortClear(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);
    				       //Sacar 0 por pin LED ROJO (enciende)
    				       GPIO_PortClear(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);
    				       //Sacar 0 por pin LED VERDE (enciende)
    				       GPIO_PortClear(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);
    				       printf("AZUL, ROJO Y VERDE\r\n");
    				       delay();
    				       //Sacar 1 por pin LED AZUL (apaga)
    				       GPIO_PortSet(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);
    				       //Sacar 1 por pin LED ROJO (apaga)
    				       GPIO_PortSet(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);
    				       //Sacar 1 por pin LED VERDE (apaga)
    				       GPIO_PortSet(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);

    			}

    	        if(nuevo_byte==116){
    				    	  //Sacar 1 por pin LED AZUL (apaga)
    				    	     GPIO_PortSet(GPIOB, 1u << BOARD_LED_BLUE_GPIO_PIN);

    				    	     //Sacar 1 por pin LED ROJO (apaga)
    				    	      GPIO_PortSet(GPIOB, 1u << BOARD_LED_RED_GPIO_PIN);

    				    	      //Sacar 1 por pin LED VERDE (apaga)
    				    	      GPIO_PortSet(GPIOB, 1u << BOARD_LED_GREEN_GPIO_PIN);
    				    	      printf("leds apagados\r\n");
    				      }




    		}else{
    			printf("error\r\n");
    		}
    	}
    }
    return 0 ;
}
