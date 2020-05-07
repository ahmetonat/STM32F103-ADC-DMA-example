ARM GAS  /tmp/cc15Trzm.s 			page 1


   1              	# 1 "startup_stm32f103xb.s"
   1              	/**
   1              	...
   0              	
   0              	
   2              	  *************** (C) COPYRIGHT 2017 STMicroelectronics ************************
   3              	  * @file      startup_stm32f103xb.s
   4              	  * @author    MCD Application Team
   5              	  * @version   V4.2.0
   6              	  * @date      31-March-2017
   7              	  * @brief     STM32F103xB Devices vector table for Atollic toolchain.
   8              	  *            This module performs:
   9              	  *                - Set the initial SP
  10              	  *                - Set the initial PC == Reset_Handler,
  11              	  *                - Set the vector table entries with the exceptions ISR address
  12              	  *                - Configure the clock system   
  13              	  *                - Branches to main in the C library (which eventually
  14              	  *                  calls main()).
  15              	  *            After Reset the Cortex-M3 processor is in Thread mode,
  16              	  *            priority is Privileged, and the Stack is set to Main.
  17              	  ******************************************************************************
  18              	  *
  19              	  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  20              	  *
  21              	  * Redistribution and use in source and binary forms, with or without modification,
  22              	  * are permitted provided that the following conditions are met:
  23              	  *   1. Redistributions of source code must retain the above copyright notice,
  24              	  *      this list of conditions and the following disclaimer.
  25              	  *   2. Redistributions in binary form must reproduce the above copyright notice,
  26              	  *      this list of conditions and the following disclaimer in the documentation
  27              	  *      and/or other materials provided with the distribution.
  28              	  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  29              	  *      may be used to endorse or promote products derived from this software
  30              	  *      without specific prior written permission.
  31              	  *
  32              	  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  33              	  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  34              	  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  35              	  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  36              	  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  37              	  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  38              	  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  39              	  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  40              	  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  41              	  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  42              	  *
  43              	  ******************************************************************************
  44              	  */
  45              	
  46              	  .syntax unified
  47              	  .cpu cortex-m3
  48              	  .fpu softvfp
  49              	  .thumb
  50              	
  51              	.global g_pfnVectors
  52              	.global Default_Handler
  53              	
ARM GAS  /tmp/cc15Trzm.s 			page 2


  54              	/* start address for the initialization values of the .data section.
  55              	defined in linker script */
  56 0000 00000000 	.word _sidata
  57              	/* start address for the .data section. defined in linker script */
  58 0004 00000000 	.word _sdata
  59              	/* end address for the .data section. defined in linker script */
  60 0008 00000000 	.word _edata
  61              	/* start address for the .bss section. defined in linker script */
  62 000c 00000000 	.word _sbss
  63              	/* end address for the .bss section. defined in linker script */
  64 0010 00000000 	.word _ebss
  65              	
  66              	.equ  BootRAM, 0xF108F85F
  67              	/**
  68              	 * @brief  This is the code that gets called when the processor first
  69              	 *          starts execution following a reset event. Only the absolutely
  70              	 *          necessary set is performed, after which the application
  71              	 *          supplied main() routine is called.
  72              	 * @param  None
  73              	 * @retval : None
  74              	*/
  75              	
  76              	  .section .text.Reset_Handler
  77              	  .weak Reset_Handler
  79              	Reset_Handler:
  80              	
  81              	/* Copy the data segment initializers from flash to SRAM */
  82 0000 0021     	  movs r1, #0
  83 0002 03E0     	  b LoopCopyDataInit
  84              	
  85              	CopyDataInit:
  86 0004 0B4B     	  ldr r3, =_sidata
  87 0006 5B58     	  ldr r3, [r3, r1]
  88 0008 4350     	  str r3, [r0, r1]
  89 000a 0431     	  adds r1, r1, #4
  90              	
  91              	LoopCopyDataInit:
  92 000c 0A48     	  ldr r0, =_sdata
  93 000e 0B4B     	  ldr r3, =_edata
  94 0010 4218     	  adds r2, r0, r1
  95 0012 9A42     	  cmp r2, r3
  96 0014 F6D3     	  bcc CopyDataInit
  97 0016 0A4A     	  ldr r2, =_sbss
  98 0018 02E0     	  b LoopFillZerobss
  99              	/* Zero fill the bss segment. */
 100              	FillZerobss:
 101 001a 0023     	  movs r3, #0
 102 001c 42F8043B 	  str r3, [r2], #4
 103              	
 104              	LoopFillZerobss:
 105 0020 084B     	  ldr r3, = _ebss
 106 0022 9A42     	  cmp r2, r3
 107 0024 F9D3     	  bcc FillZerobss
 108              	
 109              	/* Call the clock system intitialization function.*/
 110 0026 FFF7FEFF 	    bl  SystemInit
 111              	/* Call static constructors */
ARM GAS  /tmp/cc15Trzm.s 			page 3


 112 002a FFF7FEFF 	    bl __libc_init_array
 113              	/* Call the application's entry point.*/
 114 002e FFF7FEFF 	  bl main
 115 0032 7047     	  bx lr
 117              	
 118              	/**
 119              	 * @brief  This is the code that gets called when the processor receives an
 120              	 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 121              	 *         the system state for examination by a debugger.
 122              	 *
 123              	 * @param  None
 124              	 * @retval : None
 125              	*/
 126              	    .section .text.Default_Handler,"ax",%progbits
 127              	Default_Handler:
 128              	Infinite_Loop:
 129 0000 FEE7     	  b Infinite_Loop
 131              	/******************************************************************************
 132              	*
 133              	* The minimal vector table for a Cortex M3.  Note that the proper constructs
 134              	* must be placed on this to ensure that it ends up at physical address
 135              	* 0x0000.0000.
 136              	*
 137              	******************************************************************************/
 138              	  .section .isr_vector,"a",%progbits
 141              	
 142              	
 143              	g_pfnVectors:
 144              	
 145 0000 00000000 	  .word _estack
 146 0004 00000000 	  .word Reset_Handler
 147 0008 00000000 	  .word NMI_Handler
 148 000c 00000000 	  .word HardFault_Handler
 149 0010 00000000 	  .word MemManage_Handler
 150 0014 00000000 	  .word BusFault_Handler
 151 0018 00000000 	  .word UsageFault_Handler
 152 001c 00000000 	  .word 0
 153 0020 00000000 	  .word 0
 154 0024 00000000 	  .word 0
 155 0028 00000000 	  .word 0
 156 002c 00000000 	  .word SVC_Handler
 157 0030 00000000 	  .word DebugMon_Handler
 158 0034 00000000 	  .word 0
 159 0038 00000000 	  .word PendSV_Handler
 160 003c 00000000 	  .word SysTick_Handler
 161 0040 00000000 	  .word WWDG_IRQHandler
 162 0044 00000000 	  .word PVD_IRQHandler
 163 0048 00000000 	  .word TAMPER_IRQHandler
 164 004c 00000000 	  .word RTC_IRQHandler
 165 0050 00000000 	  .word FLASH_IRQHandler
 166 0054 00000000 	  .word RCC_IRQHandler
 167 0058 00000000 	  .word EXTI0_IRQHandler
 168 005c 00000000 	  .word EXTI1_IRQHandler
 169 0060 00000000 	  .word EXTI2_IRQHandler
 170 0064 00000000 	  .word EXTI3_IRQHandler
 171 0068 00000000 	  .word EXTI4_IRQHandler
 172 006c 00000000 	  .word DMA1_Channel1_IRQHandler
ARM GAS  /tmp/cc15Trzm.s 			page 4


 173 0070 00000000 	  .word DMA1_Channel2_IRQHandler
 174 0074 00000000 	  .word DMA1_Channel3_IRQHandler
 175 0078 00000000 	  .word DMA1_Channel4_IRQHandler
 176 007c 00000000 	  .word DMA1_Channel5_IRQHandler
 177 0080 00000000 	  .word DMA1_Channel6_IRQHandler
 178 0084 00000000 	  .word DMA1_Channel7_IRQHandler
 179 0088 00000000 	  .word ADC1_2_IRQHandler
 180 008c 00000000 	  .word USB_HP_CAN1_TX_IRQHandler
 181 0090 00000000 	  .word USB_LP_CAN1_RX0_IRQHandler
 182 0094 00000000 	  .word CAN1_RX1_IRQHandler
 183 0098 00000000 	  .word CAN1_SCE_IRQHandler
 184 009c 00000000 	  .word EXTI9_5_IRQHandler
 185 00a0 00000000 	  .word TIM1_BRK_IRQHandler
 186 00a4 00000000 	  .word TIM1_UP_IRQHandler
 187 00a8 00000000 	  .word TIM1_TRG_COM_IRQHandler
 188 00ac 00000000 	  .word TIM1_CC_IRQHandler
 189 00b0 00000000 	  .word TIM2_IRQHandler
 190 00b4 00000000 	  .word TIM3_IRQHandler
 191 00b8 00000000 	  .word TIM4_IRQHandler
 192 00bc 00000000 	  .word I2C1_EV_IRQHandler
 193 00c0 00000000 	  .word I2C1_ER_IRQHandler
 194 00c4 00000000 	  .word I2C2_EV_IRQHandler
 195 00c8 00000000 	  .word I2C2_ER_IRQHandler
 196 00cc 00000000 	  .word SPI1_IRQHandler
 197 00d0 00000000 	  .word SPI2_IRQHandler
 198 00d4 00000000 	  .word USART1_IRQHandler
 199 00d8 00000000 	  .word USART2_IRQHandler
 200 00dc 00000000 	  .word USART3_IRQHandler
 201 00e0 00000000 	  .word EXTI15_10_IRQHandler
 202 00e4 00000000 	  .word RTC_Alarm_IRQHandler
 203 00e8 00000000 	  .word USBWakeUp_IRQHandler
 204 00ec 00000000 	  .word 0
 205 00f0 00000000 	  .word 0
 206 00f4 00000000 	  .word 0
 207 00f8 00000000 	  .word 0
 208 00fc 00000000 	  .word 0
 209 0100 00000000 	  .word 0
 210 0104 00000000 	  .word 0
 211 0108 5FF808F1 	  .word BootRAM          /* @0x108. This is for boot in RAM mode for
 212              	                            STM32F10x Medium Density devices. */
 213              	
 214              	/*******************************************************************************
 215              	*
 216              	* Provide weak aliases for each Exception handler to the Default_Handler.
 217              	* As they are weak aliases, any function with the same name will override
 218              	* this definition.
 219              	*
 220              	*******************************************************************************/
 221              	
 222              	  .weak NMI_Handler
 223              	  .thumb_set NMI_Handler,Default_Handler
 224              	
 225              	  .weak HardFault_Handler
 226              	  .thumb_set HardFault_Handler,Default_Handler
 227              	
 228              	  .weak MemManage_Handler
 229              	  .thumb_set MemManage_Handler,Default_Handler
ARM GAS  /tmp/cc15Trzm.s 			page 5


 230              	
 231              	  .weak BusFault_Handler
 232              	  .thumb_set BusFault_Handler,Default_Handler
 233              	
 234              	  .weak UsageFault_Handler
 235              	  .thumb_set UsageFault_Handler,Default_Handler
 236              	
 237              	  .weak SVC_Handler
 238              	  .thumb_set SVC_Handler,Default_Handler
 239              	
 240              	  .weak DebugMon_Handler
 241              	  .thumb_set DebugMon_Handler,Default_Handler
 242              	
 243              	  .weak PendSV_Handler
 244              	  .thumb_set PendSV_Handler,Default_Handler
 245              	
 246              	  .weak SysTick_Handler
 247              	  .thumb_set SysTick_Handler,Default_Handler
 248              	
 249              	  .weak WWDG_IRQHandler
 250              	  .thumb_set WWDG_IRQHandler,Default_Handler
 251              	
 252              	  .weak PVD_IRQHandler
 253              	  .thumb_set PVD_IRQHandler,Default_Handler
 254              	
 255              	  .weak TAMPER_IRQHandler
 256              	  .thumb_set TAMPER_IRQHandler,Default_Handler
 257              	
 258              	  .weak RTC_IRQHandler
 259              	  .thumb_set RTC_IRQHandler,Default_Handler
 260              	
 261              	  .weak FLASH_IRQHandler
 262              	  .thumb_set FLASH_IRQHandler,Default_Handler
 263              	
 264              	  .weak RCC_IRQHandler
 265              	  .thumb_set RCC_IRQHandler,Default_Handler
 266              	
 267              	  .weak EXTI0_IRQHandler
 268              	  .thumb_set EXTI0_IRQHandler,Default_Handler
 269              	
 270              	  .weak EXTI1_IRQHandler
 271              	  .thumb_set EXTI1_IRQHandler,Default_Handler
 272              	
 273              	  .weak EXTI2_IRQHandler
 274              	  .thumb_set EXTI2_IRQHandler,Default_Handler
 275              	
 276              	  .weak EXTI3_IRQHandler
 277              	  .thumb_set EXTI3_IRQHandler,Default_Handler
 278              	
 279              	  .weak EXTI4_IRQHandler
 280              	  .thumb_set EXTI4_IRQHandler,Default_Handler
 281              	
 282              	  .weak DMA1_Channel1_IRQHandler
 283              	  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler
 284              	
 285              	  .weak DMA1_Channel2_IRQHandler
 286              	  .thumb_set DMA1_Channel2_IRQHandler,Default_Handler
ARM GAS  /tmp/cc15Trzm.s 			page 6


 287              	
 288              	  .weak DMA1_Channel3_IRQHandler
 289              	  .thumb_set DMA1_Channel3_IRQHandler,Default_Handler
 290              	
 291              	  .weak DMA1_Channel4_IRQHandler
 292              	  .thumb_set DMA1_Channel4_IRQHandler,Default_Handler
 293              	
 294              	  .weak DMA1_Channel5_IRQHandler
 295              	  .thumb_set DMA1_Channel5_IRQHandler,Default_Handler
 296              	
 297              	  .weak DMA1_Channel6_IRQHandler
 298              	  .thumb_set DMA1_Channel6_IRQHandler,Default_Handler
 299              	
 300              	  .weak DMA1_Channel7_IRQHandler
 301              	  .thumb_set DMA1_Channel7_IRQHandler,Default_Handler
 302              	
 303              	  .weak ADC1_2_IRQHandler
 304              	  .thumb_set ADC1_2_IRQHandler,Default_Handler
 305              	
 306              	  .weak USB_HP_CAN1_TX_IRQHandler
 307              	  .thumb_set USB_HP_CAN1_TX_IRQHandler,Default_Handler
 308              	
 309              	  .weak USB_LP_CAN1_RX0_IRQHandler
 310              	  .thumb_set USB_LP_CAN1_RX0_IRQHandler,Default_Handler
 311              	
 312              	  .weak CAN1_RX1_IRQHandler
 313              	  .thumb_set CAN1_RX1_IRQHandler,Default_Handler
 314              	
 315              	  .weak CAN1_SCE_IRQHandler
 316              	  .thumb_set CAN1_SCE_IRQHandler,Default_Handler
 317              	
 318              	  .weak EXTI9_5_IRQHandler
 319              	  .thumb_set EXTI9_5_IRQHandler,Default_Handler
 320              	
 321              	  .weak TIM1_BRK_IRQHandler
 322              	  .thumb_set TIM1_BRK_IRQHandler,Default_Handler
 323              	
 324              	  .weak TIM1_UP_IRQHandler
 325              	  .thumb_set TIM1_UP_IRQHandler,Default_Handler
 326              	
 327              	  .weak TIM1_TRG_COM_IRQHandler
 328              	  .thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler
 329              	
 330              	  .weak TIM1_CC_IRQHandler
 331              	  .thumb_set TIM1_CC_IRQHandler,Default_Handler
 332              	
 333              	  .weak TIM2_IRQHandler
 334              	  .thumb_set TIM2_IRQHandler,Default_Handler
 335              	
 336              	  .weak TIM3_IRQHandler
 337              	  .thumb_set TIM3_IRQHandler,Default_Handler
 338              	
 339              	  .weak TIM4_IRQHandler
 340              	  .thumb_set TIM4_IRQHandler,Default_Handler
 341              	
 342              	  .weak I2C1_EV_IRQHandler
 343              	  .thumb_set I2C1_EV_IRQHandler,Default_Handler
ARM GAS  /tmp/cc15Trzm.s 			page 7


 344              	
 345              	  .weak I2C1_ER_IRQHandler
 346              	  .thumb_set I2C1_ER_IRQHandler,Default_Handler
 347              	
 348              	  .weak I2C2_EV_IRQHandler
 349              	  .thumb_set I2C2_EV_IRQHandler,Default_Handler
 350              	
 351              	  .weak I2C2_ER_IRQHandler
 352              	  .thumb_set I2C2_ER_IRQHandler,Default_Handler
 353              	
 354              	  .weak SPI1_IRQHandler
 355              	  .thumb_set SPI1_IRQHandler,Default_Handler
 356              	
 357              	  .weak SPI2_IRQHandler
 358              	  .thumb_set SPI2_IRQHandler,Default_Handler
 359              	
 360              	  .weak USART1_IRQHandler
 361              	  .thumb_set USART1_IRQHandler,Default_Handler
 362              	
 363              	  .weak USART2_IRQHandler
 364              	  .thumb_set USART2_IRQHandler,Default_Handler
 365              	
 366              	  .weak USART3_IRQHandler
 367              	  .thumb_set USART3_IRQHandler,Default_Handler
 368              	
 369              	  .weak EXTI15_10_IRQHandler
 370              	  .thumb_set EXTI15_10_IRQHandler,Default_Handler
 371              	
 372              	  .weak RTC_Alarm_IRQHandler
 373              	  .thumb_set RTC_Alarm_IRQHandler,Default_Handler
 374              	
 375              	  .weak USBWakeUp_IRQHandler
 376              	  .thumb_set USBWakeUp_IRQHandler,Default_Handler
ARM GAS  /tmp/cc15Trzm.s 			page 8


DEFINED SYMBOLS
startup_stm32f103xb.s:143    .isr_vector:00000000 g_pfnVectors
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 Default_Handler
startup_stm32f103xb.s:66     *ABS*:f108f85f BootRAM
startup_stm32f103xb.s:79     .text.Reset_Handler:00000000 Reset_Handler
startup_stm32f103xb.s:82     .text.Reset_Handler:00000000 $t
startup_stm32f103xb.s:91     .text.Reset_Handler:0000000c LoopCopyDataInit
startup_stm32f103xb.s:85     .text.Reset_Handler:00000004 CopyDataInit
startup_stm32f103xb.s:104    .text.Reset_Handler:00000020 LoopFillZerobss
startup_stm32f103xb.s:100    .text.Reset_Handler:0000001a FillZerobss
startup_stm32f103xb.s:128    .text.Default_Handler:00000000 Infinite_Loop
startup_stm32f103xb.s:129    .text.Default_Handler:00000000 $t
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 NMI_Handler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 HardFault_Handler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 MemManage_Handler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 BusFault_Handler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 UsageFault_Handler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 SVC_Handler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 DebugMon_Handler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 PendSV_Handler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 SysTick_Handler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 WWDG_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 PVD_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 TAMPER_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 RTC_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 FLASH_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 RCC_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 EXTI0_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 EXTI1_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 EXTI2_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 EXTI3_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 EXTI4_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 DMA1_Channel1_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 DMA1_Channel2_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 DMA1_Channel3_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 DMA1_Channel4_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 DMA1_Channel5_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 DMA1_Channel6_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 DMA1_Channel7_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 ADC1_2_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 USB_HP_CAN1_TX_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 USB_LP_CAN1_RX0_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 CAN1_RX1_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 CAN1_SCE_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 EXTI9_5_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 TIM1_BRK_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 TIM1_UP_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 TIM1_TRG_COM_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 TIM1_CC_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 TIM2_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 TIM3_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 TIM4_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 I2C1_EV_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 I2C1_ER_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 I2C2_EV_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 I2C2_ER_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 SPI1_IRQHandler
ARM GAS  /tmp/cc15Trzm.s 			page 9


startup_stm32f103xb.s:127    .text.Default_Handler:00000000 SPI2_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 USART1_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 USART2_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 USART3_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 EXTI15_10_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 RTC_Alarm_IRQHandler
startup_stm32f103xb.s:127    .text.Default_Handler:00000000 USBWakeUp_IRQHandler
startup_stm32f103xb.s:126    .text.Reset_Handler:00000034 $d
                   .debug_aranges:0000000c $d

UNDEFINED SYMBOLS
_sidata
_sdata
_edata
_sbss
_ebss
SystemInit
__libc_init_array
main
_estack
