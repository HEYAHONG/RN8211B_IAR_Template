;/*****************************************************************************
; * @file:    startup_RN8215.s
; * @purpose: CMSIS Cortex-M0 Core Device Startup File
; *           for RN8215
; * @version: V1.3
; * @date:    2019/7/19
; *
; *
; *****************************************************************************/

        MODULE  ?cstartup
        
        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3) ;; 8 bytes alignment

        SECTION .intvec:CODE:NOROOT(2);; 4 bytes alignment
        
        EXTERN  SystemInit	
        EXTERN  __iar_program_start
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size
        
        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     __iar_program_start
        
        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     0
        DCD     0
        DCD     0
__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     0
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler
        
        ; External Interrupts
        DCD     SYSCLKCAL_HANDLER
        DCD     CMP_HANDLER
        DCD     VCH_HANDLER
        DCD     RTC_HANDLER
        DCD     EMU_HANDLER
        DCD     MADC_HANDLER
        DCD     UART0_HANDLER
        DCD     UART1_HANDLER
        DCD     UART2_HANDLER
        DCD     UART3_HANDLER
        DCD     SPI_HANDLER
        DCD     I2C_HANDLER   
        DCD     ISO78160_HANDLER
        DCD     ISO78161_HANDLER
        DCD     TC0_HANDLER
        DCD     TC1_HANDLER
        DCD     UART4_HANDLER
        DCD     UART5_HANDLER
        DCD     WDT_HANDLER
        DCD     KBI_HANDLER
        DCD     LCD_HANDLER
        DCD     CP_HANDLER
        DCD     DMA_HANDLER       
        DCD     NVM_HANDLER
        DCD     EXT0_HANDLER
        DCD     EXT1_HANDLER
        DCD     EXT2_HANDLER
        DCD     EXT3_HANDLER
        DCD     EXT4_HANDLER
        DCD     EXT5_HANDLER
        DCD     EXT6_HANDLER
        DCD     EXT7_HANDLER
        
__Vectors_End
__Vectors       EQU   __vector_table
__Vectors_Size  EQU     __Vectors_End - __Vectors

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
         THUMB
         PUBWEAK Reset_Handler   
         SECTION .text:CODE:REORDER(2)       ; 4 bytes alignment
Reset_Handler
         IMPORT  SystemInit
         IMPORT  __main
         LDR     R0, =SystemInit
         BLX     R0
         LDR     R0, =__iar_program_start
         BX      R0
         
         PUBWEAK NMI_Handler
         SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
         B       NMI_Handler
         
         PUBWEAK HardFault_Handler
         SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
         B       HardFault_Handler
         
         PUBWEAK SVC_Handler
         SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
         B       SVC_Handler
         
         PUBWEAK PendSV_Handler
         SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler  
         B       PendSV_Handler
         
         PUBWEAK SysTick_Handler
         SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler 
         B       SysTick_Handler
         
         PUBWEAK EXT0_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
EXT0_HANDLER    
         B       EXT0_HANDLER
         
         PUBWEAK EXT1_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
EXT1_HANDLER    
         B       EXT1_HANDLER
         
         PUBWEAK EXT2_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
EXT2_HANDLER    
         B       EXT2_HANDLER
           
         PUBWEAK EXT3_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
EXT3_HANDLER    
         B       EXT3_HANDLER
           
         PUBWEAK EXT4_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
EXT4_HANDLER    
         B       EXT4_HANDLER
           
         PUBWEAK EXT5_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
EXT5_HANDLER    
         B       EXT5_HANDLER
           
         PUBWEAK EXT6_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
EXT6_HANDLER  
         B       EXT6_HANDLER
             
         PUBWEAK EXT7_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
EXT7_HANDLER  
         B       EXT7_HANDLER
               
         PUBWEAK DMA_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
DMA_HANDLER  
         B       DMA_HANDLER
              
         PUBWEAK CP_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
CP_HANDLER   
         B       CP_HANDLER
              
         PUBWEAK RTC_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
RTC_HANDLER  
         B       RTC_HANDLER
              
         PUBWEAK EMU_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
EMU_HANDLER   
         B       EMU_HANDLER
             
         PUBWEAK MADC_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
MADC_HANDLER   
         B       MADC_HANDLER
            
         PUBWEAK UART0_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
UART0_HANDLER  
         B       UART0_HANDLER
            
         PUBWEAK UART1_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
UART1_HANDLER   
         B       UART1_HANDLER
           
         PUBWEAK UART2_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
UART2_HANDLER   
         B       UART2_HANDLER
           
         PUBWEAK UART3_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
UART3_HANDLER   
         B       UART3_HANDLER
           
         PUBWEAK SPI_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
SPI_HANDLER     
         B       SPI_HANDLER
           
         PUBWEAK I2C_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
I2C_HANDLER     
         B       I2C_HANDLER
           
         PUBWEAK ISO78161_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
ISO78161_HANDLER
         B       ISO78161_HANDLER
           
         PUBWEAK ISO78160_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
ISO78160_HANDLER
         B       ISO78160_HANDLER
           
         PUBWEAK TC0_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
TC0_HANDLER   
         B       TC0_HANDLER
             
         PUBWEAK TC1_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
TC1_HANDLER    
         B       TC1_HANDLER
            
         PUBWEAK UART4_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
UART4_HANDLER  
         B       UART4_HANDLER
           
         PUBWEAK UART5_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
UART5_HANDLER  
         B       UART5_HANDLER
            
         PUBWEAK WDT_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
WDT_HANDLER  
         B       WDT_HANDLER
              
         PUBWEAK KBI_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
KBI_HANDLER  
         B       KBI_HANDLER
              
         PUBWEAK LCD_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
LCD_HANDLER  
         B       LCD_HANDLER
              
         PUBWEAK VCH_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
VCH_HANDLER   
         B       VCH_HANDLER
         PUBWEAK NVM_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
NVM_HANDLER  
         B       NVM_HANDLER
             
         PUBWEAK CMP_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
CMP_HANDLER  
         B       CMP_HANDLER
                      
         PUBWEAK SYSCLKCAL_HANDLER
         SECTION .text:CODE:REORDER:NOROOT(1)
SYSCLKCAL_HANDLER  
         B       SYSCLKCAL_HANDLER
            
         END
