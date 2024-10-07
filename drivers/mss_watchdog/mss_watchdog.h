/*******************************************************************************
 * (c) Copyright 2009-2010 Microsemi SoC Products Group.  All rights reserved.
 * 
 * SmartFusion2 Microcontroller Subsystem (MSS) Watchdog bare metal software
 * driver.
 *
 * SVN $Revision: 4553 $
 * SVN $Date: 2012-08-23 15:08:51 +0100 (Thu, 23 Aug 2012) $
 */
/*=========================================================================*//**
  @section intro_sec Introduction
  The SmartFusion2 microcontroller subsystem (MSS) includes a watchdog timer used
  to detect system lockups.
  This driver provides a set of functions for controlling the MSS watchdog as
  part of a bare metal system where no operating system is available. These
  drivers can be adapted for use as part of an operating system but the
  implementation of the adaptation layer between this driver and the operating
  system's driver model is outside the scope of this driver.
    
  @section hw_dependencies Hardware Flow Dependencies
  The configuration of the MSS watchdog's mode of operation, reload value and
  refresh window is done as part of the hardware flow.
    
  @section theory_op Theory of Operation
  The watchdog driver uses the SmartFusion2 "Cortex Microcontroler Software
  Interface Standard - Peripheral Access Layer" (CMSIS-PAL) to access hadware
  registers. You must ensure that the SmartFusion2 CMSIS-PAL is either included
  in the software toolchain used to build your project or is included in your
  project. The most up-to-date SmartFusion2 CMSIS-PAL files can be obtained using
  the Actel Firmware Catalog.
  
  The watchdog driver functions are grouped into the following categories:
    - Initialization
    - Reading the watchdog timer current value and status
    - Refreshing the watchdog timer value
    - Time-out and wake-up interrupts control
  
  The watchdog driver is initialized through a call to the MSS_WD_init()
  function. The MSS_WD_init() function must be called before any other watchdog
  driver functions can be called.
  
  The watchdog timer cannot be disabled through software. Only the interrupts
  can be disabled.
  
  The watchdog timer current value can be read using the MSS_WD_current_value()
  function. The watchdog status can be read using the MSS_WD_status() function.
  These functions are typically required when using the watchdog configured with
  an allowed refresh window to check if a watchdog reload is currently allowed.
  
  The watchdog timer value is reloaded using the MSS_WD_reload() function. The
  value reloaded into the watchdog timer down counter is the value specified in
  the watchdog configurator software tools as part of the hardware flow.
  
  The occurance of a time out evetn can be detected through the use of the
  MSS_WD_timeout_occured() function. This function would be typically used at
  the start of the application to detect whether the application is starting as
  a result of a power-on reset or a watchdog reset. The time out event must be
  cleared through a call to function MSS_WD_clear_timeout_event() in order to 
  allow the detection of subsequent time out events ordifferentiating between
  a Cortex-M3 initiated system reset and watchdog reset.
  
  The watchdog timer can generate interrupts instead of resetting the system
  when its down-counter timer expires. These time-out interrupts are controlled
  using the following functions:
    - MSS_WD_enable_timeout_irq
    - MSS_WD_disable_timeout_irq
    - MSS_WD_clear_timeout_irq
  
  The watchdog timer is external to the Cortex-M3 processor core and operates
  even when the Cortex-M3 is in sleep mode. A wakeup interrupt can be generated
  by the watchdog timer to wakeup the Cortext-M3 when the watchdog timer value
  reaches the allowed refresh window while the Cortex-M3 is in sleep mode. The
  watchdog driver provides the following functions to control wakeup interrupts:
    - MSS_WD_enable_wakeup_irq
    - MSS_WD_disable_wakeup_irq
    - MSS_WD_clear_wakeup_irq
    
 *//*=========================================================================*/

#ifndef MSS_WATCHDOG_H_
#define MSS_WATCHDOG_H_

#ifdef __cplusplus
extern "C" {
#endif 

#include "../../CMSIS/m2sxxx.h"

/***************************************************************************//**
 * The MSS_WDOG_TIMEOUT_IRQ_ENABLE_BIT_MASK macro is a bit mask specifying the bit
 * used to enable the time out interrupt within the watchdog's WDOGCONTROL
 * register.
 */
#define MSS_WDOG_TIMEOUT_IRQ_ENABLE_BIT_MASK  (uint32_t)0x00000001U

/***************************************************************************//**
  The MSS_WDOG_WAKEUP_IRQ_ENABLE_BIT_MASK macro is a bit mask specifying the bit
  used to enable the wake up interrupt within the watchdog's WDOGCONTROL
  register.
 */
#define MSS_WDOG_WAKEUP_IRQ_ENABLE_BIT_MASK   (uint32_t)0x00000002U

/***************************************************************************//**
  The MSS_WDOG_TIMEOUT_IRQ_CLEAR_BIT_MASK macro is a bit mask specifying the bit
  used to clear the time out interrupt within the watchdog's WDOGRIS register.
 */
#define MSS_WDOG_TIMEOUT_IRQ_CLEAR_BIT_MASK   (uint32_t)0x00000001U

/***************************************************************************//**
  The MSS_WDOG_WAKEUP_IRQ_CLEAR_BIT_MASK macro is a bit mask specifying the bit
  used to clear the wake up interrupt within the watchdog's WDOGRIS register.
 */
#define MSS_WDOG_WAKEUP_IRQ_CLEAR_BIT_MASK    (uint32_t)0x00000002U

/***************************************************************************//**
  The MSS_WDOG_REFRESH_KEY macro holds the magic value which will cause a reload
  of the watchdog's down counter when written to the watchdog's WDOGREFRESH
  register.
 */
#define MSS_WDOG_REFRESH_KEY    (uint32_t)0xAC15DE42U

/***************************************************************************//**
  The MSS_WD_init() function initializes the watchdog timer. It clears any
  pending watchdog interrupts and refreshes the watchdog counter.
  Please note that calling this function does not affect the hardware reported
  time out event indicator reported by a call to function
  MSS_WD_timeout_occured(). It is therefore allowed to call the MSS_WD_init()
  function before calling MSS_WD_timeout_occured().
 
  @return
    This function does not return a value.
 */
static __INLINE void MSS_WD_init(void)
{
    uint32_t wdg_value;
    
    /* Disable interrupts. */
    WATCHDOG->WDOGCONTROL &= ~(MSS_WDOG_TIMEOUT_IRQ_ENABLE_BIT_MASK | MSS_WDOG_WAKEUP_IRQ_CLEAR_BIT_MASK);
    
    /* Clear any existing interrupts. */
    WATCHDOG->WDOGRIS = MSS_WDOG_TIMEOUT_IRQ_CLEAR_BIT_MASK | MSS_WDOG_WAKEUP_IRQ_CLEAR_BIT_MASK;
    
    /* Clear any pending wakeup interrupt from Cortex-M3 NVIC */
    NVIC_ClearPendingIRQ(WdogWakeup_IRQn);
    
    /* Reload watchdog with new load value if in allowed window. */
    wdg_value = WATCHDOG->WDOGVALUE;
    if(wdg_value < WATCHDOG->WDOGMVRP)
    {
        WATCHDOG->WDOGREFRESH = MSS_WDOG_REFRESH_KEY;
    }
}

/***************************************************************************//**
  The MSS_WD_reload() function causes the watchdog to reload its down counter timer
  with the load value configured through the call to WD_init(). This function 
  must be called regularly to avoid a system reset.
 
  @return
    This function does not return a value.
 */
static __INLINE void MSS_WD_reload( void )
{
    WATCHDOG->WDOGREFRESH = MSS_WDOG_REFRESH_KEY;
}

/***************************************************************************//**
  The MSS_WD_current_value() function returns the current value of the watchdog's
  down counter.
 
  @return
    This function returns the current value of the watchdog down counter.
 */
static __INLINE uint32_t MSS_WD_current_value( void )
{
    return WATCHDOG->WDOGVALUE;
}

/***************************************************************************//**
  The MSS_WD_status() function returns the status of the watchdog.
 
  @return 
    The MSS_WD_status() function returns the status of the watchdog. A value of
    0 indicates that watchdog counter has reached the forbidden window and that
    a reload should not be done. A value of 1 indicates that the watchdog counter
    is within the permitted window and that a reload is allowed.
 */
static __INLINE uint32_t MSS_WD_status( void )
{
    return WATCHDOG->WDOGSTATUS;
}

/***************************************************************************//**
  The MSS_WD_enable_timeout_irq() function enables the watchdog’s time out
  interrupt which is connected to the Cortex-M3 NMI interrupt.
  The NMI_Handler() function will be called when a watchdog time out occurs. You
  must provide the implementation of the NMI_Handler() function to suit your
  application.
 
  @return
    This function does not return a value.
 
  Example:
  @code
  #include "mss_watchdog.h"
  int main( void )
  {
      MSS_WD_init();
      MSS_WD_enable_timeout_irq();
      for (;;)
      {
          main_task();
      }
  }
 
  void NMI_Handler( void )
  {
      process_timeout();
      MSS_WD_clear_timeout_irq();
  }
  @endcode
 */
static __INLINE void MSS_WD_enable_timeout_irq( void )
{
/* ADD ASSERTION TO CHECK THAT HARDWARE IS CONFIGURED FOR INTERRUPT MODE? */
    WATCHDOG->WDOGCONTROL |= MSS_WDOG_TIMEOUT_IRQ_ENABLE_BIT_MASK;
}

/***************************************************************************//**
  The WD_disable_timeout_irq() function disables the generation of the NMI
  interrupt when the watchdog times out.
 
  @return
    This function does not return a value.
 */
static __INLINE void MSS_WD_disable_timeout_irq( void )
{
    WATCHDOG->WDOGCONTROL &= ~MSS_WDOG_TIMEOUT_IRQ_ENABLE_BIT_MASK;
}

/***************************************************************************//**
  The MSS_WD_enable_wakeup_irq() function enables the SmartFusion2 wakeup
  interrupt. The WdogWakeup_IRQHandler() function will be called when a wake up
  interrupt occurs. You must provide the implementation of the WdogWakeup_IRQHandler()
  function to suit your application.
 
  @return
    This function does not return a value.
 
  Example:
  @code
  #include "mss_watchdog.h"
  int main( void )
  {
      MSS_WD_init();
      MSS_WD_enable_wakeup_irq();
      for (;;)
      {
          main_task();
          cortex_sleep();
      }
  }
 
  void WdogWakeup_IRQHandler( void )
  {
      process_wakeup();
      MSS_WD_clear_wakeup_irq();
  }
  @endcode
 */
static __INLINE void MSS_WD_enable_wakeup_irq( void )
{
    WATCHDOG->WDOGCONTROL |= MSS_WDOG_WAKEUP_IRQ_ENABLE_BIT_MASK;
    NVIC_EnableIRQ( WdogWakeup_IRQn );
}

/***************************************************************************//**
  The MSS_WD_disable_wakeup_irq() function disables the SmartFusion2 wakeup
  interrupt. 
 
  @return
    This function does not return a value.
 */
static __INLINE void MSS_WD_disable_wakeup_irq( void )
{
    WATCHDOG->WDOGCONTROL &= ~MSS_WDOG_WAKEUP_IRQ_ENABLE_BIT_MASK;
}

/***************************************************************************//**
  The MSS_WD_clear_timeout_irq() function clears the watchdog’s time out
  interrupt which is connected to the Cortex-M3 NMI interrupt.
  Calling MSS_WD_clear_timeout_irq() results in clearing the Cortex-M3 NMI interrupt.
  Note: The MSS_WD_clear_timeout_irq() function must be called as part of the
  timeout interrupt service routine (ISR) in order to prevent the same interrupt
  event retriggering a call to the wakeup ISR.
 
  @return
    The example below demonstrates the use of the MSS_WD_clear_timeout_irq()
    function as part of the NMI interrupt service routine.

    Example:
  @code
  void NMI_Handler( void )
  {
      process_timeout();
      MSS_WD_clear_timeout_irq();
  }
  @endcode
 */
static __INLINE void MSS_WD_clear_timeout_irq( void )
{
    WATCHDOG->WDOGRIS = MSS_WDOG_TIMEOUT_IRQ_CLEAR_BIT_MASK;
    /*
     * Perform a second write to ensure that the first write completed before 
     * returning from this function. This is to account for posted writes across
     * the AHB matrix. The second write ensures that the first write has
     * completed and that the interrupt line has been de-asserted by the time
     * the function returns. Omitting the second write may result in a delay
     * in the de-assertion of the interrupt line going to the Cortex-M3 and a
     * retriggering of the interrupt.
     */
    WATCHDOG->WDOGRIS = MSS_WDOG_TIMEOUT_IRQ_CLEAR_BIT_MASK;
}

/***************************************************************************//**
  The MSS_WD_clear_wakeup_irq() function clears the wakeup interrupt.
  Note: The MSS_WD_clear_wakeup_irq() function must be called as part of the
  wakeup interrupt service routine (ISR) in order to prevent the same interrupt
  event retriggering a call to the wakeup ISR. This function also clears the
  interrupt in the Cortex-M3 interrupt controller through a call to
  NVIC_ClearPendingIRQ().
 
  @return
    This function does not return a value.

    Example:
    The example below demonstrates the use of the MSS_WD_clear_wakeup_irq() function
    as part of the wakeup interrupt service routine.  
    @code
    void WdogWakeup_IRQHandler( void )
    {
        do_interrupt_processing();
        
        MSS_WD_clear_wakeup_irq();
    }
    @endcode
*/
static __INLINE void MSS_WD_clear_wakeup_irq( void )
{
    WATCHDOG->WDOGRIS = MSS_WDOG_WAKEUP_IRQ_CLEAR_BIT_MASK;
    NVIC_ClearPendingIRQ( WdogWakeup_IRQn );
}

/***************************************************************************//**
  The MSS_WD_timeout_occured() function reports the occurance of a timeout
  event. It can be used to detect if the reset is resetting as part of a
  watchdog timeout.
 
  @return
    0: No watchdog timeout event occured.
    1: A timeout event occured.

  Example:
  @code
  #include "mss_watchdog.h"
  int main( void )
  {
      uint32_t wdg_reset;
      MSS_WD_init();
      wdg_reset = MSS_WD_timeout_occured();
      if(wdg_reset)
      {
          log_watchdog_event();
          MSS_WD_clear_timeout_event();
      }
      
      for(;;)
      {
          main_task();
      }
  }
*/
static __INLINE uint32_t MSS_WD_timeout_occured(void)
{
    return SYSREG->WDOGTIMEOUTEVENT;
}

/***************************************************************************//**
  The MSS_WD_clear_timeout_event() function clears the hardware's report of a
  time out event. This function would typically be called after a call to
  MSS_WD_timeout_occured() to clear the hardware's report of a time out event
  and therefore detecting the occurance of subsequent time out events.
 
  @return
    This function does not return a value.
    
  Example:
  @code
  #include "mss_watchdog.h"
  int main( void )
  {
      uint32_t wdg_reset;
      MSS_WD_init();
      wdg_reset = MSS_WD_timeout_occured();
      if(wdg_reset)
      {
          log_watchdog_event();
          MSS_WD_clear_timeout_event();
      }
      
      for(;;)
      {
          main_task();
      }
  }
*/
static __INLINE void MSS_WD_clear_timeout_event(void)
{
    /* Clear WDOGTIMEOUTEVENT system register. */
    SYSREG->WDOGTIMEOUTEVENT = 1u;
}

#ifdef __cplusplus
}
#endif

#endif /* MSS_WATCHDOG_H_ */
