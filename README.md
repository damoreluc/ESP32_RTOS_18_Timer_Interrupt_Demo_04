# ESP32_RTOS_18_Timer_Interrupt_Demo_04

## Impiego della notifica diretta al task

* Acquisizione di valori analogici dal canale 34 dell'ADC nella ISR del timer hardware, periodo 100ms
* In un task viene stampato il valore del campione acquisito dalla ISR.
  
Sincronizzazione tra ISR e task mediante notifica diretta.
