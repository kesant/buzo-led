from machine import ADC,Pin,DAC
import time
#The range of our ADC on ESP32 is 12 bits
adc=ADC(Pin(35))
adc.atten(ADC.ATTN_11DB)
adc.width(ADC.WIDTH_12BIT)
#dac =DAC(Pin(25))

try:
    
    while True:
        
        adcVal=adc.read()
        if adcVal<1900:
            adcVal=1900
            
        dacVal=int(255-((255/2195)*(4095-adcVal)))
        voltage = adcVal / 4095.0 * 3.3
        #dac.write(dacVal)
        print("ADC Val:",adcVal,"DACVal:",dacVal,"Voltage:",voltage,"V")
        time.sleep_ms(100)
        
except:
    
    pass
