## sisyphus-red

# sensor node: 
build sensor node with:  
> ```west build -b thingy52/nrf52832 mycode/apps/sensor --pristine```  

flash sensor node with:  
> ```west flash --runner jlink```  

view sensor node with:  
> ```JLinkRTTLogger -Device nrf52832_xxaa A -RTTChannel 1 -if SWD -Speed 4000 ~/rtt.log```  
> ```nc localhost 19021```  

# mobile node
build mobile node with:  
>  ```west build -b nrf52840dk/nrf52840 --pristine auto mycode/apps/mobile```  

flash mobile node with:  
> ```west flash --runner jlink```  

view mobile node with:  
> ```screen /dev/ttyACM0 115200```  