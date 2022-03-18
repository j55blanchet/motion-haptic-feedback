from time import sleep
import network

ssid = 'humanmotionlab'
password = 'hns4life'
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect(ssid, password)

print()
print('Connecting to network "' + ssid + '"')
while(sta_if.isconnected() == False):
    print('.', end='')    
    sleep(1.0)
    
print('connected')
print('Network config:', sta_if.ifconfig())

