

# This example demonstrates a UART periperhal.
#ESP32 CONECTADO AL BUZO
#VA A RECIBIR LA INFORMACION DEL SENSOR KY037 Y VA A DAR LOS VALORES A LAS LUCES LEDS DIRECCIONABLES

import bluetooth
import random
import struct
import time
import neopixel
from ble_advertising import advertising_payload
from machine import Pin

from micropython import const
# NodeMCU ESP-C3-32S-Kit onboard LEDs assignment

###########################################
button = Pin(13, Pin.IN,Pin.PULL_UP)
modo=0
pin = Pin(2, Pin.OUT)
np = neopixel.NeoPixel(pin, 8)

brightness=0.1         #brightness: 0-1.0
red=0                  #red
green=0                #green
blue=0                 #blue
###########################################
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_UART_UUID = bluetooth.UUID(
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (
    bluetooth.UUID(
        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_READ | _FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID(
        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE,
)
_UART_SERVICE = (
    _UART_UUID,
    (_UART_TX, _UART_RX),
)


class BLESimplePeripheral:
    def __init__(self, ble, name="mpy-uart"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_tx,
          self._handle_rx),) \
          = self._ble.gatts_register_services(
              (_UART_SERVICE,))
        self._connections = set()
        self._write_callback = None
        self._payload = \
                      advertising_payload(
                          name=name, services=[_UART_UUID])
        self._advertise()

    def _irq(self, event, data):
        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("New connection", conn_handle)
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Disconnected", conn_handle)
            self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle)
            if (value_handle == self._handle_rx and
                self._write_callback):
                self._write_callback(value)

    def send(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle,
                                   self._handle_tx,
                                   data)

    def is_connected(self):
        return len(self._connections) > 0

    def _advertise(self, interval_us=500000):
        print("Starting advertising")
        self._ble.gap_advertise(interval_us,
                                adv_data=self._payload)

    def on_write(self, callback):
        self._write_callback = callback

def wheel(pos):
    global red,green,blue
    WheelPos=pos%255
    if WheelPos<85:
        red=(255-WheelPos*3)
        green=(WheelPos*3)
        blue=0
    elif WheelPos>=85 and WheelPos<170:
        WheelPos -= 85;
        red=0
        green=(255-WheelPos*3)
        blue=(WheelPos*3)
    else :
        WheelPos -= 170;
        red=(WheelPos*3)
        green=0
        blue=(255-WheelPos*3)
        
def rainbow(flag):
    while flag:
        for i in range(0,255):
            for j in range(0,8):
                wheel(i+j*255//8)
                np[j]=(int(red*brightness),int(green*brightness),int(blue*brightness))
                np.write()
            time.sleep_ms(5)
        flag=False

def demo():
    global modo
    ble = bluetooth.BLE()
    p = BLESimplePeripheral(ble)

    def on_rx(v):
        # command received from central,
        # and send back the command to centrol.
        print("RX", v)
        valor_sensor=int(v.decode())
        print(valor_sensor)
        p.send(str(modo))
  """      
        if v == b"presionado":
            rainbow(True)
            print("command received: ", "presionado")
            p.send("from peripheral:")
            p.send("presionado")
        elif v == b"off":
            rainbow(False)
            print("command received: ", "off")
            p.send("from peripheral:")
            p.send("off")
"""    
    p.on_write(on_rx)

    i = 0
    
    def setear_modo():
        if modo==1:
            modo=0
        else:
            modo=1
        
    while True:
        
        if not button.value():
          time.sleep_ms(20)
          if not button.value():
              reverseGPIO()
              while not button.value():
                  time.sleep_ms(20)
              
        """
        if p.is_connected():
            # Short burst of queued notifications.
            for _ in range(3):
                data = str(i) + "_"
                print("TX", data)
                p.send(data)
                i += 1
        """
        time.sleep_ms(100)


if __name__ == "__main__":
    demo()