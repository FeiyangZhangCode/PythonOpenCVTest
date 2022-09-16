import binascii
import serial
import time
import cv2
import crcmod


crc8 = crcmod.predefined.Crc('crc-8')
hex_stop = bytes.fromhex('aa0100000101')
crc8.update(hex_stop)
str_crc8 = hex(crc8.crcValue)[2:]
hex_crc8 = bytes.fromhex(str_crc8)
data_1 = binascii.b2a_hex(hex_stop)

print(str_crc8, hex_crc8)
print(data_1)
