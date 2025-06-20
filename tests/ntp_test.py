#!/usr/bin/env python
from contextlib import closing
from socket import socket, AF_INET, SOCK_DGRAM
import struct
import time
   
NTP_PACKET_FORMAT = "!12I"
NTP_DELTA = 2208988800  # 1970-01-01 00:00:00
NTP_QUERY = b'\x1b' + 47 * b'\0'  
    
def ntp_time(host="pool.ntp.org", port=123):
    with closing(socket( AF_INET, SOCK_DGRAM)) as s:
        s.settimeout(5)
        s.sendto(NTP_QUERY, (host, port))
        msg, address = s.recvfrom(1024)
    unpacked = struct.unpack(NTP_PACKET_FORMAT,
                   msg[0:struct.calcsize(NTP_PACKET_FORMAT)])
    return unpacked[10] + float(unpacked[11]) / 2**32 - NTP_DELTA


if __name__ == "__main__":
    print(time.ctime(ntp_time()))