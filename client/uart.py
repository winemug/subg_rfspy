from serial import Serial
import time

def pb(b):
    return ("".join(["%02x" % x for x in b]))

def get_response(ser: Serial):
    while True:
        x = ser.read(1)
        if len(x) > 0:
            plen = x[0]
            print(f'len: {plen}')
            x = ser.read(plen)
            print(f'data: {pb(x)}')
            break;
    while True:
        x = ser.read(1)
        if len(x) == 0:
            break
        print(f'excess data: {pb(x)}')

def uart_listen(ser: Serial, len: int):
    x = ser.read(len)
    print(pb(x), end='')

if __name__ == '__main__':
    br = 35600 # 131/10 38400 -> 35600
    ser = Serial('/dev/ttyS0', baudrate=br, bytesize=8, parity='N', stopbits=1, timeout=0.5, xonxoff=0, rtscts=0)

    while True:
        ser.write([0x01, 0x01]);
        get_response(ser)

    ser.close()
