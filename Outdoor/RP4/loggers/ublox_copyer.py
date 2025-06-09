import datetime
import serial
import struct
import sys
import time
import os

print(sys.argv)
filename, device, baudrate = sys.argv[1:4]

SYNC_WORD = b'^D' #unique sync word for this parser, NOT SENTIBOARD SYNC WORD!
sync_id = b'\xb5\x62'
has_opened_file = False
with open(filename, 'wb+') as f:
    while True:
        try:
            if os.path.isfile(device):
                print('is file')
                if has_opened_file:
                    break
                com = open(device,'rb')
                has_opened_file = True
            else:
                print('is com')
                com = serial.Serial(device, baudrate=baudrate)
                com.timeout = 1
                print('opened com')
                com.read(3000)
                print('cleared com')


            skipped = 0
            while True:
                buf = com.read(12)
                try:
                    start_ix = buf.index(sync_id)
                except ValueError:
                    skipped+=1
                    continue

                if skipped > 0:
                    print('Skipped: ', skipped)
                    skipped = 0
                # ensure that buf contains the 12 first bytes of 
                # the ubx frame, i.e. the header and 6 bytes into 
                # the payload
                if start_ix > 0:
                    buf = buf[start_ix:] + com.read(start_ix)

                    
                pkglen = struct.unpack('<H', buf[4:6])[0]
                binarystamp = struct.pack('<d',time.time())
                
                f.write(SYNC_WORD)
                f.write(binarystamp)
                f.write(buf)
                # read the rest of the payload and the 2 checksum 
                # bytes (we started 6 bytes into the payload, and 
                # the pack ends with 2 checksum bytes: 6-2 = 4)
                f.write(com.read(pkglen - 4))
                f.flush()
                continue

                
                #print(repr(buf[2:4]), repr(buf[:6]), pkglen)

                buf += com.read(pkglen)
                continue
                sync_ix = find_sync(buf)

                if sync_ix > 2:
                    f.write(buf[:sync_ix-6])
                elif sync_ix < 0:
                    #print('No syncword found. Flushing.')
                    f.write(buf)
                    continue
                
                binarystamp = struct.pack('<d',time.time())
                
                f.write(binarystamp)
                f.write(buf[sync_ix-6:])
                f.flush()
        except Exception as e:
            print('Error {}'.format(e))
            raise

        time.sleep(0.001)
