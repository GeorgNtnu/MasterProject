# -*- coding: utf-8 -*-
"""
Created on Wed Dec 13 08:35:13 2017

@author: enfenion
"""

import sys
import struct

class UbloxParser(object):
    SYNC_BYTES = b'^D'

    def __init__(self, fname):
        self.fname = fname
        self.file = open(self.fname, 'rb')

        self.last_timestamp = 0
        self.last_r_timestamp = 0
        self.last_corr_timestamp = 0

        self.fields = ['timestamp', 'ang_x', 'ang_y', 'dist',
                       'GPS_time']

    def read(self, n):
        buf = self.file.read(n)
        if not len(buf) == n:
            self.file.close()
            raise EOFError()
        return buf

    def sync_buf(self):
        sync = self.read(2)
        while not self.SYNC_BYTES == sync:
            sync = sync[1:] + self.read(1)
        return sync

    def parse_pvt(self, timestamp, data):
        tow = struct.unpack_from('<I', data, 0)[0]
        quality = struct.unpack_from('B', data, 20)[0]
        n_sat, lat, lon, height = struct.unpack_from('<B3i', data, 23)
        pack = [timestamp, lat * 1e-7, lon * 1e-7, height * 1e-3, n_sat, tow * 1e-3, quality]
        return pack

    def get_pack(self):
        self.sync_buf()
        header_len = 8-2
        timestamp_len = 8
        tov_toa_tot_len = 12
        pre_ublox_len = header_len + timestamp_len + tov_toa_tot_len
        pre_ublox_len = timestamp_len


        ublox_header_len = 6
        buf = self.read(pre_ublox_len + ublox_header_len)

        ublox_package = buf[pre_ublox_len:]

        if not ublox_package[:2] == b'\xb5\x62':
            print('Sync error')
            return None

        ublox_pack_id = ublox_package[2:4]

        datalen = struct.unpack('H', ublox_package[4:6])[0]

        if datalen == 0:
            return None

        ublox_data = self.read(datalen + 2)

        if ublox_pack_id == b'\x01\x07':
            pc_time = struct.unpack('<d',buf[:pre_ublox_len])[0]
            # timestamp = struct.unpack_from('d', buf, header_len)[0]
            return self.parse_pvt(pc_time, ublox_data)
        return None




if __name__ == '__main__':
    FILENAME = sys.argv[1]
    print('Opening %s' % FILENAME)

    parser = UbloxParser(FILENAME)
    try:
        running = True
        while running:
            pack = parser.get_pack()
            if pack is None:
                continue
            print('%10.9f\t% 8.6f\t% 8.6f\t% 8.3f\t%10d\t%10.2f\t%d' % tuple(pack))
    except EOFError:
        pass

