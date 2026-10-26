"""Export actual application framebuffer snapshots without external packages."""
from pathlib import Path
import struct
import zlib

root = Path(__file__).resolve().parent.parent/'MDK'
names = ['menu-1','menu-2','cal-menu','servo-menu-1','servo-menu-2','menu-3','pid-menu',
         'cal','body-capture','body-ready','pid','pid-restore','pid-confirm',
         'servo-direction','left-direction','right-direction','speed-target','speed-pid','speed','servo','run','lost']
scale, gap = 4, 12
row_count = (len(names)+2)//3
width, height = 3*128*scale+4*gap, row_count*64*scale+(row_count+1)*gap
pixels = bytearray([35])*(width*height)
for n, name in enumerate(names):
    frame = (root/f'ui-{name}.bin').read_bytes()
    assert len(frame) == 1024
    for y in range(64):
        for x in range(128):
            value = 255 if frame[(y//8)*128+x] & (1 << (y%8)) else 0
            ox = gap+(n%3)*(128*scale+gap)+x*scale
            oy = gap+(n//3)*(64*scale+gap)+y*scale
            for dy in range(scale):
                pixels[(oy+dy)*width+ox:(oy+dy)*width+ox+scale] = bytes([value])*scale
def chunk(kind, data):
    return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data))
rows = b''.join(b'\0'+pixels[y*width:(y+1)*width] for y in range(height))
png = b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,0,0,0,0))
png += chunk(b'IDAT',zlib.compress(rows))+chunk(b'IEND',b'')
(root/'ui-preview.png').write_bytes(png)
print(root/'ui-preview.png')
