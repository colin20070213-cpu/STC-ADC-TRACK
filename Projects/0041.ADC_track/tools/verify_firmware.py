"""Verify HEX bounds, low-address EEPROM exclusion and linker constraints."""
from pathlib import Path
import hashlib
import json
import re

root = Path(__file__).resolve().parent.parent
hex_file = root / 'MDK/Objects/Project_Template.hex'
memory = {}
base = 0
eof = False
for line in hex_file.read_text().splitlines():
    if not line:
        continue
    assert line.startswith(':'), 'Invalid HEX line'
    row = bytes.fromhex(line[1:])
    assert len(row) == row[0] + 5 and sum(row) % 256 == 0, 'Bad HEX checksum'
    count, offset, kind = row[0], int.from_bytes(row[1:3], 'big'), row[3]
    data = row[4:4+count]
    if kind == 0:
        for i, byte in enumerate(data):
            address = base+offset+i
            assert 0xFF0000 <= address < 0xFFFC00, f'Code outside linked program region: {address:06X}'
            assert address not in memory, f'Duplicate HEX address {address:06X}'
            memory[address] = byte
    elif kind == 1:
        eof = True
    elif kind == 2:
        base = int.from_bytes(data, 'big') << 4
    elif kind == 4:
        base = int.from_bytes(data, 'big') << 16
assert memory and eof and 0xFF0000 in memory, 'Missing reset vector or EOF'
link_map = (root/'MDK/Listings/Project_Template.map').read_text(errors='replace')
assert re.search(r'FF0000H\s+FF0000H\s+FFFBFFH\s+[0-9A-F]+H\s+CODE', link_map), 'Missing CODE bound'
assert re.search(r'000000H\s+FF0000H\s+FFFBFFH\s+[0-9A-F]+H\s+HCONST', link_map), 'Missing constant bound'
result = {'hex': str(hex_file.relative_to(root)), 'sha256': hashlib.sha256(hex_file.read_bytes()).hexdigest(),
          'flash_data_bytes': len(memory), 'first_address': f'{min(memory):06X}',
          'last_address': f'{max(memory):06X}',
          'reserved_parameter_cpu_addresses': ['FE0000-FE01FF','FE0200-FE03FF'],
          'eeprom_iap_slots': ['000000', '000200'],
          'isp_eeprom_minimum_bytes': 1024}
(root/'MDK/firmware-verification.json').write_text(json.dumps(result, indent=2)+'\n')
print(f'PASS: HEX checksums, reset vector, {len(memory)} bytes, last address {max(memory):06X}; both parameter sectors excluded')
