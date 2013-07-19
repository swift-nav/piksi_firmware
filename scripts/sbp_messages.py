
import re
import os

header_file = os.path.join(
  os.path.dirname(__file__),
  '..', 'src', 'sbp_messages.h'
)

with open(header_file, 'r') as f:
  sbph = f.read()

msgs = re.findall('^\s*#define\s+MSG_([A-Z_]+)\s+0x([A-F0-9][A-F0-9])',
                  sbph, re.MULTILINE)

for name, id in msgs:
  globals()[name] = int(id, 16)

