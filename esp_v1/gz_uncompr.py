import gzip
import re
from io import BytesIO

# Step 1: Read your C array
with open('camera_index.h', 'r') as f:
    content = f.read()

# Step 2: Extract the hex values using regex
hex_data = re.findall(r'0x[0-9A-Fa-f]{2}', content)
byte_data = bytes([int(h, 16) for h in hex_data])

# Step 3: Decompress with gzip
with gzip.GzipFile(fileobj=BytesIO(byte_data)) as gz:
    html_data = gz.read()

# Step 4: Save to file
with open('output.html', 'wb') as f:
    f.write(html_data)

print("Decompressed HTML saved as output.html")
