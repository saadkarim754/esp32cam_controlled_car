import gzip
import sys

def html_to_c_array(input_file, output_header, array_name):
    with open(input_file, 'rb') as f:
        html_data = f.read()

    compressed_data = gzip.compress(html_data)

    with open(output_header, 'w') as f:
        f.write(f"// File: {output_header}, Size: {len(compressed_data)}\n")
        f.write(f"#define {array_name}_len {len(compressed_data)}\n")
        f.write(f"const unsigned char {array_name}[] = {{\n")

        for i, byte in enumerate(compressed_data):
            f.write(f" 0x{byte:02X},")
            if (i + 1) % 16 == 0:
                f.write("\n")

        f.write("\n};\n")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python html_to_header.py input.html output.h array_name")
        sys.exit(1)

    html_to_c_array(sys.argv[1], sys.argv[2], sys.argv[3])
