#!/usr/bin/env python3
import sys

def wav_to_header(input_file, output_file):
    with open(input_file, 'rb') as f:
        data = f.read()
    
    variable_name = input_file.split('/')[-1].split('\\')[-1].split('.')[0] + '_wav'
    
    with open(output_file, 'w') as f:
        f.write(f'#ifndef {variable_name.upper()}_H\n')
        f.write(f'#define {variable_name.upper()}_H\n\n')
        f.write(f'#include <stdint.h>\n\n')
        f.write(f'const uint8_t {variable_name}[] = {{\n')
        
        for i, byte in enumerate(data):
            if i % 12 == 0:
                f.write('    ')
            f.write(f'0x{byte:02x}, ')
            if (i + 1) % 12 == 0:
                f.write('\n')
        
        f.write('\n};')
        f.write(f'const uint32_t {variable_name}_len = sizeof({variable_name});\n\n')
        f.write(f'#endif // {variable_name.upper()}_H\n')

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python wav_to_header.py input.wav output.h')
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    wav_to_header(input_file, output_file)
    print(f'Converted {input_file} to {output_file}')