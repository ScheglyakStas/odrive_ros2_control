import os
import subprocess
import tempfile
import shutil

def compile_and_get_json_crc(endpoints_file='endpoints.hpp', crc_file='crc.hpp'):
    # Create temporary directory
    temp_dir = tempfile.mkdtemp()

    try:
        # Copy crc.hpp to temp directory
        shutil.copyfile(crc_file, os.path.join(temp_dir, 'crc.hpp'))

        # Extract lines 31-929 from endpoints.hpp to endpoints_slice.hpp
        with open(endpoints_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        if len(lines) < 929:
            raise RuntimeError("endpoints.hpp has less than 929 lines")

        slice_text = ''.join(lines[30:929])  # 31..929 (1-based)
        with open(os.path.join(temp_dir, 'endpoints_slice.hpp'), 'w', encoding='utf-8') as f:
            f.write(slice_text)

        # Create minimalist main.cpp (no decorative strings)
        main_cpp = r'''#include <iostream>
#include <stdint.h>
#include "crc.hpp"

// Define polynomial and protocol before including the slice
constexpr uint16_t CANONICAL_CRC16_POLYNOMIAL = 0x3d65;
constexpr uint16_t PROTOCOL_VERSION = 1;

// Include the block that defines embedded_json, embedded_json_length and json_crc_
#include "endpoints_slice.hpp"

int main() {
    std::cout << json_crc_ << '\n';
    return 0;
}
'''
        main_path = os.path.join(temp_dir, 'main.cpp')
        with open(main_path, 'w', encoding='utf-8') as f:
            f.write(main_cpp)

        # Compile
        exe_path = os.path.join(temp_dir, 'calc_crc')
        compile_cmd = ['g++', '-std=c++11', '-O2', '-o', exe_path, main_path]
        result = subprocess.run(compile_cmd, cwd=temp_dir, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip())

        # Execute and read only the number
        run = subprocess.run([exe_path], cwd=temp_dir, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(run.stderr.strip())

        out = run.stdout.strip()
        # Should be only a decimal number
        if not out.isdigit():
            # Accept hex just in case
            try:
                if out.lower().startswith('0x'):
                    return int(out, 16)
                return int(out)
            except ValueError:
                raise RuntimeError(f"Unexpected output: {out}")

        return int(out)

    finally:
        shutil.rmtree(temp_dir)


if __name__ == "__main__":
    try:
        crc_val = compile_and_get_json_crc()
        # Convert to hexadecimal with desired format
        hex_output = f"0x{crc_val:04X}" 
        print(hex_output)
    except Exception as e:
        import sys
        print(str(e), file=sys.stderr)
        sys.exit(1)