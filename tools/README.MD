# README

## Overview  
When compiling the firmware version **0.5.6** (unknown for other versions), the build system generates an **autogen** folder inside the `Firmware` directory.  
This folder contains a file called **`endpoints.hpp`**, which exposes the endpoints defined in the ODrive firmware.  

From this automatically generated file, we provide a **Python script** that processes `endpoints.hpp` and generates a new file called **`odrive_endpoints.hpp`**, which is required for this package to work properly.

---

## Workflow  
1. Compile the ODrive firmware (tested on version **0.5.6**). (See below)  
2. Locate the `endpoints.hpp` file, generated under `Firmware/autogen/` and copy to the current folder.
3. Run the provided Python script.  
4. The script will parse the embedded JSON in `endpoints.hpp`.  
5. It then generates a new header file **`odrive_endpoints.hpp`** containing:  
   - A C++ enum with all endpoint identifiers  
   - Proper type mappings (`uint8 → uint8_t`, `function → void`, etc.) 
---

## Usage  

```bash
# Run the parser script
python3 parser_endpoints_hpp.py

# This will generate:
#   - odrive_endpoints.hpp
```

The generated file can then be included in your project and used to access ODrive endpoints in a type-safe and consistent manner.  

---

## Notes  
- Currently confirmed working only with firmware **v0.5.6**.  
- Other versions may require adjustments if the `endpoints.hpp` format changes.

---

## ⚠️ Warning / Troubleshooting  

If the generated **`odrive_endpoints.hpp`** does not work as expected (for example, endpoint mismatches or communication errors), one common cause is a wrong **CRC value**.  

The **`json_crc_`** is critical because it validates the consistency between the firmware and the tools that use the generated header. If the CRC does not match, ODrive may reject endpoint operations.  

👉 If this happens:  
- Make sure the script extracted the JSON correctly from `endpoints.hpp`.  
- Re-run the auxiliary **CRC calculator** (based on `crc.hpp` from the original firmware, change from the original firmware) to recompute the value. 
- Take care with CANONICAL_CRC16_POLYNOMIAL or PROTOCOL_VERSION hardcoded in the `calc_crc.py`.
- Replace the `json_crc` constant in the generated header if necessary.  



## Build with Docker

The recommended way to build the firmware for version **fw-v0.5.6** is using the provided Docker script.  
This avoids installing the toolchain and dependencies manually on your host system.

### Clone the repository

```bash
git clone --branch fw-v0.5.6 --depth 1 https://github.com/odriverobotics/ODrive.git
cd ODrive
````

### Requirements

* [Docker](https://docs.docker.com/get-docker/) installed on your system.
* Maybe another packages

### Build

Run the build script:

```bash
./tools/dockerbuild.sh
```

This will build the firmware inside a Docker container and produce the binary at:

```
Firmware/build/ODriveFirmware.hex
```

and the autogen folder inside Firmware Folder.

¡Perfecto! Aquí tienes un `README.md` completo en **Markdown** que incluye: clonación del repo, compilación con Docker, flasheo y las reglas de udev para Linux.


## 🖧 USB udev Rules (Linux only)

On Linux systems, you may need to add a udev rule so your user can access the ODrive device without requiring `sudo`.

1. Create a new rules file:

```bash
sudo nano /etc/udev/rules.d/99-odrive.rules
```

2. Add the following line:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666"
```

3. Reload the rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

4. Unplug and replug your ODrive board.
   It should now be accessible without root privileges.