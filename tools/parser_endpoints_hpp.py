import sys
import re
import json
import calc_crc

def extract_embedded_json(file_path='endpoints.hpp'):
    """
    Extract and parse the embedded JSON from a .hpp file
    
    Args:
        file_path (str): Path to the .hpp file
        
    Returns:
        dict: Parsed JSON or None if there is an error
    """
    try:
        # Read the file
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
        
        # Search for the embedded_json variable
        pattern = r'const unsigned char embedded_json\[\] = "([^"]*(?:"[^"]*)*)"'
        match = re.search(pattern, content, re.DOTALL)
        
        if not match:
            print("❌ embedded_json variable not found")
            return None
        
        # Extract the variable content
        json_content = match.group(1)
        
        # Apply specific replacements:
        # 1. \" to "
        json_content = json_content.replace('\\"', '"')
        
        # 2. Remove "\n" completely
        json_content = json_content.replace('"\n"', '')
        
        # Try to parse the JSON
        parsed_json = json.loads(json_content)
        print("✅ JSON parsed successfully!")
        
        # Show summary
        if isinstance(parsed_json, dict):
            print(f"📊 Number of endpoints: {len(parsed_json)}")
            
            # Count by types
            types_count = {}
            for key, value in parsed_json.items():
                if isinstance(value, dict) and 'type' in value:
                    endpoint_type = value['type']
                    types_count[endpoint_type] = types_count.get(endpoint_type, 0) + 1
            
            print("📈 Endpoint types:")
            for type_name, count in types_count.items():
                print(f"  {type_name}: {count}")
        
        return parsed_json
        
    except json.JSONDecodeError as e:
        print(f"❌ Error parsing JSON: {e}")
        return None
    except FileNotFoundError:
        print(f"❌ File not found: {file_path}")
        return None
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return None

def generate_enum_from_json(json_data, output_file='odrive_endpoints.hpp', namespace='odrive'):
    """
    Generate a C++ enum from the parsed JSON data
    
    Args:
        json_data (dict): Parsed JSON data
        output_file (str): Output file for the enum
        namespace (str): Namespace for the enum
        
    Returns:
        bool: True if successfully generated, False otherwise
    """
    if not json_data:
        print("❌ No JSON data to generate enum")
        return False
    
    try:
        # Create a flat dictionary of all endpoints with ID
        flat_endpoints = {}
        
        def extract_endpoints(obj, prefix=""):
            """Recursively extract all endpoints with ID preserving hierarchy"""
            if isinstance(obj, dict):
                current_name = obj.get("name", "")

                # Build the accumulated prefix correctly
                if prefix and current_name:
                    full_prefix = f"{prefix}__{current_name}"
                elif current_name:
                    full_prefix = current_name
                else:
                    full_prefix = prefix

                # If it has an ID, it's an endpoint
                if "id" in obj:
                    flat_endpoints[full_prefix] = {
                        "id": obj["id"],
                        "type": obj.get("type", "unknown"),
                        "access": obj.get("access", "unknown"),
                    }

                # If it has members, recurse into them
                if "members" in obj:
                    for member in obj["members"]:
                        extract_endpoints(member, full_prefix)

            elif isinstance(obj, list):
                for item in obj:
                    extract_endpoints(item, prefix)
        
        # Extract all endpoints
        extract_endpoints(json_data)
        
        if not flat_endpoints:
            print("❌ No endpoints with ID found in JSON")
            return False
        
        print(f"🔍 Total endpoints with ID found: {len(flat_endpoints)}")

        # Map JSON types → C++ types
        type_mapping = {
            "function": "void",
            "uint8": "uint8_t",
            "uint16": "uint16_t",
            "uint32": "uint32_t",
            "uint64": "uint64_t",
            "int8": "int8_t",
            "int16": "int16_t",
            "int32": "int32_t",
            "int64": "int64_t",
            "float": "float",   # remains the same
            "bool": "bool"      # remains the same
        }
        
        # Generate the enum based on extracted endpoints
        enum_lines = []
        enum_lines.append("enum {")

        axis_lines = []
        axis_lines.append("")
        axis_lines.append("\t// Per-Axis endpoints (to be used with read_axis_property and write_axis_property)")
        
        axis0_error_id = 0
        axis1_error_id = 0

        endpoint_type_lines = []
        endpoint_type_axis_lines = []
        endpoint_type_axis_lines.append("")
        endpoint_type_axis_lines.append("")
        endpoint_type_axis_lines.append("// Per-axis endpoints")

        # Sort by ID for consistency
        sorted_endpoints = sorted(flat_endpoints.items(), key=lambda x: x[1]['id'])

        for name, info in sorted_endpoints:

            if 'axis0__error' in name:
                axis0_error_id = info['id']

            if 'axis1__error' in name:
                axis1_error_id = info['id']

            if 'axis' in name:
                if 'axis0' in name:
                    name = name.replace("axis0", "axis")

                    endpoint_id = info['id']
                    # Convert name to constant format (uppercase with underscores)
                    enum_name = name.upper().replace('.', '__')
                    # Clean special characters
                    enum_name = re.sub(r'[^A-Z0-9_]', '_', enum_name)
                    # Avoid names starting with numbers
                    if enum_name and enum_name[0].isdigit():
                        enum_name = f"EP_{enum_name}"
                    
                    if enum_name:  # Only add if the name is not empty
                        axis_lines.append(f"    {enum_name} = {endpoint_id},")
                        endpoint_type_axis_lines.append(f"template<> struct endpoint_type<{enum_name}> {{ typedef {type_mapping.get(info['type'], "ERROR")} type; }};")
                else:
                    continue
            elif '__endpoint' in name:
                continue
            else:
                endpoint_id = info['id']
                # Convert name to constant format (uppercase with underscores)
                enum_name = name.upper().replace('.', '__')
                # Clean special characters
                enum_name = re.sub(r'[^A-Z0-9_]', '_', enum_name)
                # Avoid names starting with numbers
                if enum_name and enum_name[0].isdigit():
                    enum_name = f"EP_{enum_name}"
                
                if enum_name:  # Only add if the name is not empty
                    enum_lines.append(f"    {enum_name} = {endpoint_id},")
                    endpoint_type_lines.append(f"template<> struct endpoint_type<{enum_name}> {{ typedef {type_mapping.get(info['type'], "ERROR")} type; }};")
                    # Unsigned integers end with _t and functions are void
                    # Endpoints named __endpoint are not included

        enum_lines.extend(axis_lines)
        enum_lines.append("};")

        per_axis_offset = axis1_error_id - axis0_error_id

        endpoint_type_lines.extend(endpoint_type_axis_lines)

        hex_output = "ERROR"
        try:
            crc_val = calc_crc.compile_and_get_json_crc()
            # Convert to hexadecimal with desired format
            hex_output = f"0x{crc_val:04X}" 
            print(f"🔍 CRC: {hex_output}")
        except Exception as e:
            pass
        
        # Save the generated enum
        enum_content = '\n'.join(enum_lines)
        endpoint_content = '\n'.join(endpoint_type_lines)
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(f"""/*
* File generated automatically from endpoints.hpp
*/

#ifndef __ODRIVE_ENDPOINTS_HPP
#define __ODRIVE_ENDPOINTS_HPP

namespace {namespace} {{

static constexpr const uint16_t json_crc = {hex_output};

static constexpr const uint16_t per_axis_offset = {per_axis_offset};

{enum_content}

template<int I>
struct endpoint_type;

{endpoint_content}

template<int I>
using endpoint_type_t = typename endpoint_type<I>::type;

}}

#endif // __ODRIVE_ENDPOINTS_HPP
""")
        
        print(f"📝 Enum generated and saved to '{output_file}'")
        print(f"🔢 Total constants in enum: {len(sorted_endpoints)}")
        
        # Show some examples
        print("\n📋 First 10 endpoints:")
        for i, (name, info) in enumerate(sorted_endpoints[:10]):
            enum_name = name.upper().replace('.', '__')
            enum_name = re.sub(r'[^A-Z0-9_]', '_', enum_name)
            if enum_name and enum_name[0].isdigit():
                enum_name = f"EP_{enum_name}"
            print(f"  {enum_name} = {info['id']} // {info['type']}")
        
        if len(sorted_endpoints) > 10:
            print("  ...")
        
        # Show statistics by type
        type_counts = {}
        for name, info in flat_endpoints.items():
            type_name = info['type']
            type_counts[type_name] = type_counts.get(type_name, 0) + 1
        
        print(f"\n📈 Statistics by type:")
        for type_name, count in sorted(type_counts.items()):
            print(f"  {type_name}: {count}")
        
        return True
        
    except Exception as e:
        print(f"❌ Error generating enum: {e}")
        return False

def main():
    """Main function that executes the complete process"""
    print("🚀 Starting JSON extraction and enum generation...")
    
    # Step 1: Extract JSON
    json_data = extract_embedded_json('endpoints.hpp')
    if json_data is None:
        print("❌ Could not extract JSON. Terminating.")
        sys.exit(1)
    
    # Step 2: Generate enum
    success = generate_enum_from_json(json_data)
    if not success:
        print("❌ Could not generate enum. Terminating.")
        sys.exit(1)
    
    print("✅ Process completed successfully!")

# Run only if called directly
if __name__ == "__main__":
    main()