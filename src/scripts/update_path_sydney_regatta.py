import sys
import json
import re

def update_world(path_x, path_y):
    # --- CONFIGURATION ---
    world_file_path = "/home/gehan/vrx_ws/src/vrx/vrx_gz/worlds/sydney_regatta.sdf" 
    
    # 1. DEFINE THE EXACT KEYS TO LOOK FOR
    start_marker = "<!-- START-MARKERS -->"
    end_marker = "<!-- END-MARKERS -->"

    # 2. GENERATE XML CONTENT
    marker_elements = []
    template = """    <model name='path_{i}'>
      <static>true</static>
      <pose>{x} {y} 0.1 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
          <sphere>
          <radius>0.3</radius>
          </sphere>
          </geometry>
          <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    for i, (x, y) in enumerate(zip(path_x, path_y)):
        marker_elements.append(template.format(i=i+1, x=round(x,3), y=round(y,3)))

    # Create the block of text to insert 
    new_inner_content = "\n" + "\n".join(marker_elements) + "\n"

    # 3. READ, FIND, AND REPLACE
    try:
        with open(world_file_path, 'r') as f:
            full_text = f.read()

        # Build the Regex Pattern dynamically using the exact marker strings
        # re.escape ensures special characters like < ! - > are handled correctly
        # (.*?) captures everything between them
        pattern = f"({re.escape(start_marker)})(.*?)({re.escape(end_marker)})"
        
        # Check if we find the block
        if re.search(pattern, full_text, re.DOTALL):
            # \1 = The Start Marker (Keep it)
            # new_inner_content = The new code (Paste it)
            # \3 = The End Marker (Keep it)
            updated_text = re.sub(pattern, r"\1" + new_inner_content + r"\3", full_text, flags=re.DOTALL)
            
            with open(world_file_path, 'w') as f:
                f.write(updated_text)
            print(f"Success: Replaced text between '{start_marker}' and '{end_marker}' with {len(path_x)} markers.")
        else:
            print(f"Error: Could not find the exact block.\nExpected:\n{start_marker}\n...text...\n{end_marker}")

    except Exception as e:
        print(f"Python Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 2:
        update_world(json.loads(sys.argv[1]), json.loads(sys.argv[2]))