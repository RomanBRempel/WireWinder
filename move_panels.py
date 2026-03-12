#!/usr/bin/env python3
"""Move Firmware Management and Network Settings panels to end of page"""

import re

# Read the file
with open('src/main.cpp', 'r', encoding='utf-8') as f:
    content = f.read()

# Pattern to find and extract the two panels
# Using non-greedy matching to get from start of firmware-panel to start of status-grid
pattern = r'(\t<div class="control-panel" id="firmware-panel">.*?</div>\s*\t<div class="control-panel" id="wifi-panel">.*?</div>\s*\n)'

match = re.search(pattern, content, re.DOTALL)

if match:
    panels_section = match.group(1)
    print(f"Found panels section: {len(panels_section)} characters")
    
    # Remove panels from current location
    content_without_panels = content.replace(panels_section, '', 1)
    
    # Find the footer
    footer_pattern = r'(\t</script>\n)'
    footer_match = re.search(footer_pattern, content_without_panels)
    
    if footer_match:
        insert_pos = footer_match.end()
        print(f"Found insertion point at position {insert_pos}")
        
        # Insert panels before footer
        new_content = (
            content_without_panels[:insert_pos] + 
            '\n' + panels_section.rstrip() + '\n\n' +
            content_without_panels[insert_pos:]
        )
        
        # Write back
        with open('src/main.cpp', 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        print("SUCCESS: Panels moved to end of page")
    else:
        print("ERROR: Could not find footer location")
else:
    print("ERROR: Could not find panels section")
