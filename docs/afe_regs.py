#!/usr/bin/env python3
import pdfplumber
import re

pdf_path = "/home/amargan/work/VA416xx_EVK_2v05_Release_Flash_Drive/docs/AFE11612-sep.pdf"
output_h = "/home/amargan/work/VA416xx_EVK_2v05_Release_Flash_Drive/software/projects/apps/ft_vor/hdr/afe11612_regs.h"

with pdfplumber.open(pdf_path) as pdf:
    with open(output_h, "w") as f:
        f.write("/**\n * @file afe11612_regs.h\n")
        f.write(" * @brief Auto-generated AFE11612 register definitions\n */\n\n")
        f.write("#ifndef __AFE11612_REGS_H__\n#define __AFE11612_REGS_H__\n\n")
        f.write("#include <stdint.h>\n\n")
        
        # Adjust page range where register table appears
        for page_num in range(56, 60):  # pages 57-60 (0-indexed: 56-59)
            if page_num >= len(pdf.pages):
                break
            page = pdf.pages[page_num]
            tables = page.extract_tables()
            
            for table in tables:
                for row in table[1:]:  # skip header
                    if len(row) < 2:
                        continue
                    addr = row[0]
                    name = row[1] if len(row) > 1 else ""
                    desc = row[2] if len(row) > 2 else ""
                    
                    # Match hex addresses like 0x6C, 6Ch, etc.
                    if re.match(r'0?x?[0-9A-Fa-f]{1,2}h?', str(addr)):
                        addr_clean = addr.strip().upper().replace('H', '')
                        if not addr_clean.startswith('0x'):
                            addr_clean = '0x' + addr_clean
                        name_clean = re.sub(r'[^A-Z0-9_]', '_', name.upper().strip())
                        f.write(f"#define AFE11612_REG_{name_clean:30s} {addr_clean}  /* {desc} */\n")
        
        f.write("\n#endif /* __AFE11612_REGS_H__ */\n")

print(f"Generated {output_h}")