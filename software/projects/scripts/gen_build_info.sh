#!/bin/bash

# Use the first argument as target dir, or default to the script's directory
if [ -n "$1" ]; then
    TARGET_DIR="$1"
else
    TARGET_DIR="$(dirname "$(realpath "$0")")"
fi

echo "TARGET_DIR: $TARGET_DIR"

# Define the output file
OUTPUT_FILE="$TARGET_DIR/hdr/build_info.h"

# Create directory if it doesn't exist
mkdir -p "$(dirname "$OUTPUT_FILE")"

# Start with a fresh file
echo "/* Auto-generated build information */" > "$OUTPUT_FILE"
echo "#ifndef __BUILD_INFO_H__" >> "$OUTPUT_FILE"
echo "#define __BUILD_INFO_H__" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"
echo "#include <stdio.h>" >> "$OUTPUT_FILE"
echo "#include <stdint.h>" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# Add build timestamp
echo "#define BUILD_TIMESTAMP \"$(date '+%Y-%m-%d %H:%M:%S')\"" >> "$OUTPUT_FILE"
echo "#define BUILD_SHORT_TS \"$(date '+%y%m%d%H')\"" >> "$OUTPUT_FILE"

# Find git repository root directory
GIT_ROOT_DIR=$(git rev-parse --show-toplevel 2>/dev/null)

# Add Git information if available
if [ -n "$GIT_ROOT_DIR" ] && [ -d "$GIT_ROOT_DIR/.git" ]; then
    # We're in a git repository, use the root directory for git commands
    echo "#define GIT_HASH \"$(git rev-parse --short HEAD)\"" >> "$OUTPUT_FILE"
    echo "#define GIT_COMMIT_HASH \"$(git rev-parse HEAD)\"" >> "$OUTPUT_FILE"
    echo "#define GIT_BRANCH \"$(git rev-parse --abbrev-ref HEAD)\"" >> "$OUTPUT_FILE"
    
    # Get tag if available
    GIT_TAG=$(git describe --tags --exact-match 2>/dev/null || echo "")
    if [ -n "$GIT_TAG" ]; then
        echo "#define GIT_TAG \"$GIT_TAG\"" >> "$OUTPUT_FILE"
    else
        echo "#warning \"No Git tag found in the repository!\"" >> "$OUTPUT_FILE"
        echo "#define GIT_TAG \"No Tag\"" >> "$OUTPUT_FILE"
    fi
    
    # Check for dirty working tree
    if [ -n "$(git status --porcelain)" ]; then
        echo "#define GIT_DIRTY 1" >> "$OUTPUT_FILE"
    else
        echo "#define GIT_DIRTY 0" >> "$OUTPUT_FILE"
    fi
    
    # Get repo URL - check for any remote, not just origin
    GIT_REPO_URL=$(git remote -v | grep fetch | head -1 | awk '{print $2}' 2>/dev/null || echo "Unknown")
    echo "#define GIT_REPO_URL \"$GIT_REPO_URL\"" >> "$OUTPUT_FILE"
else
    echo "#define GIT_HASH \"\"" >> "$OUTPUT_FILE"
    echo "#define GIT_COMMIT_HASH \"\"" >> "$OUTPUT_FILE"
    echo "#define GIT_BRANCH \"\"" >> "$OUTPUT_FILE"
    echo "#warning \"No Git tag found in the repository!\"" >> "$OUTPUT_FILE"
    echo "#define GIT_TAG \"No Tag\"" >> "$OUTPUT_FILE"
    echo "#define GIT_DIRTY 0" >> "$OUTPUT_FILE"
    echo "#define GIT_REPO_URL \"Unknown\"" >> "$OUTPUT_FILE"
fi

# Add build environment info
echo "#define BUILD_HOST \"$(hostname)\"" >> "$OUTPUT_FILE"
echo "#define BUILD_USER \"$(whoami)\"" >> "$OUTPUT_FILE"
echo "#define BUILD_OS \"$(uname -sr)\"" >> "$OUTPUT_FILE"

# Add const arrays for easy binary inclusion
echo "" >> "$OUTPUT_FILE"
echo "// Const arrays for binary inclusion in firmware" >> "$OUTPUT_FILE"
echo "const uint8_t git_hash[] = GIT_HASH;" >> "$OUTPUT_FILE"
echo "const uint8_t git_branch[] = GIT_BRANCH;" >> "$OUTPUT_FILE"
echo "const uint8_t git_tag[] = GIT_TAG;" >> "$OUTPUT_FILE"
echo "const uint8_t build_ts[] = BUILD_TIMESTAMP;" >> "$OUTPUT_FILE"
echo "const uint8_t build_short_ts[] = BUILD_SHORT_TS;" >> "$OUTPUT_FILE"

# Add the print_build_info function
echo "" >> "$OUTPUT_FILE"
echo "// Helper function to print build information" >> "$OUTPUT_FILE"
echo "static inline void print_build_info(void) {" >> "$OUTPUT_FILE"
echo "	printf(\"\\n\");" >> "$OUTPUT_FILE"
echo "	printf(\"Build Timestamp: %s\\n\", BUILD_TIMESTAMP);" >> "$OUTPUT_FILE"
echo "	printf(\"Build short Timestamp: %s\\n\", BUILD_SHORT_TS);" >> "$OUTPUT_FILE"
echo "	printf(\"Git Hash: %s\\n\", GIT_HASH);" >> "$OUTPUT_FILE"
echo "	printf(\"Git Commit Hash: %s\\n\", GIT_COMMIT_HASH);" >> "$OUTPUT_FILE"
echo "	printf(\"Git Branch: %s\\n\", GIT_BRANCH);" >> "$OUTPUT_FILE"
echo "	printf(\"Git Tag: %s\\n\", GIT_TAG);" >> "$OUTPUT_FILE"
echo "	printf(\"Repository Dirty: %d\\n\", GIT_DIRTY);" >> "$OUTPUT_FILE"
echo "	printf(\"Git Repository URL: %s\\n\", GIT_REPO_URL);" >> "$OUTPUT_FILE"
echo "	printf(\"Build Host: %s\\n\", BUILD_HOST);" >> "$OUTPUT_FILE"
echo "	printf(\"Build User: %s\\n\", BUILD_USER);" >> "$OUTPUT_FILE"
echo "	printf(\"Build OS: %s\\n\", BUILD_OS);" >> "$OUTPUT_FILE"
echo "	printf(\"\\n\");" >> "$OUTPUT_FILE"
echo "}" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# Close the header guard
echo "#endif /* __BUILD_INFO_H__ */" >> "$OUTPUT_FILE"

echo "Build info generated at $OUTPUT_FILE"
