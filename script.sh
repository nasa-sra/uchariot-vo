#!/bin/bash

# Define the source and destination subdirectories
SOURCE_SUBDIR="scripts"
DEST_SUBDIR="ORB_SLAM3"

# Get the current directory (parent directory of the subdirectories)
CURRENT_DIR=$(pwd)

# Define the full paths of the source and destination subdirectories
SOURCE_DIR="$CURRENT_DIR/$SOURCE_SUBDIR"
DEST_DIR="$CURRENT_DIR/$DEST_SUBDIR"
CMAKE_FILE="$DEST_DIR/CMakeLists.txt"

# Check if the source subdirectory exists
if [ ! -d "$SOURCE_DIR" ]; then
    echo "Source subdirectory does not exist."
    exit 1
fi

# Check if the destination subdirectory exists, if not, create it
if [ ! -d "$DEST_DIR" ]; then
    echo "Destination subdirectory does not exist. Creating it now."
    mkdir "$DEST_DIR"
fi

# If CMakeLists.txt doesn't exist, create it with initial content
if [ ! -f "$CMAKE_FILE" ]; then
    echo "Creating CMakeLists.txt in $DEST_DIR"
    echo "set(CMAKE_RUNTIME_OUTPUT_DIRECTORY \${PROJECT_SOURCE_DIR})" > "$CMAKE_FILE"
fi

# Function to check if an entry already exists in CMakeLists.txt
entry_exists() {
    grep -q "add_executable($1" "$CMAKE_FILE"
}

# Process all files from the source subdirectory
for FILE in "$SOURCE_DIR"/*; do
    if [ -f "$FILE" ]; then
        BASENAME=${FILE##*/}
        FILENAME_WITHOUT_EXT=${BASENAME%.*}
        
        echo "Processing file $BASENAME"

        # Copy the file to the destination directory if it doesn't exist or is different
        if [ ! -f "$DEST_DIR/$BASENAME" ] || ! cmp -s "$FILE" "$DEST_DIR/$BASENAME"; then
            echo "Copying $BASENAME to $DEST_DIR"
            cp "$FILE" "$DEST_DIR"
        else
            echo "$BASENAME already exists in $DEST_DIR and is identical. Skipping copy."
        fi

        # Check if entry already exists in CMakeLists.txt
        if ! entry_exists "$FILENAME_WITHOUT_EXT"; then
            echo "Adding entry for $FILENAME_WITHOUT_EXT to CMakeLists.txt"
            # Append entry to CMakeLists.txt
            echo "add_executable($FILENAME_WITHOUT_EXT" >> "$CMAKE_FILE"
            echo "       $BASENAME" >> "$CMAKE_FILE"
            echo ")" >> "$CMAKE_FILE"
            echo "target_link_libraries($FILENAME_WITHOUT_EXT \${PROJECT_NAME})" >> "$CMAKE_FILE"
            echo "" >> "$CMAKE_FILE"  # Add an empty line for readability
        else
            echo "Entry for $FILENAME_WITHOUT_EXT already exists in CMakeLists.txt. Skipping."
        fi
    fi
done

# Optional: Print a message indicating success
echo "Files have been processed and CMakeLists.txt has been updated as needed."

cd ORB_SLAM3
./build.sh