#!/bin/bash

EXECUTABLE="./Lab2"
CHECKER="./checker"
PLOTTER="plotter.py"
CASES_DIR="cases"

echo "=================================================="
echo "Test start time: $(date '+%Y-%m-%d %H:%M:%S')"
echo "=================================================="

echo "Cleaning old executables..."
make clean

echo "Compiling the latest source code..."
make

if [ $? -ne 0 ]; then
    echo "Compilation failed. Please check your source code."
    exit 1
fi

echo "Compilation successful"
echo "--------------------------------------------------"

echo "Running test cases..."

TEST_CASES=("block_30.in" "block_100.in" "block_500.in")


for FILE_NAME in "${TEST_CASES[@]}"; do
    INPUT_PATH="$CASES_DIR/$FILE_NAME"
    OUTPUT_PATH="${INPUT_PATH%.in}.out"
    IMAGE_PATH="${INPUT_PATH%.in}.png"  

    echo "=================================================="
    
    if [ ! -f "$INPUT_PATH" ]; then
        echo "Input file $INPUT_PATH not found. Skipping this test case."
        continue
    fi

    echo "Running test case: $INPUT_PATH"
    echo "Output file: $OUTPUT_PATH"
    echo "Current time: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "--------------------------------------------------"
    echo "Execution time and results:"

    time $EXECUTABLE "$INPUT_PATH" "$OUTPUT_PATH"

    if [ $? -eq 0 ]; then
        echo "Execution completed. Validating results with SolutionChecker:"
        if [ -f "$CHECKER" ]; then
            $CHECKER "$INPUT_PATH" "$OUTPUT_PATH"
            
            if [ -f "$PLOTTER" ]; then
                echo "Running the plotter. Saving the image to: $IMAGE_PATH"
                python3 "$PLOTTER" "$INPUT_PATH" "$OUTPUT_PATH" "$IMAGE_PATH"
            else
                echo "Note: $PLOTTER not found. Skipping plotting."
            fi
            # -----------------------
            
        else
            echo "Warning: SolutionChecker executable not found in the current directory."
        fi
    else
        echo "An error occurred during execution."
    fi
done

echo "=================================================="
echo "All test cases completed"
echo "Test end time: $(date '+%Y-%m-%d %H:%M:%S')"
echo "=================================================="