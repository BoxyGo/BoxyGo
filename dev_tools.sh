#!/bin/bash

# --- Function: Build & Source ---
# Usage: b [-r|--rebuild]
function b() {
    local REBUILD=false
    for arg in "$@"; do
        if [[ "$arg" == "-r" ]] || [[ "$arg" == "--rebuild" ]]; then
            REBUILD=true
        fi
    done

    if $REBUILD ; then
        echo "⚙️  Cleaning build/, install/, log/..."
        rm -rf build/ install/ log/
    fi

    echo "🏗️  Building..."
    colcon build --symlink-install --packages-up-to realsense_splitter
    colcon build --symlink-install --packages-skip-regex "nvblox*"

    if [ $? -eq 0 ]; then
        echo "✅ Build success. Sourcing..."
        if [ -f install/setup.bash ]; then
            source install/setup.bash
        fi
    else
        echo "❌ Build failed."
        return 1
    fi
}

# --- Function: Interactive Launcher ---
# Usage: l
function l() {
    if [ ! -d "install" ]; then
        echo "❌ 'install' directory not found. Run 'b' first."
        return 1
    fi

    echo "🔍 Scanning launch files..."
    
    local i=1
    local packages=()
    local files=()
    
    # Find launch files (follow symlinks with -L), filter for share folder
    while IFS= read -r filepath; do
        # Extract package name (2nd element: install/PKG/...)
        pkg_name=$(echo "$filepath" | cut -d'/' -f2)
        file_name=$(basename "$filepath")

        packages+=("$pkg_name")
        files+=("$file_name")

        echo -e "  \033[1;32m[$i]\033[0m Pkg: \033[1;34m$pkg_name\033[0m -> File: $file_name"
        ((i++))
        
    done < <(find -L install -type f \( -name "*launch.py" -o -name "*launch.xml" -o -name "*launch.yaml" \) | grep "/share/" | sort)

    if [ ${#packages[@]} -eq 0 ]; then
        echo "❌ No launch files found."
        return 1
    fi

    echo ""
    read -p "🚀 Select number (q to quit): " choice

    if [[ "$choice" == "q" ]]; then echo "Canceled."; return 0; fi
    
    # Validation
    if ! [[ "$choice" =~ ^[0-9]+$ ]]; then echo "❌ Not a number."; return 1; fi
    if [ "$choice" -lt 1 ] || [ "$choice" -ge "$i" ]; then echo "❌ Invalid number."; return 1; fi

    local idx=$((choice-1))
    
    echo "---------------------------------------------------"
    echo "🚀 Running: ros2 launch ${packages[$idx]} ${files[$idx]}"
    echo "---------------------------------------------------"
    
    ros2 launch "${packages[$idx]}" "${files[$idx]}"
}

echo "✅ Dev tools loaded. Commands: 'b' (build), 'l' (launch)"