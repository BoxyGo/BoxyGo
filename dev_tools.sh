#!/bin/bash

function g() {
    s
    if [ -z "$ROS_DISTRO" ]; then return 1; fi

    local MODE="isaac"
    local XACRO_PATH="${1:-./src/boxygo_description/urdf/luksusowy.urdf.xacro}"
    
    if [ ! -f "$XACRO_PATH" ]; then
         INSTALLED_PATH=$(ros2 pkg prefix boxygo_description)/share/boxygo_description/urdf/luksusowy.urdf.xacro
         if [ -f "$INSTALLED_PATH" ]; then
             XACRO_PATH="$INSTALLED_PATH"
         else
             return 1
         fi
    fi

    local OUTPUT_DIR=$(dirname "$XACRO_PATH")
    local OUTPUT_FILE="$OUTPUT_DIR/luksusowy_isaac.urdf"
    
    ros2 run xacro xacro "$XACRO_PATH" sim_mode:="$MODE" > "$OUTPUT_FILE"

    return $?
}

function s() {
    if [ -f install/setup.bash ]; then
        source install/setup.bash
    else
        return 1
    fi
}

function b() {
    local REBUILD=false
    local TARGET_PKG=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -r|--rebuild) REBUILD=true; shift ;;
            -p|--package) TARGET_PKG="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if $REBUILD ; then
        if [ -n "$TARGET_PKG" ]; then
            rm -rf install/"$TARGET_PKG" build/"$TARGET_PKG"
        else
            rm -rf build/ install/ log/
        fi
    fi

    if [ -n "$TARGET_PKG" ]; then
        colcon build --symlink-install --packages-select "$TARGET_PKG"
    else
        colcon build --symlink-install --packages-up-to realsense_splitter
        colcon build --symlink-install --packages-skip-regex "nvblox*"
    fi

    if [ $? -eq 0 ]; then
        if [ -f install/setup.bash ]; then
            source install/setup.bash

            local URDF_SRC_DIR="./src/boxygo_description/urdf"
            local MAIN_XACRO="$URDF_SRC_DIR/luksusowy.urdf.xacro"
            local TARGET_URDF="$URDF_SRC_DIR/luksusowy_isaac.urdf"
            
            if [ -f "$MAIN_XACRO" ]; then
                local NEED_GEN=false
                if [ ! -f "$TARGET_URDF" ]; then
                    NEED_GEN=true
                else
                    local CHANGED_FILES=$(find "$MAIN_XACRO" "$URDF_SRC_DIR/common" "$URDF_SRC_DIR/config" -type f -newer "$TARGET_URDF" 2>/dev/null)
                    if [ -n "$CHANGED_FILES" ]; then NEED_GEN=true; fi
                fi
                if $NEED_GEN; then g "$MAIN_XACRO"; fi
            fi
        fi
    else
        return 1
    fi
}

function l() {
    s
    if [ ! -d "install" ]; then return 1; fi
    
    local i=1; local packages=(); local files=()
    
    while IFS= read -r filepath; do
        pkg_name=$(echo "$filepath" | cut -d'/' -f2)
        file_name=$(basename "$filepath")
        packages+=("$pkg_name"); files+=("$file_name")
        echo -e "  \033[1;32m[$i]\033[0m Pkg: \033[1;34m$pkg_name\033[0m -> File: $file_name"
        ((i++))
    done < <(find -L install -type f \( -name "*launch.py" -o -name "*launch.xml" -o -name "*launch.yaml" \) | grep "/share/" | sort)

    if [ ${#packages[@]} -eq 0 ]; then return 1; fi
    
    echo ""
    read -p "Select # (q=quit): " choice
    if [[ "$choice" == "q" ]]; then return 0; fi
    if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -ge "$i" ]; then return 1; fi
    
    local idx=$((choice-1))
    ros2 launch "${packages[$idx]}" "${files[$idx]}"
}

function r() {
    s 

    local PANEL_DIR="rqt_panels"
    local -a files=()

    if [ ! -d "$PANEL_DIR" ]; then
        echo "Folder '$PANEL_DIR' nie istnieje."
        return 1
    fi

    echo -e "\nDostępne perspektywy RQt:\n"

    local i=1
    while IFS= read -r file; do
        files+=("$file")
        echo -e "  \033[1;32m[$i]\033[0m $(basename "$file")"
        ((i++))
    done < <(find "$PANEL_DIR" -maxdepth 1 -type f -name "*.perspective" | sort)

    if [ ${#files[@]} -eq 0 ]; then
        echo "Brak plików .perspective w $PANEL_DIR"
        return 1
    fi

    echo ""
    read -p "Wybierz numer (q=wyjście): " choice

    if [[ "$choice" == "q" ]]; then
        return 0
    fi

    if ! [[ "$choice" =~ ^[0-9]+$ ]]; then
        echo "Niepoprawny numer."
        return 1
    fi

    if (( choice < 1 || choice > ${#files[@]} )); then
        echo "Numer poza zakresem."
        return 1
    fi

    local idx=$((choice-1))
    local selected="${files[$idx]}"
    rqt --perspective-file "$selected"
}


function k() {
    s
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:="/diff_cont/cmd_vel_unstamped"
}

_b_completions()
{
  local cur prev
  cur=${COMP_WORDS[COMP_CWORD]}
  prev=${COMP_WORDS[COMP_CWORD-1]}

  case ${prev} in
    -p|--package)
        local pkgs=$(find src -maxdepth 3 -name "package.xml" -printf "%h\n" | xargs -I {} basename {})
        COMPREPLY=($(compgen -W "${pkgs}" -- ${cur}))
        return 0
        ;;
  esac
}
complete -F _b_completions b

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/isaac_ros-dev/install/boxygo_description/share


echo "   b [-r] [-p pkg] -> Build (Full or Single Pkg)"
echo "   l               -> Launch Menu"
echo "   s               -> Source setup.bash"
echo "   g               -> Generate URDF"
echo "   r               -> Open rqt panels"
echo "   k               -> Teleop Twist Keyboard"
