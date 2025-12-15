#!/bin/bash
declare -gA _L_PRESETS

function u() {
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
    local REBUILD_ALL=false
    local CLEAN_CHANGED=false
    local TARGET_PKG=""
    
    local FROZEN_PKGS=(
        "diff_drive_controller"
        "greenwave_monitor"
        "greenwave_monitor_interfaces"
        "realsense_splitter"
        "ydlidar_ros2_driver"
        "ydlidar_sdk"
        "moteus_msgs"
    )

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -r|--rebuild) REBUILD=true; shift ;;
            -a|--all)     REBUILD_ALL=true; shift ;;
            -c|--changed) CLEAN_CHANGED=true; shift ;;
            -p|--package) TARGET_PKG="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if $REBUILD; then
        if [ -n "$TARGET_PKG" ]; then
            echo -e "\033[1;33m[Rebuild]\033[0m Force cleaning package: $TARGET_PKG"
            rm -rf install/"$TARGET_PKG" build/"$TARGET_PKG"
        
        elif $REBUILD_ALL; then
            echo -e "\033[1;31m[Rebuild ALL]\033[0m Wiping entire workspace (ignoring frozen list)..."
            rm -rf build/ install/ log/
            
        else
            echo -e "\033[1;33m[Smart Rebuild]\033[0m Cleaning workspace (preserving frozen packages)..."
            rm -rf log/
            
            if [ -d "build" ]; then
                for d in build/*; do
                    pkg_name=$(basename "$d")
                    local is_frozen=false
                    
                    for frozen in "${FROZEN_PKGS[@]}"; do
                        [[ "$pkg_name" == "$frozen" ]] && is_frozen=true && break
                    done

                    if $is_frozen; then
                        echo -e "  - \033[1;36mSkipping clean\033[0m for: $pkg_name"
                    else
                        rm -rf "build/$pkg_name" "install/$pkg_name"
                    fi
                done
            fi
        fi

    elif $CLEAN_CHANGED; then
        echo -e "\033[1;35m[Clean Changed]\033[0m Scanning for modified packages..."
        
        while read -r name path _; do
            if [ -d "install/$name" ]; then
                local changes=$(find "$path" -type f -newer "install/$name" -print -quit 2>/dev/null)
                
                if [ -n "$changes" ]; then
                    echo -e "  ! Change detected in \033[1;33m$name\033[0m -> Cleaning build/install..."
                    rm -rf "build/$name" "install/$name"
                fi
            fi
        done < <(colcon list)
    fi

    echo -e "\033[1;32m[Build]\033[0m Starting colcon build..."
    if [ -n "$TARGET_PKG" ]; then
        colcon build --symlink-install --packages-select "$TARGET_PKG"
    else
        colcon build --symlink-install 
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
                    local CHECK_DIRS=("$MAIN_XACRO")
                    if [ -d "$URDF_SRC_DIR/common" ]; then CHECK_DIRS+=("$URDF_SRC_DIR/common"); fi
                    if [ -d "$URDF_SRC_DIR/config" ]; then CHECK_DIRS+=("$URDF_SRC_DIR/config"); fi

                    local CHANGED_FILES=$(find "${CHECK_DIRS[@]}" -type f -newer "$TARGET_URDF" 2>/dev/null)
                    if [ -n "$CHANGED_FILES" ]; then NEED_GEN=true; fi
                fi
                
                if $NEED_GEN; then 
                    if type u &>/dev/null; then
                        u "$MAIN_XACRO"
                    fi
                fi
            fi
        fi
    else
        echo -e "\033[1;31m[Build Failed]\033[0m"
        return 1
    fi
}

function l() {
    s
    
    if [ ! -d "install" ]; then 
        echo -e "\033[1;31m[Error]\033[0m 'install' directory not found. Are you in a ROS2 workspace?"
        return 1
    fi
    
    local i=1; local packages=(); local files=(); local full_paths=()
    
    while IFS= read -r filepath; do
        pkg_name=$(echo "$filepath" | cut -d'/' -f2)
        file_name=$(basename "$filepath")
        packages+=("$pkg_name"); files+=("$file_name"); full_paths+=("$filepath")
        ((i++))
    done < <(find -L install -type f \( -name "*launch.py" -o -name "*launch.xml" -o -name "*launch.yaml" \) | grep "/share/" | sort)

    if [ ${#packages[@]} -eq 0 ]; then echo "No launch files found."; return 1; fi
    
    echo -e "\033[1;36m=== Select Launch File ===\033[0m"
    for ((j=0; j<${#packages[@]}; j++)); do
        echo -e "  \033[1;32m[$((j+1))]\033[0m \033[1;33m${files[$j]}\033[0m \033[90m(Pkg: \033[1;34m${packages[$j]}\033[90m)\033[0m"
    done
    echo ""
    
    read -p "Select # (q=quit): " choice
    if [[ "$choice" == "q" ]]; then return 0; fi
    if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -ge "$i" ]; then echo "Invalid selection."; return 1; fi
    
    local idx=$((choice-1))
    local sel_pkg="${packages[$idx]}"
    local sel_file="${files[$idx]}"
    local sel_path="${full_paths[$idx]}"

    echo -e "\nAnalyzing arguments for \033[1;33m$sel_file\033[0m (top-level only)..."

    local raw_args
    raw_args=$(ros2 launch "$sel_pkg" "$sel_file" -s 2>/dev/null)

    local arg_names=(); local arg_values=(); local arg_defaults=()
    local arg_descs=(); local arg_choices=() 

    while IFS='|' read -r name default desc choices_str; do
        if [[ -n "$name" ]]; then
            arg_names+=("$name")
            arg_descs+=("$desc")
            arg_choices+=("$choices_str")
            arg_defaults+=("$default")

            local preset_key="${sel_pkg}:::${sel_file}:::${name}"
            if [[ -v _L_PRESETS[$preset_key] ]]; then
                arg_values+=("${_L_PRESETS[$preset_key]}")
            else
                arg_values+=("$default")
            fi
        fi
    done < <(python3 -c "
import sys, re

launch_file_path = sys.argv[1]
declared_args = set()

try:
    with open(launch_file_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
        declared_args.update(re.findall(r\"DeclareLaunchArgument\s*\(\s*['\\\"]([^'\\\"]+)['\\\"]\", content))
        declared_args.update(re.findall(r\"<arg\s+.*name=['\\\"]([^'\\\"]+)['\\\"]\", content))
except Exception as e:
    pass 

data = sys.stdin.read()
arg_starts = [m.start() for m in re.finditer(r\"^\s*'[^']+'\s*:\", data, re.MULTILINE)]
arg_starts.append(len(data))

for i in range(len(arg_starts) - 1):
    block = data[arg_starts[i]:arg_starts[i+1]]
    name_match = re.search(r\"^\s*'([^']+)'\", block, re.MULTILINE)
    if not name_match: continue
    
    name = name_match.group(1)

    if name not in declared_args:
        continue

    default = ''
    def_match = re.search(r\"default:\s*'([^']*)'\", block)
    if not def_match: def_match = re.search(r\"default:\s*([^)\s,]+)\", block)
    if def_match: default = def_match.group(1)
    
    choices = ''
    choices_match = re.search(r\"(?:allowed values|valid values|choices).*?\[(.*?)\]\", block, re.DOTALL)
    if not choices_match: choices_match = re.search(r\"(?:allowed values|valid values|choices).*?\((.*?)\)\", block, re.DOTALL)
    if choices_match: choices = re.sub(r\"['\\\"\[\]\(\)\s]\", \"\", choices_match.group(1))
    
    desc = ''
    lines = block.splitlines()
    for line in lines:
        l = line.strip()
        if l and not l.startswith(\"'\") and 'default:' not in l and 'values' not in l and 'choices' not in l:
            desc = l
            break
            
    print(f'{name}|{default}|{desc}|{choices}')
" "$sel_path" <<< "$raw_args") 

    if [ ${#arg_names[@]} -eq 0 ]; then
        echo -e "\033[1;32mNo top-level arguments found (or file parsing failed). Launching...\033[0m"
        ros2 launch "$sel_pkg" "$sel_file"
        return 0
    fi

    local cnt_args=${#arg_names[@]}
    local idx_reset=$cnt_args
    local idx_launch=$((cnt_args+1))
    local current_row=$idx_launch
    
    tput civis

    while true; do
        clear
        echo -e "\033[1;36m=== Configuration: \033[1;37m$sel_pkg / $sel_file\033[1;36m ===\033[0m"
        echo -e "Arrows: \033[33mnavigate\033[0m | Enter: \033[33medit/select\033[0m"
        echo "--------------------------------------------------------"

        for ((j=0; j<=idx_launch; j++)); do
            if [ $j -eq $idx_launch ]; then
                echo ""
                if [ $j -eq $current_row ]; then
                    echo -e "  \033[1;30;42m [  >>>  LAUNCH  <<<  ] \033[0m"
                else
                    echo -e "  \033[1;32m [       LAUNCH       ] \033[0m"
                fi
            elif [ $j -eq $idx_reset ]; then
                echo ""
                if [ $j -eq $current_row ]; then
                    echo -e "  \033[1;37;41m [  RESET TO DEFAULTS  ] \033[0m"
                else
                    echo -e "  \033[1;31m [  RESET TO DEFAULTS  ] \033[0m"
                fi
            else
                local name="${arg_names[$j]}"
                local val="${arg_values[$j]}"
                local default_val="${arg_defaults[$j]}"
                local desc="${arg_descs[$j]:0:60}"
                local choices_available="${arg_choices[$j]}"
                local display_val="$val"
                if [ -z "$val" ]; then display_val="\033[90m<empty>\033[0m"; fi
                
                local changed_mark=""
                if [[ "$val" != "$default_val" ]]; then changed_mark="\033[1;33m*\033[0m"; fi

                local list_icon=" "
                if [[ -n "$choices_available" ]]; then list_icon="\033[1;35m≡\033[0m"; fi

                if [ $j -eq $current_row ]; then
                    echo -e "\033[1;33m> $name\033[0m $list_icon $changed_mark"
                    echo -e "    Value: \033[1;37;4m$display_val\033[0m"
                    if [[ -n "$desc" ]]; then echo -e "    \033[3;90m($desc)\033[0m"; fi
                else
                    echo -e "  $name $list_icon $changed_mark : $display_val"
                fi
            fi
        done

        read -rsn3 key 
        if [[ "$key" == $'\033[A' ]]; then
            ((current_row--))
            if [ $current_row -lt 0 ]; then current_row=$idx_launch; fi
        elif [[ "$key" == $'\033[B' ]]; then
            ((current_row++))
            if [ $current_row -gt $idx_launch ]; then current_row=0; fi
        elif [[ "$key" == "" ]]; then
            if [ $current_row -eq $idx_launch ]; then
                for ((k=0; k<cnt_args; k++)); do
                    local p_key="${sel_pkg}:::${sel_file}:::${arg_names[$k]}"
                    _L_PRESETS[$p_key]="${arg_values[$k]}"
                done
                break
            elif [ $current_row -eq $idx_reset ]; then
                for ((k=0; k<cnt_args; k++)); do
                    arg_values[$k]="${arg_defaults[$k]}"
                    unset _L_PRESETS["${sel_pkg}:::${sel_file}:::${arg_names[$k]}"]
                done
                tput cnorm; echo -e "\nDefaults restored!"; sleep 0.5; tput civis
            else
                local edit_name="${arg_names[$current_row]}"
                local current_val="${arg_values[$current_row]}"
                local available_choices="${arg_choices[$current_row]}"

                if [[ -n "$available_choices" ]]; then
                    IFS=',' read -r -a choices_arr <<< "$available_choices"
                    local c_idx=0
                    local c_max=$((${#choices_arr[@]} - 1))
                    
                    for ((c=0; c<=c_max; c++)); do
                        if [[ "${choices_arr[$c]}" == "$current_val" ]]; then c_idx=$c; break; fi
                    done

                    while true; do
                        clear
                        echo -e "\033[1;36m=== Select value for: \033[1;33m$edit_name\033[1;36m ===\033[0m"
                        echo "Arrows: up/down | Enter: confirm"
                        echo "----------------------------------------"
                        for ((c=0; c<=c_max; c++)); do
                            if [ $c -eq $c_idx ]; then
                                echo -e " \033[1;32m> ${choices_arr[$c]} \033[0m \033[90m<--\033[0m"
                            else
                                echo -e "   ${choices_arr[$c]}"
                            fi
                        done
                        read -rsn3 c_key
                        if [[ "$c_key" == $'\033[A' ]]; then 
                            ((c_idx--)); if [ $c_idx -lt 0 ]; then c_idx=$c_max; fi
                        elif [[ "$c_key" == $'\033[B' ]]; then 
                            ((c_idx++)); if [ $c_idx -gt $c_max ]; then c_idx=0; fi
                        elif [[ "$c_key" == "" ]]; then
                            arg_values[$current_row]="${choices_arr[$c_idx]}"
                            break
                        fi
                    done
                else
                    tput cnorm
                    echo -e "\n--------------------------------------------------------"
                    read -e -p "New value for '$edit_name': " -i "$current_val" new_val
                    arg_values[$current_row]="$new_val"
                    tput civis
                fi
            fi
        fi
    done

    tput cnorm
    
    local cmd_args=()
    for ((k=0; k<cnt_args; k++)); do
        local n="${arg_names[$k]}"
        local v="${arg_values[$k]}"
        if [[ -n "$v" ]]; then cmd_args+=("$n:=$v"); fi
    done

    echo -e "\n\033[1;32m>>> Launching:\033[0m ros2 launch $sel_pkg $sel_file ${cmd_args[*]}"
    ros2 launch "$sel_pkg" "$sel_file" "${cmd_args[@]}"
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

function g() {
    s || return 1

    if ros2 pkg list | grep -q "^greenwave_monitor$"; then
        ros2 run greenwave_monitor ncurses_dashboard
    else
        echo "Greenwave Monitor is not built!"
    fi
}

function t() {
    function _t_analyze() {
        local selected_topic="$1"
        while true; do
            clear
            echo -e "\033[1;35m>>> Analysis: \033[1;37m$selected_topic\033[0m"
            echo "----------------------------------------------------"
            
            echo -e "\033[1;33m[1. Snapshot (Echo Once)]:\033[0m"
            echo -e " \033[90m... listening for 1.5s ...\033[0m"
            
            local echo_out
            echo_out=$(timeout 1.5s ros2 topic echo "$selected_topic" --once --no-arr 2>&1)
            local exit_code=$?

            if [ $exit_code -eq 124 ]; then
                echo -e "  \033[1;31m[!] No message received yet (Topic silent?)\033[0m"
            elif [ -z "$echo_out" ]; then
                echo -e "  (Received empty data)"
            else
                echo "$echo_out" | head -n 15
            fi
            
            echo -e "\n\033[1;33m[2. Info Verbose (Summary)]:\033[0m"
            ros2 topic info -v "$selected_topic" \
                | grep -E "Type:|Publisher count:|Subscription count:|Node name:" \
                | sed 's/Node name:/    - Node:/'
            
            echo -e "\n----------------------------------------------------"
            echo -e " \033[1;32m[e]\033[0m Echo (Live)   | \033[1;32m[v]\033[0m Info Verbose  | \033[1;32m[i]\033[0m Info"
            echo -e " \033[1;32m[h]\033[0m Hz            | \033[1;32m[b]\033[0m Bandwidth     | \033[1;32m[t]\033[0m Show Type Def"
            echo -e " \033[1;31m[q]\033[0m Back"
            echo -e "----------------------------------------------------"
            read -rsn1 -p "Select action: " action_key

            case "$action_key" in
                e)
                    clear; echo -e "\033[1;32m>>> Echo Live: $selected_topic (Ctrl+C to exit)\033[0m"
                    ros2 topic echo "$selected_topic"
                    read -n 1 -s -r -p "Press any key..." ;;
                v)
                    clear; echo -e "\033[1;32m>>> Info Verbose: $selected_topic\033[0m"
                    ros2 topic info -v "$selected_topic"; echo ""; read -n 1 -s -r -p "Press any key..." ;;
                i)
                    clear; echo -e "\033[1;32m>>> Info: $selected_topic\033[0m"
                    ros2 topic info "$selected_topic"; echo ""; read -n 1 -s -r -p "Press any key..." ;;
                h)
                    clear; echo -e "\033[1;32m>>> Hz: $selected_topic (Ctrl+C to exit)\033[0m"
                    ros2 topic hz "$selected_topic" ;;
                b)
                    clear; echo -e "\033[1;32m>>> Bandwidth: $selected_topic (Ctrl+C to exit)\033[0m"
                    ros2 topic bw "$selected_topic" ;;
                t)
                    clear; echo -e "\033[1;32m>>> Type Definition\033[0m"
                    local msg_type=$(ros2 topic info "$selected_topic" | grep "Type:" | awk '{print $2}')
                    if [ -n "$msg_type" ]; then
                        echo -e "Type: \033[1;34m$msg_type\033[0m\n"
                        ros2 interface show "$msg_type"
                    else echo "Could not determine type."; fi
                    echo ""; read -n 1 -s -r -p "Press any key..." ;;
                q) return 0 ;;
            esac
        done
    }

    if ! command -v ros2 &> /dev/null; then
        echo -e "\033[1;31m[Error]\033[0m ros2 command not found."
        return 1
    fi

    echo -e "\033[1;36mScanning topics...\033[0m"
    local all_topics_raw
    all_topics_raw=$(ros2 topic list)

    if [ -z "$all_topics_raw" ]; then echo "No topics found."; return 0; fi

    tput civis 

    while true; do
        local display_items=()
        local real_values=()
        local is_namespace=()

        while IFS= read -r ns; do
            if [[ -n "$ns" ]]; then
                local disp_ns=${ns%/}
                display_items+=("$disp_ns")
                real_values+=("$ns")
                is_namespace+=("true")
            fi
        done < <(echo "$all_topics_raw" | grep -o '^/[^/]*/' | sort -u)

        while IFS= read -r topic; do
            if [[ -n "$topic" ]]; then
                display_items+=("$topic")
                real_values+=("$topic")
                is_namespace+=("false")
            fi
        done < <(echo "$all_topics_raw" | grep -v '^/[^/]*/')

        local total_rows=${#display_items[@]}
        local current_row=0

        while true; do
            clear
            echo -e "\033[1;36m=== Topic Explorer (Root) ===\033[0m"
            echo -e "\033[90mArrows: Navigate | Enter: Open | q: Quit\033[0m"
            echo "-----------------------------"
            
            for ((i=0; i<total_rows; i++)); do
                local label="${display_items[$i]}"
                local is_ns="${is_namespace[$i]}"
                local icon=" " 
                local color="\033[0m"

                if [[ "$is_ns" == "true" ]]; then
                    icon="\033[1;33m📁\033[0m"
                    color="\033[1;34m"
                fi

                if [ $i -eq $current_row ]; then
                    echo -e "\033[1;32m > $icon $color$label \033[0m"
                else
                    echo -e "   $icon $color$label \033[0m"
                fi
            done

            read -rsn3 key
            if [[ "$key" == $'\033[A' ]]; then
                ((current_row--))
                if [ $current_row -lt 0 ]; then current_row=$((total_rows-1)); fi
            elif [[ "$key" == $'\033[B' ]]; then
                ((current_row++))
                if [ $current_row -ge $total_rows ]; then current_row=0; fi
            elif [[ "$key" == "" ]]; then
                local selected_val="${real_values[$current_row]}"
                local selected_type="${is_namespace[$current_row]}"

                if [[ "$selected_type" == "true" ]]; then
                    local ns_prefix="$selected_val"
                    local sub_current=0
                    
                    while true; do
                        local sub_topics=()
                        while IFS= read -r t; do sub_topics+=("$t"); done < <(echo "$all_topics_raw" | grep "^$ns_prefix")
                        local sub_total=${#sub_topics[@]}
                        
                        clear
                        echo -e "\033[1;36m=== Namespace: \033[1;33m$ns_prefix\033[1;36m ===\033[0m"
                        echo -e "\033[90mq / Backspace: Go back\033[0m"
                        echo "-----------------------------"

                        for ((j=0; j<sub_total; j++)); do
                            local full_name="${sub_topics[$j]}"
                            local rel_name="${full_name#$ns_prefix}" 
                            
                            if [ $j -eq $sub_current ]; then
                                echo -e "\033[1;32m > $rel_name \033[0m"
                            else
                                echo -e "   $rel_name"
                            fi
                        done

                        read -rsn3 sub_key
                        if [[ "$sub_key" == $'\033[A' ]]; then 
                            ((sub_current--))
                            if [ $sub_current -lt 0 ]; then sub_current=$((sub_total-1)); fi
                        elif [[ "$sub_key" == $'\033[B' ]]; then 
                            ((sub_current++))
                            if [ $sub_current -ge $sub_total ]; then sub_current=0; fi
                        elif [[ "$sub_key" == "" ]]; then
                            tput cnorm
                            _t_analyze "${sub_topics[$sub_current]}"
                            tput civis
                        elif [[ "$sub_key" == "q" ]] || [[ "$sub_key" == $'\177' ]]; then
                            break
                        fi
                    done
                else
                    tput cnorm
                    _t_analyze "$selected_val"
                    tput civis
                fi

            elif [[ "$key" == "q" ]]; then
                break 2
            fi
        done
        break
    done
    tput cnorm
}


function k() {
    s
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:="/diff_cont/cmd_vel_unstamped"
}

function f(){
    s
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
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


echo "   b [-r|-a|-c] [-p] -> Smart Build (Smart/All/Changed/Pkg)"
echo "   l               -> Interactive Launch (Param Editor)"
echo "   t               -> Topic Explorer (Tree View & Analysis)"
echo "   s               -> Source setup.bash"
echo "   g               -> Run Greenwave Monitor"
echo "   r               -> Open rqt panels"
echo "   k               -> Teleop Twist Keyboard"
echo "   u               -> Generate URDF for Isaac Sim"
echo "   f               -> Run Foxglove bridge"