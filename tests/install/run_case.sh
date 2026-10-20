#!/usr/bin/env bash
# Characterization-test harness for the AFC installer's unit-handling functions.
#
# Sources the same include files install-afc.sh does (minus the interactive
# menu files), applies whatever env-var overrides the caller set, runs one
# MODE against a scratch AFC config dir, and prints a deterministic dump of
# the result to stdout. Used by tests/install/test_characterization.py to
# compare pre-refactor and post-refactor behavior byte-for-byte.
#
# MODE=copy_unit_files       -> run copy_unit_files() into a fresh config dir
# MODE=install_additional_unit -> run install_additional_unit() into a config
#                                  dir that already looks like an existing install
# MODE=buffer_target         -> run get_unit_buffer_target() and dump the globals it sets
# MODE=message                -> run build_install_message() and dump $message
# MODE=name_additional_unit   -> run name_additional_unit() (default answer, empty stdin)
#                                  and dump $boxturtle_name / $turtle_renamed
# MODE=installation_options   -> dump the installation_options array, one entry per line
# MODE=bridgebox_ask_pool     -> answer bridgebox_ask_pool() with $answers and dump the
#                                  pool counts and $message
# MODE=bridgebox_ask_connection -> answer bridgebox_ask_connection() with $answers and
#                                  dump the connection, the address and $message
# MODE=install_menu_options   -> run unit_print_install_menu_options() (install_menu.sh's
#                                  type-specific option rows) and dump its stdout
# MODE=additional_menu_row    -> run unit_print_additional_menu_row() (additional_system_menu.sh's
#                                  name + option rows) and dump its stdout plus the resulting
#                                  boxturtle_name (the function can mutate it as a side effect)
# MODE=no_adapter_survival    -> runs under `set -e` (unlike every other mode) and calls
#                                  each unit_* dispatcher plus print_unit_art for a type
#                                  with no registered adapter. install-afc.sh itself runs
#                                  under `set -e` and calls these as bare statements, so a
#                                  dispatcher that returns nonzero for "no adapter" would
#                                  silently kill the whole script instead of no-op'ing the
#                                  way an unmatched if/elif branch used to. This mode fails
#                                  (via `set -e`, no "still alive" line) if that regresses.

set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export afc_path="$REPO_ROOT"

# shellcheck source=/dev/null
source "$REPO_ROOT/include/units/registry.sh"
source "$REPO_ROOT/include/constants.sh"
# constants.sh unconditionally sets afc_path="$HOME/AFC-Klipper-Add-On"; force
# it back to the checked-out repo so tests exercise this tree's templates,
# not whatever (if anything) happens to be installed at that fixed path.
export afc_path="$REPO_ROOT"
source "$REPO_ROOT/include/colors.sh"
source "$REPO_ROOT/include/utils.sh"
source "$REPO_ROOT/include/check_commands.sh"
source "$REPO_ROOT/include/buffer_configurations.sh"
source "$REPO_ROOT/include/emu_templater.sh"
source "$REPO_ROOT/include/install_functions.sh"
source "$REPO_ROOT/include/unit_functions.sh"

# Apply any CASE_* overrides the python driver exported, stripping the prefix.
while IFS='=' read -r name _; do
  case "$name" in
    CASE_*)
      var="${name#CASE_}"
      printf -v "$var" '%s' "${!name}"
      ;;
  esac
done < <(env)

WORKDIR=$(mktemp -d)
trap 'rm -rf "$WORKDIR"' EXIT

printer_config_dir="$WORKDIR/printer_data/config"
klipper_dir="$WORKDIR/klipper"
afc_config_dir="$WORKDIR/AFC"
afc_file="$afc_config_dir/AFC.cfg"

dump_config_dir() {
  echo "=== FILES ==="
  find "$afc_config_dir" -type f | sed "s#^$afc_config_dir/##" | sort | while read -r rel; do
    echo "--- $rel ---"
    cat "$afc_config_dir/$rel"
    echo "--- end $rel ---"
  done
}

case "${MODE:-}" in
  copy_unit_files)
    mkdir -p "$afc_config_dir/mcu" "$afc_config_dir/macros"
    copy_unit_files
    dump_config_dir
    ;;
  install_additional_unit)
    mkdir -p "$afc_config_dir/mcu" "$afc_config_dir/macros"
    echo "placeholder" > "$afc_config_dir/AFC_Turtle_1.cfg"
    install_additional_unit
    dump_config_dir
    ;;
  buffer_target)
    get_unit_buffer_target
    echo "=== VARS ==="
    printf 'buffer_unit_name=%s\n' "$buffer_unit_name"
    printf 'buffer_unit_section_prefix=%s\n' "$buffer_unit_section_prefix"
    printf 'buffer_extruder_file=%s\n' "${buffer_extruder_file#"$afc_config_dir/"}"
    printf 'buffer_section_name=%s\n' "$buffer_section_name"
    printf 'buffer_prebaked_header=%s\n' "$buffer_prebaked_header"
    ;;
  message)
    mkdir -p "$afc_config_dir/mcu" "$afc_config_dir/macros"
    build_install_message
    echo "=== MESSAGE ==="
    normalized="${message//$WORKDIR/<WORKDIR>}"
    normalized="${normalized//$REPO_ROOT/<REPO_ROOT>}"
    normalized="${normalized//$HOME/<HOME>}"
    printf '%s' "$normalized"
    ;;
  name_additional_unit)
    name_additional_unit < /dev/null
    echo "=== VARS ==="
    printf 'boxturtle_name=%s\n' "$boxturtle_name"
    printf 'turtle_renamed=%s\n' "$turtle_renamed"
    ;;
  installation_options)
    printf '%s\n' "${installation_options[@]}"
    ;;
  bridgebox_ask_pool)
    bridgebox_ask_pool < <(printf '%b' "$answers") > /dev/null
    printf 'bridgebox_pool_ams=%s\n' "$bridgebox_pool_ams"
    printf 'bridgebox_pool_ht=%s\n' "$bridgebox_pool_ht"
    printf 'message=%s\n' "$message"
    ;;
  bridgebox_ask_connection)
    bridgebox_ask_connection < <(printf '%b' "$answers") > /dev/null
    printf 'bridgebox_connection=%s\n' "$bridgebox_connection"
    printf 'bridgebox_address=%s\n' "$bridgebox_address"
    printf 'message=%s\n' "$message"
    ;;
  install_menu_options)
    echo "=== OUTPUT ==="
    unit_print_install_menu_options "$installation_type"
    ;;
  additional_menu_row)
    echo "=== OUTPUT ==="
    unit_print_additional_menu_row "$installation_type"
    echo "=== VARS ==="
    printf 'boxturtle_name=%s\n' "$boxturtle_name"
    ;;
  no_adapter_survival)
    set -e
    source "$REPO_ROOT/include/menus/unit_art.sh"
    mkdir -p "$afc_config_dir/mcu" "$afc_config_dir/macros"
    boxturtle_name="Whatever_1"
    unit_copy_files "NoSuchType"
    unit_install_additional "NoSuchType"
    unit_set_buffer_target "NoSuchType"
    unit_append_message "NoSuchType"
    print_unit_art "NoSuchType"
    echo "still alive"
    ;;
  *)
    echo "Unknown MODE: ${MODE:-}" >&2
    exit 1
    ;;
esac
