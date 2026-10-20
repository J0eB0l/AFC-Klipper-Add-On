#!/usr/bin/env bash
# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

additional_system_menu() {
  local message
  local choice
  local counter board_counter qb_board_counter qb_motor_counter
  if ! check_existing_unit_installed; then
    echo ""
    echo "No existing unit installation found in ${afc_config_dir}."
    echo "Please install your first unit from the main Install menu before adding additional units."
    echo ""
    read -p "Press Enter to return to the main menu..." _
    main_menu
    return
  fi
  message="This menu allows you to add ADDITIONAL unit types. It should NOT be used for your first unit.\n"
  message+="This is a best effort in adding an additional unit. You will probably be required\n"
  message+="to manually edit the configuration files to ensure proper operation.\n"
  # Override the default boxturtle_name to prevent conflicts with the default name.
  boxturtle_name="Turtle_2"
  counter=0
  board_counter=0
  emu_board_counter=0
  qb_board_counter=0
  qb_motor_counter=0
  motor=0
  while true; do
    clear
    print_unit_art "$installation_type"
    printf "%b▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ \n" "$MENU_GREEN"
    printf "█%b                                    AFC Script Help      %b                            █\n" "$RESET" "$MENU_GREEN"
    printf "%b▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀%b \n" "$MENU_GREEN" "$RESET"
    printf "%b\n" "$message"
    printf "%b▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ \n" "$MENU_GREEN"
    printf "█%b         Please review the following options to add an additional unit type%b          █\n" "$RESET" "$MENU_GREEN"
    printf "█%b           Type a number or letter and press Enter/Return to toggle choice%b           █\n" "$RESET" "$MENU_GREEN"
    printf "%b▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀%b \n" "$MENU_GREEN" "$RESET"
    if [ "$files_updated_or_installed" == "False" ]; then
      printf "T. Installation Type: %s \n" "$installation_type"
      unit_print_additional_menu_row "$installation_type"
    fi
    echo ""
    if [ "$files_updated_or_installed" == "False" ]; then
      printf "${GREEN}I. Install system with current selections${NC}\n"
    fi
    printf "M. Return to Main Menu\n"
    printf "Q. Quit\n"
    echo ""
    read -p "Enter your choice: " choice

    choice="${choice^^}"

    case $choice in
      T)
        # Increment the counter and reset if it exceeds the array length
        counter=$(( (counter + 1) % ${#installation_options[@]} ))

        # Set the installation type to the current option
        installation_type="${installation_options[$counter]}"

        # Update the message
        message="Installation Type: $installation_type"
        export message ;;
      D)
        # Increment the counter and reset if it exceeds the array length
        board_counter=$(( (board_counter + 1) % ${#htlf_board_types[@]} ))

        # Set the installation type to the current option
        htlf_board_type="${htlf_board_types[$board_counter]}"

        # Update the message
        message="HTLF Board Type: $htlf_board_type"
        export message ;;
      E)
        # Increment the counter and reset if it exceeds the array length
        qb_board_counter=$(( (qb_board_counter + 1) % ${#qb_board_types[@]} ))

        # Set the installation type to the current option
        qb_board_type="${qb_board_types[$qb_board_counter]}"

        # Update the message
        message="QuattroBox Board Type: $qb_board_type"
        export message ;;
      F)
        # Increment the counter and reset if it exceeds the array length
        qb_motor_counter=$(( (qb_motor_counter + 1) % ${#qb_motor_types[@]} ))

        # Set the installation type to the current option
        qb_motor_type="${qb_motor_types[$qb_motor_counter]}"

        # Update the message
        message="QuattroBox Motor Type: $qb_motor_type"
        export message ;;
      G)
        cycle_array emu_num_lanes_options emu_num_lanes_index emu_num_lanes "EMU Lane Count" ;;
        
      H)
        if [ "$installation_type" == "EMU" ]; then
          cycle_array emu_board_types emu_board_counter emu_board_type "EMU Board Type"
        elif  [ "$installation_type" == "Claymore" ]; then
          # Increment the counter and reset if it exceeds the array length
          board_counter=$(( (board_counter + 1) % ${#htlf2_board_types[@]} ))

          # Set the installation type to the current option
          htlf2_board_type="${htlf2_board_types[$board_counter]}"

          # Update the message
          message="Claymore Board Type: $htlf2_board_type"
          export message
        fi ;;
      1)
        name_additional_unit
        export message ;;
      P)
        if [ "$installation_type" == "BridgeBox" ]; then
          bridgebox_ask_pool
        fi ;;
      W)
        if [ "$installation_type" == "BridgeBox" ]; then
          bridgebox_ask_connection
        fi ;;
      S)
        if [ "$installation_type" == "BridgeBox" ] && [ "$bridgebox_connection" != "wifi" ]; then
          bridgebox_ask_serial
        fi ;;
      X)
        if [ "$installation_type" == "BridgeBox" ]; then
          bridgebox_ask_extruder
        fi ;;
      Q) exit_afc_install ;;
      M) main_menu ;;
      I)
        verify_name_not_in_use ${boxturtle_name}
        if [ "$invalid_name" == "False" ]; then
          install_additional_unit
          message="${boxturtle_name} created successfully, please look over config file and update lane numbers."
          message+="\nAdditionally, please ensure any MCU connections are updated in the appropriate files (CANBus, serial, etc)"
          message+="\nThis is not a 100% turn-key solution and will require some manual configuration based on your specific setup."
          message+="\n\n${RED}Please restart the script after installation to install additional units or make changes to the current unit.${NC}"
          export message
        fi
        ;;
      *) echo "Invalid selection" ;;
    esac
  done
}
