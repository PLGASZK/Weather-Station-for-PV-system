#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/WeatherStation_AC164164.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/WeatherStation_AC164164.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=mcc_generated_files/ext_int.c mcc_generated_files/i2c1.c mcc_generated_files/interrupt_manager.c mcc_generated_files/mcc.c mcc_generated_files/pin_manager.c mcc_generated_files/tmr1.c mcc_generated_files/traps.c mcc_generated_files/uart2.c WINC1500/Board_driver/wf_mcu_driver_stub.c WINC1500/Driver/wf_asic.c WINC1500/Driver/wf_hif.c WINC1500/Driver/wf_ota.c WINC1500/Driver/wf_socket.c WINC1500/Driver/wf_spi.c WINC1500/Driver/wf_spi_flash.c WINC1500/Driver/wf_utils.c WINC1500/Driver/wf_wifi.c WINC1500/Example/winc1500_fw_update_over_serial.c WINC1500/Example/winc1500_http_download.c WINC1500/Example/winc1500_locate_ip_address.c WINC1500/Example/winc1500_mac_address.c WINC1500/Example/winc1500_mode_ap.c WINC1500/Example/winc1500_mode_change.c WINC1500/Example/winc1500_mode_p2p.c WINC1500/Example/winc1500_mode_sta.c WINC1500/Example/winc1500_ota_firmware_update.c WINC1500/Example/winc1500_power_save_mode.c WINC1500/Example/winc1500_provision_http.c WINC1500/Example/winc1500_security_wep_wpa.c WINC1500/Example/winc1500_security_wps.c WINC1500/Example/winc1500_send_email.c WINC1500/Example/winc1500_signal_strength.c WINC1500/Example/winc1500_ssl_connect.c WINC1500/Example/winc1500_tcp_client.c WINC1500/Example/winc1500_tcp_server.c WINC1500/Example/winc1500_time_client.c WINC1500/Example/winc1500_weather_client.c WINC1500/Support/winc1500_support_stub.c bsp.c main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/mcc_generated_files/ext_int.o ${OBJECTDIR}/mcc_generated_files/i2c1.o ${OBJECTDIR}/mcc_generated_files/interrupt_manager.o ${OBJECTDIR}/mcc_generated_files/mcc.o ${OBJECTDIR}/mcc_generated_files/pin_manager.o ${OBJECTDIR}/mcc_generated_files/tmr1.o ${OBJECTDIR}/mcc_generated_files/traps.o ${OBJECTDIR}/mcc_generated_files/uart2.o ${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o ${OBJECTDIR}/WINC1500/Driver/wf_asic.o ${OBJECTDIR}/WINC1500/Driver/wf_hif.o ${OBJECTDIR}/WINC1500/Driver/wf_ota.o ${OBJECTDIR}/WINC1500/Driver/wf_socket.o ${OBJECTDIR}/WINC1500/Driver/wf_spi.o ${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o ${OBJECTDIR}/WINC1500/Driver/wf_utils.o ${OBJECTDIR}/WINC1500/Driver/wf_wifi.o ${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o ${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o ${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o ${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o ${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o ${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o ${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o ${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o ${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o ${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o ${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o ${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o ${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o ${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o ${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o ${OBJECTDIR}/bsp.o ${OBJECTDIR}/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/mcc_generated_files/ext_int.o.d ${OBJECTDIR}/mcc_generated_files/i2c1.o.d ${OBJECTDIR}/mcc_generated_files/interrupt_manager.o.d ${OBJECTDIR}/mcc_generated_files/mcc.o.d ${OBJECTDIR}/mcc_generated_files/pin_manager.o.d ${OBJECTDIR}/mcc_generated_files/tmr1.o.d ${OBJECTDIR}/mcc_generated_files/traps.o.d ${OBJECTDIR}/mcc_generated_files/uart2.o.d ${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o.d ${OBJECTDIR}/WINC1500/Driver/wf_asic.o.d ${OBJECTDIR}/WINC1500/Driver/wf_hif.o.d ${OBJECTDIR}/WINC1500/Driver/wf_ota.o.d ${OBJECTDIR}/WINC1500/Driver/wf_socket.o.d ${OBJECTDIR}/WINC1500/Driver/wf_spi.o.d ${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o.d ${OBJECTDIR}/WINC1500/Driver/wf_utils.o.d ${OBJECTDIR}/WINC1500/Driver/wf_wifi.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o.d ${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o.d ${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o.d ${OBJECTDIR}/bsp.o.d ${OBJECTDIR}/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/mcc_generated_files/ext_int.o ${OBJECTDIR}/mcc_generated_files/i2c1.o ${OBJECTDIR}/mcc_generated_files/interrupt_manager.o ${OBJECTDIR}/mcc_generated_files/mcc.o ${OBJECTDIR}/mcc_generated_files/pin_manager.o ${OBJECTDIR}/mcc_generated_files/tmr1.o ${OBJECTDIR}/mcc_generated_files/traps.o ${OBJECTDIR}/mcc_generated_files/uart2.o ${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o ${OBJECTDIR}/WINC1500/Driver/wf_asic.o ${OBJECTDIR}/WINC1500/Driver/wf_hif.o ${OBJECTDIR}/WINC1500/Driver/wf_ota.o ${OBJECTDIR}/WINC1500/Driver/wf_socket.o ${OBJECTDIR}/WINC1500/Driver/wf_spi.o ${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o ${OBJECTDIR}/WINC1500/Driver/wf_utils.o ${OBJECTDIR}/WINC1500/Driver/wf_wifi.o ${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o ${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o ${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o ${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o ${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o ${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o ${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o ${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o ${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o ${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o ${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o ${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o ${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o ${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o ${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o ${OBJECTDIR}/bsp.o ${OBJECTDIR}/main.o

# Source Files
SOURCEFILES=mcc_generated_files/ext_int.c mcc_generated_files/i2c1.c mcc_generated_files/interrupt_manager.c mcc_generated_files/mcc.c mcc_generated_files/pin_manager.c mcc_generated_files/tmr1.c mcc_generated_files/traps.c mcc_generated_files/uart2.c WINC1500/Board_driver/wf_mcu_driver_stub.c WINC1500/Driver/wf_asic.c WINC1500/Driver/wf_hif.c WINC1500/Driver/wf_ota.c WINC1500/Driver/wf_socket.c WINC1500/Driver/wf_spi.c WINC1500/Driver/wf_spi_flash.c WINC1500/Driver/wf_utils.c WINC1500/Driver/wf_wifi.c WINC1500/Example/winc1500_fw_update_over_serial.c WINC1500/Example/winc1500_http_download.c WINC1500/Example/winc1500_locate_ip_address.c WINC1500/Example/winc1500_mac_address.c WINC1500/Example/winc1500_mode_ap.c WINC1500/Example/winc1500_mode_change.c WINC1500/Example/winc1500_mode_p2p.c WINC1500/Example/winc1500_mode_sta.c WINC1500/Example/winc1500_ota_firmware_update.c WINC1500/Example/winc1500_power_save_mode.c WINC1500/Example/winc1500_provision_http.c WINC1500/Example/winc1500_security_wep_wpa.c WINC1500/Example/winc1500_security_wps.c WINC1500/Example/winc1500_send_email.c WINC1500/Example/winc1500_signal_strength.c WINC1500/Example/winc1500_ssl_connect.c WINC1500/Example/winc1500_tcp_client.c WINC1500/Example/winc1500_tcp_server.c WINC1500/Example/winc1500_time_client.c WINC1500/Example/winc1500_weather_client.c WINC1500/Support/winc1500_support_stub.c bsp.c main.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/WeatherStation_AC164164.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ128GA705
MP_LINKER_FILE_OPTION=,--script=p24FJ128GA705.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/mcc_generated_files/ext_int.o: mcc_generated_files/ext_int.c  .generated_files/flags/default/3690ee32971d5c53c2da9bf28c5b110514dc32b .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/ext_int.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/ext_int.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/ext_int.c  -o ${OBJECTDIR}/mcc_generated_files/ext_int.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/ext_int.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/i2c1.o: mcc_generated_files/i2c1.c  .generated_files/flags/default/e7d7ab6ee7e5e75dd78e15ee96b0bef8459c85d9 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/i2c1.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/i2c1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/i2c1.c  -o ${OBJECTDIR}/mcc_generated_files/i2c1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/i2c1.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/interrupt_manager.o: mcc_generated_files/interrupt_manager.c  .generated_files/flags/default/7b0731627b5e63682ecced2a6469cfb8e4a2f260 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/interrupt_manager.c  -o ${OBJECTDIR}/mcc_generated_files/interrupt_manager.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/interrupt_manager.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/mcc.o: mcc_generated_files/mcc.c  .generated_files/flags/default/f913b5ae96b7d76a94b16721f626800eb8626feb .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/mcc.c  -o ${OBJECTDIR}/mcc_generated_files/mcc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/mcc.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/pin_manager.o: mcc_generated_files/pin_manager.c  .generated_files/flags/default/3274e002ccbcb26889cded282302213338516336 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/pin_manager.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/pin_manager.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/pin_manager.c  -o ${OBJECTDIR}/mcc_generated_files/pin_manager.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/pin_manager.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/tmr1.o: mcc_generated_files/tmr1.c  .generated_files/flags/default/16254165bab32c65b2236dc7309bc58a126e409c .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/tmr1.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/tmr1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/tmr1.c  -o ${OBJECTDIR}/mcc_generated_files/tmr1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/tmr1.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/traps.o: mcc_generated_files/traps.c  .generated_files/flags/default/e9ccc323000743586c2625d6c5daf51d536d5371 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/traps.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/traps.c  -o ${OBJECTDIR}/mcc_generated_files/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/traps.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/uart2.o: mcc_generated_files/uart2.c  .generated_files/flags/default/4f059831d285c2c505589e3cb1aad2ba5211829a .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart2.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/uart2.c  -o ${OBJECTDIR}/mcc_generated_files/uart2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/uart2.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o: WINC1500/Board_driver/wf_mcu_driver_stub.c  .generated_files/flags/default/4d4618fc9113a47a2789141e53a018b69bb173b .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Board_driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Board_driver/wf_mcu_driver_stub.c  -o ${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_asic.o: WINC1500/Driver/wf_asic.c  .generated_files/flags/default/a4901644899b520ffd2027df7fcf5b29f3f24050 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_asic.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_asic.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_asic.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_asic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_asic.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_hif.o: WINC1500/Driver/wf_hif.c  .generated_files/flags/default/372a3107a07ba10faa902235aad32037dc62544 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_hif.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_hif.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_hif.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_hif.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_hif.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_ota.o: WINC1500/Driver/wf_ota.c  .generated_files/flags/default/ac22bfa26c394062543d31d2600e1de755d14480 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_ota.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_ota.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_ota.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_ota.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_ota.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_socket.o: WINC1500/Driver/wf_socket.c  .generated_files/flags/default/36f4ccb344b98132a94170d56ee9590cacd99d26 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_socket.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_socket.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_socket.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_socket.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_socket.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_spi.o: WINC1500/Driver/wf_spi.c  .generated_files/flags/default/f94b662c7e0eabaa661ac7665b5c046c9e5ddb09 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_spi.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_spi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_spi.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_spi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_spi.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o: WINC1500/Driver/wf_spi_flash.c  .generated_files/flags/default/833ae08715ac08685f9cd95d7677c8722d649ad6 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_spi_flash.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_utils.o: WINC1500/Driver/wf_utils.c  .generated_files/flags/default/7164e38567e8dc4774cf0d7429236c74ae902b84 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_utils.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_utils.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_utils.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_utils.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_utils.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_wifi.o: WINC1500/Driver/wf_wifi.c  .generated_files/flags/default/2e34644a5221cbdff4abe9922d9f77ff2026460b .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_wifi.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_wifi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_wifi.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_wifi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_wifi.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o: WINC1500/Example/winc1500_fw_update_over_serial.c  .generated_files/flags/default/aab290efd52350ca0b53a9d6ce36d672d525d6a .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_fw_update_over_serial.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o: WINC1500/Example/winc1500_http_download.c  .generated_files/flags/default/b13f2299279e8882f9a41bcd3dcdb85be27e0668 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_http_download.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o: WINC1500/Example/winc1500_locate_ip_address.c  .generated_files/flags/default/7c87e3d6835d15439ca60dee0d865bf08384cf52 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_locate_ip_address.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o: WINC1500/Example/winc1500_mac_address.c  .generated_files/flags/default/c36d21d8c7199182841e32c8aa78853c83d75b20 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mac_address.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o: WINC1500/Example/winc1500_mode_ap.c  .generated_files/flags/default/925967470eeb6ba956685f3675028b1f9dc10022 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mode_ap.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o: WINC1500/Example/winc1500_mode_change.c  .generated_files/flags/default/ae73b9ca20bafdb5b98888203d49be78ff7e7598 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mode_change.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o: WINC1500/Example/winc1500_mode_p2p.c  .generated_files/flags/default/43a4dea452e8701e650c444db8cda20bed866a3c .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mode_p2p.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o: WINC1500/Example/winc1500_mode_sta.c  .generated_files/flags/default/7bed763f1a2d09715fc981298c433cea90c9f8f .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mode_sta.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o: WINC1500/Example/winc1500_ota_firmware_update.c  .generated_files/flags/default/b08a5b1fd5da0d7ceb8c650d8735b8c19c3a36c9 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_ota_firmware_update.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o: WINC1500/Example/winc1500_power_save_mode.c  .generated_files/flags/default/1d77edcbda4b95cb51ad2cbb23d7a7be2dc618df .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_power_save_mode.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o: WINC1500/Example/winc1500_provision_http.c  .generated_files/flags/default/ee9c3ffe1f2d9868c2042a7dc61b12a8b1b3b614 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_provision_http.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o: WINC1500/Example/winc1500_security_wep_wpa.c  .generated_files/flags/default/eac80d8a8e9409d212c6f89e41d6c4617e215648 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_security_wep_wpa.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o: WINC1500/Example/winc1500_security_wps.c  .generated_files/flags/default/3fd47037a42fce6fc82cb21b3cefdf32bae4103f .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_security_wps.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o: WINC1500/Example/winc1500_send_email.c  .generated_files/flags/default/52468019419ace61ecc25987c92d8c032d611ead .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_send_email.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o: WINC1500/Example/winc1500_signal_strength.c  .generated_files/flags/default/11169d205d82382d596480728b7db8f8e52f98ad .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_signal_strength.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o: WINC1500/Example/winc1500_ssl_connect.c  .generated_files/flags/default/d0bea6ca134fb69c4770e909648c450f25451819 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_ssl_connect.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o: WINC1500/Example/winc1500_tcp_client.c  .generated_files/flags/default/fbd57baeb6c384a343cd6348a270f5fb8e416ca8 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_tcp_client.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o: WINC1500/Example/winc1500_tcp_server.c  .generated_files/flags/default/d17756216b63a79db3cfaae09a2405a10ee00c68 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_tcp_server.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o: WINC1500/Example/winc1500_time_client.c  .generated_files/flags/default/c8f910e4d252616fc10a47330dfe8d0a8f36879c .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_time_client.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o: WINC1500/Example/winc1500_weather_client.c  .generated_files/flags/default/12b400e5ce940f8d77012b35b42ec27d3612c98b .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_weather_client.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o: WINC1500/Support/winc1500_support_stub.c  .generated_files/flags/default/f11ab9028171728dce06768696a4a6016edfb02a .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Support" 
	@${RM} ${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Support/winc1500_support_stub.c  -o ${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/bsp.o: bsp.c  .generated_files/flags/default/318f98aefe375eed3b07d838fbf931af8dee7dcc .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/bsp.o.d 
	@${RM} ${OBJECTDIR}/bsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  bsp.c  -o ${OBJECTDIR}/bsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/bsp.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/d9a0558162f6d2f5f490f8adb1b15854e09b94a .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
else
${OBJECTDIR}/mcc_generated_files/ext_int.o: mcc_generated_files/ext_int.c  .generated_files/flags/default/6e27e01a11c153895ff57e992f4aad9118598e0f .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/ext_int.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/ext_int.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/ext_int.c  -o ${OBJECTDIR}/mcc_generated_files/ext_int.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/ext_int.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/i2c1.o: mcc_generated_files/i2c1.c  .generated_files/flags/default/265dde3cb58d255794b9930de9e0db3c4e4da364 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/i2c1.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/i2c1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/i2c1.c  -o ${OBJECTDIR}/mcc_generated_files/i2c1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/i2c1.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/interrupt_manager.o: mcc_generated_files/interrupt_manager.c  .generated_files/flags/default/9120de191e5ce7d9689b9f78c99fc999b7a54438 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/interrupt_manager.c  -o ${OBJECTDIR}/mcc_generated_files/interrupt_manager.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/interrupt_manager.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/mcc.o: mcc_generated_files/mcc.c  .generated_files/flags/default/7ce015f81bf143b5b1f824f5d34daf1da1e4baf .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/mcc.c  -o ${OBJECTDIR}/mcc_generated_files/mcc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/mcc.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/pin_manager.o: mcc_generated_files/pin_manager.c  .generated_files/flags/default/727f08f2c1b582a80433428bbefa50428092c431 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/pin_manager.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/pin_manager.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/pin_manager.c  -o ${OBJECTDIR}/mcc_generated_files/pin_manager.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/pin_manager.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/tmr1.o: mcc_generated_files/tmr1.c  .generated_files/flags/default/a9b3d1552da872adbaf154e5c431a7a7d7730f7b .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/tmr1.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/tmr1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/tmr1.c  -o ${OBJECTDIR}/mcc_generated_files/tmr1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/tmr1.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/traps.o: mcc_generated_files/traps.c  .generated_files/flags/default/45f0fa64c2a8596cd44a9cdf9f15f045cb303902 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/traps.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/traps.c  -o ${OBJECTDIR}/mcc_generated_files/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/traps.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/mcc_generated_files/uart2.o: mcc_generated_files/uart2.c  .generated_files/flags/default/1f87663ce694d755d5a401a74516d68c0f248ba1 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart2.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  mcc_generated_files/uart2.c  -o ${OBJECTDIR}/mcc_generated_files/uart2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/mcc_generated_files/uart2.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o: WINC1500/Board_driver/wf_mcu_driver_stub.c  .generated_files/flags/default/6fbd99a1a22ca5c1b1090a22cb9e7438e6e7a044 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Board_driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Board_driver/wf_mcu_driver_stub.c  -o ${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Board_driver/wf_mcu_driver_stub.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_asic.o: WINC1500/Driver/wf_asic.c  .generated_files/flags/default/6a3e1c652bd2062a4a5934ab3007bc143895956a .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_asic.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_asic.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_asic.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_asic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_asic.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_hif.o: WINC1500/Driver/wf_hif.c  .generated_files/flags/default/da37d579d39a3ff7457c8480d34f405fd79d0bb3 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_hif.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_hif.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_hif.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_hif.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_hif.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_ota.o: WINC1500/Driver/wf_ota.c  .generated_files/flags/default/c6415f4ba28d7b34dbe863198b05b77bb1ec7438 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_ota.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_ota.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_ota.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_ota.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_ota.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_socket.o: WINC1500/Driver/wf_socket.c  .generated_files/flags/default/277cee1dc6203cc1aca9993e16503a0b9c0f92a6 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_socket.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_socket.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_socket.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_socket.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_socket.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_spi.o: WINC1500/Driver/wf_spi.c  .generated_files/flags/default/d3499dea36157efe9ab4c501d4205b7f4465719e .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_spi.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_spi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_spi.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_spi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_spi.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o: WINC1500/Driver/wf_spi_flash.c  .generated_files/flags/default/c4e9e43d223b2a33753d2f3f6a9be2e2d4c535ec .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_spi_flash.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_spi_flash.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_utils.o: WINC1500/Driver/wf_utils.c  .generated_files/flags/default/a74c55bc2becfee62c6faa8fb771a933ca5c47de .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_utils.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_utils.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_utils.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_utils.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_utils.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Driver/wf_wifi.o: WINC1500/Driver/wf_wifi.c  .generated_files/flags/default/5fba138ae8db39f25cdba37eed8f8c81b9e5033a .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Driver" 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_wifi.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Driver/wf_wifi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Driver/wf_wifi.c  -o ${OBJECTDIR}/WINC1500/Driver/wf_wifi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Driver/wf_wifi.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o: WINC1500/Example/winc1500_fw_update_over_serial.c  .generated_files/flags/default/ae501252fc7aec0cb40d1547cb56341fa93a5920 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_fw_update_over_serial.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_fw_update_over_serial.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o: WINC1500/Example/winc1500_http_download.c  .generated_files/flags/default/3025fae902639c4d1251e38ff908393c5b11e6de .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_http_download.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_http_download.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o: WINC1500/Example/winc1500_locate_ip_address.c  .generated_files/flags/default/ca847ba130fe6311d5994595c58514c5c6bdbe10 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_locate_ip_address.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_locate_ip_address.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o: WINC1500/Example/winc1500_mac_address.c  .generated_files/flags/default/8084960fd5738df29f893c5950fd148de800b0a5 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mac_address.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mac_address.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o: WINC1500/Example/winc1500_mode_ap.c  .generated_files/flags/default/9af527fdfc60d0c358953826aff7e7feba8f2d14 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mode_ap.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mode_ap.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o: WINC1500/Example/winc1500_mode_change.c  .generated_files/flags/default/3f3841d8ae176b924ed1d912f32f53689d8cc4ed .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mode_change.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mode_change.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o: WINC1500/Example/winc1500_mode_p2p.c  .generated_files/flags/default/4592a338623e6714658692a299a44d878da6a120 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mode_p2p.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mode_p2p.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o: WINC1500/Example/winc1500_mode_sta.c  .generated_files/flags/default/cc7deac75f6ab12008b416c277e19ed4106bc65e .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_mode_sta.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_mode_sta.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o: WINC1500/Example/winc1500_ota_firmware_update.c  .generated_files/flags/default/25cde4fedb9f6a144df42598027782e0ead2400a .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_ota_firmware_update.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_ota_firmware_update.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o: WINC1500/Example/winc1500_power_save_mode.c  .generated_files/flags/default/67da25084810c99b02e9ca7cf49ec3564733f3e6 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_power_save_mode.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_power_save_mode.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o: WINC1500/Example/winc1500_provision_http.c  .generated_files/flags/default/499ffd77e54f4edc2a43784edeb48d88c83a7f6f .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_provision_http.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_provision_http.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o: WINC1500/Example/winc1500_security_wep_wpa.c  .generated_files/flags/default/e3d2973b5dabbc8ea9b8675dc319d42378b90d2e .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_security_wep_wpa.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_security_wep_wpa.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o: WINC1500/Example/winc1500_security_wps.c  .generated_files/flags/default/fda848c97bf1b9a50452686dc82385d04d4de34b .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_security_wps.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_security_wps.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o: WINC1500/Example/winc1500_send_email.c  .generated_files/flags/default/cb60ffaecc2b02bc416c1813cdc60292253501d6 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_send_email.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_send_email.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o: WINC1500/Example/winc1500_signal_strength.c  .generated_files/flags/default/2687fea6e9a32c0e82469d614b84683cedc8a14d .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_signal_strength.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_signal_strength.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o: WINC1500/Example/winc1500_ssl_connect.c  .generated_files/flags/default/6e7fde5c33f9797a61196a2a20753bc25dc6b427 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_ssl_connect.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_ssl_connect.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o: WINC1500/Example/winc1500_tcp_client.c  .generated_files/flags/default/5dbbc1baa776b8538ce279f2248c1dc0c6ef37d3 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_tcp_client.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_tcp_client.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o: WINC1500/Example/winc1500_tcp_server.c  .generated_files/flags/default/8697130c640182d2ffbba863db3fde6c60e8a0c5 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_tcp_server.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_tcp_server.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o: WINC1500/Example/winc1500_time_client.c  .generated_files/flags/default/88a42c66bbf2f98cadbdda54d76116ebd9abb62 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_time_client.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_time_client.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o: WINC1500/Example/winc1500_weather_client.c  .generated_files/flags/default/85745651c5280700f55593542c8fb6b6f722b198 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Example" 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Example/winc1500_weather_client.c  -o ${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Example/winc1500_weather_client.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o: WINC1500/Support/winc1500_support_stub.c  .generated_files/flags/default/b60323495001f9a47386e81b5bf81462556074fa .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}/WINC1500/Support" 
	@${RM} ${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o.d 
	@${RM} ${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  WINC1500/Support/winc1500_support_stub.c  -o ${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/WINC1500/Support/winc1500_support_stub.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/bsp.o: bsp.c  .generated_files/flags/default/700d3ba89214afc4c141e35aaf93a83ade949dd1 .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/bsp.o.d 
	@${RM} ${OBJECTDIR}/bsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  bsp.c  -o ${OBJECTDIR}/bsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/bsp.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/a622cd9ff63e9b4b3ecdb21d1997580d02f5a43c .generated_files/flags/default/7dab310884e5de58b737751714b62c5515706f0f
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/WeatherStation_AC164164.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/WeatherStation_AC164164.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG   -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common"  -mreserve=data@0x800:0x81B -mreserve=data@0x81C:0x81D -mreserve=data@0x81E:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x827 -mreserve=data@0x82A:0x84F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  
	
else
${DISTDIR}/WeatherStation_AC164164.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/WeatherStation_AC164164.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -I"mcc_generated_files" -I"WINC1500" -I"WINC1500/Board_driver" -I"WINC1500/Example" -I"WINC1500/Support" -I"WINC1500/Driver" -I"WINC1500/Timer1" -I"WINC1500/Common" -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  
	${MP_CC_DIR}\\xc16-bin2hex ${DISTDIR}/WeatherStation_AC164164.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
