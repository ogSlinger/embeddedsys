################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-m4 -mfloat-abi=soft -mfpu=none -mlittle-endian -mthumb -Oz -I"C:/Users/Derek/workspace_v12/cs350project" -I"C:/Users/Derek/workspace_v12/cs350project/MCU+Image" -I"C:/ti/simplelink_cc32xx_sdk_7_10_00_13/source" -I"C:/ti/simplelink_cc32xx_sdk_7_10_00_13/kernel/nortos" -I"C:/ti/simplelink_cc32xx_sdk_7_10_00_13/kernel/nortos/posix" -DDeviceFamily_CC3220 -DNORTOS_SUPPORT -gdwarf-3 -march=armv7e-m -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/Derek/workspace_v12/cs350project/MCU+Image/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1057762799: ../gpiointerrupt.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/sysconfig_1_12_0/sysconfig_cli.bat" --script "C:/Users/Derek/workspace_v12/cs350project/gpiointerrupt.syscfg" -o "syscfg" -s "C:/ti/simplelink_cc32xx_sdk_7_10_00_13/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_drivers_config.c: build-1057762799 ../gpiointerrupt.syscfg
syscfg/ti_drivers_config.h: build-1057762799
syscfg/ti_utils_build_linker.cmd.genlibs: build-1057762799
syscfg/syscfg_c.rov.xs: build-1057762799
syscfg: build-1057762799

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-m4 -mfloat-abi=soft -mfpu=none -mlittle-endian -mthumb -Oz -I"C:/Users/Derek/workspace_v12/cs350project" -I"C:/Users/Derek/workspace_v12/cs350project/MCU+Image" -I"C:/ti/simplelink_cc32xx_sdk_7_10_00_13/source" -I"C:/ti/simplelink_cc32xx_sdk_7_10_00_13/kernel/nortos" -I"C:/ti/simplelink_cc32xx_sdk_7_10_00_13/kernel/nortos/posix" -DDeviceFamily_CC3220 -DNORTOS_SUPPORT -gdwarf-3 -march=armv7e-m -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/Derek/workspace_v12/cs350project/MCU+Image/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-688870817: ../image.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/sysconfig_1_12_0/sysconfig_cli.bat" --script "C:/Users/Derek/workspace_v12/cs350project/image.syscfg" -o "syscfg" -s "C:/ti/simplelink_cc32xx_sdk_7_10_00_13/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_drivers_net_wifi_config.json: build-688870817 ../image.syscfg
syscfg: build-688870817


