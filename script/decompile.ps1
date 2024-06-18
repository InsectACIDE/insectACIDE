

$sfile= "$env:InsectACIDE_DIR\core\Example\Sherloc_FreeRTOS_MPU_S_NS\Sherloc_s\Objects\Sherloc_s.axf"
$nsfile ="$env:InsectACIDE_DIR\core\Example\Sherloc_FreeRTOS_MPU_S_NS\FreeRTOS_MPU_ns\Objects\FreeRTOS_MPU_ns.axf"

arm-none-eabi-objdump.exe -D $nsfile| Out-File -FilePath "ns_output.txt"
arm-none-eabi-objdump.exe -D $sfile| Out-File -FilePath "s_output.txt"