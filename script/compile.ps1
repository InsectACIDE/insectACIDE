$sProject = "${env:InsectACIDE_DIR}\core\Example\Sherloc_FreeRTOS_MPU_S_NS\Sherloc_s\Sherloc_s.uvprojx"
$nsProject = "${env:InsectACIDE_DIR}\core\Example\Sherloc_FreeRTOS_MPU_S_NS\FreeRTOS_MPU_ns\FreeRTOS_MPU_ns.uvprojx"

$compiler = "C:\Users\penguin\Documents\yujie\insectacide\ide_space\core\UV4\UV4.exe"

& $compiler -j4 -b $sProject -o "${env:InsectACIDE_DIR}\build_logs\s_log.log"

& $compiler -j4 -b $nsProject -o "${env:InsectACIDE_DIR}\build_logs\ns_log.log"
