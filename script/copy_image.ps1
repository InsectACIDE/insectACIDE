# Set the paths for the source and destination files
$sfile= "$env:InsectACIDE_DIR\core\Example\Sherloc_FreeRTOS_MPU_S_NS\Sherloc_s\Objects\Sherloc_s.axf"
$nsfile ="$env:InsectACIDE_DIR\core\Example\Sherloc_FreeRTOS_MPU_S_NS\FreeRTOS_MPU_ns\Objects\FreeRTOS_MPU_ns.axf"
$destsfile = "E:\SOFTWARE\s.axf"
$destnsfile = "E:\SOFTWARE\ns.axf"
# Copy the file from the source path to the destination path
Copy-Item $sfile $destsfile
Copy-Item $nsfile $destnsfile 

