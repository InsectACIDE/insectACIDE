
$workspaceDir = "$(Get-Location)\.."
Write-Host $workspaceDir


$out_Project="$workspaceDir\code\Sherloc-Cortex-M-CFVD"
$current_dir="$workspaceDir\script"
# $out_sfile="$out_Project\host_tools\evaluation\elf_s\full_rtos.axf"

### project compile
$compiler = "$workspaceDir\ide_space\core\UV4\UV4.exe"
$out_sProject="$out_Project\Example\Sherloc_FreeRTOS_MPU_S_NS\Sherloc_s\Sherloc_s.uvprojx"
& $compiler -j4 -b $out_sProject -o "${out_Project}\build_logs\s_log.log"
$out_sfile="$out_Project\Example\Sherloc_FreeRTOS_MPU_S_NS\Sherloc_s\Objects\Sherloc_s.axf"
###

### ns project compile
cd "$out_Project\host_tools\evaluation"
python3 eval_run.py
cd $current_dir
###

$out_nsfile="$out_Project\Example\out\eval\O3\elf_ns\FreeRTOS_MPU_ns_1.axf"

$out_bs= "$out_Project\Example\out\eval\O3\metadata\FreeRTOS_MPU_ns_1.bin"

$destsfile = "E:\SOFTWARE\s.axf"
$destnsfile = "E:\SOFTWARE\ns.axf"
$destbsfile="E:\SOFTWARE\bs.bin"

arm-none-eabi-objdump.exe -D $out_nsfile| Out-File -FilePath "ns_output.txt"
arm-none-eabi-objdump.exe -D $out_sfile| Out-File -FilePath "s_output.txt"

# Copy the file from the source path to the destination path

Copy-Item $out_sfile $destsfile
Copy-Item $out_nsfile $destnsfile 
Copy-Item $out_bs $destbsfile 

