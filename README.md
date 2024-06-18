
# InsectACIDE: Debugger-Based Holistic Asynchronous CFI for Embedded System

  

InsectACIDE is the first kernel-level CFI for embedded and real-time systems that does not require binary instrumentation and is real-time friendly.

  

The results of this project were published in the paper entitled "[InsectACIDE: Debugger-Based Holistic Asynchronous CFI for Embedded System]" in the IEEE Real-Time and Embedded Technology and Applications Symposium (RTAS) 2024. If you want to cite our paper in your work, please use the following BibTeX entry.

```

@inproceedings{wang2024insectacide,

title = {InsectACIDE: Debugger-Based Holistic Asynchronous CFI for Embedded System},

author = {Wang, Yujie and Mack, Cailani Lemieux and Tan, Xi and Zhang, Ning and Zhao, Ziming and Baruah, Sanjoy and Ward, Bryan C.},

booktitle = {IEEE Real-Time and Embedded Technology and Applications Symposium (RTAS)},

year = {2024},

}

```

---

  

## Repository Structure

  

This repository is organized into three main directories:

  

1. *code\Sherloc-Cortex-M-CFVD\Sherloc_runtime*: InsectACIDE implementation based on [Sherloc](https://cactilab.github.io/assets/pdf/sherloc2023.pdf).

2.  *code\Sherloc-Cortex-M-CFVD\Example\Sherloc_FreeRTOS_MPU_S_NS\FreeRTOS_MPU_ns*:  example protection target.

3. *code\Sherloc-Cortex-M-CFVD\host_tool*: tools for static analysis for InsectACIDE.

  

### Prerequisites

  

- Environment prerequisites

- Hardware: [MPS2+ board](https://developer.arm.com/Tools%20and%20Software/MPS2%20Plus%20FPGA%20Prototyping%20Board)

- System: Windows. We have tested on Windows 11. Windows 10 may also work.

- Software: licensed Keil [uVision5](https://www.keil.com/demo/eval/arm.htm), python3, Jupyter notebook

- Required Python libraries: please refer to `code\Sherloc-Cortex-M-CFVD\host_tools\evaluation\requirements.txt`

- Knowledge prerequisites

- C and Python programming languages.

- Keil uVision5 IDE.

- Cortex-M33 architecture.

  

### The InsectACIDE_runtime Directory

  

This folder contains the InsectACIDE implementation and is organized into the following sub-folders:

  

-  _src_: source code for InsectACIDE.

-  _inc_: header files for InsectACIDE.

-  _src_: source files for InsectACIDE.

-  _freertos_: FreeRTOS kernel code.
  

## Getting Started

  

### Step 1: Environment setup

  

- Configure the board loading files.

- Refer to [using the Cortex-M33 IoT Kit Image on MPS2+](https://www.keil.com/appnotes/files/apnt_300.pdf).

- Find the file system of this board, usually the drive name is `V2M_MPS2`.

- Using `AN505` FPGA Image.

- Set the environment variables ( e.g., compiler toolchain path) in `scripts\auto_run.ps1`.

### Step 2: Compile the Protected Target and InsectACIDE

- execute the script `scripts\auto_run.ps1`. This script will compile and copy the compiled images to the board `E:\SOFTWARE`.

### (Optional) Using Your Own Task

- Place your task in `task1.cpp`, and your task name into the file `code\Sherloc-Cortex-M-CFVD\host_tools\evaluation\rtos\task.json`.


## License

  

InsectACIDE is released under the Apache License.