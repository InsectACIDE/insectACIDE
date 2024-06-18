
import os
import shutil

from eval_freertos import *






target_opt_levels = ['O3']
eval_elf_s = '../../host_tools/evaluation/elf_s/'
example_path = '../../Example/'
eval_working_path = example_path + 'out/eval/'
out_binary_s_path = eval_working_path + 'elf_s/'




class EvaluationRunnerForFreeRTOS(object):
    def __init__(self, opt_level):
        self.opt_level = opt_level
       
        self.TargetName = 'FreeRTOS_MPU_ns'
        self.in_FreeRTOS_path = example_path + 'Sherloc_FreeRTOS_MPU_S_NS/'
        self.in_FreeRTOS_path_ns = self.in_FreeRTOS_path + 'FreeRTOS_MPU_ns/'
        self.eval_FreeRTOS_src_path = './freertos/'
        self.task_number = ['1']

      

    def FreeRTOS_prepare_run(self):
        start = time.time()
        # print(f'-- build keil project with -{self.opt_level}')
        for task_number in self.task_number:
            # build the project to get the binary file
            
            
            UVPROJXBuilderForFreeRTOS(self.TargetName, task_number, self.in_FreeRTOS_path_ns,
                                      self.eval_FreeRTOS_src_path, out_uvprojx_path, out_binary_ns_path, out_tmp_path, self.opt_level).build_run()
            
            print(
                f'-- static analysis to create metadata with {self.opt_level}')
            # create metadata for all binaries
            MetadataCreatorForFreeRTOS(self.TargetName, task_number, out_binary_ns_path,
                                       out_metadata_path, out_log_path, self.opt_level).create_metadata_run()
        print(f'prepare for the evaluation costs {time.time() - start}')

   
    
    def FreeRTOS_eval_run_all(self):
        self.FreeRTOS_prepare_run()
        

if __name__ == '__main__':
    # clean output
    if os.path.exists(eval_working_path):
        pass
        
    else:
        # create folders for evaluation for output
        os.mkdir(eval_working_path)
    if not os.path.exists(out_binary_s_path):
        os.makedirs(out_binary_s_path, exist_ok=True)
    # Copy the source directory to the destination directory
    # print("copytree: ",eval_elf_s, os.path.join(out_binary_s_path))
    shutil.copytree(eval_elf_s, os.path.join(out_binary_s_path,
                    os.path.basename(eval_elf_s)), dirs_exist_ok=True)
    # shutil.copytree(eval_elf_s, out_binary_s_path)
    print('prepare for elf_s success')

    for opt_level in target_opt_levels:
        # parent directory for working path
        eval_working_subpath = eval_working_path + opt_level + '/'

        out_uvprojx_path = eval_working_subpath + 'proj/'
        out_binary_ns_path = eval_working_subpath + 'elf_ns/'
        out_metadata_path = eval_working_subpath + 'metadata/'
        out_tmp_path = eval_working_subpath + 'build_tmp/'
        out_log_path = eval_working_subpath + 'eval_log/'
        out_eval_log = out_log_path + 'eval.log'

        check_and_create_directory(eval_working_subpath)
        check_and_create_directory(out_metadata_path)
        check_and_create_directory(out_uvprojx_path)
        check_and_create_directory(out_binary_ns_path)
        check_and_create_directory(out_tmp_path)
        check_and_create_directory(out_log_path)
        check_and_create_file(out_eval_log)

        EvaluationRunnerForFreeRTOS(opt_level).FreeRTOS_eval_run_all()
        