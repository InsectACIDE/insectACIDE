#!/usr/bin/env python

import copy
import json

import angr
from capstone import *
from elftools.elf.elffile import ELFFile

import warnings

# from angrutils import *

class analyze_on_target():
    def __init__(self, filepath, metadata_path,context_level=1):
        
        ###comment temporalily
        self.context_level=context_level
        self.proj, self.cfg, self.code, self.edge_table =self.cs_load_analyze_cfg(filepath)
        ###

        # self.proj, self.cfg, self.code = self.load_cfg(filepath)
    
        # self.edge_table = self.analyze_cfg()
        print("origiinal edge_table:",self.edge_table)
        
    def is_inset(self,pair,pairs):
        for p in pairs:
            if str(pair)==str(p):
                return True
        return False
        
    def get_func_entry_node(self,init_node):
        predecessors = self.cfg.graph.predecessors(init_node)
        worklist = [(p,init_node) for p in predecessors]
        visited=[]
        callsite_edges=[]
        # node=init_node
        while worklist:
            predecessor,node = worklist.pop(0)
            visited.append((predecessor,node))
            edge_data = self.cfg.graph.get_edge_data(predecessor, node)
            # print("Edge nodes: ",str((predecessor,node)))
            # edges=list(self.cfg.graph.edges(predecessor, node))
            # print("Edges: ",edges, predecessor, node)
            predecessor_function_address = predecessor.function_address
            predecessor_function = self.proj.kb.functions[predecessor_function_address]
            node_function_address = node.function_address
            node_function = self.proj.kb.functions[node_function_address]
            # print(f"Function name: {function.name} -> {nodefunction.name}")

            

            if edge_data['jumpkind'] == 'Ijk_Call' or (edge_data['jumpkind'] == 'Ijk_Ret' and node_function.name != predecessor_function.name):
                if not self.is_inset((predecessor,node) ,callsite_edges):
                    callsite_edges.append((predecessor,node))
          

            elif edge_data['jumpkind'] == 'Ijk_FakeRet' or edge_data['jumpkind'] == 'Ijk_Boring':
                next_predecessors = self.cfg.graph.predecessors(predecessor)
                for n in next_predecessors:
                    if not self.is_inset((n,predecessor),visited):
                        worklist.append((n,predecessor))
 
        # print("results:", callsite_edges)
        callsite_addresses = []

        return callsite_edges

    def get_src_dst_address(self, src_node,dst_node):
        block = self.proj.factory.block(src_node.addr)
         
        insns = block.capstone.insns
        if insns:
            las_insns = insns[-1]
                       
            isrc = las_insns.address
            idst = dst_node.addr
           
            return (isrc-1, idst-1)
                    
        return None

    def cs_load_analyze_cfg(self, filename):

        proj = angr.Project(filename, main_opts={'arch': 'ArchARMCortexM'}, auto_load_libs=False)
        self.proj=proj
        print(proj.arch)
        print(hex(proj.entry))
        
        try:
            elf = ELFFile(open(filename, 'rb'))
        except IOError as e:
            print(e)
        code_section = elf.get_section_by_name('ER_ROM')
        if code_section:
            pass
        else:  # For FreeRTOS+MPU
            code_section = elf.get_section_by_name('ER_IROM_NS_PRIVILEGED')

        code = code_section.data()

        import sys
        import os
        import warnings
        import logging
        # warnings.filterwarnings("ignore")
        # sys.stdout = open(os.devnull, 'w')
        # sys.stderr = open(os.devnull, 'w')
        # warnings.simplefilter("ignore")
        logging.getLogger("pyvex").setLevel(logging.CRITICAL) 
        # static CFG
        self.cfg_fast = proj.analyses.CFGFast()
        # cfg=self.cfg_fast
        # func_addr = proj.loader.find_symbol("main").rebased_addr
        # start_state = proj.factory.blank_state()
        # cfg = proj.analyses.CFGEmulated(context_sensitivity_level=self.context_level, keep_state=True)#,starts=[func_addr],base_graph=self.cfg_fast.graph)
        # self.cfg=cfg
 
        # #obtain analysis entry 
        import json
        task_json = None
        try:
            f = open('./rtos/task.json', )
        except IOError as e:
            print(e)
        # returns JSON object as a dictionary
        try:
            task_json = json.load(f)
        except ValueError:
            print("load task json file error.")
        f.close()
        analysis_entry = task_json.keys()
        analyses_entry = list(analysis_entry)
        analyses_entry.append("main")
        analyses_entry.append("updateOutputs")
        # print("task_json:",task_json,analyses_entry)
        analyses_entry_addrs=[]
        for func_name in analyses_entry:
            if proj.loader.find_symbol(func_name) is not None:
                func_addr = proj.loader.find_symbol(func_name).rebased_addr
                start_state = proj.factory.blank_state(addr=func_addr)
                analyses_entry_addrs.append(func_addr)
        print("analyses_entry_addrs:", (analyses_entry_addrs))
        # while(True):
        #     pass
        # cfgs=[]
        # for func_name in analyses_entry:
            
        #     if proj.loader.find_symbol(func_name) is not None:
        #         # Function exists in proj
        #         func_addr = proj.loader.find_symbol(func_name).rebased_addr
        #         start_state = proj.factory.blank_state(addr=func_addr)
        #         # Create a CFGEmulated instance, starting from the current function's address
        #         cfg_emu = proj.analyses.CFGEmulated(fail_fast=False, starts=[func_addr], context_sensitivity_level=self.context_level, keep_state=True)
        #         cfgs.append(cfg_emu)
        #         # print("Function exists ",func_addr, func_name)
        #     else:
        #         pass
               

        # print(len(cfgs))
        # sys.exit(0)
        cfg_emu = proj.analyses.CFGEmulated(fail_fast=False, starts=analyses_entry_addrs, context_sensitivity_level=self.context_level, keep_state=True)
        cfg=cfg_emu
        self.cfg=cfg
        # sys.exit(0)       
        # self.cfg=cfgs[-1]
        # cfg=cfgs[-1]
        print("It has %d nodes and %d edges" % (len(cfg.graph.nodes()), len(cfg.graph.edges())))
        import networkx as nx
        dot_file_path = 'example_function.dot'
        nx.drawing.nx_pydot.write_dot(cfg.graph, dot_file_path)
        

       
        edge_table = {}
        conn = {}
        conn_pair = {}
        idx = 0
        branch_insns = ["blx", "bx"]

        analyzed_edges = []
        
        for edges in cfg.graph.edges():

            src_node = edges[0]
            dst_node = edges[1]
            block=src_node.block
            if block is None:
                continue
            # if src_node.addr not in proj.kb.cfg.model:
            #     continue
            # block = proj.factory.block(src_node.addr)
            # Rest of the code...
            insns = block.capstone.insns
            if insns:
                las_insns = insns[-1]
                mnemonic = las_insns.mnemonic
                op_str = las_insns.op_str
                isrc = las_insns.address
                ret=None
                if mnemonic == "blx":
                    ret = isrc + 2
                    conn_pair['type'] = "icall"
                elif mnemonic == "bl":
                    ret = isrc + 4
                if ret == dst_node.addr:
                    # print("ret")
                    continue
                if mnemonic in branch_insns:
                    if op_str == "lr":
                        # print("ret: bx lr")
                        continue
                    if not self.is_inset(edges,analyzed_edges):
                        analyzed_edges.append(edges)
                    else:
                        continue
                    # print("inst: ", las_insns)
                    print("edges: ",edges)
                    callsites=[[(src_node,dst_node)]]
                    
                    for i in range(self.context_level):
                        
                        to_add=[]
                        for j in range(len(callsites)):
                            call_seq = callsites[j]
                            
                            # print("call_seq:",call_seq)
                            n=call_seq[0][0]
                            callsite_edges=self.get_func_entry_node(n)    
                            if len(callsite_edges)==0:
                                to_add.append(call_seq)  
                            else:
                                for edge in callsite_edges:
                                    to_add.append([edge]+call_seq)
                        
                        callsites=to_add
                    print("callsites:",callsites)
                    for call_seq in callsites:
                        encode_src=0
                        encode_dst=dst_node.addr 
                        for i,j in call_seq:
                            i_addr,j_addr=self.get_src_dst_address(i,j)
                            encode_src ^= i_addr
                            # encode_dst ^= j_addr
                        idst = dst_node.addr
                        conn_pair['src_name'] = src_node.name
                        conn_pair['dst_name'] = dst_node.name
                        conn_pair['ins'] = mnemonic + ' ' + op_str
                        conn_pair['src_addr'] = copy.copy('0x%x' % (encode_src))
                        conn_pair['dst_addr'] = copy.copy('0x%x' % (encode_dst))
                        # print(f'src_node: [0x{isrc-1:x}] {src_node.name}, dst_node: [0x{idst-1:x}] {dst_node.name}')
                        conn[idx] = copy.copy(conn_pair)
                        conn_pair = {}
                        idx = idx + 1
        edge_table = conn
        print("edge_table:",edge_table)

        # Save edge_table as JSON
        with open("CFG_encode_table.json", 'w') as f:
            json.dump(edge_table, f)
        # sys.exit(0)
        # return edge_table        
        # print("This-is-the graph:", cfg.graph)
        # sys.exit(0)

        return proj, cfg, code, edge_table

    def load_cfg(self, filename):
        
        proj = angr.Project(filename, main_opts={'arch': 'ArchARMCortexM'})
        print(proj.arch)
        print(hex(proj.entry))
        
        try:
            elf = ELFFile(open(filename, 'rb'))
        except IOError as e:
            print(e)
        code_section = elf.get_section_by_name('ER_ROM')
        if code_section:
            pass
        else:  # For FreeRTOS+MPU
            code_section = elf.get_section_by_name('ER_IROM_NS_PRIVILEGED')

        code = code_section.data()

        import sys
        import os
        import warnings
        import logging
        # warnings.filterwarnings("ignore")
        # sys.stdout = open(os.devnull, 'w')
        # sys.stderr = open(os.devnull, 'w')
        # warnings.simplefilter("ignore")
        logging.getLogger("pyvex").setLevel(logging.CRITICAL) 
        # static CFG
        cfg = proj.analyses.CFGFast()

        # cfg = proj.analyses.CFGEmulated(keep_state=True)
        # logging.getLogger("pyvex").setLevel(logging.NOTSET) 

        return proj, cfg, code

    def analyze_cfg(self):
        # dynamic CFG
       
        print("This is the graph:", self.cfg.graph)
        print("It has %d nodes and %d edges" %
              (len(self.cfg.graph.nodes()), len(self.cfg.graph.edges())))

        print(type(self.proj.entry))

        entry_node = self.cfg.get_any_node(self.proj.entry)

        print("There were %d contexts for the entry block" %
              len(self.cfg.get_all_nodes(self.proj.entry)))
        print("Predecessors of the entry point:", entry_node.predecessors)
        print("Successors of the entry point:", entry_node.successors)

        print("Successors (and type of jump)  of the entry point:", [jumpkind + " to " + str(
            node.addr) for node, jumpkind in self.cfg.get_successors_and_jumpkind(entry_node)])

        conn = {}
        conn_pair = {}
        idx = 0

        edge_table = {}
        conn = {}
        conn_pair = {}
        idx = 0
        branch_insns = ["blx", "bx"]

        analyzed_node = []

        # current_node = "__main"
        for edges in self.cfg.graph.edges():
            src_node = edges[0]
            dst_node = edges[1]

            block = self.proj.factory.block(src_node.addr)

            # print("--------------------BB start----------------------")
            # print('src_node: [0x%x] %s (src_node_size: %x), dst_node: [0x%x] %s (dst_node_size: %x)' % (
            #     src_node.addr-1, src_node.name, src_node.size, dst_node.addr-1, dst_node.name, dst_node.size))

            insns = block.capstone.insns

            if insns:
                las_insns = insns[-1]
                mnemonic = las_insns.mnemonic
                op_str = las_insns.op_str
                isrc = las_insns.address
                if mnemonic == "blx":
                    ret = isrc + 2
                    conn_pair['type'] = "icall"
                elif mnemonic == "bl":
                    ret = isrc + 4
                if ret == dst_node.addr:
                    # print("ret")
                    continue
                if mnemonic in branch_insns:
                    if op_str == "lr":
                        # print("ret: bx lr")
                        continue
                    idst = dst_node.addr
                    conn_pair['src_name'] = src_node.name
                    conn_pair['dst_name'] = dst_node.name
                    conn_pair['ins'] = mnemonic + ' ' + op_str
                    conn_pair['src_addr'] = copy.copy('0x%x' % (isrc-1))
                    conn_pair['dst_addr'] = copy.copy('0x%x' % (idst-1))
                    conn[idx] = copy.copy(conn_pair)
                    conn_pair = {}
                    idx = idx + 1
                    # print(
                    #     f'indirect branch: {las_insns}, src: {hex(isrc-1)}, dst: {hex(idst-1)}')

            # print("==> insns_list")
            for i in insns:
                # insn: <CsInsn 0x200419 [01d1]: bne #0x20041f>, size: 2, address: 2098201, mnemonic: bne, op_str: #0x20041f
                continue
                print(
                    f'insn: {i.insn}, size: {i.size}, address: 0x{i.address:x}, mnemonic: {i.mnemonic}, op_str: {i.op_str}')

            if src_node in analyzed_node:
                pass
            else:
                analyzed_node.append(src_node)

        edge_table = conn
        return edge_table

    def write_json(self, metadata_path):
        # Save analyzed address pairs
        with open(metadata_path, 'w', encoding='utf-8') as outfile:
            json.dump(self.edge_table, outfile, ensure_ascii=False, indent=4)
