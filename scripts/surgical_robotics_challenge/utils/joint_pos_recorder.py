# this script will save the joint position to a file
# from queue import Queue
from __future__ import division
from __future__ import print_function
from datetime import datetime
# from datetime import timezone
import json
import os
import warnings
import gc
from glob import glob
from pprint import pprint
# import PyKDL
# from PyKDL import Frame
# import numpy as np
import time


def compare_name(name_str):
    datestr = os.path.basename(name_str).replace(".jplist", "").split("#")[1]
    datetime_ins = datetime.strptime(datestr, '%Y-%m-%d %H:%M:%S.%f')
    return datetime_ins


class JointPosRecorder():
    ##### This recorder only work for list
    def __init__(self, save_path='.', record_size=500):
        #### can self-define the save_path
        if not os.path.exists(save_path):
            print('[*]creating a new path at ' + save_path)
            os.makedirs(save_path)
        else:
            print('[*]using existing path at ' + save_path)
        self.__save_path = save_path
        self.__record_size = record_size
        self.__record_queue = []
        self.__success_save = 0
        self.__total_save = 0
        self.tm_format = '%Y-%m-%d %H:%M:%S.%f'

    def record(self, joint_pos):
        # joint_pos is a list
        assert type(joint_pos) == list, "The join_pos should be a list"
        assert len(joint_pos) == 6, "The length of joint_pos should be 6"

        dt = datetime.now()
        # utc_time = dt.replace()
        # utc_timestamp = utc_time.timestamp() # float
        queue_item = {'time': str(dt), 'pos': joint_pos}
        self.__record_queue.append(queue_item)
        if self.__is_full():
            # do the flush operation
            self.__flush()
            self.__record_queue = []
            gc.collect()  # clean RAM

    def __generate_file_name(self):
        this_time = datetime.now().strftime(self.tm_format)
        fname = 'JP#' + this_time + '.jplist'
        return fname

    def __is_full(self):
        return len(self.__record_queue) >= self.__record_size

    def __flush(self):
        fname = self.__generate_file_name()
        path = os.path.join(self.__save_path, fname)
        self.__total_save += 1
        try:
            with open(path, 'w') as f:
                json.dump(self.__record_queue, f)
                self.__success_save += 1
                print("[*]record file generated at " + path)
        except Exception as e:
            warnings.warn("[!]recording failed. \n" + str(e))
            # self.add_to_recover_list(self.record_queue)
            return False
        return True

    def flush(self):
        # deal with all the rest part
        print('flush remaining ' + str(len(self.__record_queue)) + ' records')
        self.__flush()

    def get_success_rate(self):
        return self.__success_save / self.__total_save
    # def add_to_recover_list(self, item):
    #     self.recover_list.append(item)


class JointPosLoader():
    @staticmethod
    def load(filepath):
        with open(filepath) as f:
            s = json.load(f)
        return s

    @staticmethod
    def load_range(folder_path='.', datetime_str=None, suffix_record='.*jplist'):
        # datetime_str should be like 2021-05-06
        # if datetime is None: the default is today's date
        if datetime_str is None:
            datetime_str = str(datetime.now().strftime(
                '%Y-%m-%d %H:%M:%S.%f')).split(' ')[0]
        datetime_str = datetime_str.strip()
        jp_list = glob(os.path.join(folder_path, suffix_record))
        jp_list.sort(key=compare_name)
        jps = [json.load(open(p)) for p in jp_list]
        jp_times = jp_list
        return jps, jp_times

    @staticmethod
    def load_by_prefix(prefix, folder_path='.'):
        jp_list = glob(os.path.join(folder_path, prefix + '*'))
        jp_list.sort(key=compare_name)
        jps = [json.load(open(p)) for p in jp_list]
        jp_times = jp_list
        return jps, jp_times


if __name__ == '__main__':
    # choice = int(input('which operation to test?\n1)generate\n2)load single file\n3)load by date\n4)load by prefix\n5)generate stream\n\nInput=').strip())
    # if choice == 1:
    #     import random
    #     recorder = JointPosRecorder()
    #     for i in range(450):
    #         sample_pos = list(random.sample(range(0, 30), 6))
    #         recorder.record(sample_pos)
    #     recorder.flush() # so the remaining part will also be saved
    # elif choice == 2:
    #     m,l = JointPosLoader.load('JP#2021-05-06 21:23:22.855854.jplist')
    #     pprint(m)
    # elif choice == 3:
    #     m,l = JointPosLoader.load_range(datetime_str='2021-05-06')
    #     pprint(m)
    # elif choice == 4:
    m, l = JointPosLoader.load_by_prefix('JP#2021-05-11 01')
    pprint(m)
    # elif choice == 5:
    #     import random
    #     recorder = JointPosRecorder()
    #     try:
    #         while True:
    #             for i in range(478):
    #                 sample_pos = list(random.sample(range(0, 30), 6))
    #                 recorder.record(sample_pos)
    #             time.sleep(1)
    #     except KeyboardInterrupt:
    #         recorder.flush()
