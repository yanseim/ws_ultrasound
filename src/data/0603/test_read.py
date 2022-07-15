import numpy as np
import matplotlib.pyplot as plt
import os
import struct

if __name__=="__main__":
    pre_path = "/home/yan/yxj/ws_ultrasound/src/data/0714/traj/"

    # reading log_error
    filename = pre_path + 'log_error.bin'
    log_error = []
    f = open(filename, "rb")
    with open(filename, mode="rb") as f:
        # # 移至指定字节位置
        # f.seek(8)
        a = f.read(4)
        length = struct.unpack("i", a)
        a = f.read(4)
        total_size = struct.unpack("i", a)
        print(length)
        print(total_size)
        # 读入 16 个字节
        for i in range(length[0]):
            a = f.read(48)
            # # 打印 a 类型 bytes
            # print(type(a))
            # # 打印 a 内字节数目
            # print(len(a))
            # # 打印 a 内数据，以 16 进制数显示
            # print(a)
            # 16 个字节解析为 4 个 unsigned short 数据和 2 个 unsigned int 数据，字节排序为小端，返回元组
            val_tuple = struct.unpack("dddddd", a) # 如果解析 1 个数据，则应当读取与数据存储空间大小一致的字节数目，unpack 仍返回元组
            # 将元组转为 list
            log_error.append(list(val_tuple))

    # reading log_F_ext
    filename = pre_path + 'log_F_ext.bin'
    log_F_ext = []
    f = open(filename, "rb")
    with open(filename, mode="rb") as f:
        a = f.read(4)
        length = struct.unpack("i", a)
        a = f.read(4)
        total_size = struct.unpack("i", a)
        print(length)
        print(total_size)
        for i in range(length[0]):
            a = f.read(48)
            val_tuple = struct.unpack("dddddd", a)
            log_F_ext.append(list(val_tuple))

    # reading log_F_ext_filtered
    filename = pre_path + 'log_F_ext_filtered.bin'
    log_F_ext_filtered = []
    f = open(filename, "rb")
    with open(filename, mode="rb") as f:
        a = f.read(4)
        length = struct.unpack("i", a)
        a = f.read(4)
        total_size = struct.unpack("i", a)
        print(length)
        print(total_size)
        for i in range(length[0]):
            a = f.read(48)
            val_tuple = struct.unpack("dddddd", a)
            log_F_ext_filtered.append(list(val_tuple))

    # reading log_tau
    filename = pre_path + 'log_tau.bin'
    log_tau = []
    f = open(filename, "rb")
    with open(filename, mode="rb") as f:
        a = f.read(4)
        length = struct.unpack("i", a)
        a = f.read(4)
        total_size = struct.unpack("i", a)
        print(length)
        print(total_size)
        for i in range(length[0]):
            a = f.read(56)
            val_tuple = struct.unpack("ddddddd", a)
            log_tau.append(list(val_tuple))

    # reading log_t
    filename = pre_path + 'log_t.bin'
    log_t = []
    f = open(filename, "rb")
    with open(filename, mode="rb") as f:
        a = f.read(4)
        length = struct.unpack("i", a)
        a = f.read(4)
        total_size = struct.unpack("i", a)
        print(length)
        print(total_size)
        for i in range(length[0]):
            a = f.read(8)
            val_tuple = struct.unpack("d", a)
            log_t.append(val_tuple[0])

    plt.figure()
    plt.plot(log_t,log_error)
    plt.legend(['x','y','z','ax','ay','az'])
    plt.title("log_error")
    plt.savefig(pre_path+"log_error")

    plt.figure()
    plt.plot(log_t,log_F_ext)
    plt.legend(['x','y','z','ax','ay','az'])
    plt.title("log_F_ext")
    plt.savefig(pre_path+"log_F_ext")

    plt.figure()
    plt.plot(log_t,log_F_ext_filtered)
    plt.legend(['x','y','z','ax','ay','az'])
    plt.title("log_F_ext_filtered")
    plt.savefig(pre_path+"log_F_ext_filtered")

    plt.figure()
    plt.plot(log_t,log_tau)
    plt.legend(['1','2','3','4','5','6','7'])
    plt.title("log_tau")
    plt.savefig(pre_path+"log_tau")

    plt.show()

