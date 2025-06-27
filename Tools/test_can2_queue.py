#!/usr/bin/env python3
"""
CAN2轨迹数据队列测试工具

这个脚本用于测试CAN2轨迹数据队列机制，通过快速发送多个轨迹点来验证队列是否正常工作。

使用方法:
    python3 test_can2_queue.py [CAN接口] [发送频率(Hz)]

示例:
    python3 test_can2_queue.py can1 50  # 50Hz发送频率
"""

import sys
import time
import struct
import socket
import argparse
import threading
from typing import List

class CAN2QueueTester:
    """CAN2轨迹数据队列测试器"""
    
    def __init__(self, can_interface: str = "can0"):
        self.can_interface = can_interface
        self.socket = None
        self.running = False
        
    def connect(self) -> bool:
        """连接到CAN接口"""
        try:
            self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.socket.bind((self.can_interface,))
            print(f"已连接到CAN接口: {self.can_interface}")
            return True
        except Exception as e:
            print(f"连接CAN接口失败: {e}")
            return False
    
    def disconnect(self):
        """断开CAN连接"""
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def send_trajectory(self, joint_positions: List[float], is_final: bool = False) -> bool:
        """发送轨迹数据"""
        if not self.socket:
            return False
        
        try:
            # 构建CAN帧数据
            can_id = 0x100
            data = bytearray()
            
            # 添加6个关节位置 (小端格式)
            for pos in joint_positions:
                data.extend(struct.pack('<f', pos))
            
            # 添加控制标志
            control_byte = 0x01 if is_final else 0x00
            data.append(control_byte)
            data.append(0x00)  # 保留字节
            
            # 构建CAN帧
            can_frame = struct.pack('<I3x8s', can_id, data)
            
            # 发送CAN帧
            self.socket.send(can_frame)
            return True
            
        except Exception as e:
            print(f"发送失败: {e}")
            return False
    
    def rapid_send_test(self, frequency: float = 50.0, duration: float = 10.0):
        """快速发送测试"""
        print(f"开始快速发送测试: {frequency}Hz, 持续{duration}秒")
        
        self.running = True
        start_time = time.time()
        sent_count = 0
        
        # 生成测试轨迹序列
        test_sequences = [
            # 序列1: 快速变化的位置
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], False),
            ([10.0, 10.0, 10.0, 10.0, 10.0, 10.0], False),
            ([20.0, 20.0, 20.0, 20.0, 20.0, 20.0], False),
            ([30.0, 30.0, 30.0, 30.0, 30.0, 30.0], False),
            ([40.0, 40.0, 40.0, 40.0, 40.0, 40.0], False),
            ([50.0, 50.0, 50.0, 50.0, 50.0, 50.0], False),
            ([60.0, 60.0, 60.0, 60.0, 60.0, 60.0], False),
            ([70.0, 70.0, 70.0, 70.0, 70.0, 70.0], False),
            ([80.0, 80.0, 80.0, 80.0, 80.0, 80.0], False),
            ([90.0, 90.0, 90.0, 90.0, 90.0, 90.0], False),
        ]
        
        interval = 1.0 / frequency
        sequence_index = 0
        
        while self.running and (time.time() - start_time) < duration:
            # 获取当前轨迹点
            positions, is_final = test_sequences[sequence_index % len(test_sequences)]
            
            # 添加一些随机变化
            import random
            for i in range(6):
                positions[i] += random.uniform(-5.0, 5.0)
                positions[i] = max(-180.0, min(180.0, positions[i]))  # 限制范围
            
            # 发送轨迹数据
            if self.send_trajectory(positions, is_final):
                sent_count += 1
                if sent_count % 10 == 0:
                    print(f"已发送 {sent_count} 个轨迹点")
            
            # 更新序列索引
            sequence_index += 1
            
            # 等待下一个发送周期
            time.sleep(interval)
        
        self.running = False
        elapsed_time = time.time() - start_time
        actual_frequency = sent_count / elapsed_time
        
        print(f"\n快速发送测试完成:")
        print(f"  发送时间: {elapsed_time:.2f}秒")
        print(f"  发送数量: {sent_count}")
        print(f"  实际频率: {actual_frequency:.1f}Hz")
        print(f"  目标频率: {frequency:.1f}Hz")
    
    def burst_send_test(self, burst_size: int = 20, burst_interval: float = 2.0):
        """突发发送测试"""
        print(f"开始突发发送测试: 每次{burst_size}个点, 间隔{burst_interval}秒")
        
        burst_count = 0
        total_sent = 0
        
        while True:
            try:
                print(f"\n突发 {burst_count + 1}:")
                
                # 发送突发数据
                for i in range(burst_size):
                    # 生成递增的位置
                    positions = [float(i * 5) for _ in range(6)]
                    is_final = (i == burst_size - 1)  # 最后一个点为最终点
                    
                    if self.send_trajectory(positions, is_final):
                        total_sent += 1
                        print(f"  发送点 {i+1}: [{', '.join([f'{p:.1f}' for p in positions])}] Final:{is_final}")
                    else:
                        print(f"  发送点 {i+1} 失败")
                
                burst_count += 1
                print(f"  突发 {burst_count} 完成，总计发送: {total_sent}")
                
                # 等待下一次突发
                time.sleep(burst_interval)
                
            except KeyboardInterrupt:
                break
        
        print(f"\n突发发送测试完成，总计发送: {total_sent} 个轨迹点")

def main():
    parser = argparse.ArgumentParser(description="CAN2轨迹数据队列测试工具")
    parser.add_argument("can_interface", nargs="?", default="can0", 
                       help="CAN接口名称 (默认: can0)")
    parser.add_argument("test_type", nargs="?", choices=["rapid", "burst"], default="rapid",
                       help="测试类型: rapid(快速发送) 或 burst(突发发送)")
    parser.add_argument("param1", nargs="?", type=float, default=50.0,
                       help="参数1: rapid模式为频率(Hz), burst模式为突发大小")
    parser.add_argument("param2", nargs="?", type=float, default=10.0,
                       help="参数2: rapid模式为持续时间(秒), burst模式为间隔(秒)")
    
    args = parser.parse_args()
    
    # 创建测试器
    tester = CAN2QueueTester(args.can_interface)
    
    # 连接CAN接口
    if not tester.connect():
        sys.exit(1)
    
    try:
        if args.test_type == "rapid":
            # 快速发送测试
            tester.rapid_send_test(args.param1, args.param2)
        else:
            # 突发发送测试
            tester.burst_send_test(int(args.param1), args.param2)
            
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    finally:
        tester.disconnect()
        print("已断开CAN连接")

if __name__ == "__main__":
    main() 