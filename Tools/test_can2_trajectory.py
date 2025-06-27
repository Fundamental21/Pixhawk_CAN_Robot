#!/usr/bin/env python3
"""
CAN2轨迹数据发送测试工具

这个脚本用于测试CAN2轨迹数据接收功能，通过发送不同位置的轨迹数据来验证系统响应。

使用方法:
    python3 test_can2_trajectory.py [CAN接口] [发送间隔(秒)]

示例:
    python3 test_can2_trajectory.py can0 0.5
"""

import sys
import time
import struct
import socket
import argparse
from typing import List, Tuple

class CAN2TrajectorySender:
    """CAN2轨迹数据发送器"""
    
    def __init__(self, can_interface: str = "can0"):
        self.can_interface = can_interface
        self.socket = None
        
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
        """
        发送轨迹数据
        
        Args:
            joint_positions: 6个关节位置 (度)
            is_final: 是否为最终停止点
        
        Returns:
            bool: 发送是否成功
        """
        if not self.socket:
            print("CAN接口未连接")
            return False
        
        if len(joint_positions) != 6:
            print("关节位置数量必须为6")
            return False
        
        try:
            # 构建CAN帧数据
            can_id = 0x100  # 轨迹命令ID
            data = bytearray()
            
            # 添加6个关节位置 (每个4字节，小端格式)
            for pos in joint_positions:
                data.extend(struct.pack('<f', pos))
            
            # 添加控制标志
            control_byte = 0x01 if is_final else 0x00
            data.append(control_byte)
            
            # 添加保留字节
            data.append(0x00)
            
            # 构建CAN帧
            can_frame = struct.pack('<I3x8s', can_id, data)
            
            # 发送CAN帧
            self.socket.send(can_frame)
            
            print(f"发送轨迹: [{', '.join([f'{pos:.1f}' for pos in joint_positions])}] Final:{is_final}")
            return True
            
        except Exception as e:
            print(f"发送轨迹数据失败: {e}")
            return False
    
    def send_test_sequence(self, interval: float = 1.0):
        """发送测试序列"""
        test_positions = [
            # 测试位置1: 所有关节0度
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], False),
            
            # 测试位置2: 所有关节45度
            ([45.0, 45.0, 45.0, 45.0, 45.0, 45.0], False),
            
            # 测试位置3: 所有关节90度
            ([90.0, 90.0, 90.0, 90.0, 90.0, 90.0], False),
            
            # 测试位置4: 不同角度
            ([30.0, 60.0, 90.0, 120.0, 150.0, 180.0], False),
            
            # 测试位置5: 回到0度，最终停止
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], True),
        ]
        
        print("开始发送测试序列...")
        for i, (positions, is_final) in enumerate(test_positions, 1):
            print(f"\n测试位置 {i}:")
            if self.send_trajectory(positions, is_final):
                time.sleep(interval)
            else:
                print("发送失败，停止测试")
                break
        
        print("\n测试序列完成")

def main():
    parser = argparse.ArgumentParser(description="CAN2轨迹数据发送测试工具")
    parser.add_argument("can_interface", nargs="?", default="can0", 
                       help="CAN接口名称 (默认: can0)")
    parser.add_argument("interval", nargs="?", type=float, default=1.0,
                       help="发送间隔时间(秒) (默认: 1.0)")
    
    args = parser.parse_args()
    
    # 创建发送器
    sender = CAN2TrajectorySender(args.can_interface)
    
    # 连接CAN接口
    if not sender.connect():
        sys.exit(1)
    
    try:
        # 发送测试序列
        sender.send_test_sequence(args.interval)
        
        # 交互式模式
        print("\n进入交互式模式 (输入 'quit' 退出):")
        while True:
            try:
                user_input = input("输入6个关节位置 (度，用空格分隔): ").strip()
                
                if user_input.lower() == 'quit':
                    break
                
                # 解析用户输入
                parts = user_input.split()
                if len(parts) != 6:
                    print("请输入6个数字，用空格分隔")
                    continue
                
                try:
                    positions = [float(x) for x in parts]
                    is_final = input("是否为最终停止点? (y/n): ").strip().lower() == 'y'
                    
                    sender.send_trajectory(positions, is_final)
                    
                except ValueError:
                    print("输入格式错误，请输入数字")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
                
    finally:
        sender.disconnect()
        print("\n已断开CAN连接")

if __name__ == "__main__":
    main() 