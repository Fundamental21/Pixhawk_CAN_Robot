#!/usr/bin/env python3
"""
CAN2报文编码-解码器
完全按照process_can2_trajectory_command的逻辑处理CAN2报文

✅ 报文格式 (process_can2_trajectory_command):
  data[0] = point_id      - 位置点编号 (固定为0x00)
  data[1] = motor_id      - 电机编号 (0x00-0x05)
  data[2] = pos_byte0     - 位置数据字节0 (LSB)
  data[3] = pos_byte1     - 位置数据字节1
  data[4] = pos_byte2     - 位置数据字节2
  data[5] = pos_byte3     - 位置数据字节3 (MSB)
  data[6] = unused        - 未使用字节
  data[7] = end_flag      - 结束标志 (最后一帧为0x01，其他为0x00)

🎯 本脚本实现完整的编码→解码验证流程！
"""

import struct
import sys
from typing import List, Optional, Tuple

class CAN2Encoder:
    """
    CAN2编码器 - 按照process_can2_trajectory_command的逻辑将电机位置编码为CAN报文
    """
    
    @staticmethod
    def encode_motor_positions(motor_positions: List[float]) -> List[str]:
        """
        将6个电机位置编码为CAN报文
        
        Args:
            motor_positions: 6个电机的位置（度）
            
        Returns:
            List[str]: 6个CAN帧的十六进制字符串
        """
        if len(motor_positions) != 6:
            raise ValueError("Must provide exactly 6 motor positions")
            
        frames = []
        
        for i in range(6):
            # Convert float position to 4-byte representation (小端序)
            position_union = struct.pack('<f', motor_positions[i])  # 小端序打包
            
            # 构造CAN帧 (按照process_can2_trajectory_command期待的格式)
            frame_data = bytearray(8)
            frame_data[0] = 0x00                        # Point ID (位置点编号，固定为0)
            frame_data[1] = i                           # Motor ID (电机编号 0-5)
            frame_data[2] = position_union[0]           # Position byte 0 (LSB)
            frame_data[3] = position_union[1]           # Position byte 1
            frame_data[4] = position_union[2]           # Position byte 2
            frame_data[5] = position_union[3]           # Position byte 3 (MSB)
            frame_data[6] = 0x00                        # 未使用字节
            frame_data[7] = 0x01 if i == 5 else 0x00   # 最后一帧标志 (电机5是最后一帧)
            
            # 转换为十六进制字符串
            hex_data = frame_data.hex().upper()
            frame_str = f"200#{hex_data}"
            frames.append(frame_str)
            
        return frames

class CAN2Decoder:
    def __init__(self):
        # 6个电机的位置数据存储
        self.motor_positions = [0.0] * 6
        self.motor_data_valid = [False] * 6
        self.frame_count = 0
        self.complete_cycles = 0
        
    def decode_can_frame(self, can_id: int, data: bytes) -> Optional[Tuple[int, float]]:
        """
        解码单个CAN帧
        
        Args:
            can_id: CAN ID (应该是0x200)
            data: CAN数据字节 (8字节)
            
        Returns:
            Tuple[motor_id, position] 或 None (如果解码失败)
        """
        # 检查CAN ID
        if can_id != 0x200:
            print(f"Warning: Unexpected CAN ID 0x{can_id:03X}, expected 0x200")
            return None
            
        # 检查数据长度
        if len(data) != 8:
            print(f"Error: CAN frame length {len(data)}, expected 8 bytes")
            return None
            
        # 按照process_can2_trajectory_command的解析逻辑
        point_id = data[0]  # 位置点编号
        motor_id = data[1]  # 电机编号 (0-5)
        
        # 检查电机ID是否有效
        if motor_id > 5:
            print(f"Error: Invalid motor ID {motor_id}, expected 0-5")
            return None
            
        # 提取位置数据 (按照process_can2_trajectory_command的格式)
        try:
            # 根据process_can2_trajectory_command的解析逻辑：
            # position_union.bytes[0] = msg.data[2]; // Position byte 0 (LSB)
            # position_union.bytes[1] = msg.data[3]; // Position byte 1
            # position_union.bytes[2] = msg.data[4]; // Position byte 2
            # position_union.bytes[3] = msg.data[5]; // Position byte 3 (MSB)
            position_bytes = data[2:6]  # 提取字节2-5
            position = struct.unpack('<f', position_bytes)[0]  # 小端序解码float
        except struct.error as e:
            print(f"Error: Failed to decode position data: {e}")
            return None
            
        # 检查是否为最终帧 (第8字节是01表示最后一个点的最后一帧)
        is_final_frame = (data[7] == 0x01)
        
        # 验证最后一帧标志 (电机5应该是0x01，其他是0x00)
        expected_last_byte = 0x01 if motor_id == 5 else 0x00
        if data[7] != expected_last_byte:
            print(f"Warning: Motor {motor_id} final frame flag 0x{data[7]:02X}, expected 0x{expected_last_byte:02X}")
            
        # 记录接收到的数据
        self.motor_positions[motor_id] = position
        self.motor_data_valid[motor_id] = True
        self.frame_count += 1
        
        # 检查是否收到完整的一轮数据 (当收到最后一帧时)
        if is_final_frame and motor_id == 5:
            if all(self.motor_data_valid):
                self.complete_cycles += 1
                print(f"\n=== Complete cycle #{self.complete_cycles} received ===")
                self._print_all_positions()
                # 重置标志为下一轮做准备
                self.motor_data_valid = [False] * 6
            else:
                missing_motors = [i for i in range(6) if not self.motor_data_valid[i]]
                print(f"Warning: Incomplete cycle, missing motors: {missing_motors}")
                
        return (motor_id, position)
        
    def _print_all_positions(self):
        """打印所有电机的位置信息"""
        print("Motor positions:")
        for i in range(6):
            print(f"  Motor {i+1}: {self.motor_positions[i]:8.3f}°")
            
    def decode_hex_string(self, hex_string: str) -> Optional[Tuple[int, float]]:
        """
        从十六进制字符串解码CAN帧
        
        Args:
            hex_string: 格式如 "200#00666634420000XX" (CAN_ID#8字节数据)
        """
        try:
            # 解析CAN ID和数据
            if '#' not in hex_string:
                print(f"Error: Invalid format, expected 'CAN_ID#DATA'")
                return None
                
            can_id_str, data_str = hex_string.strip().split('#', 1)
            can_id = int(can_id_str, 16)
            
            # 将十六进制字符串转换为字节
            if len(data_str) != 16:  # 8字节 = 16个十六进制字符
                print(f"Error: Data length {len(data_str)}, expected 16 hex characters")
                return None
                
            data = bytes.fromhex(data_str)
            return self.decode_can_frame(can_id, data)
            
        except ValueError as e:
            print(f"Error: Failed to parse hex string '{hex_string}': {e}")
            return None
            
    def print_statistics(self):
        """打印统计信息"""
        print(f"\n=== Statistics ===")
        print(f"Total frames received: {self.frame_count}")
        print(f"Complete cycles: {self.complete_cycles}")
        print(f"Current motor data valid: {self.motor_data_valid}")

def verify_encode_decode_flow(input_positions: List[float]):
    """
    完整验证编码-解码流程
    
    Args:
        input_positions: 6个电机的输入位置（度）
    """
    print("=" * 80)
    print("🔍 完整编码-解码验证流程")
    print("=" * 80)
    
    # 步骤1：显示输入
    print("📥 输入的6个电机位置:")
    for i, pos in enumerate(input_positions):
        print(f"   Motor {i+1}: {pos:8.3f}°")
    print()
    
    # 步骤2：编码为CAN报文
    print("🔧 按照robot_can2_tx_loop逻辑编码为CAN报文:")
    try:
        encoder = CAN2Encoder()
        encoded_frames = encoder.encode_motor_positions(input_positions)
        
        for i, frame in enumerate(encoded_frames):
            print(f"   Frame {i+1}: {frame}")
        print()
        
    except Exception as e:
        print(f"❌ 编码失败: {e}")
        return
    
    # 步骤3：解码CAN报文
    print("🔍 按照接收端逻辑解码CAN报文:")
    decoder = CAN2Decoder()
    decoded_positions = [0.0] * 6
    decode_success = True
    
    for frame in encoded_frames:
        result = decoder.decode_hex_string(frame)
        if result:
            motor_id, position = result
            decoded_positions[motor_id] = position
            print(f"   {frame} -> Motor {motor_id+1}: {position:.6f}°")
        else:
            print(f"   ❌ 解码失败: {frame}")
            decode_success = False
    print()
    
    # 步骤4：对比结果
    print("📊 输入与输出对比:")
    print("   Motor  |    输入     |     输出    |   误差")
    print("   -------|-------------|-------------|----------")
    
    max_error = 0.0
    for i in range(6):
        error = abs(input_positions[i] - decoded_positions[i])
        max_error = max(max_error, error)
        status = "✅" if error < 0.001 else "⚠️"
        print(f"   {i+1:2d}     | {input_positions[i]:10.6f}° | {decoded_positions[i]:10.6f}° | {error:8.6f}° {status}")
    
    print()
    print(f"📈 最大误差: {max_error:.6f}°")
    
    if max_error < 0.001:
        print("🎉 验证通过！编码-解码流程完全正确")
    elif max_error < 0.01:
        print("✅ 验证基本通过，存在轻微浮点精度误差")
    else:
        print("❌ 验证失败，存在显著误差")
    
    print("=" * 80)

def input_motor_positions() -> List[float]:
    """
    交互式输入6个电机的位置
    
    Returns:
        List[float]: 6个电机的位置（度）
    """
    print("📥 请输入6个电机的位置（度）:")
    positions = []
    
    for i in range(6):
        while True:
            try:
                pos_str = input(f"   Motor {i+1} 位置 (度): ").strip()
                position = float(pos_str)
                positions.append(position)
                break
            except ValueError:
                print("   ❌ 请输入有效的数字！")
            except KeyboardInterrupt:
                print("\n❌ 用户取消输入")
                return []
                
    return positions

def interactive_mode():
    """交互模式，手动输入CAN帧进行解码"""
    decoder = CAN2Decoder()
    
    print("=== CAN2 Interactive Decoder ===")
    print("Enter CAN frames in format: CAN_ID#DATA (e.g., 200#00666634420000000)")
    print("Enter 'quit' to exit, 'stats' to show statistics")
    print()
    
    while True:
        try:
            user_input = input("CAN Frame: ").strip()
            
            if user_input.lower() in ['quit', 'exit', 'q']:
                break
            elif user_input.lower() == 'stats':
                decoder.print_statistics()
                continue
            elif user_input == '':
                continue
                
            result = decoder.decode_hex_string(user_input)
            if result:
                motor_id, position = result
                print(f"  -> Motor {motor_id+1}: {position:.6f}°")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            
    decoder.print_statistics()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "input":
            # 交互输入模式 - 用户输入角度，动态生成和解析报文
            print("🎯 交互输入模式 - 根据用户输入的角度动态生成和解析CAN报文")
            positions = input_motor_positions()
            if positions:
                verify_encode_decode_flow(positions)
            else:
                print("❌ 未获取到有效的电机位置数据")
                
        elif sys.argv[1] == "interactive":
            interactive_mode()
            
        elif sys.argv[1] == "verify":
            # 验证模式：测试几组不同的电机位置
            test_cases = [
                [45.1, 0.0, 0.1, 10.0, -10.0, 90.0],     # 基本测试
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],          # 全零测试
                [180.0, -180.0, 360.0, -360.0, 45.5, -45.5],  # 边界测试
                [1.234567, -1.234567, 123.456789, -123.456789, 0.000001, -0.000001]  # 精度测试
            ]
            
            for i, positions in enumerate(test_cases):
                print(f"\n🧪 测试案例 {i+1}:")
                verify_encode_decode_flow(positions)
                print()
        else:
            print("Usage: python3 can2_decoder.py [input|interactive|verify]")
            print("  input     - 交互输入电机角度，动态生成和解析报文")
            print("  interactive - 交互模式，手动输入CAN帧")
            print("  verify    - 验证预设测试案例的编码-解码流程")
    else:
        # 默认运行交互输入模式
        print("🔥 默认模式：交互输入电机角度")
        print("=" * 50)
        positions = input_motor_positions()
        if positions:
            print()
            verify_encode_decode_flow(positions)
        else:
            print("❌ 未获取到有效的电机位置数据")
            
        print("\n" + "="*50)
        print("使用说明:")
        print("  python3 can2_decoder.py input     - 交互输入电机角度")
        print("  python3 can2_decoder.py interactive - 手动输入CAN帧解码")  
        print("  python3 can2_decoder.py verify    - 验证预设测试案例") 