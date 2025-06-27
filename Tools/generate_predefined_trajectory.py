#!/usr/bin/env python3
"""
Generate CAN2 trajectory messages based on predefined joint positions
Based on the original predefined_joints array from Rover.cpp
"""

import struct
import time
import can

# Original predefined_joints array from Rover.cpp
predefined_joints = [
    [-45.0, -45.0, -45.0, -45.0, -45.0, -45.0],  # Point 1
    [90.0, 90.0, 90.0, 90.0, 90.0, 90.0],        # Point 2
    [135.0, 135.0, 135.0, 135.0, 135.0, 135.0],  # Point 3
    [45.0, 45.0, 45.0, 45.0, 45.0, 45.0],        # Point 4
    [180.0, 180.0, 180.0, 180.0, 180.0, 180.0],  # Point 5
    [135.0, 135.0, 135.0, 135.0, 135.0, 135.0],  # Point 6
    [-90.0, -90.0, -90.0, -90.0, -90.0, -90.0],  # Point 7
    [45.0, 45.0, 45.0, 45.0, 45.0, 45.0],        # Point 8
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]              # Point 9 (final)
]

# CAN2 configuration
CAN2_TRAJECTORY_CMD_ID = 0x100
CAN2_INTERFACE = 'can2'  # or 'vcan2' for virtual CAN

def float_to_bytes(value):
    """Convert float to 4 bytes (little endian)"""
    return struct.pack('<f', value)

def create_trajectory_message(joint_positions, is_final_point=False):
    """
    Create CAN2 trajectory message
    
    Args:
        joint_positions: List of 6 joint angles (degrees)
        is_final_point: Whether this is the final trajectory point
    
    Returns:
        CAN message data (8 bytes)
    """
    # Pack 6 joint positions (4 bytes each = 24 bytes total)
    # But CAN message is only 8 bytes, so we need to send multiple messages
    # For now, we'll pack 2 joints per message (8 bytes = 2 * 4 bytes)
    
    data = bytearray()
    
    # Pack first 2 joint positions (8 bytes)
    for i in range(2):
        data.extend(float_to_bytes(joint_positions[i]))
    
    # Add control flags (2 bytes)
    control_flags = 0x0000
    if is_final_point:
        control_flags |= 0x0001  # Set final point flag
    
    data.extend(struct.pack('<H', control_flags))
    
    return data

def send_trajectory_sequence():
    """Send the complete trajectory sequence"""
    try:
        # Initialize CAN interface
        bus = can.interface.Bus(channel=CAN2_INTERFACE, bustype='socketcan')
        print(f"Connected to {CAN2_INTERFACE}")
        
        print("Sending predefined trajectory sequence...")
        print("=" * 60)
        
        for i, joint_pos in enumerate(predefined_joints):
            is_final = (i == len(predefined_joints) - 1)  # Last point is final
            
            print(f"Point {i+1}: {joint_pos}")
            print(f"  Final point: {is_final}")
            
            # Create and send message
            data = create_trajectory_message(joint_pos, is_final)
            
            msg = can.Message(
                arbitration_id=CAN2_TRAJECTORY_CMD_ID,
                data=data,
                is_extended_id=False
            )
            
            bus.send(msg)
            print(f"  Sent CAN2 message: ID=0x{CAN2_TRAJECTORY_CMD_ID:03X}, Data={data.hex()}")
            
            # Wait between points (adjust timing as needed)
            time.sleep(0.1)  # 100ms between points
            
            print()
        
        print("Trajectory sequence completed!")
        
    except can.CanError as e:
        print(f"CAN error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'bus' in locals():
            bus.shutdown()

def print_message_format():
    """Print the CAN2 message format for reference"""
    print("CAN2 Trajectory Message Format:")
    print("=" * 40)
    print("Message ID: 0x100")
    print("Data Length: 8 bytes")
    print("Data Structure:")
    print("  Bytes 0-3:   Joint 1 position (float, degrees)")
    print("  Bytes 4-7:   Joint 2 position (float, degrees)")
    print("  Bytes 8-9:   Control flags (uint16)")
    print("    Bit 0:     Final point flag")
    print("    Bits 1-15: Reserved")
    print()

def print_predefined_sequence():
    """Print the predefined trajectory sequence"""
    print("Predefined Trajectory Sequence:")
    print("=" * 40)
    for i, joint_pos in enumerate(predefined_joints):
        is_final = (i == len(predefined_joints) - 1)
        print(f"Point {i+1:2d}: [{joint_pos[0]:6.1f}, {joint_pos[1]:6.1f}, {joint_pos[2]:6.1f}, "
              f"{joint_pos[3]:6.1f}, {joint_pos[4]:6.1f}, {joint_pos[5]:6.1f}] "
              f"{'[FINAL]' if is_final else ''}")
    print()

if __name__ == "__main__":
    print("CAN2 Trajectory Message Generator")
    print("=" * 40)
    print()
    
    print_message_format()
    print_predefined_sequence()
    
    # Ask user if they want to send the sequence
    response = input("Do you want to send this trajectory sequence? (y/n): ")
    if response.lower() in ['y', 'yes']:
        send_trajectory_sequence()
    else:
        print("Sequence not sent. Exiting.") 