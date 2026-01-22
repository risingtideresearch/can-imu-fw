import can
import numpy as np
import struct


def main():
    filters = [
        {"can_id": 0x200, "can_mask": 0x7FF, "extended": False},
        {"can_id": 0x201, "can_mask": 0x7FF, "extended": False},
    ]
    bus = can.Bus(interface="socketcan", channel="can0", can_filters=filters)

    msgs = [[], []]

    N_SAMPLES = 100

    while True:
        msg = bus.recv()
        msgs[msg.arbitration_id - 0x200].append(msg.data)

        if all([len(f) > N_SAMPLES for f in msgs]):
            break
        
    def average(msgs):
        sums = np.zeros(3)
        for m in msgs:
            values = struct.unpack("<hhh", m[0:6])
            sums += values  
        return sums / len(msgs)

    gyros = np.array(average(msgs[0]))
    accels = np.array(average(msgs[1]))

    print(f"Gyro: {gyros}")
    print(f"Accel: {accels}")
    
    bus.shutdown()


if __name__ == "__main__":
    main()