#!/usr/bin/env python
import subprocess
import time
import signal

def play_bag(bag_path, speed=2.0):
    # 通过subprocess启动 rosbag play 命令，以指定的速度因子播放
    process = subprocess.Popen(['rosbag', 'play', bag_path, '--loop', '-r', str(speed)], preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN))
    return process

def main():
    bag_path = '/home/hlc/RM_Radar2023/Record/bag/2024-04-12-15-35-38.bag'  # 替换为你的bag文件路径
    print("Starting to play the bag file at 2x speed in a loop...")

    try:
        # 开始以2倍速循环播放
        process = play_bag(bag_path, speed=3.0)
        
        # 使脚本运行直到被外部中断
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping the bag playback...")
        # 当按下Ctrl+C时停止播放并退出
        process.send_signal(subprocess.signal.SIGINT)  # 发送中断信号到rosbag播放进程
        process.wait()  # 等待进程结束
    except Exception as e:
        print("Error occurred: ", e)
    finally:
        if process.poll() is None:  # 如果进程仍在运行，则结束它
            process.terminate()
        print("Bag playback has been stopped.")

if __name__ == '__main__':
    main()

