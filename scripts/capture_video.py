"""Record actual Ubuntu ROS/RViz execution and add explanatory Chinese captions."""
import json
import os
from pathlib import Path
import subprocess
import time

os.environ.update(ROS_DOMAIN_ID='86', ROS_LOCALHOST_ONLY='1', LIBGL_ALWAYS_SOFTWARE='1', QT_X11_NO_MITSHM='1')
import rclpy
from a2_demo.acceptance import Probe
from integration import stop

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/media'
LABELS = {'PICK_LIFT': 'pick', 'WAIT_PROCESS': 'processing', 'UNLOAD_LIFT': 'unload',
          'COMPLETE': 'complete', 'FAILED': 'failed'}


def screenshot(name):
    subprocess.run(['import', '-window', 'root', str(OUT/(name+'.png'))], check=True, timeout=15)


def hold(probe, seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)


def record(scenario, parameters):
    directory = OUT/scenario
    directory.mkdir(parents=True, exist_ok=True)
    with (directory/'launch.log').open('w', encoding='utf-8') as log:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'a2_demo', 'demo.launch.py', f'output_dir:={directory}', *parameters],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        probe = Probe()
        recorder = None
        try:
            probe.until(lambda: probe.status and probe.arm and probe.machine and probe.piece, 20)
            probe.until(lambda: any(info.node_name in ('rviz', 'rviz2')
                                    for info in probe.get_subscriptions_info_by_topic('/a2/scene')), 20)
            hold(probe, 6)
            if launch.poll() is not None:
                raise RuntimeError('ROS launch stopped unexpectedly')
            subprocess.run(['wmctrl', '-r', 'RViz', '-b', 'add,maximized_vert,maximized_horz'], check=True)
            hold(probe, 2)
            screenshot(scenario+'-initial')
            raw = directory/'raw.mp4'
            with (directory/'ffmpeg.log').open('w', encoding='utf-8') as encoding_log:
                recorder = subprocess.Popen(
                    ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'warning', '-f', 'x11grab',
                     '-video_size', '1440x900', '-framerate', '20', '-i', os.environ['DISPLAY'],
                     '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '24',
                     '-pix_fmt', 'yuv420p', str(raw)], stdin=subprocess.PIPE,
                    stdout=encoding_log, stderr=subprocess.STDOUT)
                begin = time.monotonic()
                hold(probe, 22 if scenario == 'normal' else 8)
                started = time.monotonic()-begin
                response = probe.call('start')
                assert response.success, response.message
                probe.until(lambda: probe.status.active, 5)
                run_id = probe.status.run_id
                snapshots = set()
                observed = []
                last_phase = ''
                deadline = time.monotonic()+90
                while time.monotonic() < deadline:
                    rclpy.spin_once(probe, timeout_sec=0.05)
                    phase = probe.status.phase
                    if phase != last_phase:
                        observed.append({'video_s': round(time.monotonic()-begin, 2), 'phase': phase})
                        last_phase = phase
                    if phase in LABELS and phase not in snapshots:
                        hold(probe, 0.3)
                        screenshot(scenario+'-'+LABELS[phase])
                        snapshots.add(phase)
                    if phase in ('COMPLETE', 'FAILED', 'CANCELED'):
                        break
                expected = 'COMPLETE' if scenario == 'normal' else 'FAILED'
                assert probe.status.phase == expected, (probe.status.phase, probe.status.error)
                if scenario != 'normal':
                    assert 'WAIT_PROCESS timeout' in probe.status.error
                terminal = time.monotonic()-begin
                hold(probe, 12 if scenario == 'normal' else 8)
                reset_at = None
                if scenario != 'normal':
                    response = probe.call('reset')
                    assert response.success, response.message
                    probe.until(lambda: probe.status.phase == 'IDLE', 5)
                    reset_at = time.monotonic()-begin
                    hold(probe, 8)
                    screenshot('timeout-reset')
                recorder.communicate(input=b'q\n', timeout=30)
                assert recorder.returncode == 0, 'Screen recorder failed'
                recorder = None
            evidence = json.loads((directory/f'{run_id}.json').read_text(encoding='utf-8'))
            return {'scenario': scenario, 'raw': str(raw), 'start_s': started, 'terminal_s': terminal,
                    'reset_s': reset_at, 'evidence': evidence, 'observed_phases': observed}
        finally:
            if recorder is not None:
                recorder.terminate()
                recorder.wait(timeout=15)
            probe.destroy_node()
            stop(launch)


def caption_filter(captions, stem):
    font = subprocess.check_output(['fc-match', '-f', '%{file}', 'Noto Sans CJK SC'], text=True).strip()
    filters = []
    for index, (start, end, content) in enumerate(captions):
        path = OUT/f'{stem}-caption-{index}.txt'
        path.write_text(content, encoding='utf-8')
        condition = f'between(t,{start:.3f},{end:.3f})'
        filters.extend([
            f"drawbox=x=0:y=806:w=1440:h=94:color=black@0.88:t=fill:enable='{condition}'",
            f"drawtext=fontfile='{font}':textfile='{path}':fontsize=25:fontcolor=white:x=24:y=822:line_spacing=8:enable='{condition}'",
        ])
    return ','.join(filters)


def annotate(recording):
    start, end = recording['start_s'], recording['terminal_s']
    if recording['scenario'] == 'normal':
        captions = [
            (0, 10, 'A2 模拟机床上下料\n目标：把料盘毛坯送入工位加工，再取回成品并归位'),
            (10, start, 'ROS 2 系统：任务节点、机械臂执行、加工工位、场景可视化\nAction 控制动作，Service 请求加工，Topic 反馈状态'),
            (start, end, '完整正常流程，实时 1×，连续录制\n取料、上料、退出工位、等待加工、下料、返回料盘、归位'),
            (end, end+15, '最终结果：COMPLETE，完成数 1/1，绿色成品回到料盘\n系统保存每个阶段的 CSV 和最终 JSON，可重复检查'),
        ]
    else:
        reset = recording['reset_s']
        captions = [
            (0, start, '异常演示：模拟工位不报告加工完成\n加工等待上限为 8 秒，观察超时处理和复位'),
            (start, end, '机械臂完成上料并退出工位，系统等待加工完成信号\n信号持续缺失，系统不会继续下料或计为成功'),
            (end, reset, '失败结果：WAIT_PROCESS timeout，完成数 0/1\n任务中止加工并保存异常原因，等待操作者复位'),
            (reset, reset+12, '复位结果：IDLE，机械臂归位，毛坯恢复到料盘\n故障注入属于启动参数，恢复正常加工需以正常参数启动'),
        ]
    output = OUT/(recording['scenario']+'.mp4')
    subprocess.run(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'warning', '-i', recording['raw'],
                    '-vf', caption_filter(captions, recording['scenario']), '-an', '-c:v', 'libx264',
                    '-preset', 'fast', '-crf', '23', '-pix_fmt', 'yuv420p', str(output)], check=True)
    return output


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    window_manager = subprocess.Popen(['openbox'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    try:
        normal = record('normal', ['motion_seconds:=1.8', 'process_seconds:=7.0'])
        timeout = record('timeout', ['motion_seconds:=1.0', 'stall:=true', 'process_timeout:=8.0'])
        paths = [annotate(normal), annotate(timeout)]
        closing = OUT/'closing.mp4'
        text = '实现：任务状态机、加工互锁、异常处理、结果记录和可视化\n复用：ROS 2、RViz、robot_state_publisher；运动学仿真，无接触力学'
        subprocess.run(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'warning', '-loop', '1',
                        '-framerate', '20', '-i', str(OUT/'normal-complete.png'), '-t', '12',
                        '-vf', caption_filter([(0, 12, text)], 'closing'), '-an', '-c:v', 'libx264',
                        '-preset', 'fast', '-crf', '23', '-pix_fmt', 'yuv420p', str(closing)], check=True)
        paths.append(closing)
        concat = OUT/'concat.txt'
        concat.write_text(''.join(f"file '{p}'\n" for p in paths), encoding='utf-8')
        final = OUT/'A2完整演示.mp4'
        subprocess.run(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'warning', '-f', 'concat',
                        '-safe', '0', '-i', str(concat), '-c', 'copy', '-movflags', '+faststart', str(final)], check=True)
        metadata = json.loads(subprocess.check_output(
            ['ffprobe', '-v', 'quiet', '-show_format', '-show_streams', '-of', 'json', str(final)], text=True))
        duration = float(metadata['format']['duration'])
        assert 120 <= duration <= 180, f'Video must be 2-3 minutes; got {duration}'
        (OUT/'video_manifest.json').write_text(json.dumps(
            {'duration_s': duration, 'normal': normal, 'timeout': timeout,
             'description': 'Actual ROS 2 Humble RViz capture on Ubuntu Xvfb, Chinese captions, real time 1x'},
            ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Video ready: {final}; duration={duration:.1f}s', flush=True)
    finally:
        rclpy.shutdown()
        window_manager.terminate()
        window_manager.wait(timeout=10)


if __name__ == '__main__':
    main()
